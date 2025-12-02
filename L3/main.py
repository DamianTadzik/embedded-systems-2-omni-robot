import time
import numpy as np
import cv2
import argparse
import pyrealsense2 as rs
from ultralytics import YOLO
import random
import logging
logging.getLogger("ultralytics").setLevel(logging.ERROR)
import paho.mqtt.client as mqtt
import json

# Fix for numpy compatibility
np.int = int

def get_color_for_id(id):
    """Deterministic BGR color for a given track id"""
    if id is None or id < 0:
        return (48, 170, 73)
    r = (id * 37) % 255
    g = (id * 123) % 255
    b = (id * 73) % 255
    return (int(b), int(g), int(r))


def main(display_image, use_realsense, use_mqtt):
    print("Starting the application...")
    print(f'Display image: {display_image}\nUse RealSense: {use_realsense} \nUse MQTT: {use_mqtt}')

    # MQTT setup
    if use_mqtt:
        MQTT_PUBLISH_TOPIC = "robot/nn_output"
        MQTT_BROKER_IP = "localhost"

        mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        mqttc.connect(MQTT_BROKER_IP, 1883, 60)
        mqttc.loop_start()

    if use_realsense:
        pipeline = rs.pipeline()
        config = rs.config()
        # Enable depth and color; change resolutions if needed
        config.enable_stream(rs.stream.depth, 640, 480,  rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # Start streaming
        profile = pipeline.start(config)

        align_to = rs.stream.color
        align = rs.align(align_to)

        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 2)
        colorizer.set_option(rs.option.histogram_equalization_enabled, 1)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()

        color_intr = color_stream.get_intrinsics()
        depth_intr = depth_stream.get_intrinsics()

    else:
        cam = cv2.VideoCapture(0)

    frame_count = 0
    t0 = time.time()
    fps = 0.0

    # Load YOLO model
    yolo = YOLO("./L3/monster_net_0.2.pt")

    # Tracking State
    selected_target = None
    lost_counter = 0
    MAX_LOST_FRAMES = 200

    last_bbox = None
    last_template = None
    TEMPLATE_MIN_SIZE = 20
    TEMPLATE_MATCH_THRESH = 0.60
    SEARCH_EXPAND = 1.8
    TEMPLATE_UPDATE_INTERVAL = 10
    template_age = 0

    try:
        while True:
            if use_realsense:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                depth_display = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                color_image = np.asanyarray(color_frame.get_data())
            else:
                ret, color_image = cam.read()
                if not ret:
                    continue

            frame_count += 1
            if frame_count % 15 == 0:
                t1 = time.time()
                fps = 15.0 / (t1 - t0)
                t0 = t1

            results = yolo.track(color_image, stream=True)

            found_selected = False

            for result in results:
                class_names = result.names

                for box in result.boxes:
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    if conf > 0.4 and cls == 1:
                        track_id = int(box.id[0]) if box.id is not None else -1
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        if selected_target is None:
                            selected_target = track_id

                        if track_id == selected_target:
                            found_selected = True
                            lost_counter = 0

                            # Save template
                            last_bbox = (x1, y1, x2, y2)
                            template_age = 0
                            w = x2 - x1
                            h = y2 - y1
                            if w >= TEMPLATE_MIN_SIZE and h >= TEMPLATE_MIN_SIZE:
                                last_template = color_image[y1:y2, x1:x2].copy()

                            # 💡 TU jest logika łączona — używamy Twojej funkcji get_bbox_distance_percentile
                            if use_realsense:
                                distance_m = depth_frame.get_distance((x1 + x2)//2, (y1 + y2)//2)
                                class_name += f" {distance_m:.2f} m"

                                print(f"Detected {class_name} with confidence {conf:.2f} at "
                                f"({x1}, {y1}), ({x2}, {y2}) {distance_m:.2f} m away. FPS: {fps:.1f}")

                            # MQTT
                            if use_mqtt:
                                out_data = {
                                    "Cx": float(cx),
                                    "Cy": float(cy),
                                    "distance": float(distance_m) if distance_m is not None else None,
                                }
                                mqttc.publish(MQTT_PUBLISH_TOPIC, json.dumps(out_data, separators=(',', ':')), 0)

                            if display_image:
                                color = get_color_for_id(selected_target)
                                label = f"ID:{selected_target} {distance_m:.2f}m"
                                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                                cv2.putText(color_image, label,
                                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.6, color, 2)

            # ------------------------- TEMPLATE FALLBACK TRACKING -----------------------
            if selected_target is not None and not found_selected and last_template is not None and last_bbox is not None:
                template_age += 1
                ih, iw = color_image.shape[:2]
                x1, y1, x2, y2 = last_bbox
                bw = x2 - x1
                bh = y2 - y1

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                sw = int(min(iw, max(bw * SEARCH_EXPAND, bw + 20)))
                sh = int(min(ih, max(bh * SEARCH_EXPAND, bh + 20)))
                sx1 = max(0, cx - sw // 2)
                sy1 = max(0, cy - sh // 2)
                sx2 = min(iw, sx1 + sw)
                sy2 = min(ih, sy1 + sh)

                search_region = color_image[sy1:sy2, sx1:sx2]
                if search_region.size > 0:
                    search_gray = cv2.cvtColor(search_region, cv2.COLOR_BGR2GRAY)
                    tpl_gray = cv2.cvtColor(last_template, cv2.COLOR_BGR2GRAY)

                    try:
                        res = cv2.matchTemplate(search_gray, tpl_gray, cv2.TM_CCOEFF_NORMED)
                        _, max_val, _, max_loc = cv2.minMaxLoc(res)
                    except Exception:
                        max_val = 0
                        max_loc = (0, 0)

                    if max_val >= TEMPLATE_MATCH_THRESH:
                        mx, my = max_loc
                        nx1 = sx1 + mx
                        ny1 = sy1 + my
                        nx2 = nx1 + last_template.shape[1]
                        ny2 = ny1 + last_template.shape[0]

                        last_bbox = (nx1, ny1, nx2, ny2)
                        if template_age >= TEMPLATE_UPDATE_INTERVAL:
                            last_template = color_image[ny1:ny2, nx1:nx2].copy()
                            template_age = 0

                        found_selected = True
                        lost_counter = 0

                        if use_realsense:
                                distance_m = depth_frame.get_distance((x1 + x2)//2, (y1 + y2)//2)
                                class_name += f" {distance_m:.2f} m"

                        print(f"Detected {class_name} with confidence {conf:.2f} at "
                              f"({x1}, {y1}), ({x2}, {y2}) {distance_m:.2f} m away. FPS: {fps:.1f}")

                        if use_mqtt:
                            out_data = {
                                "Cx": float((nx1 + nx2) // 2),
                                "Cy": float((ny1 + ny2) // 2),
                                "distance": float(distance_m),
                            }
                            mqttc.publish(MQTT_PUBLISH_TOPIC, json.dumps(out_data, separators=(',', ':')), 0)

                        if display_image:
                            color = get_color_for_id(selected_target)
                            cv2.rectangle(color_image, (nx1, ny1), (nx2, ny2), color, 2)
                            cv2.putText(color_image, f"ID:{selected_target} (templ:{max_val:.2f})",
                                        (nx1, ny1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if selected_target is not None and not found_selected:
                lost_counter += 1
                if lost_counter > MAX_LOST_FRAMES:
                    print("Lost target — resetting.")
                    selected_target = None
                    last_bbox = None
                    last_template = None
                    lost_counter = 0
                    template_age = 0

            if display_image:
                if use_realsense:
                    cv2.putText(depth_display, f"FPS: {fps:.1f}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (255, 255, 255), 2)
                    depth_display = cv2.resize(depth_display, (1280, 720))
                    cv2.imshow("RealSense - Depth Information", depth_display)

                cv2.putText(color_image, f"FPS: {fps:.1f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                color_display = cv2.resize(color_image, (1280, 720))
                cv2.imshow("RealSense - Color with Tracking", color_display)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break

    except rs.error as e:
        print("RealSense error:", e)
    except KeyboardInterrupt:
        pass
    finally:
        if use_realsense:
            pipeline.stop()
        else:
            cam.release()
        if use_mqtt:
            mqttc.loop_stop()
        cv2.destroyAllWindows()


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--display_image", default=False, type=str2bool)
    parser.add_argument("-r", "--use_realsense", default=True, type=str2bool)
    parser.add_argument("-m", "--use_mqtt", default=False, type=str2bool)

    args = parser.parse_args()

    main(display_image=args.display_image,
         use_realsense=args.use_realsense,
         use_mqtt=args.use_mqtt)
