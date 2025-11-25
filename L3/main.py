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
from collections import deque
import math

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
        # --- Configure streams ---
        pipeline = rs.pipeline()
        config = rs.config()
        # Enable depth and color; change resolutions if needed
        config.enable_stream(rs.stream.depth, 640, 480,  rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = pipeline.start(config)
        # profile = pipeline.start()

        # Align depth to color stream
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Nice-looking depth display
        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 2)  # 0..9; 2 = “Jet”
        colorizer.set_option(rs.option.histogram_equalization_enabled, 1)

        # Print intrinsics once
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()

        color_intr = color_stream.get_intrinsics()
        depth_intr = depth_stream.get_intrinsics()

    else:
        cam = cv2.VideoCapture(0)

        # Get the default frame width and height
        frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # FPS tracking
    frame_count = 0
    t0 = time.time()
    fps = 0.0

    # Load YOLO model
    yolo = YOLO("./monster_net_0.2.pt")

    # Model notes:
    # monster_net_0.2.pt - currently best
    # monster_net_0.3.pt - more conservative modell

# Global tracking state
    selected_target = None          # Track ID we're following
    lost_counter = 0                # How many frames it's been lost
    MAX_LOST_FRAMES = 120            # After this, we drop tracking (shorter than before)

    # Template-match fallback params
    last_bbox = None                 # (x1,y1,x2,y2) of last known bbox
    last_template = None             # last cropped template (BGR)
    last_depth = None                # last measured depth at template center (meters)
    last_centers = deque(maxlen=5)   # recent centers for simple motion check

    TEMPLATE_MIN_SIZE = 20           # smallest template side to consider
    TEMPLATE_MATCH_THRESH = 0.65     # normalized matchTemplate threshold (0..1) - raised
    HIST_THRESH = 0.5                # HSV histogram correlation threshold (0..1)
    SEARCH_EXPAND = 1.4              # expand search window by this factor (slightly smaller)
    TEMPLATE_UPDATE_INTERVAL = 10    # frames before refreshing template from detection
    MAX_SHIFT_PIXELS = 80            # reject matches that jump too far from recent motion
    DEPTH_TOLERANCE = 0.35           # meters - require similar depth when using RealSense

    template_age = 0
    try:
        while True:
            if use_realsense:
                # Wait for a coherent pair of frames
                frames = pipeline.wait_for_frames()

                # Align depth to color
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert to NumPy
                depth_display = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                color_image = np.asanyarray(color_frame.get_data())
            else:
                ret, color_image = cam.read()
                if not ret:
                    # skip when camera read fails
                    continue

            # FPS update
            frame_count += 1
            if frame_count % 15 == 0:
                t1 = time.time()
                fps = 15.0 / (t1 - t0)
                t0 = t1

            # Can detection and visualization
            results = yolo.track(color_image, stream=True)

            found_selected = False

            for result in results:
                class_names = result.names

                for box in result.boxes:
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    if conf > 0.4 and cls == 1:   # Class 1 = Monster can

                        # Extract track ID
                        track_id = int(box.id[0]) if box.id is not None else -1

                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        # First time: lock onto first seen target
                        if selected_target is None:
                            selected_target = track_id

                        # If this detection corresponds to selected target, use it
                        if track_id == selected_target:
                            found_selected = True
                            lost_counter = 0  # reset lost flag

                            # Save last bbox and template for fallback tracking
                            last_bbox = (x1, y1, x2, y2)
                            template_age = 0
                            w = x2 - x1
                            h = y2 - y1
                            if w >= TEMPLATE_MIN_SIZE and h >= TEMPLATE_MIN_SIZE:
                                last_template = color_image[y1:y2, x1:x2].copy()

                            # Depth reading
                            distance_m = None
                            if use_realsense and depth_frame:
                                try:
                                    distance_m = float(depth_frame.get_distance(cx, cy))
                                    last_depth = distance_m
                                except:
                                    distance_m = None

                            # update recent centers
                            last_centers.append((cx, cy))

                            # Publish minimal target data
                            if use_mqtt:
                                out_data = {
                                    "Cx": float(cx),
                                    "Cy": float(cy),
                                    "distance": float(distance_m) if distance_m is not None else None
                                }
                                out_msg = json.dumps(out_data, separators=(',', ':'))
                                mqttc.publish(MQTT_PUBLISH_TOPIC, out_msg, 0)

                            # Show tracking on screen
                            if display_image:
                                color = get_color_for_id(selected_target)
                                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                                label = f"ID:{selected_target} {distance_m:.2f}m" if use_realsense and distance_m is not None else f"ID:{selected_target}"
                                cv2.putText(color_image, label,
                                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.6, color, 2)

            # If selected target wasn't found by detector, try template-match fallback
            if selected_target is not None and not found_selected and last_template is not None and last_bbox is not None:
                template_age += 1

                ih, iw = color_image.shape[:2]
                x1, y1, x2, y2 = last_bbox
                bw = x2 - x1
                bh = y2 - y1

                # Build expanded search window around last bbox
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                sw = int(min(iw, max(bw * SEARCH_EXPAND, bw + 20)))
                sh = int(min(ih, max(bh * SEARCH_EXPAND, bh + 20)))
                sx1 = max(0, cx - sw // 2)
                sy1 = max(0, cy - sh // 2)
                sx2 = min(iw, sx1 + sw)
                sy2 = min(ih, sy1 + sh)

                search_region = color_image[sy1:sy2, sx1:sx2]
                ok_size = (search_region.size > 0 and last_template.size > 0 and
                           last_template.shape[0] < search_region.shape[0] and last_template.shape[1] < search_region.shape[1])

                if ok_size:
                    # Use grayscale normalized cross-correlation
                    search_gray = cv2.cvtColor(search_region, cv2.COLOR_BGR2GRAY)
                    tpl_gray = cv2.cvtColor(last_template, cv2.COLOR_BGR2GRAY)

                    try:
                        res = cv2.matchTemplate(search_gray, tpl_gray, cv2.TM_CCOEFF_NORMED)
                        _, max_val, _, max_loc = cv2.minMaxLoc(res)
                    except Exception:
                        max_val = 0
                        max_loc = (0, 0)

                    # Basic HSV histogram similarity check to reduce false positives
                    cand_x, cand_y = max_loc
                    cand_x_full = sx1 + cand_x
                    cand_y_full = sy1 + cand_y
                    nx1 = cand_x_full
                    ny1 = cand_y_full
                    nx2 = nx1 + last_template.shape[1]
                    ny2 = ny1 + last_template.shape[0]

                    hist_ok = False
                    try:
                        tpl_hsv = cv2.cvtColor(last_template, cv2.COLOR_BGR2HSV)
                        cand_patch = color_image[ny1:ny2, nx1:nx2]
                        if cand_patch.shape[:2] == tpl_hsv.shape[:2]:
                            cand_hsv = cv2.cvtColor(cand_patch, cv2.COLOR_BGR2HSV)
                            h_bins = 50
                            s_bins = 60
                            tpl_hist = cv2.calcHist([tpl_hsv], [0, 1], None, [h_bins, s_bins], [0, 180, 0, 256])
                            cand_hist = cv2.calcHist([cand_hsv], [0, 1], None, [h_bins, s_bins], [0, 180, 0, 256])
                            cv2.normalize(tpl_hist, tpl_hist, 0, 1, cv2.NORM_MINMAX)
                            cv2.normalize(cand_hist, cand_hist, 0, 1, cv2.NORM_MINMAX)
                            hist_score = cv2.compareHist(tpl_hist, cand_hist, cv2.HISTCMP_CORREL)
                            hist_ok = (hist_score >= HIST_THRESH)
                        else:
                            hist_ok = False
                    except Exception:
                        hist_ok = False

                    # depth and motion checks
                    center_new = ((nx1 + nx2) // 2, (ny1 + ny2) // 2)
                    shift = math.hypot(center_new[0] - ((x1 + x2)//2), center_new[1] - ((y1 + y2)//2))
                    motion_ok = True
                    if last_centers:
                        # require not too far from recent motion (prevents long jumps)
                        avg_cx = int(sum(c[0] for c in last_centers) / len(last_centers))
                        avg_cy = int(sum(c[1] for c in last_centers) / len(last_centers))
                        motion_ok = (math.hypot(center_new[0]-avg_cx, center_new[1]-avg_cy) <= max(MAX_SHIFT_PIXELS, bw*1.5))

                    depth_ok = True
                    if use_realsense and depth_frame and last_depth is not None:
                        try:
                            d_new = float(depth_frame.get_distance(center_new[0], center_new[1]))
                            depth_ok = (abs(d_new - last_depth) <= DEPTH_TOLERANCE)
                        except:
                            depth_ok = True

                    # Accept match only if combined checks pass
                    if max_val >= TEMPLATE_MATCH_THRESH and hist_ok and motion_ok and depth_ok:
                        # Found a good match; compute new bbox in full image coords
                        last_bbox = (nx1, ny1, nx2, ny2)
                        if template_age >= TEMPLATE_UPDATE_INTERVAL:
                            w = nx2 - nx1
                            h = ny2 - ny1
                            if w >= TEMPLATE_MIN_SIZE and h >= TEMPLATE_MIN_SIZE:
                                last_template = color_image[ny1:ny2, nx1:nx2].copy()
                            template_age = 0

                        found_selected = True
                        lost_counter = 0

                        # Depth reading
                        cx = (nx1 + nx2) // 2
                        cy = (ny1 + ny2) // 2
                        if use_realsense and depth_frame:
                            try:
                                last_depth = float(depth_frame.get_distance(cx, cy))
                            except:
                                pass

                        last_centers.append((cx, cy))

                        # Publish if needed
                        if use_mqtt:
                            out_data = {
                                "Cx": float(cx),
                                "Cy": float(cy),
                                "distance": float(last_depth) if last_depth is not None else None,
                                "id": int(selected_target),
                                "match": float(max_val)
                            }
                            mqttc.publish(MQTT_PUBLISH_TOPIC, json.dumps(out_data, separators=(',', ':')), 0)

                        # Draw fallback bbox
                        if display_image:
                            color = get_color_for_id(selected_target)
                            cv2.rectangle(color_image, (nx1, ny1), (nx2, ny2), color, 2)
                            cv2.putText(color_image, f"ID:{selected_target} (templ:{max_val:.2f})",
                                        (nx1, ny1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # If template matching failed, increment lost counter below

            # After iterating detections & fallback, handle lost logic
            if selected_target is not None and not found_selected:
                lost_counter += 1
                if lost_counter > MAX_LOST_FRAMES:
                    print("Lost target, resetting tracking...")
                    selected_target = None
                    last_bbox = None
                    last_template = None
                    last_depth = None
                    last_centers.clear()
                    lost_counter = 0
                    template_age = 0


            if display_image:
                if use_realsense:
                    # Add FPS to depth image as well
                    cv2.putText(depth_display, f"FPS: {fps:.1f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    depth_display = cv2.resize(depth_display, (1280, 720))

                    cv2.imshow("RealSense - Depth Information", depth_display)


                # Display FPS on color image
                cv2.putText(color_image, f"FPS: {fps:.1f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # Resize images for display
                color_display = cv2.resize(color_image, (1280, 720))

                # Display both windows
                cv2.imshow("RealSense - Color with Face Detection", color_display)

                # Quit with 'q' or ESC
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
    parser.add_argument("-d", "--display_image", default=False, type=str2bool,
                        help="Display image (True/False)")
    parser.add_argument("-r", "--use_realsense", default=True, type=str2bool,
                        help="Use a RealSense camera (True/False)")
    parser.add_argument("-m", "--use_mqtt", default=False, type=str2bool,
                        help="Use MQTT to publish data (True/False)")

    args = parser.parse_args()

    main(display_image=args.display_image,
         use_realsense=args.use_realsense,
         use_mqtt=args.use_mqtt)
