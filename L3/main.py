import time
import numpy as np
import cv2
import argparse
import pyrealsense2 as rs
from ultralytics import YOLO
import random

# Fix for numpy compatibility
np.int = int

def getColors(cls_num):
    """Generate unique colors for each class ID"""
    random.seed(cls_num)
    return tuple(random.randint(0, 255) for _ in range(3))


def main(display_image, use_realsense):
    print("Starting the application...")
    print(f'Display image: {display_image}, Use RealSense: {use_realsense}')
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

        print("[Color intrinsics]", color_intr)
        print("[Depth intrinsics]", depth_intr)
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
    yolo = YOLO("monster_net_0.2.pt")

    # Model notes:
    # monster_net_0.2.pt - currently best
    # monster_net_0.3.pt - more conservative model


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


            # FPS update
            frame_count += 1
            if frame_count % 15 == 0:
                t1 = time.time()
                fps = 15.0 / (t1 - t0)
                t0 = t1

            # Can detection and visualization
            results = yolo.track(color_image, stream=True)

            if display_image:
                for result in results:
                    class_names = result.names
                    for box in result.boxes:
                        if box.conf[0] > 0.4 and box.cls[0] == 1:
                            cls = int(box.cls[0])
                            class_name = class_names[cls]

                            x1, y1, x2, y2 = map(int, box.xyxy[0])

                            conf = float(box.conf[0])

                            colour = [48, 170, 73] # Monsterish colour

                            cv2.rectangle(color_image, (x1, y1), (x2, y2), colour, 2)

                            if use_realsense:
                                distance_m = depth_frame.get_distance((x1 + x2)//2, (y1 + y2)//2)
                                class_name += f" {distance_m:.2f} m"

                            cv2.putText(color_image, f"{class_name} {conf:.2f}",
                                        (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, colour, 2)


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

    args = parser.parse_args()

    main(display_image=args.display_image, use_realsense=args.use_realsense)
