import pyrealsense2 as rs
import numpy as np
import cv2

# Checkerboard size (INNER corners)
CHECKERBOARD = (8, 5)

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

print("Starting checkerboard detection... Press 'q' to quit")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect checkerboard
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            # refine corners for better accuracy
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            )

            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            cv2.putText(frame, "Detected", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Not detected", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("RealSense Checkerboard", frame)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()


import pyrealsense2 as rs
import numpy as np
import cv2
import os

CHECKERBOARD = (8, 5)

save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

img_count = 0

print("Press SPACE to save image, Q to quit")

try:
    while True:
        frames = pipeline.wait_for_frames()
        frame = frames.get_color_frame()

        if not frame:
            continue

        img = np.asanyarray(frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)

        if ret:
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
            status = "Detected"
        else:
            status = "Not detected"

        cv2.putText(img, status, (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow("Capture Calibration", img)

        key = cv2.waitKey(1)

        # SPACE = save image
        if key == 32 and ret:
            filename = f"{save_dir}/img_{img_count}.png"
            cv2.imwrite(filename, img)
            print("Saved:", filename)
            img_count += 1

        # Q = quit
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()