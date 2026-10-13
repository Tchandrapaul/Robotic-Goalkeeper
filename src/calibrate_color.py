"""
Interactive HSV calibration tool.

Run this with the ball in front of the camera, in the same lighting you'll
use for the real thing (lighting changes the numbers a lot). Drag the
trackbars until the "mask" window shows ONLY the ball as solid white and
everything else as black. Press 's' to save the range to
config/hsv_config.json, 'q' to quit without saving.
"""

import json
import sys

import cv2
import numpy as np

CONFIG_PATH = "config/hsv_config.json"


def nothing(_value):
    pass


def main(camera_index: int = 0) -> None:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}")
        sys.exit(1)

    cv2.namedWindow("controls")
    cv2.createTrackbar("H min", "controls", 0, 179, nothing)
    cv2.createTrackbar("H max", "controls", 179, 179, nothing)
    cv2.createTrackbar("S min", "controls", 0, 255, nothing)
    cv2.createTrackbar("S max", "controls", 255, 255, nothing)
    cv2.createTrackbar("V min", "controls", 0, 255, nothing)
    cv2.createTrackbar("V max", "controls", 255, 255, nothing)

    print("Adjust trackbars until only the ball is white in the mask window.")
    print("Press 's' to save, 'q' to quit without saving.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame from camera.")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H min", "controls")
        h_max = cv2.getTrackbarPos("H max", "controls")
        s_min = cv2.getTrackbarPos("S min", "controls")
        s_max = cv2.getTrackbarPos("S max", "controls")
        v_min = cv2.getTrackbarPos("V min", "controls")
        v_max = cv2.getTrackbarPos("V max", "controls")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        cv2.imshow("camera", frame)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            config = {
                "h_min": h_min,
                "h_max": h_max,
                "s_min": s_min,
                "s_max": s_max,
                "v_min": v_min,
                "v_max": v_max,
                "min_area": 300,
            }
            with open(CONFIG_PATH, "w") as f:
                json.dump(config, f, indent=2)
            print(f"Saved to {CONFIG_PATH}: {config}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
