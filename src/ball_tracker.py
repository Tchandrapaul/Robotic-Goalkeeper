"""
Real-time ball detection and tracking.

Loads the HSV range saved by calibrate_color.py, finds the largest matching
blob in each frame, and tracks its center over time. This is the core CV
building block everything else (trajectory prediction, sending a target
position to the motors) will be built on top of.
"""

import json
import sys
import time
from collections import deque

import cv2
import numpy as np

CONFIG_PATH = "config/hsv_config.json"
TRAIL_LENGTH = 32


def load_config() -> dict:
    with open(CONFIG_PATH) as f:
        return json.load(f)


def find_ball(mask: np.ndarray, min_area: float):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None

    (x, y), radius = cv2.minEnclosingCircle(largest)
    return (x, y), radius


def main(camera_index: int = 0) -> None:
    config = load_config()
    lower = np.array([config["h_min"], config["s_min"], config["v_min"]])
    upper = np.array([config["h_max"], config["s_max"], config["v_max"]])
    min_area = config["min_area"]

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}")
        sys.exit(1)

    trail = deque(maxlen=TRAIL_LENGTH)
    prev_time = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame from camera.")
            break

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        result = find_ball(mask, min_area)

        now = time.time()
        fps = 1.0 / (now - prev_time) if now > prev_time else 0.0
        prev_time = now

        if result is not None:
            (x, y), radius = result
            center = (int(x), int(y))
            trail.appendleft(center)

            cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
            cv2.circle(frame, center, 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"({center[0]}, {center[1]}) r={radius:.0f}",
                (center[0] + 10, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        else:
            trail.appendleft(None)

        for i in range(1, len(trail)):
            if trail[i - 1] is None or trail[i] is None:
                continue
            thickness = int(np.sqrt(TRAIL_LENGTH / float(i + 1)) * 2)
            cv2.line(frame, trail[i - 1], trail[i], (0, 255, 255), thickness)

        cv2.putText(
            frame,
            f"FPS: {fps:.0f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        cv2.imshow("ball tracker", frame)
        cv2.imshow("mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
