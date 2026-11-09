"""
Real-time ball detection and tracking.

Loads the HSV range saved by calibrate_color.py, finds the largest matching
blob in each frame, and tracks its center over time. This is the core CV
building block everything else (trajectory prediction, sending a target
position to the motors) will be built on top of.
"""


"""
Updates as of Nov 8th 2026.

Groups nearby color patches, estimates image velocity from timestamped
detections, and uses predicted positions to guide the next search.
A bounded fallback searches near the last accepted detection.

Predictions guide detection only; missing detections are not displayed as
tracked positions. Coordinates and velocities remain in image pixels.
Press 's' to toggle automatic snapshots and 'q' to quit.


"""

import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np


CONFIG_PATH = "config/pink_ball.json"
TRAIL_LENGTH = 32
MAX_SPEED_PX_S = 2500.0
POSITION_SLACK_PX = 20.0
TRACK_TIMEOUT_S = 0.25


def load_config() -> dict:
    with open(CONFIG_PATH) as f:
        return json.load(f)


def find_ball(mask, min_area, previous_ball=None, search_padding=0.0):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    if previous_ball is None:
        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < min_area:
            return None

        return cv2.minEnclosingCircle(largest)

    (previous_x, previous_y), previous_radius = previous_ball
    search_radius = max(40.0, previous_radius * 1.5) + search_padding

    nearby = []

    for contour in contours:
        if cv2.contourArea(contour) < 30:
            continue

        moments = cv2.moments(contour)

        if moments["m00"] <= 0:
            continue

        patch_x = moments["m10"] / moments["m00"]
        patch_y = moments["m01"] / moments["m00"]

        distance = np.hypot(
            patch_x - previous_x,
            patch_y - previous_y
        )

        if distance <= search_radius:
            nearby.append(contour)

    if not nearby:
        return None

    total_area = sum(cv2.contourArea(c) for c in nearby)

    if total_area < min_area:
        return None

    combined_points = np.concatenate(nearby, axis=0)

    return cv2.minEnclosingCircle(combined_points)


def estimate_velocity(observations):
    if len(observations) >= 2:
    
        samples = np.array(observations, dtype=float)
    
        times = samples[:, 0]
        x_positions = samples[:, 1]
        y_positions = samples[:, 2]
    
        relative_times = times - times[-1]
        centered_times = relative_times - relative_times.mean()
     
        denominator = np.sum(centered_times ** 2)
     
        if denominator > 1e-12:
        
            vx = np.sum(centered_times * (x_positions - x_positions.mean())) / denominator
            vy = np.sum(centered_times * (y_positions - y_positions.mean())) / denominator
     
            return  float(vx), float(vy)                  


def main(camera_index: int = 0) -> None:
    config = load_config()
    lower = np.array([config["h_min"], config["s_min"], config["v_min"]])
    upper = np.array([config["h_max"], config["s_max"], config["v_max"]])
    min_area = config["min_area"]

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}")
        sys.exit(1)

    #Request 720p60; actual delivered FPS must be measured separately.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)

    #Four buffers improved camera FPS in Brio tests; frame latency is currently untested.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    print("Camera reports:", int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "x", int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), "at", cap.get(cv2.CAP_PROP_FPS), "FPS")


    trail = deque(maxlen=TRAIL_LENGTH)
    observations = deque(maxlen=10);
    prev_time = time.time()
    last_diagnostic_print = time.monotonic()
    previous_ball = None
    auto_snapshots = False
    last_snapshot_time = 0.0
    
    while True:
       
        read_started = time.monotonic()
        ok, frame = cap.read()
        read_finished = time.monotonic()


        if not ok:
            print("Failed to read frame from camera.")
            break


        raw_frame = frame.copy()
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        close_size = config.get("close_kernel", 0)


        if(close_size > 0):
            kernel = np.ones((close_size, close_size), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2. CHAIN_APPROX_SIMPLE
        )

        #Diagnostic only: largest individual patch, not the combined tracking group.
        largest_area = max(
            (cv2.contourArea(contour) for contour in contours),
            default=0.0,
        )

        observation_time = time.monotonic()

        #Discards stale measurements before searching for the ball again.
        if observations:
            time_since_seen = observation_time - observations[-1][0]

            if time_since_seen > TRACK_TIMEOUT_S:
                observations.clear()
                previous_ball = None


        search_ball = previous_ball


        if previous_ball is not None and observations:
            search_velocity = estimate_velocity(observations)


            if search_velocity is not None:
                last_time, last_x, last_y = observations[-1]
                vx_search, vy_search = search_velocity

                prediction_dt = observation_time - last_time

                predicted_x = last_x + vx_search * prediction_dt
                predicted_y = last_y + vy_search * prediction_dt

                _, previous_radius = previous_ball

                search_ball = (
                    (predicted_x, predicted_y),
                    previous_radius,
                )

        result = find_ball(mask, min_area, search_ball)
        used_fallback = False

        #If prediction fails, retry near the last measurement with extra reach.
        if result is None and previous_ball is not None and observations:
            elapsed = max( 0.0, observation_time - observations[-1][0])

            padding = min(120.0, MAX_SPEED_PX_S * elapsed + POSITION_SLACK_PX)

            result = find_ball(mask, min_area, previous_ball, search_padding=padding)

            used_fallback = result is not None


        if result is None:
            tracking_status = "No qualifying color group"
        elif used_fallback:
            tracking_status = "Fallback candidate found"
        else:
            tracking_status = "Color candidate found"


        #Reject not possible displacement before updating the measured history.
        if result is not None and observations:
            (candidate_x, candidate_y), _ = result
            last_time, last_x, last_y = observations[-1]

            dt = observation_time - last_time
            distance = np.hypot(candidate_x - last_x, candidate_y - last_y)

            allowed_distance = (MAX_SPEED_PX_S * dt + POSITION_SLACK_PX)

            if dt <= 0 or distance > allowed_distance:
                result = None
                tracking_status = "Rejected: position jump"

        now = time.time()
        fps = 1.0 / (now - prev_time) if now > prev_time else 0.0
        prev_time = now
        velocity = None


        if result is not None:

            (x, y), radius = result
            previous_ball = result

            #Store accepted detections only, never predicted positions
            observations.append((observation_time, x, y))
            velocity = estimate_velocity(observations)
            
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


        if velocity is not None:
            vx, vy = velocity
            arrow_seconds = 0.15
            end_point = (int(center[0] + vx * arrow_seconds), int(center[1] + vy * arrow_seconds))
            cv2.arrowedLine(frame, center, end_point,(255, 255, 0), 2, tipLength=0.25)
            label = f"vx: {vx:.0f} vy: {vy:.0f} px/s"
        else:
            label = "Velocity waiting for observations"

        cv2.putText(frame, label, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.putText(frame, tracking_status, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        processing_finished = time.monotonic()
        read_ms = (read_finished - read_started) * 1000
        processing_ms = (processing_finished - read_finished) * 1000

        diagnostic_time = time.monotonic()



        if diagnostic_time - last_diagnostic_print >= 0.5:
            print(
                f"FPS: {fps:.1f} | "
                f"Read: {read_ms:.1f} ms | "
                f"Process: {processing_ms:.1f} ms | "
                f"Area: {largest_area:.0f}/{min_area} | "
                f"{tracking_status}",
                flush=True,
            )

            last_diagnostic_print = diagnostic_time

        cv2.putText(
            frame, 
            f"Read: {read_ms:.1f} ms  Process: {processing_ms:.1f} ms", 
            (10, 100), 
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        cv2.putText(
            frame,
            f"Largest color area: {largest_area:.0f} / minimum {min_area}",
            (10, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        cv2.imshow("ball tracker", frame)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF


        if key == ord("q"):
            break

        elif key == ord("s"):
            auto_snapshots = not auto_snapshots
            print(
                "Automatic snapshots ON"
                if auto_snapshots
                else "Automatic snapshots OFF",
                flush=True,
            )

        snapshot_time = time.monotonic()

        # Disk writes can interrupt capture; disable snapshots when measuring FPS.
        if auto_snapshots and snapshot_time - last_snapshot_time >= 0.5:
            last_snapshot_time = snapshot_time

            snapshot_dir = (
                Path(__file__).resolve().parents[1]
                / "snapshots"
                / str(time.time_ns())
            )
            snapshot_dir.mkdir(parents=True, exist_ok=False)

            raw_saved = cv2.imwrite(
                str(snapshot_dir / "camera.png"), raw_frame
            )
            overlay_saved = cv2.imwrite(
                str(snapshot_dir / "overlay.png"), frame
            )
            mask_saved = cv2.imwrite(
                str(snapshot_dir / "mask.png"), mask
            )

            if raw_saved and overlay_saved and mask_saved:
                print(f"Snapshot saved: {snapshot_dir}", flush=True)
            else:
                print(f"Snapshot incomplete: {snapshot_dir}", flush=True)


    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
