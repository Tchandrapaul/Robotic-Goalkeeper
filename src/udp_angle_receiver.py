"""
UDP receiver for the Pi: listens for paddle-angle commands sent over the
network by src/udp_angle_sender.py (running on the laptop) and drives the
stepper via move_goalie.Goalie.

Drops packets that are out of order (lower seq than the last one acted on)
or too old (sat around too long before arriving) rather than queuing them --
with a moving ball, a stale angle is worse than a dropped one.

Run on the Pi:
    python3 src/udp_angle_receiver.py --dry-run     # no motor, just prints received angles
    python3 src/udp_angle_receiver.py                # live, drives the motor
"""

import argparse
import json
import socket
import time

from move_goalie import Goalie, max_position, min_position

MAX_PACKET_AGE_S = 0.5  # drop packets older than this (stale/queued on a flaky link)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--max-rpm", type=float, default=60)
    parser.add_argument("--chip", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true", help="Print received angles without opening GPIO or moving the motor.")
    args = parser.parse_args()

    print("GOALIE UDP RECEIVER | 0-180 degrees | center = 90 degrees")
    print("WARNING: NO HOMING SENSOR, NO PHYSICAL LIMIT MONITORING, NO PI ENCODER FEEDBACK.")
    print("Use a physical emergency cutoff; STOP if the paddle binds or driver alarms.")
    if not args.dry_run:
        prompt = input(
            "Power OFF to align the paddle physically at 90deg, "
            "then power ON. Type CENTERED to arm: "
        ).strip()
        if prompt != "CENTERED":
            print("Not armed. Exiting without motor movement.")
            return

    motor = Goalie(args.max_rpm, args.chip, args.dry_run)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening for UDP angle commands on port {args.port}...")

    last_seq = -1
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            try:
                msg = json.loads(data.decode())
                seq = int(msg["seq"])
                sent_at = float(msg["ts"])
                angle = float(msg["angle"])
            except (KeyError, ValueError, json.JSONDecodeError):
                print(f"Dropping malformed packet from {addr}")
                continue

            if seq <= last_seq:
                print(f"Dropping out-of-order packet (seq={seq}, last={last_seq})")
                continue
            last_seq = seq

            age = time.time() - sent_at
            if age > MAX_PACKET_AGE_S:
                print(f"Dropping stale packet (seq={seq}, age={age:.3f}s)")
                continue

            if not min_position <= angle <= max_position:
                print(f"Dropping out-of-range angle {angle} from {addr}")
                continue

            print(f"seq={seq} angle={angle:.1f} age={age * 1000:.0f}ms from {addr}")
            motor.move_to(angle)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sock.close()
        motor.close()
        print("Pulse output stopped. Driver may still hold motor torque.")


if __name__ == "__main__":
    main()
