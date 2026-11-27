"""
UDP sender for the laptop: sends paddle-angle commands over the network to
src/udp_angle_receiver.py running on the Pi.

For now this just sends manually-typed angles so you two can test the
network link (laptop -> Pi over WiFi) before wiring it into
goalie_vision.py's live CV loop. Once this works, goalie_vision.py's
`motor.move_to(target_angle)` call gets replaced with a sock.sendto() like
the one below.

Run on the laptop:
    python3 src/udp_angle_sender.py --pi-ip 192.168.1.42
"""

import argparse
import json
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-ip", required=True, help="IP address of the Pi on the local network")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0

    print(f"Sending angle commands to {args.pi_ip}:{args.port}. Ctrl-C to quit.")
    try:
        while True:
            response = input("\nTarget angle 0-180 (q to quit): ").strip().lower()
            if response in ("q", "quit", "exit"):
                break
            try:
                angle = float(response)
                if not 0 <= angle <= 180:
                    print("Enter an angle between 0 and 180.")
                    continue
            except ValueError:
                print("Enter an angle or q.")
                continue

            seq += 1
            msg = json.dumps({"seq": seq, "ts": time.time(), "angle": angle}).encode()
            sock.sendto(msg, (args.pi_ip, args.port))
            print(f"sent seq={seq} angle={angle:g}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
