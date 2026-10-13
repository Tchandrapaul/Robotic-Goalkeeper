"""
Diagnostic: checks each camera index, reports whether it opens and whether
it returns real (non-black) frames, and saves one snapshot per working
camera so you can inspect it without relying on a live window.
"""

import cv2
import numpy as np

MAX_INDEX_TO_TRY = 4


def main() -> None:
    found_any = False

    for index in range(MAX_INDEX_TO_TRY + 1):
        cap = cv2.VideoCapture(index)
        opened = cap.isOpened()

        if not opened:
            print(f"index {index}: could not open")
            cap.release()
            continue

        ok, frame = cap.read()
        cap.release()

        if not ok or frame is None:
            print(f"index {index}: opened but could not read a frame")
            continue

        brightness = float(np.mean(frame))
        found_any = True
        print(f"index {index}: opened, frame shape {frame.shape}, mean brightness {brightness:.1f}")

        if brightness < 1.0:
            print(f"  -> frame is solid black. Likely a camera permission issue, not a code issue.")

        filename = f"snapshot_index{index}.jpg"
        cv2.imwrite(filename, frame)
        print(f"  -> saved {filename}, open it to check if it looks like a real picture")

    if not found_any:
        print("\nNo camera could be opened at all.")
        print("On macOS: System Settings -> Privacy & Security -> Camera -> ")
        print("make sure your terminal app (Terminal/iTerm2/VS Code) is checked ON.")
        print("You may need to fully quit and reopen the terminal app after granting permission.")


if __name__ == "__main__":
    main()
