"""Restrict color detection without changing camera/calibration coordinates."""
import cv2
import numpy as np


def select_tracking_area(frame):
    title = 'Select ball approach area - drag, ENTER confirm, C cancel'
    print('Drag around the floor/ball approach area, excluding furniture. ENTER confirms; C cancels.', flush=True)
    try:
        x, y, width, height = cv2.selectROI(title, frame, showCrosshair=True, fromCenter=False)
    finally:
        cv2.destroyWindow(title)
    if width <= 0 or height <= 0:
        return None
    return int(x), int(y), int(width), int(height)


def area_mask(frame_shape, area):
    height, width = frame_shape[:2]
    x, y, w, h = area
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > width or y+h > height:
        raise ValueError('Tracking area is outside the image; select it again.')
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y:y+h, x:x+w] = 255
    return mask
