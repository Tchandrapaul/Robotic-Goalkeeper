"""Guided notebook calibration. Detected outlines require human confirmation."""
import json
import shutil
import time
from pathlib import Path

import cv2
from camera_setup import configure_camera_timing
import numpy as np

LABELS = ('NEAR LEFT', 'NEAR RIGHT', 'FAR RIGHT', 'FAR LEFT')
WIDTH_M, DEPTH_M = 8 * 0.0254, 10.5 * 0.0254


def ask_number(prompt, minimum=None):
    while True:
        try:
            value = float(input(prompt))
            if np.isfinite(value) and (minimum is None or value >= minimum):
                return value
        except ValueError:
            pass
        print('Enter a finite number within the requested range.')


def guide_bounds(shape):
    height, width = shape[:2]
    return int(width*.20), int(height*.28), int(width*.80), int(height*.93)


def ordered_corners(points):
    """Assumes notebook is aligned with goal, near edge lower in the image."""
    points = np.float32(points).reshape(4, 2)
    by_height = points[np.argsort(points[:, 1])]
    far = by_height[:2][np.argsort(by_height[:2, 0])]
    near = by_height[2:][np.argsort(by_height[2:, 0])]
    return np.float32([near[0], near[1], far[1], far[0]])


def find_notebook_candidate(frame):
    """Find a plausible quadrilateral inside the guide, not notebook identity."""
    x1, y1, x2, y2 = guide_bounds(frame.shape)
    gray = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    height, width = gray.shape
    best, best_score = None, -1
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        corners = cv2.approxPolyDP(contour, .025 * perimeter, True)
        area = abs(cv2.contourArea(corners))
        if len(corners) != 4 or not cv2.isContourConvex(corners):
            continue
        if not 1200 <= area <= .75 * width * height:
            continue
        p = corners.reshape(4, 2)
        if (p[:, 0] <= 3).any() or (p[:, 0] >= width-4).any():
            continue
        if (p[:, 1] <= 3).any() or (p[:, 1] >= height-4).any():
            continue
        p = ordered_corners(p)
        if np.min(np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1)) < 20:
            continue
        if not cv2.isContourConvex(p):
            continue
        offset = np.linalg.norm((p.mean(axis=0)-[width/2, height/2])/[width,height])
        score = area / (1 + 4*offset)
        if score > best_score:
            best, best_score = p + [x1, y1], score
    return None if best is None else np.float32(best)


def make_transform(points, world):
    if len(points) != 4:
        return None
    p = np.float32(points)
    if not np.isfinite(p).all() or not cv2.isContourConvex(p) or abs(cv2.contourArea(p)) < 100:
        return None
    matrix = cv2.getPerspectiveTransform(p, np.float32(world))
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-15:
        return None
    return matrix


def load_saved_placement(path):
    if not path.exists():
        return None, None
    try:
        data = json.loads(path.read_text())
        floor = np.asarray(data['floor_points_m'], dtype=float)
        if floor.shape != (4, 2) or not np.isfinite(floor).all():
            raise ValueError('Invalid floor points')
        near = float(floor[0, 1])
        center = float((floor[0, 0] + floor[1, 0]) / 2)
        expected = np.array([
            [center-WIDTH_M/2, near], [center+WIDTH_M/2, near],
            [center+WIDTH_M/2, near+DEPTH_M], [center-WIDTH_M/2, near+DEPTH_M],
        ])
        if near < 0 or not np.allclose(floor, expected, atol=1e-5, rtol=0):
            raise ValueError('Saved reference is not the expected notebook rectangle')
        return (center, near), data
    except (ValueError, KeyError, TypeError, OSError) as error:
        print(f'Cannot reuse saved location: {error}. Measurements required on save.')
        return None, None


def main(camera=0, output=None):
    print('Lay notebook flat: 8 inches across goal, 10.5 inches toward kicker.')
    print('Guide is only a search area. Do not resize the notebook to match it.')
    print('ENTER confirms and saves the outline; SPACE reviews; M selects corners manually.')
    cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'Could not open camera {camera}.')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    configure_camera_timing(camera)
    window = 'Notebook floor calibration'
    frozen, probe, dragging = None, None, None
    points = []
    world = np.float32([[-WIDTH_M/2, 0], [WIDTH_M/2, 0], [WIDTH_M/2, DEPTH_M], [-WIDTH_M/2, DEPTH_M]])

    def click(event, x, y, flags, param):
        nonlocal probe, dragging
        if frozen is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 4:
                points.append([float(x), float(y)])
            else:
                distances = np.linalg.norm(np.float32(points)-[x,y], axis=1)
                if distances.min() <= 25:
                    dragging = int(distances.argmin())
                else:
                    probe = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and dragging is not None:
            points[dragging] = [float(x), float(y)]
        elif event == cv2.EVENT_LBUTTONUP:
            dragging = None

    output = Path(output) if output is not None else Path(__file__).resolve().parents[1] / 'config' / 'floor_calibration.json'
    placement, saved = load_saved_placement(output)
    edit_placement = False
    if placement is not None:
        print(f'Reusing floor location: near edge {placement[1]/.0254:.2f} in, sideways {placement[0]/.0254:.2f} in.')
        print('Use the SAME physical floor spot. Press E to change its saved measurements.')
    else:
        print('First setup: measurements will be requested once after confirming the outline.')
    try:
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, click)
        while True:
            candidate, matrix = None, None
            if frozen is None:
                ok, frame = cap.read()
                if not ok:
                    print('Camera read failed.')
                    break
                canvas = frame.copy()
                x1,y1,x2,y2 = guide_bounds(frame.shape)
                cv2.rectangle(canvas, (x1,y1), (x2,y2), (255,180,0), 2)
                candidate = find_notebook_candidate(frame)
                lines = ['Place the entire notebook inside the blue guide.',
                         '8-inch edge across goal; near edge toward camera.',
                         'M: manual corners | Q: quit']
                if candidate is not None:
                    cv2.polylines(canvas, [candidate.astype(np.int32)], True, (0,255,255), 2)
                    lines[2] = 'ENTER: confirm notebook + save | SPACE: review | M: manual | Q: quit'
                else:
                    lines[2] = 'No outline found. M: select manually | Q: quit'
            else:
                canvas = frozen.copy()
                matrix = make_transform(points, world)
                lines = ['Check corners match the measured rectangle; EXCLUDE spirals.',
                         'Drag corners to adjust | M: clear corners | R: live view | Q: quit',
                         ('Click '+LABELS[len(points)]) if len(points)<4 else
                         ('ENTER: confirm + save | E: change floor location' if matrix is not None else 'Invalid corners. M: select again')]
                if len(points) == 4:
                    cv2.polylines(canvas, [np.int32(points)], True, (0,255,255), 2)
                for index, point in enumerate(points):
                    pixel = tuple(np.int32(point))
                    cv2.circle(canvas, pixel, 6, (0,255,255), -1)
                    cv2.putText(canvas, LABELS[index], (pixel[0]+8,pixel[1]-8), cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                if matrix is not None and probe is not None:
                    xy = cv2.perspectiveTransform(np.float32([[probe]]), matrix)[0,0]
                    cv2.circle(canvas, probe, 5, (0,255,0), -1)
                    lines.append(f'Notebook-relative floor point: x={xy[0]:.3f} m, y={xy[1]:.3f} m')
            if placement is not None and not edit_placement:
                lines.append(f'Saved spot: near {placement[1]/.0254:.2f} in, sideways {placement[0]/.0254:.2f} in | E: change')
                if frozen is None and saved is not None and saved.get('image_size') == [canvas.shape[1], canvas.shape[0]]:
                    old_points = np.asarray(saved.get('image_points', []), dtype=float)
                    if old_points.shape == (4, 2) and np.isfinite(old_points).all() and np.abs(old_points).max() < 100000:
                        cv2.polylines(canvas, [old_points.astype(np.int32)], True, (255, 0, 255), 1)
                        lines.append('Purple: previous outline, valid placement guide ONLY if camera is unchanged.')
            else:
                lines.append('Floor location: enter measurements in terminal when saving.')
            cv2.rectangle(canvas, (0,0), (canvas.shape[1], 28*len(lines)+12), (20,20,20), -1)
            for index, line in enumerate(lines):
                cv2.putText(canvas,line,(12,26+28*index),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
            if key == ord('e'):
                edit_placement = True
            if key in (10, 13, ord('s')) and frozen is None and candidate is not None:
                frozen, points = frame.copy(), candidate.tolist()
                matrix = make_transform(points, world)
            if key == ord(' ') and frozen is None and candidate is not None:
                frozen, points = frame.copy(), candidate.tolist()
            elif key == ord('m'):
                if frozen is None:
                    frozen = frame.copy()
                points, probe, dragging = [], None, None
            elif key == ord('r'):
                frozen, probe, dragging = None, None, None
                points = []
            elif key in (10, 13, ord('s')) and frozen is not None and matrix is not None:
                if placement is None or edit_placement:
                    print('\nOutline confirmed. Enter this fixed floor location once; it will be remembered.')
                    near = ask_number('Near edge distance in front of blocking line (inches, >=0): ',0)*.0254
                    center = ask_number('Notebook center sideways offset (inches, image-right positive; centered=0): ')*.0254
                else:
                    center, near = placement
                actual_world = world + np.float32([center,near])
                actual_matrix = make_transform(points, actual_world)
                if actual_matrix is None:
                    print('Invalid transform; not saved.')
                    continue
                data = {
                    'image_to_floor': actual_matrix.tolist(), 'image_points': points,
                    'floor_points_m': actual_world.tolist(),
                    'image_size': [frozen.shape[1], frozen.shape[0]],
                    'notebook_size_m': [WIDTH_M, DEPTH_M],
                    'placement_m': {'center_x': center, 'near_y': near},
                    'coordinates': 'x right as viewed by camera; y outward from blocking line; meters',
                    'limitations': 'Floor only, not ball centers. Lens distortion uncorrected. Small reference; independently validate. Recalibrate after camera/optics change.',
                }
                output.parent.mkdir(parents=True, exist_ok=True)
                if output.exists():
                    backup = output.with_name(f'floor_calibration.backup-{time.time_ns()}.json')
                    shutil.copy2(output, backup)
                    print('Previous calibration backed up:',backup)
                temporary = output.with_suffix('.json.tmp')
                temporary.write_text(json.dumps(data,indent=2)+'\n')
                temporary.replace(output)
                print('Saved:',output)
                return True
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return False


if __name__ == '__main__':
    main()
