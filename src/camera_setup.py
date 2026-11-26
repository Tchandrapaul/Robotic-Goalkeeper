"""Apply the camera control verified in Brio tests on every startup."""
import re
import subprocess


def configure_camera_timing(camera_index=0):
    device = f'/dev/video{camera_index}'
    command = [
        'v4l2-ctl', f'--device={device}',
        '--set-ctrl=exposure_dynamic_framerate=0',
        '--get-ctrl=exposure_dynamic_framerate',
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    except FileNotFoundError:
        print('CAMERA WARNING: v4l2-ctl missing; could not disable dynamic frame rate.', flush=True)
        return False
    except subprocess.TimeoutExpired:
        print('CAMERA WARNING: setting frame-rate control timed out.', flush=True)
        return False
    if result.returncode != 0 or not re.search(r'exposure_dynamic_framerate:\s*0\b', result.stdout):
        detail = (result.stderr or result.stdout).strip()
        print(f'CAMERA WARNING: dynamic frame rate was not verified off on {device}: {detail}', flush=True)
        return False
    print(f'CAMERA: dynamic frame rate OFF on {device} (verified); delivered FPS still measured separately.', flush=True)
    return True
