# Robotic-Goalkeeper

CV pipeline for a robotic goalie that detects an incoming soccer ball and predicts where to block it.

## Setup

```
python3 -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Step 1: Calibrate the ball's color

Lighting affects this a lot, so calibrate in the same room/lighting you'll actually test in.

```
python3 src/calibrate_color.py
```

Point the camera at the ball. Drag the trackbars until the `mask` window shows
the ball as a solid white blob and everything else black (ignore small
speckles). Press `s` to save, `q` to quit.

## Step 2: Track the ball live

```
python3 src/ball_tracker.py
```

Uses the saved calibration to find the ball each frame, draws a circle on it,
a trail of recent positions, and the current FPS. Press `q` to quit.

If detection is jittery or picks up the wrong thing:
- Re-run calibration with tighter HSV bounds.
- Raise `min_area` in `config/hsv_config.json` if it's picking up small noise.
- Use a ball color that contrasts strongly with the floor/background — this
  matters more than any code tweak.

## Roadmap (what's next)

1. **Color-based detection** (done above) — fast, simple, good enough if the
   ball has a distinct, consistent color.
2. **Camera-to-real-world calibration** — figure out the mapping from pixel
   coordinates to real-world distance (e.g. cm along the goal line), so
   "ball at pixel x=400" becomes "ball is 30cm left of center."
3. **Trajectory prediction** — use several recent (position, timestamp) pairs
   to fit a line/curve and extrapolate where the ball will cross the goal
   line, and roughly when.
4. **Actuator interface** — send the predicted position to whatever is
   controlling the goalie's movement (serial to an Arduino, a socket, etc.),
   early enough for it to actually move in time.
5. **(Optional) Upgrade detection** — if color thresholding isn't reliable
   enough (bad lighting, ball color similar to background), move to a small
   trained object detector (e.g. YOLO) instead of HSV masking.

Steps 2-4 depend on the physical goal setup and actuator hardware, so we'll
tackle those once that's in place.
