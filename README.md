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

## Experimental camera-to-paddle preview

This branch adds `src/live_goalie.py` alongside the original tracker. It uses
color-seeded ball outlines, short optical-flow/prediction bridges, approximate
single-camera 3D estimation, and a simulated paddle. It does not command a motor
or implement GPIO homing. The roadmap above describes the original baseline;
these additions are experimental implementations of calibration and prediction.

### Setup and calibration

Use the existing Python environment and requirements. Camera capture currently
uses Linux V4L2. Install `v4l-utils` if `v4l2-ctl` is missing; the runner uses it
to disable dynamic frame rate on each launch when the camera supports it.
A requested 60 FPS is not a guarantee of delivered frame rate.

```bash
cp config/live_settings.example.json config/live_settings.json
```

Edit the local settings for your actual ball diameter, camera position, goal,
and paddle dimensions (meters). The example uses a 6-inch ball and a centered
camera 30 inches above the floor; its 4.5-inch paddle shaft offset is provisional.
The preview speed is an assumption, not a measured motor limit.

Calibrate color for your own ball and lighting with the existing color tool.
Back up shared HSV settings before saving over them; `--config` below can select
an alternative HSV JSON without changing the shared file.

```bash
python3 src/calibrate_floor.py
```

Keep the camera fixed. Place a measured 8-by-10.5-inch notebook flat on the floor,
with the 8-inch edge across the goal and the near edge toward the camera.
Exclude spirals from the detected rectangle. Press Enter to confirm; Space
allows review and corner adjustment, and M allows manual selection.
The first setup asks for the near-edge distance from the blocking line and the
notebook center's sideways offset (zero if centered, image-right positive).
These are physical measurements, not distances inferred from the guide box.
On subsequent runs, use the same physical location or press E to change it.
Recalibrate after moving or tilting the camera. The standalone tool uses camera 0; integrated calibration uses `--camera`.

### Run and capture a problem

```bash
python3 src/live_goalie.py --config config/hsv_config.json
```

Select a tracking rectangle that includes the ball's floor path AND airborne
path. `--camera`, `--settings`, and `--calibration` select alternative inputs;
use `--help` for details. Floor calibration must match the camera and resolution.

- Q: quit.
- R: reselect tracking area and reset tracking history.
- M: show/hide the color mask.
- S: save one snapshot set: camera, overlay, mask, simulated paddle, and state JSON.

Snapshots go into `snapshots/` and are ignored by Git. Capture both successful
and failed tracking and describe what the ball was doing when sharing a set.
These controls belong to `live_goalie.py`; the original tracker is separate.

FOLLOW points the preview toward the estimated current ball location even when
there is no predicted crossing. INTERCEPT uses a predicted goal-line crossing.
HOLD indicates no usable command. Following does not prove a shot can be blocked.

### Current limits

Single-camera airborne depth depends on the detected ball size and measured
geometry. Fast kicks, occlusion, similar-colored objects, and inaccurate outlines
can still break tracking. Prediction bridges are deliberately brief; a drawn
trajectory is not a new measurement. Floor mapping does not correct lens distortion.
The simulation does not establish real motor travel time, switch behavior, floor
clearance, or successful physical interception. Local calibration and snapshots
are intentionally excluded from the branch.

### Recalibrate and track in one launch

```bash
python3 src/live_goalie.py --config config/pink_ball.json --calibrate
```

`--calibrate` opens notebook calibration first, then starts tracking after a
successful save. Omit it to reuse saved calibration. Q cancels calibration and
exits without starting tracking with an old mapping. `--camera` and
`--calibration` apply to both calibration and tracking.

Keep the notebook in its previously measured floor location or press E to edit
its placement. Moving/tilting the camera requires a new calibration. If its
height or position relative to the blocking line changes, also update
`camera_center_m` in your local geometry settings; notebook calibration does not
measure camera height automatically. Existing calibration is backed up on save.
