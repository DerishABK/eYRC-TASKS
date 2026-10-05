# Task 2A - Arena Navigation Boilerplate

Niti Vahan (NV), eYRC 2026-27.

## Files

| File | What it is |
|---|---|
| `task2a.py` | The boilerplate. Fill in `detect_lane()` and `compute_control()`; leave everything else alone. |
| `Task_2A.ttt` | The CoppeliaSim scene. Open it before you run anything. |

## Requirements

Your `NV_<Team-ID>` environment from Task 0 already has what you need - nothing
new to install since Task 1:

```sh
conda activate NV_<Team-ID>
python -c "import cv2, numpy, coppeliasim_zmqremoteapi_client; print('ok')"
```

## Usage

1. Launch CoppeliaSim and open `Task_2A.ttt`.
2. Press **▶ Play**.
3. In a terminal:

```sh
conda activate NV_<Team-ID>
python task2a.py
```

Press **⏹ Stop** then **▶ Play** to put the vehicle back at its starting point
between runs.

## What you implement

Two functions, and nothing else:

```python
def detect_lane(frame):
    # frame is a corrected BGR image from the vehicle's camera
    return <anything compute_control() can use>

def compute_control(detection, stop_requested):
    return left_angle, right_angle, speed
```

- `left_angle`, `right_angle` - the two **front-wheel** angles, in **radians**.
  The scene's steering joints are **positive to the left**. Nothing is clamped
  for you; the vehicle's own limit is 45°. Use your Task 1A geometry to split
  one steering angle into these two.
- `speed` - drive motor target, in rad/s. `0.0` holds the vehicle still.
- `stop_requested` - `True` while the evaluation is holding you at a stop point.
  It is a **level**, not a pulse: stay stationary for as long as it is `True`.
  Do not time the stop yourself.

Everything below the `DO NOT EDIT BELOW THIS LINE` marker is provided: the
connection, the handle lookup, the camera read, the four joint writes and the
loop. None of it is a decision you are marked on - it is there so every team
reads the same picture and drives the same vehicle.

## Where the marks are

Most of Task 2A is **how closely you hold the middle of your lane**, measured
against a centreline surveyed from the painted road edges and the dashed
divider. Being on the correct side of the divider is not enough - weaving
across your own lane loses marks. The inner lane is worth more than the outer
one.

The evaluation tool on your machine **records** the two laps; the marks are
worked out on the portal when you upload the recording. So the tool will not
print a score - watch the simulator instead, and submit to see the marks.

## What your script must never do

The evaluation resets the scene, places the vehicle in the lane under test, and
starts and stops the simulation. Your script only reads the camera and drives
the joints. Calling `sim.startSimulation()`, `sim.stopSimulation()` or
`sim.setObjectPosition()` on the vehicle fights it for control of the run - the
evaluator reports these when it finds them.

Full instructions are in the theme book.
