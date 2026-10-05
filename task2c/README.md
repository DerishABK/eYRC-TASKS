# Task 2C - Parking with ArUco

Park the vehicle in one of two bays, using the ArUco marker on that bay.

## What is in here

| File | What it is |
|---|---|
| `Task_2C.ttt` | The CoppeliaSim scene: a road, two parking bays to its right with an ArUco marker on each, and the vehicle spawned on the road |
| `task2c.py` | The boilerplate. You fill in **one function**, `parking_manoeuvre()` |

## What is done for you

Everything to do with the camera: reading it, finding the marker, and working
out from it where the vehicle is relative to the bay. You also get the
steering split across the two front wheels (your Task 1A geometry).

## What you write

The parking manoeuvre. Every frame, `parking_manoeuvre(pose)` is handed where
the vehicle is, relative to the target bay:

| Key | What it is |
|---|---|
| `lateral` | metres off the bay's centre line - positive = to the left of it |
| `heading` | radians, relative to the bay's long axis - positive = turned left |
| `forward` | metres from the vehicle's centre to the marker line |
| `bearing` | radians, where the marker is in the camera view - positive = left |

`pose` is `None` on a frame where the marker was not seen. You return a
steering angle (radians, positive = left), a speed (rad/s), and whether you
are parked.

The bays are 0.22 m wide and the vehicle is 0.18 m - about 20 mm to spare
each side. Steering straight at the marker brings you in on a diagonal and
does not fit: `lateral` and `heading` both have to reach zero.

## Running it

Open `Task_2C.ttt` in CoppeliaSim, leave the simulation **stopped**, then:

```sh
conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task2c
python task2c.py --bay 0
```

The script prints `Waiting for the simulation to start - press Play in CoppeliaSim.` - press Play then. Choose the
bay on the command line, and make sure both work:

```sh
python task2c.py --bay 0
python task2c.py --bay 1
```

Without `--bay`, the script uses `TARGET_ID` at the top of the file.

## Rules

- Write code **only** inside the `ADD YOUR IMPLEMENTATION HERE` block.
- Do **not** move the vehicle yourself with `sim.setObjectPosition()`.
- Do **not** read positions out of the simulator - everything you need is in `pose`.
- No `print()`, plots or `input()` inside `parking_manoeuvre()` when you submit.
