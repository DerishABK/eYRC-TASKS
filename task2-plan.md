Aim

Fill in the two functions of the Task 1B boilerplate so that the Ackermann vehicle in CoppeliaSim moves into whichever lane it is asked for while the simulation is running, and holds it until asked for the other one.

The Problem

The vehicle drives along a straight two-lane road at a constant speed the harness sets. The lane it should be in is asked for during the run, and your controller has one job: steer to whichever lateral position it is currently being asked for, and stay there.

The Functions You Implement
1. ackermann_wheel_angles() - copied from Task 1A

Paste your Task 1A implementation in unchanged. It is not marked again; it is here because the control loop needs it to turn one steering angle into the two front-wheel angles the joints take. The WHEELBASE and TRACK_WIDTH constants it reads are declared near the top of the file, as before.

compute_steering() - the controller

Called once per simulation step. It receives the lane to head for and the vehicle’s current measurements, and returns one number:
target_y is a float in metres and can change between calls. current_values is a dict:

y	current lateral position, in metres
yaw	current heading, in radians, 0.0 when aligned with the road
speed	current forward speed, in m/s
dt	seconds elapsed since the previous call
t	seconds since the run started

Return a steering angle in radians, where positive turns the vehicle left - and therefore toward smaller yy. Get this the wrong way round and your controller drives away from the lane, smoothly and confidently.
You need	Compute it from
Cross-track error	current_values["y"] - target_y, written this way round so a positive error asks for a left turn
Heading error	-current_values["yaw"], since the road runs straight
Speed, for the Stanley denominator	current_values["speed"]
Timestep, for the I and D terms	current_values["dt"]

The harness clamps what you return to the steering limit before passing it to ackermann_wheel_angles(), so a large value is safe - but a controller that relies on being clamped is not a controller.

Keeping State Between Calls

compute_steering() is called fresh each step, so anything in a local variable is gone by the next call. A PID controller has to remember its accumulated error and previous error: keep those in globals inside the implementation block and reach them with a global statement. Stanley needs none of this - it works from the current measurements alone.

Add as many helpers and globals as you like, above the END OF YOUR IMPLEMENTATION line, and list them in the file header.

Rules

Read this before you start writing code

    Write code only inside the ADD YOUR IMPLEMENTATION HERE block. Everything outside it must stay exactly as shipped.
    No print(), plotting or input() inside either function when you submit. They are called hundreds of times per run, and the terminal belongs to the lane input.
    No sim.* calls. Your function reads its arguments and returns a number; the harness owns the simulator. Placing the vehicle where you want it is not solving the task.
    Do not rename the file or either function, or change what they return.
    Return a plain float from compute_steering() and a pair of floats from ackermann_wheel_angles(). None or a string crashes the run; NaN does not crash it but makes every later row worthless.


Running It
Terminal window

python path_tracking.py                     # start in the left lane
python path_tracking.py --out run1.csv      # write the CSV elsewhere
python path_tracking.py --log-rate 20       # rows per second (default 10)

While it runs, type a lane and press Enter:

lane L (y = 0.00 m) - type L or R and press Enter to change lane, q to stop
R
  lane -> R  (y = 0.20 m)
L
  lane -> L  (y = 0.00 m)
q
stopping
wrote 412 rows to trajectory.csv

Every run lasts 120 simulated seconds, or until you type q.
The evaluation drives it instead

When you submit, the lane changes come from a fixed schedule passed to this same script, not from anything you type:
Terminal window

python path_tracking.py --schedule "15:R,60:L,95:R"

Times are in simulated seconds, which is what makes a run reproducible: the simulator only advances when the script steps it, so the same schedule gives the same changes at the same points on every machine. Every team gets the same schedule, and it is not published. With a schedule the terminal is not read at all. See Submission.