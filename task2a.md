## AIM
Task 2A has no new theory. You join two things you built separately: the detector from Task 1C and the controller from Task 1B.

## Important
Drive the vehicle one full lap of the arena using only its camera. Stay in the lane it was placed in, and stop whenever you are told to.

You have already written both halves of this task. What is new is making them work together.

| Task 1C | Task 1B | Task 2A |
|-----------|----------|---------|
| Where the input came from | a recorded video file | the simulator, as ground truth | the vehicle’s own camera, live
| What you produced | center_x and a lane name | a steering angle | a steering angle, from the camera
| Did your output affect the next input? | No | No | Yes

That last row is the whole task. In Task 1C, a bad frame gave you one bad number, and the next frame arrived anyway. Here, a bad number steers the vehicle. The vehicle’s new position then decides what the camera sees next. So mistakes build on each other instead of averaging out. That is why a detector that scored well on the Task 1C clips can still drive straight off the road.

# Stages

## Stage 1
Stage 1 - The frame (provided)

You do not write this part. task2a.py connects to the simulator and looks up every handle once, before the loop starts. It then hands detect_lane() a corrected BGR frame. You only need to know that the frame is ready to use.

Why is this part given to you? The camera read is the only call in this task that does not return something usable straight away. Its problems are also silent. Each one gives you a detector that seems to work, on a frame that is wrong:

What read_frame() does	| What would go wrong without it
sim.handleVisionSensor(...)	| This scene’s sensor is in explicit-handling mode, so it renders only when asked. Skip this call and every read returns the same frame for the whole lap. Your vehicle then steers on a picture of where it started
cv2.flip(frame, 0)	| The buffer’s first row is the bottom of the image. Without the flip, the road comes out upside down, and the lane appears to bend the wrong way
cv2.cvtColor(..., RGB2BGR)	| The buffer is RGB, but OpenCV assumes BGR. Without the swap, every HSV threshold you tuned in Task 1C picks the wrong colour. On this arena, an HSV yellow mask finds zero pixels instead of thousands

# Extra stuff
Read the function anyway. The ZMQ Remote API page explains what it does, and you will need the same ideas in Task 2B. If you want I will be providing it, just ask me if you are stuck and require this.

## The shape of the file

The boilerplate has two halves, split by a marker line. Above the marker are the two functions you write. Below it is everything else: the connection, the handle lookup, the camera read, the four joint writes and the loop itself. You are not marked on anything below the marker.

def detect_lane(frame):
    # Stage 2 - your Task 1C detector, extended
    return None                       # anything compute_control() can use

def compute_control(detection, stop_requested):
    # Stage 3 - your Task 1B controller, your Task 1A split, plus the stop rule
    return 0.0, 0.0, 0.0              # (left angle, right angle, speed)

#################### DO NOT EDIT BELOW THIS LINE ####################

The loop that calls them is already written:

while sim.getSimulationState() == sim.simulation_advancing_running:
    frame = read_frame(sim, handles)
    if frame is None:
        continue
    stop_requested = bool(sim.getInt32Signal('NV_stop'))
    detection = detect_lane(frame)                                   # yours
    left, right, speed = compute_control(detection, stop_requested)  # yours
    drive(sim, handles, left, right, speed)

You can add helper functions. Put them above the marker, next to the two functions, and call them from there.

## Warnings
Two things the harness owns, and you must not

The harness owns starting and stopping the simulation, and where the vehicle is. Your script only reads the camera and drives the joints.

Do not call sim.startSimulation(), and do not call sim.setObjectPosition() on the vehicle. Either one fights the harness for control of the run. Placing the vehicle yourself is also not solving the task. The evaluator reports these calls when it finds them in your file.

## Stage 2 - Detection, extended

This is detect_lane(), the first function you write. It receives the frame and returns whatever compute_control() needs. That can be a tuple, a dictionary or your own class. The two functions only have to agree with each other.

Your Task 1C detector answered two questions: where is the lane centre, and which lane am I in. A controller cannot use either answer directly. It needs to know how far off the vehicle is right now. That takes two numbers:
Signal	| What it means	| Where it comes from
Cross-track error e	| how far the vehicle is from the lane centre, sideways	| the lane centre you already compute, compared with the vehicle’s own axis in the image
Heading error ψ| 	how far the vehicle is pointing away from the direction the lane runs	| the slope of the curve you fitted, not its position

The heading error is the part Task 1C never asked for. It is also why the Lane Detection page tells you to fit a curve instead of finding only a centre point. A fitted curve gives you a slope at any row you choose. On the bird’s-eye view, its second-order term also tells you how sharply the road is about to bend.

## Keep in mind
Pixels are not metres

Your Task 1B controllers were tuned against an error in metres. The camera gives you an error in pixels. The two are not interchangeable. A gain that was stable in Task 1B will be far too large or far too small here.

You have two options. Re-tune the gains against pixel error, or convert the error to metres first. Converting is the better habit. In a bird’s-eye view the scale is constant, so a single number converts one into the other. Whichever you pick, know which units your controller is working in.

## Stage 3 - Control

This is compute_control(), the second function you write. It returns exactly three numbers, which go straight to the joints:
Return value	| Units	| Convention
left_angle, right_angle	| radians	| the two front-wheel angles. The scene’s steering joints are positive to the left. Nothing is clamped for you, and the vehicle’s limit is 45°
speed	| rad/s, both drive motors	| 0.0 holds the vehicle still

Two pieces of your earlier work meet here:

    compute_steering() from Task 1B turns an error into one steering angle.
    ackermann_wheel_angles() from Task 1A splits that angle into the two angles the joints take.

Both carry over almost unchanged. Only two things are different: where the error comes from, and what units it is in.

Stanley is worth a second look here. It is built from exactly the two signals a camera gives you: cross-track error and heading error. That is why it is a common choice for camera-based lane keeping. PID on cross-track error alone will also drive the vehicle. But PID has no term that knows which way the road is turning.

## Staying in Your Lane

The vehicle must hold the middle of the lane it was placed in, for the whole lap. Most of the marks are here.

Staying on the correct side of the dashed line is not enough. The evaluation measures your distance from the centre of your lane. That centre is surveyed from the painted road edges and the divider. It is not taken from any reference lap. You get credit only for the part of the lap you drive close to it. Weaving from line to line inside your lane loses marks. So does riding the boundary.

You do not choose the lane, and you do not place the vehicle. The evaluation puts the vehicle in one lane and runs your script for a lap. It then puts the vehicle in the other lane and runs your script again. Both laps have to work. A pipeline that quietly relies on the boundary being on one particular side loses most of its score.

This affects how you write the loop in two ways:

    Do not hard-code a lane. Work out which lane you are in from what the camera sees, as in Task 1C. Seen from the other lane, the arena looks mirrored.
    Do not steer differently depending on which lane you think you are in. Use one controller that follows whichever lane the detector reports. It is simpler, and it does not fail when the detector is briefly wrong.

## Stopping When You Are Told

At two places on each lap, the vehicle must come to a complete stop. It waits about five seconds, then carries on in the same lane.

You do not need to work out where those places are, and you must not try. The evaluation watches the vehicle’s position. When the vehicle reaches a stop point, the evaluation raises a flag. Your loop only has to notice the flag and obey it.

How the flag reaches your code

The flag is an integer signal in the simulator, named NV_stop. The boilerplate reads it once per iteration. It passes the value to you as the second argument of compute_control():

def compute_control(detection, stop_requested):
    left, right = ackermann_wheel_angles(steering_angle)  # your Task 1A split
    if stop_requested:
        return left, right, 0.0       # hold still - steering can stay as it is
    return left, right, CRUISE        # drive normally

You do not need to read or touch the signal yourself. In case you are curious: sim.getInt32Signal('NV_stop') returns None until the signal is first set. The boilerplate treats “not set” as “not stopping”, so your first few frames behave correctly.

The flag is a level, not a pulse

You do not time the stop. The evaluation raises the flag and keeps it raised for the whole stop. It lowers the flag when the wait is over. So your rule is simple:
NV_stop | What your vehicle must be doing
1 | stationary
0 or None | driving, in its lane

Four things follow from this. Each one costs marks if you miss it:

    Stop promptly. You have a moment to slow down, not a hundred metres. The flag is read once per iteration. If detect_lane() is slow, you see the flag late, and you stop late.
    Keep remote calls to a minimum. While the simulation is running, each remote call waits for the next simulation step. On this scene that takes 6-10 ms, and every call pays it. To keep the loop quick, make fewer calls. This is also why the boilerplate looks up its handles once, not every frame. See Performance.
    Keep returning values while stopped. compute_control() must still return all three numbers, with a speed of 0.0. Do not raise an error, and do not block inside it with a sleep. Either one stops the loop from seeing the flag drop.
    Resume in the same lane. The flag dropping is not a fresh start. You are mid-lap and in a lane, and the detector should still know which one.


## Keep in mind

Do not tie stops to time or distance

The flag is raised based on where the vehicle is, not on how long it has been driving. Say you write “stop after 12 seconds, then again after 30”. That works at the speed you tested at, and fails at any other speed. Guessing the locations does not help either. Read the signal.

## Running the thing

conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task2a
python task2a.py

## keep in minf
Do not move objects, rename them, or change camera or joint settings. Any of these will invalidate your run. If you think you have changed something, re-open Task_2A.ttt from the repository.

What You Write

You submit one file, task2a.py, built from the boilerplate in task2a/. In it you write two functions, and nothing else:

You write | It receives | It returns
detect_lane(frame) | one corrected BGR frame | anything compute_control() can use
compute_control(detection, stop_requested) | that, and the stop flag | (left_angle, right_angle, speed)

## Before You Submit

    Run it from both lanes, a full lap each, more than once.
    Watch the whole route, not only the first straight. Most failures happen on the bends.
    Check what your program does when detection fails. Force a failure by starting the vehicle from a bad position. Then see whether it recovers or drives away.
    Confirm it completes a full lap and is still in its lane at the end.
    Test the stop yourself. From a second terminal, connect to the simulator and call sim.setInt32Signal('NV_stop', 1). Check that the vehicle halts. Then set the signal back to 0, and check that the vehicle drives on in the same lane.
gi