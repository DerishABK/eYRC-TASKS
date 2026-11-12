ask 3 - Instructions
Aim

    The vehicle should start from its spawn location, drive to the designated pickup location, wait for a few seconds there, then drive towards the parking and park itself, all while maintaing the speed limits of the specific area.

No new theory here

Task 3 has no new learning resources. Every piece of the run comes from earlier tasks. Keep these pages open, because you will go back to parts of them : Task 2A · Task 2B · Task 2C · Sign Detection · Obstacle Detection · ZMQ Remote API
What is actually new
	Task 2	Task 3
When to stop	The evaluator raised NV_stop for you	Your code decides, from the signs it sees
Signs and obstacles	Detected in still images, and no action was taken based on them	Detected live, and they change how the vehicle drives
Lanes	Stay in the lane you start in	Change lane when an obstacle blocks the lane your vehicle is driving in
Parking	Spawned just before the bays	Reached after completing the pickup task
What the run asks for	One job	Several jobs in one lap: halt, slow down, change lane and park

In Task 2 each piece did one job and something else told it when to do it. In Task 3 your algorithm has to keep track of where it is and what it has already done.
The Arenas

Task 3 comes with three arenas. They share the same road, signs, pickup points and parking. The only difference is the obstacles:
Arena	Scene	Obstacles
Easy	Task_3_Easy.ttt	None
Medium	Task_3_Medium.ttt	2
Hard	Task_3_Hard.ttt	3

Only the Hard arena is graded. Easy and Medium are there to practise on.

Work through them in order. The Easy arena has no obstacles, so get the halts, the speed limit and parking working there first. Then add lane changes to avoid obstacles in Medium, and make them reliable in Hard.
What is in the arena
What	What it means for you
Traffic signs: Hospital, School, Speed 40, Parking	The same four signs your detect_signs() learned in 2B.
Pickup points	Where the vehicle halts: one beside the Hospital, one beside the School
Obstacles	The same red obstacle as in 2B. Each one blocks one lane
Parking bay with an ArUco marker	As in 2C, a bay with an AruCo marker at its base. The vehicle enters and leaves it from the right-hand straight

Figure 4 shows where each of these is:

Do not hard-code the layout

The three arenas share the road and the signs but not the obstacles, and only the Hard arena is graded. A run built on “turn after 20 seconds” or “change lane at the second bend” works in one arena at one speed and fails everywhere else. Act on what the camera shows.
The Graded Run

Your run is graded in the Hard arena, Task_3_Hard.ttt, with its three obstacles. In one lap the vehicle must:

    Halt at the Hospital pickup point for 5 seconds.
    Slow down in the Speed 40 zone.
    Change lane round every obstacle in its lane.
    Park in the Parking Bay.

The School is not a stop in the graded run: drive past its sign. Practise the same lap in the Easy and Medium arenas first.

No Boilerplate for Task 3

Unlike in previous tasks, no boilerplate will be provided for Task 3. You have to stitch your implementations from previous task and additional features on your own.
Halting at a sign

    Halt at the pickup point, in its lane. The sign tells you the pickup point is near. Figure 4 shows where each pickup point is relative to its sign.
    Halt at the pickup point for 5 seconds. The vehicle has to stand still for 5 seconds of simulation time:
        Start counting when the vehicle has actually come to rest, not when your code first sends the stop command. The vehicle keeps rolling for a moment after the speed is set to 0.
        Count with sim.getSimulationTime(), not with time.sleep() or your computer’s clock. Simulation time does not run at the same speed as real time, and the halt is measured in simulation time.
    Halt only at the Hospital. Drive past the School sign and any other sign.
    Stop at the right distance. Stop too early and the vehicle halts far from the sign. Leave it too late and the sign leaves the camera’s view before you stop.
    Do not halt at the same sign twice. When the wait is over, the sign is still in view. Mark the halt done before you drive on.

Obeying the speed sign

This applies in every arena. Speed here means the speed your code sends to the drive motors, in rad/s, as in 2A and 2C.
Where	Speed limit
Everywhere on the arena	3.0 rad/s
In the Speed 40 zone, between the two Speed 40 signs	1.2 rad/s

    These are limits, not targets : Drive at or below them. Slowing down more for a bend, a lane change or parking is fine.
    There are two Speed 40 signs on the road (Figure 4). The first one starts the Speed 40 zone and the second one ends it.
    First Speed 40 sign: slow down. As soon as your code sees the first Speed 40 sign, slow down to 1.2 rad/s or below, and stay at that speed. Keep the reduced speed even after Speed 40 sign is out of sight.
    Second Speed 40 sign: speed up again. When your code sees a Speed 40 sign for the second time, you can go back up to 3.0 rad/s.

Changing lane

    Only an obstacle in your own lane counts. detect_obstacle() should tell you whether the obstacle is present, how large it looks and whether it is in your current lane.
    Start the change early enough to get round it. Please make sure your vehicle changes lane only when a obstacle is detected, it should not go back to its original lane after the obstacle is passed.

Parking

    The P sign may come into view before the lap is done. Ignore it until every earlier instruction is complete.
    Once you commit to parking, hand over to your 2C manoeuvre. That needs the ArUco marker in view, so begin from a position where the camera can see it.

The Rules

    Use only the camera. Do not read the positions of the vehicle, signs or obstacles out of the simulator. Everything you act on must come from the image.
    Keep to the speed limits. At most 3.0 rad/s anywhere on the arena, and 1.2 rad/s between the two Speed 40 signs.
    Do not move the vehicle yourself. No sim.setObjectPosition() and no sim.setObjectOrientation() on the vehicle. Your program only sets steering angles and speeds.
    Do not start or stop the simulation. In a graded run the evaluation tool does that. Your script waits for the simulation to run and exits when it stops, as in 2A.
    Do not edit the scene. Moving, renaming or deleting objects, or changing the camera or joint settings, invalidates the run.
    Drive into nothing. That means obstacles, signs and ArUco boards.

Setting Up

Pull the repository first. The task3/ folder holds the three arena scenes and the evaluation tool, one build for each operating system:

task3/
├── Task_3_Easy.ttt
├── Task_3_Medium.ttt
├── Task_3_Hard.ttt
├── Ubuntu_eval
└── Windows_eval.exe

You only need the evaluation tool for the graded run (see the submission page).
Terminal window

cd ~/eYRC_26-27_Niti-Vahan
git pull

1. Launch CoppeliaSim.

    Ubuntu: from the folder you extracted it into, run ./coppeliaSim.sh
    Windows: use the desktop shortcut, or the Start menu entry

Only one CoppeliaSim at a time

If a second CoppeliaSim is already running, the new one quietly takes a different remote-API port. Your script then connects to the wrong simulator. Close any other instance before you begin.

2. Open the scene. Go to File → Open scene… and pick the arena you are working on (Task_3_Easy.ttt, Task_3_Medium.ttt or Task_3_Hard.ttt) from:

    Ubuntu: ~/eYRC_26-27_Niti-Vahan/task3/
    Windows: %USERPROFILE%\eYRC_26-27_Niti-Vahan\task3\

3. Run your script from a terminal (Ubuntu: Terminal; Windows: Anaconda Prompt):
Terminal window

conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task3
python task3.py

On Windows, replace the cd with cd %USERPROFILE%\eYRC_26-27_Niti-Vahan\task3.

4. Press ▶ Play in CoppeliaSim. Your script waits for the simulation to start, as in 2A and 2C. To start again from the beginning, press ⏹ Stop, then ▶ Play.
What You Write

You write the code yourself and keep it in task3/:

    task3.py is your solution and the only file you submit. The evaluation tool runs this file, so it must have exactly this name.
    Put all of your code in this one file. Do not split it into other .py files.
    Your sign detector from 2B may need its reference_images to run. Keep them in the task3/ folder while you practise and run the evaluation tool, but DO NOT add them to your zip.

Before You Submit

    Get each piece working on its own before you join them: lane following, one halt and parking in the Easy arena, then lane changes in Medium and Hard.
    Run the full lap in the Hard arena, the graded one, more than once. One good run is not proof that it is reliable.
    Run the lap in each arena (see The Arenas), and check the vehicle slows down in the Speed 40 zone in all three.
    Check that the vehicle drives past signs it was not asked to halt at.
    Check that a halt lasts the right time in simulation time, by printing sim.getSimulationTime() when the vehicle stops and when it moves off.
    Check that the script exits when the simulation stops.
    Remove cv2.imshow() windows, plots and input() calls.
