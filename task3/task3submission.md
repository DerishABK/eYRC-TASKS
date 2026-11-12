## The graded run
The graded run will be conducted on Task_3_Hard.ttt arena. Do not try to hard-code anything, it will be caught by the evaluation tools.
Halt at the Hospital pickup point, then park. Keep to 3.0 rad/s everywhere, and 1.2 rad/s in the Speed 40 zone.

## Marking scheme
Task 3 is marked out of 80. Your score is calculated out of 100 first, and then scaled down by a ratio of 0.8. You earn marks for what the run achieves and lose marks for mistakes along the way. Your score cannot go below 0.

Marks you earn
For	Marks
Halt for 5 seconds at the designated stop position (the Hospital pickup point)	| +50
Speed maintained: keeping to the speed limits, including slowing down in the Speed 40 zone	| +20
No collision with an obstacle during the run| 	+20
Parking	| +10

A run that does everything right earns 100.

Marks you lose
For	Marks
Going outside the road, or colliding with an obstacle: every time it happens	−30 each time
Taking too long: every second over 3 minutes, up to 5 minutes	−0.5 per second
Exceeding the speed limit: every second the speed limit is exceeded	−1 per second

## How each mark is measured

    Halt. Both drive wheels must be stopped for 5 seconds of simulation time, with the vehicle at rest inside the Hospital pickup area. That is a patch of road about 10 cm across and 20 cm along, around the Hospital pickup point (see Figure 4 on the instruction page). The vehicle has to stay put while it waits.
    Speed. Your speed is the wheel speed your code commands, in rad/s. The limit is 3.0 rad/s, and 1.2 rad/s in the Speed 40 zone. The zone starts just before the first Speed 40 sign and ends at the second one. You earn the +20 if you stay at or below 1.2 rad/s all the way through the zone. Every second over a limit, inside the zone or outside it, costs 1 mark.
    Collisions. Your vehicle counts as a 9 cm square. If it overlaps an obstacle, that is a collision, and each separate contact is one collision. The +20 for no collision is shared over the three obstacles: you earn it for each obstacle you drive past, and you lose all of it if you touch any.
    Going off the road. A wheel that goes more than 1 cm outside the road is one event. The yellow edge lines count as road. Coming back and leaving again later is another event.
    Parking. At the end of the run the vehicle is at rest inside the parking spot, a patch of about 20 cm by 10 cm at the parking bay, and you have been round the whole lap, past every obstacle.

Time is measured in simulation time, not by your machine’s clock, so a slow computer does not cost you marks. It is counted from the first step of the run. The tool stops the run after 5 minutes of simulation time, so being slow can cost you at most 60 marks. As an example, a run that halts, keeps to the speed limits, avoids every obstacle and parks earns 50 + 20 + 20 + 10 = 100. If it finishes after 3 minutes 20 seconds, it loses 20 × 0.5 = 10 marks and scores 90.

What You Submit
File	What it is	Where it comes from
task3.py	Your solution: all of your code, in this one file, the one the tool runs	You
NV_Task3_run.csv	The encrypted recording of the run	Written by the evaluation tool

## Submission Procedure
get the latest code(which I already did)


## Open the scene

The graded run is on the Hard arena. CoppeliaSim must be open with Task_3_Hard.ttt loaded. Leave the simulation stopped. The tool starts and stops it. Close any other CoppeliaSim window.

The evaluator tool checks that the open scene is Task_3_Hard.ttt and that its obstacles and signs are where they should be. If any attempts are made to alter the grading arena, the evaluation tool will fail to grade your run.

## Start the screen recording(you can skip this for now)

Start recording before you start the tool, and stop only after the tool has finished. Any screen recorder software works. The video must be:

    One continuous recording, not cut, edited, sped up or joined from parts.
    The whole CoppeliaSim window, with the scene and the simulation time visible, because reviewers will compare what the video shows with the recording.
    The terminal running the tool, visible at the start and the end. It shows the tool starting and its final message.

The simulation time is the line at the top-left of the CoppeliaSim window, as in Figure 1. Keep it in the frame for the whole recording, so record the whole window and do not crop or cover it.

## Run the tool

Open a terminal, activate your environment and run the tool from task3/. The tool runs your task3.py with the Python of the active environment, so the environment has to be active in this terminal.

Ubuntu, in a Terminal:
Terminal window

conda activate NV_<Team-ID>
cd ~/eYRC_26-27_Niti-Vahan/task3
./Ubuntu_eval
If the shell answers Permission denied, the file has lost its execute permission. Run chmod +x Ubuntu_eval once, then ./Ubuntu_eval again.

Do not press Play. The tool starts the simulation itself.

The tool then takes over:

    It checks the open scene,
    It starts the simulation and moves the whole arena, the vehicle and the CoppeliaSim view by a random offset (you will see the arena jump to its new place),
    It runs your task3.py as a separate program, the way you run it yourself, and shows its output in the same terminal,
    It records the vehicle until your script finishes, or until 5 minutes of simulation time have passed and the tool stops the run,
    It stops the simulation, which puts the arena back where it was,
    It finally writes NV_Task3_run.csv into the same folder as your task3.py.


Reading Your Result

After you upload, the portal shows your marks out of 80 and a few lines of feedback. They go through each mark and each penalty, say what happened in your run and what to change. For example:

SCORE 72.67/100 = 58.14/80 on the portal.
HALT 50/50: stopped for 5.2 s inside the Hospital pickup area.
SPEED 20/20: at most 1.2 rad/s all through the Speed 40 zone.
NO COLLISION 20.00/20: no obstacle was touched.
PARKING 10/10: at rest inside the parking spot.
TIME -27.33: the run took 3 min 55 s, 55 s over 3 minutes (0.5 per second).

The portal may show these lines joined by | on one line. The last line gives a run id. Quote it if you ask us about a result.

When a mark is lost, its line says why. These are the usual reasons:
Line	What it says, and what to change
HALT	You stopped a few cm short of or past the pickup area: adjust the distance at which you stop. Or the wait was shorter than 5 s, or the vehicle moved while it waited, or it never stopped near the Hospital pickup point.
SPEED	You went over 1.2 rad/s inside the Speed 40 zone: slow down as soon as you see the first Speed 40 sign. Or the vehicle never went through the zone.
SPEED LIMITS	The number of seconds you were over a limit, at 1 mark per second.
NO COLLISION	The vehicle hit an obstacle (and which one, and when), or it never got past an obstacle, so only some of the 20 marks count.
OFF THE ROAD	A wheel left the road at the time shown: −30 each time.
PARKING	Parking only counts after you have driven past every obstacle. Or the vehicle was still moving when the run ended, or it stopped a few cm outside the parking spot.
TIME	How long the run took, and how much of it was over 3 minutes.
If your submission could not be graded

The portal then shows 0 marks and a line starting with Not graded:, which says what is wrong:
It says	What to do
task3.py is not in the zip	Name your script exactly task3.py.
NV_Task3_run.csv is not in the zip	Put the recording in the zip, exactly as the tool wrote it.
… is not a Task 3 recording, or was written by another version of the tool	Run git pull and record again with the tool in your task3/ folder.
… was sealed for another key	Your tool is out of date: git pull, then record again.
… does not open - it has been edited or damaged	Do not open or save the recording. Upload the file the tool wrote.
… has no signature, or the signature … does not match	The recording was not written by the evaluation tool. Make it with the tool.
the run was interrupted	Record again and let the tool finish without pressing Ctrl+C.
the simulation was stopped during the run	Record again, and leave CoppeliaSim alone until the tool finishes.
task3.py is not the file that made the recording	You changed task3.py after the run. Record again with the code you submit.
the floor, or … is not where Task_3_Hard.ttt puts it	The scene was edited. Reopen Task_3_Hard.ttt from the repository and record again.
not a real run, or time runs backwards	The recording is too short or damaged. Record the whole run again.
no .zip, or zip files in submission/, expected one	Upload exactly one file, NV_Task3.zip.

A message that starts with Grader error is not your fault. Tell us the run id, if you have one.
