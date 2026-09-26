ZMQ Remote API

Read this first

This page follows on from CoppeliaSim, which covers scenes, objects, joints and the simulation loop. This one is about the wire between your Python file and the simulator.

Every task from Task 1 onward uses this API. It is worth reading properly once.
Getting the Name Right

There are three similar-sounding names here, and mixing them up is the first source of confusion:
Name	What it actually is
ZeroMQ (also written ØMQ or ZMQ)	A general-purpose messaging library. Nothing to do with robotics - it is how the messages get from A to B
pyzmq	The Python bindings for ZeroMQ. A dependency, installed for you. You will never import zmq yourself
coppeliasim-zmqremoteapi-client	The package you actually use. Coppelia Robotics’ client, built on top of pyzmq

So the thing you install is the long one:
Terminal window

pip install coppeliasim-zmqremoteapi-client

…except you do not even need to run that, because it is already listed in the environment.yml you used in Task 0. If conda activate NV_<Team-ID> succeeded, it is there.

The import name uses underscores rather than hyphens, which trips up nearly everyone the first time:

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

How It Works

CoppeliaSim ships with a plugin that opens a TCP socket and listens on it - by default on port 23000 of your own machine. Your Python script opens a connection to that socket and sends requests down it.

┌──────────────────────┐        tcp://localhost:23000        ┌──────────────────────┐
│   your_script.py     │  ────── "sim.getObjectPosition" ──> │     CoppeliaSim      │
│   (the client)       │  <───── [1.2, 0.4, 0.15]  ───────── │     (the server)     │
└──────────────────────┘                                     └──────────────────────┘

Each call is a round trip: your script sends a request, then waits for the answer before doing anything else. That single fact explains most of the performance advice further down.

The important consequence of the design is that the sim object in your script is not a local library. It is a proxy. When you write sim.getObjectPosition(handle, -1), no maths happens on your side - the name and arguments are packed into a message, sent to CoppeliaSim, executed there, and the result is sent back. This is why the API mirrors CoppeliaSim’s own scripting functions exactly: it is those functions, called remotely.
Connecting

Four lines get you from nothing to a live connection:

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()          # defaults to localhost:23000
sim = client.require('sim')         # the sim API namespace

RemoteAPIClient() takes host and port if you need them - RemoteAPIClient(host='192.168.1.7', port=23000) connects to a simulator on another machine - but for this theme the defaults are correct.

require
or
getObject
?

You will see both client.require('sim') and client.getObject('sim') in examples online, including in Coppelia’s own manual. Both return the same sim namespace. require is the current, documented form and is the one to use - it also loads the named module server-side first, which matters for namespaces other than sim.

CoppeliaSim exposes several namespaces this way. sim is the general one and the only one you need for now; simIK (inverse kinematics), simVision and others are requested the same way when a task calls for them.
Stepping Mode

By default the simulator runs on its own clock and your script races alongside it. That is fine for watching, and useless for control - two runs of the same code would sample the world at different moments and produce different results.

Stepping mode puts your script in charge of the clock:

sim.setStepping(True)
sim.startSimulation()

while (t := sim.getSimulationTime()) < 20:
    # read the world
    position = sim.getObjectPosition(vehicle, -1)

    # decide something
    steering = compute_steering(position)

    # act on it
    sim.setJointTargetPosition(steer_left, steering)

    sim.step()          # <- advance the simulation by exactly one time step

sim.stopSimulation()

Nothing moves between your sim.step() calls. The loop above is therefore exactly one controller decision per physics step - which is what makes a run repeatable, and what lets the evaluation compare your result with anyone else’s.

The loop that never ends

Forget sim.step() inside a stepping-mode loop and the simulation time never advances. Your while condition stays true forever, the vehicle sits perfectly still, and nothing errors. If your script appears to hang with a frozen scene, this is the first thing to check.
The Functions You Will Use

Object handles first: every call needs a numeric handle, which you look up once by path before the loop starts.

vehicle = sim.getObject("/ackermann_vehicle")
steer_left = sim.getObject("/ackermann_vehicle/steer_left_joint")

Call	What it does
sim.getObject(path)	Path from the Scene Hierarchy → handle. Do this once, outside the loop
sim.getObjectPosition(h, -1)	[x, y, z] in world coordinates - -1 means “relative to the world”
sim.getObjectOrientation(h, -1)	Euler angles [α, β, γ] in radians; the yaw you want for a ground vehicle is γ
sim.getObjectVelocity(h)	Returns two lists: linear [vx, vy, vz] and angular velocity
sim.setJointTargetPosition(h, angle)	Command a steering joint to an angle, in radians
sim.setJointTargetVelocity(h, rate)	Command a drive joint to spin, in radians per second
sim.getSimulationTime()	Simulated seconds elapsed - use this for timing, never time.time()
sim.getSimulationTimeStep()	The dt between steps
sim.startSimulation() / sim.stopSimulation()	Start and stop the run from your script
sim.step()	Advance one step (stepping mode only)

The full list - several hundred functions - is in the regular API reference.

Simulated time, not wall-clock time

sim.getSimulationTime() returns time inside the simulation, which may run faster or slower than the clock on your wall. Every controller calculation that involves time - the dt in a PID integral or derivative, a settling-time measurement - must use simulated time. Using time.time() makes your controller’s behaviour depend on how busy your laptop is.
Stopping Cleanly

sim.stopSimulation() returns immediately, but the simulator takes a few moments to actually wind down. Start the next run before it has finished and you inherit a half-stopped state. Wait for it:

sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    pass

Note sim.simulation_stopped - the API’s constants come through the same proxy as its functions, so you refer to them as attributes of sim rather than importing them.
Performance

Every call is a network round trip, so the number of calls per step is what governs speed, far more than what any individual call does.

    Fetch handles once. sim.getObject() inside the loop turns one round trip into thousands. Look everything up before startSimulation().
    Do not read the same value twice in a step. Read the vehicle’s position once and pass it around.
    Do your maths locally. Your controller arithmetic costs nothing; the calls around it cost everything.

When It Does Not Connect
Symptom	Cause
The script hangs at RemoteAPIClient() with no error	CoppeliaSim is not running, or a firewall is blocking port 23000. The client waits rather than failing fast
ModuleNotFoundError: No module named 'coppeliasim_zmqremoteapi_client'	The NV_<Team-ID> environment is not active, or you ran the file through a different interpreter (a VS Code interpreter set to base is the usual culprit)
An exception naming the object path	The path does not match the Scene Hierarchy exactly, or the scene has not been opened
It connects, but nothing moves	The simulation was not started, or sim.step() is missing from the loop
Odd behaviour on a second run	The previous run was not stopped cleanly - see above