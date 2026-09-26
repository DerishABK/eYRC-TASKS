'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 1B of Niti Vahan (NV) Theme.
*
*  Filename:         path_tracking.py
*  Created:          2026
*  Last Modified:
*  Author:           e-Yantra Team
*
*  You are ONLY allowed to write your code inside the block marked
*  "ADD YOUR IMPLEMENTATION HERE". Do not change anything outside it - the
*  evaluation script relies on the rest of this file staying as it is.
*
*****************************************************************************************
'''

# Team ID:          < 5761 >
# Author List:      < Derish Abraham, Joshua Varghese, Reibin Chacko Thomas, Delvin Varghese >
# Filename:         path_tracking.py
# Functions:        ackermann_wheel_angles, compute_steering
# Global variables: _integral_cte


####################### IMPORT MODULES #######################
import argparse
import csv
import math
import queue
import sys
import threading
import time
##############################################################


#################### VEHICLE CONSTANTS #######################
WHEELBASE = 0.120           # L: distance between front and rear axle centrelines
TRACK_WIDTH = 0.110         # W: distance between left and right wheel centre
WHEEL_OFFSET = 0.0275       # O: distance between kingpin axis and wheel centre.
##############################################################


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################


# --------------- Ackermann geometry constants (same as Task 1A) ---------------
# WHEELBASE, TRACK_WIDTH, WHEEL_OFFSET are already declared above.

# --------------- Stanley controller tuning constants --------------------------
# Vehicle speed is ~0.029 m/s; Stanley term = atan(K*cte / (K_SOFT+speed)).
# With K_STANLEY=1.5 and speed=0.03 the term saturates at MAX_STEER for any
# cte > 0.001 m -- causing constant large yaw and oscillation.
# Tuned for actual speed: K_SOFT ~ speed, K_STANLEY small.
K_STANLEY = 0.3     # cross-track gain (reduced for slow vehicle)
K_SOFT    = 0.05    # softening constant (~vehicle speed to keep term linear)
K_I       = 0.01    # integral gain (small to avoid wind-up across lane changes)
INTEGRAL_CLAMP = 0.10   # m·s  anti-windup: cap the accumulated integral

# --------------- Persistent controller state ----------------------------------
_integral_cte = 0.0  # accumulated cross-track error integral (NOT reset on lane change)


def ackermann_wheel_angles(delta):
    '''
    Purpose:
    ---
    Convert one desired steering angle into the two real front-wheel angles
    the vehicle's Ackermann geometry implies.

    Input Arguments:
    ---
    `delta` :          [ float ]
        Desired steering angle in radians, as if the vehicle had a single
        centred front wheel. Positive means turning left.

    Returns:
    ---
    `left_wheel_angle`  : [ float ]  angle for the left front wheel, radians
    `right_wheel_angle` : [ float ]  angle for the right front wheel, radians
    '''

    if delta == 0:
        return 0.0, 0.0

    # Effective half-track: kingpin axis is inset by WHEEL_OFFSET from wheel centre
    half_track = (TRACK_WIDTH / 2) - WHEEL_OFFSET
    tangent = math.tan(delta)

    # Ackermann geometry: each wheel steers around the same instantaneous centre.
    # Inner wheel (left when turning left) steers more sharply than the outer.
    left_tangent  = tangent / (1 - half_track * tangent / WHEELBASE)
    right_tangent = tangent / (1 + half_track * tangent / WHEELBASE)

    left_wheel_angle  = math.atan(left_tangent)
    right_wheel_angle = math.atan(right_tangent)

    return left_wheel_angle, right_wheel_angle


def compute_steering(target_y, current_values):
    '''
    Purpose:
    ---
    Compute the steering angle that drives the vehicle to `target_y` and keeps
    it there. Called once per simulation step.

    Input Arguments:
    ---
    `target_y` :        [ float ]
        Lateral position to converge to, in metres - see LANE_Y below.

    `current_values` :  [ dict ]
        {
            "y"     : float,  current lateral position of the vehicle, in metres
            "yaw"   : float,  current heading in radians, 0.0 when aligned with the road
            "speed" : float,  current forward speed, in m/s
            "dt"    : float,  seconds elapsed since the previous call
            "t"     : float,  seconds since this run started
        }

    Returns:
    ---
    `steering` : [ float ]
        Steering angle in radians. Positive turns the vehicle left, which on
        this road means toward SMALLER y.

    Controller:
    ---
    Stanley method:
        steering = heading_error + arctan(K_STANLEY * cte / (K_SOFT + speed))
    Plus a small integral term to kill steady-state offset:
        steering += K_I * integral_cte

    Sign convention:
        cte = current_y - target_y
        A positive cte means the vehicle is too far in the +y direction,
        so a positive (left-turn) correction is required. ✓
    '''

    global _integral_cte

    # --- Extract measurements ---
    current_y = current_values["y"]
    yaw       = current_values["yaw"]
    speed     = current_values["speed"]
    dt        = current_values["dt"]

    # --- Cross-track error ---
    # Positive cte → vehicle is right of target → need to steer left (positive steering). ✓
    cte = current_y - target_y

    # --- Integrate cross-track error for steady-state correction ---
    _integral_cte += cte * dt
    # Anti-windup: clamp integral so it cannot drive steering past the limit
    _integral_cte = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, _integral_cte))

    # --- Heading error ---
    # Road runs along world -x, so desired yaw = 0.  A positive yaw means the
    # nose is pointing toward +y (too far right), so we need a left correction.
    heading_error = -yaw

    # --- Stanley lateral correction term ---
    stanley_term = math.atan2(K_STANLEY * cte, K_SOFT + speed)

    # --- Integral term ---
    integral_term = K_I * _integral_cte

    # --- Combined steering command ---
    steering = heading_error + stanley_term + integral_term

    return float(steering)


##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

VEHICLE_PATH = "/Niti_Vahan"
STEER_LEFT_JOINT = "/Niti_Vahan/steeringLeft"
STEER_RIGHT_JOINT = "/Niti_Vahan/steeringRight"
DRIVE_LEFT_JOINT = "/Niti_Vahan/motorLeft"
DRIVE_RIGHT_JOINT = "/Niti_Vahan/motorRight"

WHEEL_RATE = 1.3                        # rad/s, constant drive speed
MAX_STEER = math.radians(30.0)          # steering limit, radians
RUN_TIME = 120.0                        # s of simulated time, every run

# The vehicle drives along world -x, so its left-hand side faces -y.
LANE_Y = {"L": 0.0, "R": 0.2}
DEFAULT_LANE = "L"
DEFAULT_CSV = "trajectory.csv"
DEFAULT_LOG_RATE = 10.0                 # Hz, rows written to the CSV

CSV_COLUMNS = ("t", "x", "y", "yaw", "speed", "target_y",
               "steering", "left_wheel", "right_wheel")


def _stdin_reader(commands):
    '''Read the terminal in a background thread so the control loop never blocks.'''
    for line in sys.stdin:
        commands.put(line.strip())
    commands.put("q")           # stdin closed - treat it as "stop"


def read_target(commands, lane):
    '''Apply every lane command typed since the last step. Returns (lane, keep_running).'''
    while True:
        try:
            command = commands.get_nowait().strip().upper()
        except queue.Empty:
            return lane, True

        if not command:
            continue
        if command in ("Q", "QUIT", "EXIT"):
            return lane, False
        if command not in LANE_Y:
            print("  ignored %r - type L, R or q" % command)
            continue
        if command != lane:
            lane = command
            print("  lane -> %s  (y = %.2f m)" % (lane, LANE_Y[lane]))


def parse_schedule(text):
    '''Turn "12:R,45:L" into [(12.0, "R"), (45.0, "L")], sorted by simulated time.'''
    schedule = []
    for entry in text.split(","):
        entry = entry.strip()
        if not entry:
            continue
        when, _, side = entry.partition(":")
        side = side.strip().upper()
        if side not in LANE_Y:
            raise ValueError("%r: lane must be L or R" % entry)
        schedule.append((float(when), side))
    return sorted(schedule)


def heading(matrix):
    '''Heading relative to the road, in radians, from the vehicle's pose matrix.

    The body's Euler angles sit at beta = -90 degrees, where the decomposition
    is degenerate and the reported angles jump around, so the heading comes
    from the rotation matrix instead. Column 3 is the local +z axis - the
    direction the vehicle faces - and the road runs along world -x.
    '''
    yaw = math.atan2(matrix[6], matrix[2]) - math.pi
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def run_session(sim, commands, csv_path, log_rate, schedule=None):
    '''Drive one RUN_TIME-second run, logging the trajectory.

    With a `schedule` the lane changes at the simulated times it lists;
    without one it comes from whatever is typed at the terminal.
    '''
    vehicle = sim.getObject(VEHICLE_PATH)
    steer_left = sim.getObject(STEER_LEFT_JOINT)
    steer_right = sim.getObject(STEER_RIGHT_JOINT)
    drive_left = sim.getObject(DRIVE_LEFT_JOINT)
    drive_right = sim.getObject(DRIVE_RIGHT_JOINT)

    log_interval = 1.0 / log_rate
    next_log = 0.0
    lane = DEFAULT_LANE
    pending = list(schedule or ())
    rows = 0

    handle = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    writer.writerow(CSV_COLUMNS)

    sim.startSimulation()
    try:
        t0 = sim.getSimulationTime()
        prev_t = t0

        while True:
            now = sim.getSimulationTime()
            elapsed = now - t0
            if elapsed >= RUN_TIME:
                print("run complete at %.1f s" % elapsed)
                break

            while pending and elapsed >= pending[0][0]:
                _, lane = pending.pop(0)
                print("  t=%6.2f s  lane -> %s  (y = %.2f m)"
                      % (elapsed, lane, LANE_Y[lane]))

            lane, keep_running = read_target(commands, lane)
            if not keep_running:
                print("stopping")
                break

            dt = now - prev_t
            if dt <= 0.0:
                sim.step()
                continue
            prev_t = now

            pos = sim.getObjectPosition(vehicle, -1)
            yaw = heading(sim.getObjectMatrix(vehicle, -1))
            linear, _ = sim.getObjectVelocity(vehicle)
            target_y = LANE_Y[lane]

            steering = compute_steering(target_y, {
                "y": pos[1],
                "yaw": yaw,
                "speed": math.hypot(linear[0], linear[1]),
                "dt": dt,
                "t": elapsed,
            })
            steering = max(-MAX_STEER, min(MAX_STEER, steering))
            left_angle, right_angle = ackermann_wheel_angles(steering)

            sim.setJointTargetPosition(steer_left, left_angle)
            sim.setJointTargetPosition(steer_right, right_angle)
            sim.setJointTargetVelocity(drive_left, WHEEL_RATE)
            sim.setJointTargetVelocity(drive_right, WHEEL_RATE)

            if elapsed >= next_log:
                writer.writerow([
                    "%.3f" % elapsed, "%.4f" % pos[0], "%.4f" % pos[1],
                    "%.4f" % yaw, "%.4f" % math.hypot(linear[0], linear[1]),
                    "%.2f" % target_y, "%.4f" % steering,
                    "%.4f" % left_angle, "%.4f" % right_angle,
                ])
                rows += 1
                next_log += log_interval

            sim.step()
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        handle.close()
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)

    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Task 1B - drive the Ackermann vehicle to the lane you ask for."
    )
    parser.add_argument(
        "--out", default=DEFAULT_CSV,
        help="Where to write the trajectory CSV (default: %s)." % DEFAULT_CSV,
    )
    parser.add_argument(
        "--log-rate", type=float, default=DEFAULT_LOG_RATE,
        help="Rows per second written to the CSV (default: %g Hz)." % DEFAULT_LOG_RATE,
    )
    parser.add_argument(
        "--schedule",
        help="Lane changes at fixed SIMULATED times, e.g. '15:R,60:L,95:R'. "
             "The evaluation supplies this so every team drives the same run; "
             "without it, the lane is whatever you type.",
    )
    args = parser.parse_args()
    schedule = parse_schedule(args.schedule) if args.schedule else None

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    sim = RemoteAPIClient().require("sim")
    sim.setStepping(True)

    commands = queue.Queue()
    if schedule:
        # No terminal reader: the schedule is the only thing that may change lane.
        print("lane %s (y = %.2f m) - fixed schedule: %s"
              % (DEFAULT_LANE, LANE_Y[DEFAULT_LANE],
                 ", ".join("%gs->%s" % entry for entry in schedule)))
    else:
        threading.Thread(target=_stdin_reader, args=(commands,), daemon=True).start()
        print("lane %s (y = %.2f m) - type L or R and press Enter to change lane, q to stop"
              % (DEFAULT_LANE, LANE_Y[DEFAULT_LANE]))

    rows = run_session(sim, commands, args.out, args.log_rate, schedule)
    print("wrote %d rows to %s" % (rows, args.out))


if __name__ == "__main__":
    main()
