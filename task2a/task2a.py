'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 2A of Niti Vahan (NV) Theme.
*
*  Filename:         task2a.py
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

# Team ID:          < Team-ID >
# Author List:      < Names of the team members who worked on this file, comma separated >
# Filename:         task2a.py
# Functions:        detect_lane, compute_control
# Global variables: < List any global variables you add, "None" if you add none >


####################### IMPORT MODULES #######################
import sys
import time

import cv2
import numpy as np

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
##############################################################


##################### SCENE CONSTANTS ########################
# Paths into the CoppeliaSim scene hierarchy, read off Task_2A.ttt.
VISION_SENSOR = '/Niti_Vahan/Camera_joint/Cuboid/visionSensor'
STEER_LEFT    = '/steeringLeft'
STEER_RIGHT   = '/steeringRight'
MOTOR_LEFT    = '/motorLeft'
MOTOR_RIGHT   = '/motorRight'

# The integer signal the evaluation raises when the vehicle must stop.
STOP_SIGNAL = 'NV_stop'
##############################################################


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

# Task 1A geometry and Task 1B controller parameters.
_WHEELBASE = 0.120
_TRACK_WIDTH = 0.110
_WHEEL_OFFSET = 0.0275
_CRUISE_SPEED = 1.3
_RECOVERY_SPEED = 0.12
_TASK1B_SPEED_MPS = 0.029
_TASK1B_LANE_CENTER_SPACING_M = 0.20
_LANE_WIDTH_M = _TASK1B_LANE_CENTER_SPACING_M
_STANLEY_K = 0.3
_STANLEY_SOFT = 0.05
_INTEGRAL_K = 0.01
_INTEGRAL_CLAMP = 0.10
# The image tangent is only a proxy for the vehicle's world yaw. Dampen that
# term so a curved preview cannot pull the car across the detected divider.
_HEADING_WEIGHT = 0.50

# Camera segmentation/search settings. Lane position itself is always detected
# from the two painted markings in the current camera image.
_YELLOW_LOWER = np.array([15, 100, 100], dtype=np.uint8)
_YELLOW_UPPER = np.array([40, 255, 255], dtype=np.uint8)
_WHITE_SATURATION_MAX = 80
_WHITE_VALUE_MIN = 180
_ROI_TOP = 0.30
_ROI_BOTTOM = 0.90
_LOOKAHEAD_FAR = 0.40
# Task 1C scores its lane-center estimate at 0.62 of image height; use the
# same reference row for live cross-track error. The far point supplies yaw.
_LOOKAHEAD_NEAR = 0.62

# The 1B controller works in metres and m/s. Camera cross-track is normalized
# by the observed lane width, so convert it using the Task 1B lane spacing.
# Task 2A's joint limit is 45 degrees; 39 degrees virtual keeps the inner
# Ackermann wheel below that limit with the supplied Task 1A geometry.
_MAX_STEER = np.deg2rad(39.0)
_FILTER_ALPHA = 0.50
_MAX_STEER_STEP = 0.08

# Per-run state: the previous boundary fit keeps dashed-marking search on the
# same lane, and the controller filters frame-to-frame image noise.
_tracked_markings = None
_previous_lane_center = None
_locked_lane_sides = None
_detection_misses = 0
_filtered_cte = None
_filtered_heading = None
_last_steering = 0.0
_last_speed = _CRUISE_SPEED
_integral_cte = 0.0
_last_control_time = None


def _fit_marking(mask, frame_center, y_start, y_end, y_far, y_near,
                 side_hint, previous_fit=None):
    """Track a paint line near-to-far, bridging gaps in the dashed divider."""
    observations = []
    span = max(1.0, float(y_near - y_far))
    recent = []
    start_y = min(y_near, y_end - 1)
    for y in range(start_y, y_start - 1, -2):
        xs = np.flatnonzero(mask[y])
        if previous_fit is not None:
            fraction = (y - y_far) / span
            expected_x = previous_fit[0] + fraction * (previous_fit[1] - previous_fit[0])
        elif len(recent) >= 2:
            y0, x0 = recent[-2]
            y1, x1 = recent[-1]
            expected_x = x1 + (x1 - x0) / max(1.0, y1 - y0) * (y - y1)
        elif recent:
            expected_x = recent[-1][1]
        else:
            expected_x = frame_center

        if not xs.size:
            continue
        runs = np.split(xs, np.flatnonzero(np.diff(xs) > 2) + 1)
        runs = [run for run in runs
                if 2 <= run.size <= max(8, int(0.10 * mask.shape[1]))]
        if not runs:
            continue

        def run_score(run):
            x = float(np.mean(run))
            # The side hint seeds the first observed paint segment. Once
            # tracking starts, continuity takes over so bends may cross x=mid.
            wrong_side = (not recent and previous_fit is None and side_hint and
                          (x - frame_center) * side_hint < 0)
            return abs(x - expected_x) + (0.20 * mask.shape[1] if wrong_side else 0.0)

        run = min(runs, key=run_score)
        x = float(np.mean(run))
        if recent and abs(x - expected_x) > 0.20 * mask.shape[1]:
            continue
        observations.append((y, x))
        recent.append((y, x))
        if len(recent) > 2:
            recent.pop(0)

    if len(observations) < 5:
        return None
    ys = np.asarray([point[0] for point in observations], dtype=np.float64)
    xs = np.asarray([point[1] for point in observations], dtype=np.float64)
    degree = 2 if len(observations) >= 8 else 1
    keep = np.ones(len(observations), dtype=bool)
    for _ in range(3):
        coefficients = np.polyfit(ys[keep], xs[keep], degree)
        residuals = xs - np.polyval(coefficients, ys)
        median = np.median(residuals[keep])
        mad = np.median(np.abs(residuals[keep] - median))
        limit = max(6.0, 3.5 * 1.4826 * mad)
        updated = np.abs(residuals - median) <= limit
        if updated.sum() < degree + 1 or np.array_equal(updated, keep):
            break
        keep = updated
    coefficients = np.polyfit(ys[keep], xs[keep], degree)
    return (float(np.polyval(coefficients, y_far)),
            float(np.polyval(coefficients, y_near)),
            float(np.polyval(np.polyder(coefficients), y_near)))


def _task1c_fallback(frame, yellow_mask, white_mask):
    """Use Task 1C's scored center estimate when paired tracking is occluded."""
    height, width = frame.shape[:2]
    frame_center = width / 2.0
    y_start = int(0.52 * height)
    y_end = int(0.86 * height)
    y_far = int(0.52 * height)
    y_ref = int(0.62 * height)

    def fit(mask, side=None):
        observations = []
        for y in range(y_start, y_end, 2):
            xs = np.flatnonzero(mask[y])
            if not xs.size:
                continue
            runs = np.split(xs, np.flatnonzero(np.diff(xs) > 2) + 1)
            runs = [run for run in runs if run.size >= 3]
            if not runs:
                continue
            if side is None:
                run = max(runs, key=lambda item: item.size)
            else:
                eligible = [run for run in runs
                            if (float(np.mean(run)) - frame_center) * side > 0]
                if not eligible:
                    continue
                run = min(eligible,
                          key=lambda item: abs(float(np.mean(item)) - frame_center))
            observations.append((y, float(np.mean(run))))

        if len(observations) < 5:
            return None
        ys = np.asarray([point[0] for point in observations], dtype=np.float64)
        xs = np.asarray([point[1] for point in observations], dtype=np.float64)
        degree = 2 if len(observations) >= 8 else 1
        keep = np.ones(len(observations), dtype=bool)
        for _ in range(3):
            coefficients = np.polyfit(ys[keep], xs[keep], degree)
            residuals = xs - np.polyval(coefficients, ys)
            median = np.median(residuals[keep])
            mad = np.median(np.abs(residuals[keep] - median))
            limit = max(8.0, 3.0 * 1.4826 * mad)
            updated = np.abs(residuals - median) <= limit
            if updated.sum() < degree + 1 or np.array_equal(updated, keep):
                break
            keep = updated
        coefficients = np.polyfit(ys[keep], xs[keep], degree)
        return (float(np.polyval(coefficients, y_far)),
                float(np.polyval(coefficients, y_ref)),
                float(np.polyval(np.polyder(coefficients), y_ref)))

    yellow = fit(yellow_mask)
    if yellow is None:
        return None
    white_side = -1 if yellow[1] > frame_center else 1
    white = fit(white_mask, white_side)
    if white is None:
        return None
    if (yellow[1] - frame_center) * (white[1] - frame_center) >= 0:
        return None
    lane_width = abs(white[1] - yellow[1])
    if lane_width < 0.15 * width:
        return None

    center_far = 0.5 * (yellow[0] + white[0])
    center_ref = 0.5 * (yellow[1] + white[1])
    lane = "left" if white[1] > frame_center else "right"
    heading_slope = 0.5 * (yellow[2] + white[2])
    return {
        "valid": True,
        "cte": (center_ref - frame_center) / lane_width,
        "heading_error": float(np.arctan(heading_slope)),
        "lane_width_px": lane_width,
        "lane": lane,
        "center_far": center_far,
        "center_ref": center_ref,
    }


def _ackermann_wheel_angles(steering):
    """Split the virtual steering angle into Task 1A front-wheel angles."""
    if abs(steering) < 1e-12:
        return 0.0, 0.0

    half_track = (_TRACK_WIDTH / 2.0) - _WHEEL_OFFSET
    tangent = np.tan(steering)
    left = np.arctan(tangent / (1.0 - half_track * tangent / _WHEELBASE))
    right = np.arctan(tangent / (1.0 + half_track * tangent / _WHEELBASE))
    return float(left), float(right)


def detect_lane(frame):
    '''
    Purpose:
    ---
    Work out, from a single camera frame, how wrong the vehicle currently is -
    and hand back whatever `compute_control()` needs in order to correct it.

    This is your Task 1C detector, extended. Task 1C answered *where the lane
    centre is* and *which lane the vehicle is in*; a controller cannot use
    either of those directly. It needs to know how far the vehicle is from
    where it should be, and which way it is pointing.

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]
        One BGR frame from the vehicle's camera, of shape (height, width, 3),
        the right way up and in the right colour order.

    Returns:
    ---
    Anything you like - a tuple, a dictionary, your own class. Whatever you
    return here is passed straight to compute_control() and nowhere else, so
    the two functions only have to agree with each other.

    Example call:
    ---
    detection = detect_lane(frame)

    REMEMBER:
    ---
    Both lanes are driven during evaluation, and the arena looks mirrored from
    the other one. Do not hard-code which side the boundary is on.

    Do NOT call cv2.imshow() or cv2.imwrite() from inside this function.

    You are marked on how closely you hold the MIDDLE of your lane, not merely
    on staying the correct side of the dashed line.
    '''

    global _tracked_markings, _previous_lane_center
    global _locked_lane_sides, _detection_misses
    global _filtered_cte, _filtered_heading

    height, width = frame.shape[:2]
    frame_center = width / 2.0
    y_start = int(_ROI_TOP * height)
    y_end = int(_ROI_BOTTOM * height)
    y_far = int(_LOOKAHEAD_FAR * height)
    y_near = int(_LOOKAHEAD_NEAR * height)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    yellow = cv2.inRange(hsv, _YELLOW_LOWER, _YELLOW_UPPER)
    white = ((gray >= _WHITE_VALUE_MIN) &
             (hsv[:, :, 1] <= _WHITE_SATURATION_MAX) &
             (hsv[:, :, 2] >= _WHITE_VALUE_MIN)).astype(np.uint8) * 255
    kernel = np.ones((3, 3), dtype=np.uint8)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)

    def make_candidate(yellow_side, white_side, yellow_fit=None, white_fit=None):
        if yellow_fit is None:
            yellow_fit = _fit_marking(
                yellow, frame_center, y_start, y_end, y_far, y_near, yellow_side)
        if white_fit is None:
            white_fit = _fit_marking(
                white, frame_center, y_start, y_end, y_far, y_near, white_side)
        if yellow_fit is None or white_fit is None:
            return None
        left_fit, right_fit = ((yellow_fit, white_fit) if yellow_side < 0
                               else (white_fit, yellow_fit))
        if left_fit[0] >= right_fit[0] or left_fit[1] >= right_fit[1]:
            return None
        lane_width = 0.5 * ((right_fit[0] - left_fit[0]) +
                            (right_fit[1] - left_fit[1]))
        markings_bracket_vehicle = (
            left_fit[1] < frame_center < right_fit[1]
        )
        if not markings_bracket_vehicle or not (0.15 * width <= lane_width <= 0.95 * width):
            return None

        # These are the two virtual centerline points: each is halfway between
        # the fitted yellow edge and dashed divider at the same lookahead row.
        lane_far = 0.5 * (left_fit[0] + right_fit[0])
        lane_near = 0.5 * (left_fit[1] + right_fit[1])
        heading_slope = 0.5 * (left_fit[2] + right_fit[2])
        score = abs(lane_near - frame_center) / lane_width
        return {"far": lane_far, "near": lane_near, "width": lane_width,
                "heading_slope": heading_slope,
                "score": score, "yellow_side": yellow_side,
                "white_side": white_side, "yellow_fit": yellow_fit,
                "white_fit": white_fit}

    candidates = []
    previous_center = _previous_lane_center
    if _tracked_markings is not None:
        yellow_side, previous_yellow, white_side, previous_white = _tracked_markings
        yellow_fit = _fit_marking(yellow, frame_center, y_start, y_end,
                                  y_far, y_near, 0, previous_yellow)
        white_fit = _fit_marking(white, frame_center, y_start, y_end,
                                 y_far, y_near, 0, previous_white)
        tracked = make_candidate(yellow_side, white_side, yellow_fit, white_fit)
        if tracked is not None and _previous_lane_center is not None:
            old_far, old_near = _previous_lane_center
            jump = (abs(tracked["near"] - old_near) +
                    0.35 * abs(tracked["far"] - old_far)) / tracked["width"]
            if jump <= 0.45:
                candidates.append(tracked)

    if not candidates:
        # Keep the same yellow/divider pair through short corner occlusions.
        # Re-selecting by image-centre proximity on every miss can switch the
        # target to the divider or the adjacent lane while turning.
        _tracked_markings = None
        _detection_misses += 1
        alternatives = ((-1, 1), (1, -1))
        if _locked_lane_sides is not None:
            # Prefer the current pair through brief losses. If it remains
            # missing through a corner, allow the other image orientation,
            # but only a pair that brackets the camera (Task 1C rule) qualifies.
            other = ((1, -1) if _locked_lane_sides == (-1, 1) else (-1, 1))
            alternatives = ((_locked_lane_sides,) if _detection_misses <= 4
                           else (_locked_lane_sides, other))
        for yellow_side, white_side in alternatives:
            candidate = make_candidate(yellow_side, white_side)
            if candidate is not None:
                if previous_center is not None:
                    old_far, old_near = previous_center
                    jump = (abs(candidate["near"] - old_near) +
                            0.35 * abs(candidate["far"] - old_far)) / candidate["width"]
                    candidate["score"] += 1.5 * jump
                if (_locked_lane_sides is not None and
                        (yellow_side, white_side) != _locked_lane_sides):
                    candidate["score"] += 0.25
                candidates.append(candidate)

    if not candidates:
        _tracked_markings = None
        fallback = _task1c_fallback(frame, yellow, white)
        if fallback is not None:
            _detection_misses = 0
            fallback_sides = ((-1, 1) if fallback["lane"] == "left"
                              else (1, -1))
            _locked_lane_sides = fallback_sides
            _previous_lane_center = (fallback["center_far"],
                                     fallback["center_ref"])
            _filtered_cte = fallback["cte"]
            _filtered_heading = fallback["heading_error"]
            return fallback
        return {"valid": False}

    relock = (_locked_lane_sides is None or _detection_misses > 4)
    candidate = min(candidates, key=lambda item: item["score"])
    _detection_misses = 0
    if relock:
        _locked_lane_sides = (candidate["yellow_side"], candidate["white_side"])
    _tracked_markings = (candidate["yellow_side"], candidate["yellow_fit"],
                         candidate["white_side"], candidate["white_fit"])
    _previous_lane_center = (candidate["far"], candidate["near"])
    return {
        "valid": True,
        "cte": (candidate["near"] - frame_center) / candidate["width"],
        # Use the local lane tangent at Task 1C's reference row. Positive image
        # x points right, matching positive Task 1B heading toward a left turn.
        "heading_error": float(np.arctan(candidate["heading_slope"])),
        "lane_width_px": candidate["width"],
    }


def compute_control(detection, stop_requested):
    '''
    Purpose:
    ---
    Turn the errors from detect_lane() into the three numbers the vehicle is
    driven with, and obey the stop flag.

    This is where your Task 1B controller and your Task 1A steering geometry
    meet: work out one steering angle from the error, then split it into the
    two front-wheel angles the joints actually take.

    Input Arguments:
    ---
    `detection` :   [ any ]
        Whatever detect_lane() returned for this frame.

    `stop_requested` :  [ bool ]
        True while the evaluation is holding the vehicle at a stop point.
        This is a LEVEL, not a pulse: it stays True for the whole duration of
        the stop and goes False when the wait is over. While it is True the
        vehicle must be STATIONARY, so return a speed of 0.0. There is no
        stopwatch for you to write and no duration to hard-code.

    Returns:
    ---
    `left_angle`, `right_angle` :   [ float ]
        The two front-wheel steering angles, in RADIANS, written straight to
        the steering joints. The scene's steering joints are POSITIVE TO THE
        LEFT. Nothing is clamped for you - the vehicle's own limit is 45
        degrees, and commanding past it will not steer any harder.

    `speed` :   [ float ]
        Target velocity for both drive motors, in rad/s. 0.0 holds the
        vehicle still.

    Example call:
    ---
    left_angle, right_angle, speed = compute_control(detection, stop_requested)

    REMEMBER:
    ---
    Keep the loop running while stopped - set the SPEED to zero, not your
    program. If you stop reading frames you will not see the flag drop.

    Sooner or later a frame comes back with nothing usable in it. Holding the
    last steering angle, easing off the throttle and stopping dead are all
    defensible - doing nothing is not.
    '''

    global _filtered_cte, _filtered_heading, _last_steering, _last_speed
    global _integral_cte, _last_control_time

    if stop_requested:
        _last_speed = 0.0
        _last_control_time = time.monotonic()
        left, right = _ackermann_wheel_angles(_last_steering)
        return left, right, 0.0

    if isinstance(detection, dict) and detection.get("valid"):
        cte = float(detection["cte"])
        heading_error = float(detection["heading_error"])

        if _filtered_cte is None:
            _filtered_cte = cte
            _filtered_heading = heading_error
        else:
            _filtered_cte += _FILTER_ALPHA * (cte - _filtered_cte)
            _filtered_heading += _FILTER_ALPHA * (heading_error - _filtered_heading)

        # Task 1B defines cte = vehicle_y - target_y. Positive image-space
        # error means the target is to the right, hence the vehicle is left:
        # negate it before applying the Task 1B Stanley equation.
        cte_m = -_filtered_cte * _LANE_WIDTH_M
        now = time.monotonic()
        dt = 0.0 if _last_control_time is None else min(
            0.25, max(0.0, now - _last_control_time))
        _last_control_time = now
        _integral_cte = float(np.clip(
            _integral_cte + cte_m * dt, -_INTEGRAL_CLAMP, _INTEGRAL_CLAMP))
        speed_mps = _TASK1B_SPEED_MPS * max(
            0.30, _last_speed / _CRUISE_SPEED)
        cross_track_term = np.arctan2(
            _STANLEY_K * cte_m, _STANLEY_SOFT + speed_mps)
        requested = float(np.clip(
            _HEADING_WEIGHT * _filtered_heading + cross_track_term
            + _INTEGRAL_K * _integral_cte,
            -_MAX_STEER, _MAX_STEER))
        steering = _last_steering + float(np.clip(
            requested - _last_steering, -_MAX_STEER_STEP, _MAX_STEER_STEP))

        # Reduce speed for large lane error or a sharper bend; the task has no
        # lap-time target, and the wheels need time to follow the path.
        speed = _CRUISE_SPEED / (1.0 + 1.2 * abs(steering))
        speed = min(speed, _CRUISE_SPEED * max(0.20, 1.0 - 1.5 * abs(_filtered_cte)))
        speed = float(max(_CRUISE_SPEED * 0.20, speed))
    else:
        steering = _last_steering
        # Keep the last lane-hold steering and creep so the camera can see
        # beyond a brief marking loss. The persistent lane lock prevents a
        # recovery frame from silently selecting the adjacent lane.
        speed = _RECOVERY_SPEED

    _last_steering = steering
    _last_speed = speed
    left, right = _ackermann_wheel_angles(steering)
    return left, right, float(speed)


##############################################################
############## END OF YOUR IMPLEMENTATION ####################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################
#
# Connecting to the simulator, reading the camera and writing to the joints.
# None of it involves a decision you are being marked on - it is here so that
# every team reads the same picture and drives the same vehicle.
#
#####################################################################

def connect_and_locate():
    '''Open the remote API connection and look up every scene handle once.

    Each getObject() is a network round trip, which is why they all happen
    here rather than inside the loop.
    '''
    try:
        sim = RemoteAPIClient().getObject('sim')
        sim.getSimulationTime()
    except Exception as err:
        raise ConnectionError(
            f'Could not reach CoppeliaSim on the ZMQ Remote API port.\n'
            f'  Is CoppeliaSim running with Task_2A.ttt open?\n'
            f'  Simulator said: {err}')

    handles = {}
    for name, path in (('vision_sensor', VISION_SENSOR),
                       ('steer_left', STEER_LEFT),
                       ('steer_right', STEER_RIGHT),
                       ('motor_left', MOTOR_LEFT),
                       ('motor_right', MOTOR_RIGHT)):
        try:
            handles[name] = sim.getObject(path)
        except Exception as err:
            raise RuntimeError(
                f'No object at "{path}" in the open scene.\n'
                f'  Open Task_2A.ttt from the repository.\n'
                f'  Simulator said: {err}')
    return sim, handles


def read_frame(sim, handles):
    '''Grab one frame from the vehicle's camera as a BGR image.

    The vision sensor hands back a raw byte buffer and a resolution, not an
    image, and three corrections are needed to turn one into the other. None
    of these problems announces itself - each produces a detector that looks
    like it is working on a frame that is wrong:

      * handleVisionSensor() - this scene's sensor renders only when asked.
        Without it every read returns the SAME frame for the whole lap.
      * cv2.flip(frame, 0) - the buffer's first row is the BOTTOM of the
        image, so the road comes out vertically mirrored.
      * RGB2BGR - the buffer is RGB and OpenCV assumes BGR. Without the swap
        every HSV threshold you tuned in Task 1C selects the wrong colour.

    Note `res[1], res[0]` in the reshape too: the resolution comes back as
    (width, height) while NumPy wants rows first.

    Returns None if no frame is ready yet, which happens on the first few
    iterations after the simulation starts.
    '''
    sim.handleVisionSensor(handles['vision_sensor'])
    img, res = sim.getVisionSensorImg(handles['vision_sensor'], 0)
    if len(img) == 0:
        return None
    frame = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
    frame = cv2.flip(frame, 0)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def drive(sim, handles, left_angle, right_angle, speed):
    '''Write the two steering angles and the drive speed to the joints.'''
    sim.setJointTargetPosition(handles['steer_left'], float(left_angle))
    sim.setJointTargetPosition(handles['steer_right'], float(right_angle))
    sim.setJointTargetVelocity(handles['motor_left'], float(speed))
    sim.setJointTargetVelocity(handles['motor_right'], float(speed))


def main():
    sim, handles = connect_and_locate()

    # While you are developing you press Play yourself. Under evaluation the
    # harness resets the scene, places the vehicle in the lane being tested
    # and starts the simulation - so wait for that rather than starting your
    # own, which would race the harness into position.
    #
    # Your script must never call startSimulation(), stopSimulation() or move
    # the vehicle. Reading the camera and driving the joints is all it does.
    deadline = time.monotonic() + 120.0
    while sim.getSimulationState() != sim.simulation_advancing_running:
        if time.monotonic() > deadline:
            print('The simulation never started. Press Play in CoppeliaSim, '
                  'then run this script again.')
            return 1
        time.sleep(0.05)
    print('Simulation running - driving.')

    frames = 0
    try:
        # The harness stops the simulation when the lap is over, which is the
        # cue to exit - hence testing the state rather than looping forever.
        while sim.getSimulationState() == sim.simulation_advancing_running:
            frame = read_frame(sim, handles)
            if frame is None:
                continue
            frames += 1

            # getInt32Signal() returns None until the signal is first set, so
            # "not set" reads as "not stopping" and the first frames behave.
            stop_requested = bool(sim.getInt32Signal(STOP_SIGNAL))

            detection = detect_lane(frame)
            left_angle, right_angle, speed = compute_control(detection,
                                                             stop_requested)
            drive(sim, handles, left_angle, right_angle, speed)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    except Exception as err:
        # A call raising because the simulation stopped mid-frame is normal at
        # the end of a run; anything else is a bug worth seeing.
        print(f'Loop ended: {type(err).__name__}: {err}')
    finally:
        # Without this, whatever speed was last commanded is still being
        # applied and the vehicle drives on unsteered.
        try:
            sim.setJointTargetVelocity(handles['motor_left'], 0.0)
            sim.setJointTargetVelocity(handles['motor_right'], 0.0)
        except Exception:
            pass

    print(f'{frames} frames processed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
