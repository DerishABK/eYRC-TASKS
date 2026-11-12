"""Task 3A Mini Theme Run: camera-only lane navigation, signs, obstacles, parking.

All solution code is intentionally in this file, as required by the Task 3
submission instructions.  The evaluator owns simulation start/stop and scene
placement; this program only reads the vehicle camera and commands joints.
"""

import math
import sys
import time

import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    sys.stdout.reconfigure(line_buffering=True)
except (AttributeError, OSError):
    pass


# Scene handles are looked up by their documented paths. No object poses are
# read from CoppeliaSim.
VISION_SENSOR = "/Niti_Vahan/Camera_joint/Cuboid/visionSensor"
STEER_LEFT = "/steeringLeft"
STEER_RIGHT = "/steeringRight"
MOTOR_LEFT = "/motorLeft"
MOTOR_RIGHT = "/motorRight"

WHEELBASE = 0.120
TRACK_WIDTH = 0.110
WHEEL_OFFSET = 0.0275
STEER_LIMIT = math.radians(43.0)
CAMERA_FOV_DEG = 95.0
MARKER_SIDE = 0.096
CAMERA_PITCH = math.radians(30.0)
CAMERA_ABOVE_MARKER = 0.066
CAMERA_AHEAD = 0.047
IMG_W, IMG_H = 640, 480
FOCAL_PX = (IMG_W / 2.0) / math.tan(math.radians(CAMERA_FOV_DEG) / 2.0)
CAMERA_MATRIX = np.array([[FOCAL_PX, 0, IMG_W / 2],
                          [0, FOCAL_PX, IMG_H / 2],
                          [0, 0, 1]], dtype=np.float64)
HALF_MARKER = MARKER_SIDE / 2.0
MARKER_CORNERS_3D = np.array([[-HALF_MARKER, HALF_MARKER, 0],
                              [HALF_MARKER, HALF_MARKER, 0],
                              [HALF_MARKER, -HALF_MARKER, 0],
                              [-HALF_MARKER, -HALF_MARKER, 0]], dtype=np.float32)

YELLOW_LOWER = np.array([15, 90, 90], np.uint8)
YELLOW_UPPER = np.array([42, 255, 255], np.uint8)
WHITE_SAT_MAX = 115
WHITE_VAL_MIN = 150
LANE_ROI_TOP = 0.34
LANE_ROI_BOTTOM = 0.91
LANE_WIDTH_M = 0.20

MAX_SPEED = 3.0
SPEED40_LIMIT = 1.2
BASE_SPEED = 2.0
RECOVERY_SPEED = 0.25
LANE_LOSS_GRACE_FRAMES = 2
LANE_CHANGE_LOSS_GRACE_FRAMES = 6
LANE_CHANGE_OFFSET_STEP = 0.06
LANE_CHANGE_MIN_SPEED = 0.80
TASK1B_SPEED_MPS = 0.029
TASK1B_STANLEY_K = 0.3
TASK1B_STANLEY_SOFT = 0.05
TASK1B_INTEGRAL_K = 0.01
TASK1B_HEADING_WEIGHT = 0.50
TASK1B_MAX_STEER = math.radians(39.0)
TASK1B_MAX_STEER_STEP = 0.08

BLUE_LOWER = np.array([88, 70, 45], np.uint8)
BLUE_UPPER = np.array([140, 255, 255], np.uint8)
RED_LOWER_1 = np.array([0, 110, 90], np.uint8)
RED_UPPER_1 = np.array([12, 255, 255], np.uint8)
RED_LOWER_2 = np.array([168, 110, 90], np.uint8)
RED_UPPER_2 = np.array([180, 255, 255], np.uint8)


def ackermann_wheel_angles(steering):
    """Split one virtual steering angle into the two front-wheel angles."""
    if abs(steering) < 1e-10:
        return 0.0, 0.0
    kingpin_track = TRACK_WIDTH - 2.0 * WHEEL_OFFSET
    tangent = math.tan(steering)
    left = math.atan2(2.0 * WHEELBASE * tangent,
                      2.0 * WHEELBASE - kingpin_track * tangent)
    right = math.atan2(2.0 * WHEELBASE * tangent,
                       2.0 * WHEELBASE + kingpin_track * tangent)
    return float(left), float(right)


# Task 1A geometry and Task 1B controller parameters.
_WHEELBASE = 0.120
_TRACK_WIDTH = 0.110
_WHEEL_OFFSET = 0.0275
_CRUISE_SPEED = 1.3
_RECOVERY_SPEED = 0.25  # minimum fraction of cruise while markings are lost
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
_last_lane_width_profile = None
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


def _task1c_fallback(frame, yellow_mask, white_mask, lane_sides=None):
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

    if lane_sides is None:
        yellow = fit(yellow_mask)
        if yellow is None:
            return None
        white_side = -1 if yellow[1] > frame_center else 1
        yellow_side = -1 if yellow[1] < frame_center else 1
        white = fit(white_mask, white_side)
    else:
        # Once a lap's lane is known, anchor recovery on that lane's yellow
        # edge and look for its dashed divider on the opposite side. In the
        # inner/right lane this means yellow on the car's right, divider left.
        yellow_side, white_side = lane_sides
        yellow = fit(yellow_mask, yellow_side)
        white = fit(white_mask, white_side)
    if yellow is None:
        return None
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


def _detect_task2a_lane(frame, desired_white_side=None,
                       allow_unbracketed=False,
                       continuity_reacquire=False):
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
    global _last_lane_width_profile
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

    def make_candidate(yellow_side, white_side, yellow_fit=None, white_fit=None,
                       require_bracket=True):
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
        markings_bracket_vehicle = left_fit[1] < frame_center < right_fit[1]
        # Bracketing the camera is a good way to identify the starting lane,
        # but it is a bad recovery rule: once the car drifts across a marking,
        # the correct pair no longer brackets the camera and the controller
        # loses the very lane it needs to steer back into. After lane lock,
        # retain the same ordered pair and let its signed centre error recover.
        if ((require_bracket and not markings_bracket_vehicle) or
                not (0.15 * width <= lane_width <= 0.95 * width)):
            return None

        # These are the two virtual centerline points: each is halfway between
        # the fitted yellow edge and dashed divider at the same lookahead row.
        lane_far = 0.5 * (left_fit[0] + right_fit[0])
        lane_near = 0.5 * (left_fit[1] + right_fit[1])
        lane_width_far = max(1.0, right_fit[0] - left_fit[0])
        lane_width_near = max(1.0, right_fit[1] - left_fit[1])
        heading_slope = 0.5 * (left_fit[2] + right_fit[2])
        score = abs(lane_near - frame_center) / lane_width
        if score > 1.20:
            return None
        return {"far": lane_far, "near": lane_near, "width": lane_width,
                "width_far": lane_width_far, "width_near": lane_width_near,
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
        tracked = make_candidate(
            yellow_side, white_side, yellow_fit, white_fit,
            require_bracket=(_locked_lane_sides is None and
                             not allow_unbracketed))
        if tracked is not None and _previous_lane_center is not None:
            old_far, old_near = _previous_lane_center
            jump = (abs(tracked["near"] - old_near) +
                    0.35 * abs(tracked["far"] - old_far)) / tracked["width"]
            # A frame-to-frame jump close to a full lane width means the fit
            # likely latched onto the neighbouring pair. Reject it and use the
            # locked lane's one-edge recovery or stop safely for that frame.
            if jump <= 0.85:
                tracked["score"] += 0.5 * jump
                candidates.append(tracked)

    if not candidates:
        # Keep the same yellow/divider pair through short corner occlusions.
        # Re-selecting by image-centre proximity on every miss can switch the
        # target to the divider or the adjacent lane while turning.
        _tracked_markings = None
        _detection_misses += 1
        alternatives = ((-1, 1), (1, -1))
        if _locked_lane_sides is not None:
            # The evaluation starts a fresh script for each lap, so this
            # orientation identifies the lane selected at that lap's start.
            # Never flip it after a corner occlusion: the opposite yellow /
            # divider pairing is the adjacent lane, not a recovery target.
            alternatives = (_locked_lane_sides,)
        elif desired_white_side is not None:
            alternatives = tuple(pair for pair in alternatives
                                 if pair[1] == int(desired_white_side))
        for yellow_side, white_side in alternatives:
            candidate = make_candidate(
                yellow_side, white_side,
                require_bracket=(_locked_lane_sides is None and
                                 not allow_unbracketed))
            if candidate is not None:
                if previous_center is not None:
                    old_far, old_near = previous_center
                    jump = (abs(candidate["near"] - old_near) +
                            0.35 * abs(candidate["far"] - old_far)) / candidate["width"]
                    if jump > 0.85:
                        continue
                    candidate["score"] += 1.5 * jump
                if (_locked_lane_sides is not None and
                        (yellow_side, white_side) != _locked_lane_sides):
                    candidate["score"] += 0.25
                candidates.append(candidate)

    if not candidates:
        _tracked_markings = None
        if desired_white_side is not None:
            return {"valid": False}
        if continuity_reacquire:
            return {"valid": False}
        fallback = _task1c_fallback(
            frame, yellow, white, lane_sides=_locked_lane_sides)
        if fallback is not None:
            # Task 1C's fallback is useful when one marking is briefly
            # occluded. Do not demand a small image-space jump here: camera
            # bearing changes quickly on a tight bend, even when the physical
            # lane has not changed. The persistent side lock below guards the
            # lane identity.
            _detection_misses = 0
            fallback_sides = ((-1, 1) if fallback["lane"] == "left"
                              else (1, -1))
            if (_locked_lane_sides is not None and
                    fallback_sides != _locked_lane_sides):
                return {"valid": False}
            if abs(float(fallback["cte"])) > 1.20:
                fallback = None
        if fallback is not None:
            # A fallback may recover the centre estimate; it must not redefine
            # which lane the evaluator placed the vehicle in.
            if _locked_lane_sides is None:
                _locked_lane_sides = fallback_sides
            _previous_lane_center = (fallback["center_far"],
                                     fallback["center_ref"])
            _filtered_cte = fallback["cte"]
            _filtered_heading = fallback["heading_error"]
            return fallback

        # At tight bends the dashed divider can disappear behind the vehicle
        # or another object while the solid yellow edge remains visible. Keep
        # the locked lane identity and infer its centre from that one observed
        # edge plus the last measured perspective width. This is a short-term
        # visual recovery, not a map or a fixed steering command.
        if _locked_lane_sides is not None and _last_lane_width_profile is not None:
            yellow_side, white_side = _locked_lane_sides
            yellow_fit = _fit_marking(
                yellow, frame_center, y_start, y_end, y_far, y_near,
                yellow_side)
            white_fit = _fit_marking(
                white, frame_center, y_start, y_end, y_far, y_near,
                white_side)
            left_fit = yellow_fit if yellow_side < 0 else white_fit
            right_fit = white_fit if white_side > 0 else yellow_fit
            old_far_width, old_near_width = _last_lane_width_profile
            if left_fit is not None:
                far = left_fit[0] + old_far_width * 0.5
                near = left_fit[1] + old_near_width * 0.5
                slope = left_fit[2]
            elif right_fit is not None:
                far = right_fit[0] - old_far_width * 0.5
                near = right_fit[1] - old_near_width * 0.5
                slope = right_fit[2]
            else:
                return {"valid": False}
            width = 0.5 * (old_far_width + old_near_width)
            inferred = {
                "valid": True,
                "cte": (near - frame_center) / max(width, 1.0),
                "heading_error": float(np.arctan(slope)),
                "lane_width_px": width,
                "lane": "left" if white_side > 0 else "right",
                "center_far": far,
                "center_ref": near,
                "center": near,
                "far": far,
                "width": width,
                "width_far": old_far_width,
                "width_near": old_near_width,
                "white_side": white_side,
                "inferred_from_single_edge": True,
            }
            if abs(inferred["cte"]) > 1.20:
                return {"valid": False}
            _previous_lane_center = (far, near)
            _filtered_cte = inferred["cte"]
            _filtered_heading = inferred["heading_error"]
            return inferred
        return {"valid": False}

    relock = _locked_lane_sides is None
    candidate = min(candidates, key=lambda item: item["score"])
    _detection_misses = 0
    left_fit, right_fit = ((candidate["yellow_fit"], candidate["white_fit"])
                           if candidate["yellow_side"] < 0 else
                           (candidate["white_fit"], candidate["yellow_fit"]))
    _last_lane_width_profile = (
        max(1.0, right_fit[0] - left_fit[0]),
        max(1.0, right_fit[1] - left_fit[1]))
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
        "center": candidate["near"],
        "far": candidate["far"],
        "width": candidate["width"],
        "width_far": candidate["width_far"],
        "width_near": candidate["width_near"],
        "white_side": candidate["white_side"],
    }


class LaneVision:
    """Use the validated Task 2A paired-boundary tracker for Task 3."""

    def detect(self, frame, desired_white_side=None, allow_unbracketed=False):
        detection = _detect_task2a_lane(
            frame, desired_white_side=desired_white_side,
            allow_unbracketed=allow_unbracketed)
        if not isinstance(detection, dict) or not detection.get("valid"):
            return detection
        width = float(detection.get("width", detection.get("lane_width_px", 1.0)))
        center = float(detection.get("center", detection.get("center_ref", frame.shape[1] / 2.0 + detection.get("cte", 0.0) * width)))
        detection["width"] = width
        detection["width_far"] = float(detection.get("width_far", width))
        detection["width_near"] = float(detection.get("width_near", width))
        detection["center"] = center
        detection["far"] = float(detection.get("far", detection.get("center_far", center)))
        detection["heading"] = float(detection.get("heading_error", 0.0))
        if "white_side" not in detection:
            detection["white_side"] = (_locked_lane_sides[1] if _locked_lane_sides else 1)
        return detection

    def reacquire(self, frame, desired_white_side=None,
                  allow_unbracketed=False):
        """Reset old-lane tracking and find the lane currently around the car."""
        global _tracked_markings, _previous_lane_center, _locked_lane_sides
        global _detection_misses, _filtered_cte, _filtered_heading
        _tracked_markings = None
        _previous_lane_center = None
        _locked_lane_sides = None
        _detection_misses = 0
        _filtered_cte = None
        _filtered_heading = None
        return self.detect(frame, desired_white_side=desired_white_side,
                           allow_unbracketed=allow_unbracketed)

    def try_reacquire_lane(self, frame, desired_white_side):
        """Try the requested adjacent lane without losing the current fit."""
        global _tracked_markings, _previous_lane_center, _locked_lane_sides
        global _detection_misses, _filtered_cte, _filtered_heading
        saved = (_tracked_markings, _previous_lane_center, _locked_lane_sides,
                 _detection_misses, _filtered_cte, _filtered_heading)
        candidate = self.reacquire(
            frame, desired_white_side=desired_white_side,
            allow_unbracketed=True)
        if candidate and "cte" in candidate and int(
                candidate.get("white_side", 0)) == int(desired_white_side):
            return candidate
        (_tracked_markings, _previous_lane_center, _locked_lane_sides,
         _detection_misses, _filtered_cte, _filtered_heading) = saved
        return None

    def reacquire_current_lane(self, frame):
        """Relock either image orientation while favoring the previous lane."""
        global _tracked_markings, _previous_lane_center, _locked_lane_sides
        global _detection_misses, _filtered_cte, _filtered_heading
        previous_center = _previous_lane_center
        _tracked_markings = None
        _locked_lane_sides = None
        _detection_misses = 0
        _filtered_cte = None
        _filtered_heading = None
        _previous_lane_center = previous_center
        detection = _detect_task2a_lane(
            frame, allow_unbracketed=True, continuity_reacquire=True)
        if isinstance(detection, dict) and detection.get("valid"):
            detection["width"] = float(detection.get("width", detection["lane_width_px"]))
            detection["center"] = float(detection.get(
                "center", frame.shape[1] / 2.0 + detection["cte"] * detection["width"]))
            detection["far"] = float(detection.get("far", detection["center"]))
            detection["heading"] = float(detection.get("heading_error", 0.0))
        return detection

    def center_at(self, lane, y):
        if not lane or lane.get("width", 0) <= 1:
            return None
        # `center` is fitted at Task 1C's 0.62H reference row, not at the
        # bottom of the image. Use matching rows when projecting the lane
        # centre to an obstacle's image depth.
        y_near = _LOOKAHEAD_NEAR * IMG_H
        y_far = _LOOKAHEAD_FAR * IMG_H
        alpha = float(np.clip((y - y_far) / (y_near - y_far), -0.8, 1.5))
        return lane["far"] + alpha * (lane["center"] - lane["far"])

    def width_at(self, lane, y):
        if not lane or lane.get("width", 0) <= 1:
            return None
        y_near, y_far = _LOOKAHEAD_NEAR * IMG_H, _LOOKAHEAD_FAR * IMG_H
        # Lane width is measured only between the scored far/near rows. A
        # clipped obstacle box can end well below the near row; extrapolating
        # lane width toward the image bottom inflated the lane enough to
        # classify a vehicle in the neighboring lane as blocking this one.
        alpha = float(np.clip((y - y_far) / (y_near - y_far), 0.0, 1.0))
        far = float(lane.get("width_far", lane["width"]))
        near = float(lane.get("width_near", lane["width"]))
        return max(1.0, far + alpha * (near - far))


def _blue_label(crop):
    """Distinguish the H and P boards from the white glyph inside blue."""
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # Ignore the white frame and retain only the inner board/glyph.
    h, w = hsv.shape[:2]
    inner = hsv[int(0.18 * h):int(0.82 * h), int(0.18 * w):int(0.82 * w)]
    if inner.size == 0:
        return None
    glyph = ((inner[:, :, 2] > 150) & (inner[:, :, 1] < 100)).astype(np.uint8) * 255
    points = cv2.findNonZero(glyph)
    if points is None:
        return None
    x, y, gw, gh = cv2.boundingRect(points)
    glyph = glyph[y:y + gh, x:x + gw]
    if gh < 5 or gw < 3:
        return None
    glyph = cv2.resize(glyph, (32, 40), interpolation=cv2.INTER_AREA)
    ink = glyph > 75
    # H has two full-height stems. P has only its left stem reaching the
    # baseline. This silhouette test is more stable than raw font matching
    # when a board is small or viewed obliquely.
    left_ink = float(ink[:, :12].sum())
    right_bottom = float(ink[24:, 18:].sum())
    total_ink = float(ink.sum())
    if total_ink > 0 and left_ink > 0:
        baseline_ratio = right_bottom / left_ink
        if baseline_ratio < 0.12:
            return "Parking"
        if baseline_ratio > 0.24:
            return "Hospital"
    best = {"H": -1.0, "P": -1.0}
    for label, char in (("H", "H"), ("P", "P")):
        for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX,
                     cv2.FONT_HERSHEY_COMPLEX):
            for thickness in (1, 2, 3):
                templ = np.zeros((48, 40), np.uint8)
                cv2.putText(templ, char, (3, 39), font, 1.25, 255, thickness,
                            cv2.LINE_AA)
                templ = cv2.resize(templ, (32, 40), interpolation=cv2.INTER_AREA)
                a = glyph.astype(np.float32) / 255.0
                b = templ.astype(np.float32) / 255.0
                denom = float(np.linalg.norm(a) * np.linalg.norm(b))
                score = float((a * b).sum() / denom) if denom else -1.0
                best[label] = max(best[label], score)
    if max(best.values()) < 0.28:
        return None
    return "Hospital" if best["H"] >= best["P"] else "Parking"


def detect_signs(frame):
    """Return camera-visible sign labels with image boxes and pixel area."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []

    blue = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE,
                            np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, bw, bh = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if (area < 180 or bw < 18 or bh < 18 or
                not (0.45 <= bw / max(bh, 1) <= 1.60) or
                area / max(bw * bh, 1) < 0.45):
            continue
        pad = max(2, int(0.12 * max(bw, bh)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(w, x + bw + pad), min(h, y + bh + pad)
        label = _blue_label(frame[y0:y1, x0:x1])
        if label:
            detections.append({"label": label, "area": int(area),
                               "bbox": (x, y, bw, bh),
                               "center": (x + bw / 2.0, y + bh / 2.0),
                               "clipped": bool(x <= 1 or y <= 1 or
                                               x + bw >= w - 1 or
                                               y + bh >= h - 1)})

    red = cv2.bitwise_or(cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1),
                         cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2))
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 180:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.045 * perimeter, True)
        x, y, bw, bh = cv2.boundingRect(contour)
        if bw < 12 or bh < 12:
            continue
        ratio = bw / max(bh, 1)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        if len(approx) == 3 and area > 250:
            detections.append({"label": "School", "area": int(area),
                               "bbox": (x, y, bw, bh),
                               "center": (x + bw / 2.0, y + bh / 2.0)})
        else:
            sign_crop = frame[y:y + bh, x:x + bw]
            sign_hsv = cv2.cvtColor(sign_crop, cv2.COLOR_BGR2HSV)
            inner = sign_hsv[int(0.18 * bh):max(int(0.82 * bh), 1),
                             int(0.18 * bw):max(int(0.82 * bw), 1)]
            white_inside = (float(np.mean((inner[:, :, 1] < 100) &
                                          (inner[:, :, 2] > 155)))
                            if inner.size else 0.0)
            if circularity > 0.62 and 0.72 <= ratio <= 1.35 and white_inside > 0.18:
                detections.append({"label": "Speed40", "area": int(area),
                                   "bbox": (x, y, bw, bh),
                                   "center": (x + bw / 2.0, y + bh / 2.0)})
    return detections


def detect_obstacles(frame):
    """Return solid red obstacles using the updated Task 2B classifier."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1),
                         cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2))
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        ratio = bw / float(max(bh, 1))
        if not (0.70 <= ratio <= 1.35):
            continue
        fill = np.zeros((bh, bw), dtype=np.uint8)
        cv2.drawContours(fill, [contour - [x, y]], -1, 255, -1)
        red_points = np.count_nonzero(
            (red[y:y + bh, x:x + bw] > 0) & (fill > 0))
        if red_points / max(area, 1.0) < 0.75:
            continue
        extent = area / float(max(bw * bh, 1))
        if extent < 0.68:
            continue
        perimeter = cv2.arcLength(contour, True)
        circularity = (4.0 * math.pi * area / max(perimeter * perimeter, 1.0))
        if circularity < 0.55:
            continue
        result.append({"area": int(round(area)),
                       "bbox": (x, y, bw, bh),
                       "center": (x + bw / 2.0, y + bh / 2.0)})
    return sorted(result, key=lambda item: item["area"], reverse=True)


class MarkerPose:
    """Estimate bay-relative pose from ArUco corners using the Task 2C model."""

    def __init__(self):
        self.heading = None
        self.rho = None

    @staticmethod
    def _bearing(corners):
        cx, cy = corners[:, 0].mean(), corners[:, 1].mean()
        xc = (cx - IMG_W / 2.0) / FOCAL_PX
        yc = (IMG_H / 2.0 - cy) / FOCAL_PX
        return float(math.atan2(xc, math.cos(CAMERA_PITCH) +
                                yc * math.sin(CAMERA_PITCH)))

    @staticmethod
    def _pnp(corners):
        try:
            _, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                MARKER_CORNERS_3D, corners.astype(np.float32), CAMERA_MATRIX,
                np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return None, None
        solutions = []
        for rv, tv, error in zip(rvecs, tvecs, np.asarray(errors).flatten()):
            rotation, _ = cv2.Rodrigues(rv)
            forward = rotation.T @ np.array([0.0, 0.0, 1.0])
            heading = float(math.atan2(forward[0], -forward[2]))
            vec = tv.flatten()
            rho = math.sqrt(max(float(vec @ vec) - CAMERA_ABOVE_MARKER ** 2, 1e-6))
            solutions.append((float(error), heading, rho))
        if not solutions:
            return None, None
        solutions.sort(key=lambda p: p[0])
        clear = (len(solutions) == 1 or
                 solutions[1][0] > 1.5 * max(solutions[0][0], 1e-6))
        return solutions[0][1] if clear else None, solutions[0][2]

    def update(self, corners, last_steering):
        if corners is None:
            return None
        bearing = self._bearing(corners)
        yaw_seen, rho_now = self._pnp(corners)
        if rho_now is None:
            return None
        first_observation = self.rho is None
        if first_observation:
            self.rho = rho_now
        rho_prev = self.rho
        self.rho = 0.6 * self.rho + 0.4 * rho_now
        if first_observation:
            # Task 3 reaches the parking marker after driving the full route,
            # so its approach heading is not the parallel-at-start heading
            # assumed by the standalone Task 2C parking exercise. Initialize
            # from a clear marker pose when available.
            self.heading = yaw_seen if yaw_seen is not None else 0.0
        else:
            ds = float(np.clip(
                (rho_prev - self.rho) / max(math.cos(bearing), 0.3),
                0.0, 0.01))
            self.heading -= math.tan(last_steering) / WHEELBASE * ds
            if (yaw_seen is not None and
                    abs(yaw_seen - self.heading) < math.radians(15)):
                self.heading += 0.12 * (yaw_seen - self.heading)
        los = bearing + self.heading
        lateral_cam = -self.rho * math.sin(los)
        forward_cam = self.rho * math.cos(los)
        lateral_front = lateral_cam + (WHEELBASE / 2.0 - CAMERA_AHEAD) * math.sin(self.heading)
        forward = forward_cam + CAMERA_AHEAD * math.cos(self.heading)
        return {"lateral": float(-lateral_front), "heading": float(-self.heading),
                "forward": float(forward), "bearing": float(-bearing)}


def _find_marker(detector, frame):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None
    # Task 3 has one parking target. Select the largest visible marker instead
    # of assuming a target ID from the two-bay Task 2C practice scene.
    choices = sorted(zip(corners, ids.flatten()),
                     key=lambda p: cv2.contourArea(p[0].reshape(-1, 1, 2)),
                     reverse=True)
    return choices[0][0].reshape(4, 2) if choices else None


def _read_frame(sim, handles):
    sim.handleVisionSensor(handles["camera"])
    raw, resolution = sim.getVisionSensorImg(handles["camera"], 0)
    if not raw:
        return None
    frame = np.frombuffer(raw, dtype=np.uint8).reshape(
        (resolution[1], resolution[0], 3))
    return cv2.cvtColor(cv2.flip(frame, 0), cv2.COLOR_RGB2BGR)


def _connect():
    client = RemoteAPIClient()
    sim = client.getObject("sim")
    paths = {"camera": VISION_SENSOR, "steer_left": STEER_LEFT,
             "steer_right": STEER_RIGHT, "motor_left": MOTOR_LEFT,
             "motor_right": MOTOR_RIGHT}
    handles = {}
    for key, path in paths.items():
        try:
            handles[key] = sim.getObject(path)
        except Exception as exc:
            raise RuntimeError("Task 3 scene object missing: " + path) from exc
    return sim, handles


def _drive(sim, handles, steering, speed):
    steering = float(np.clip(steering, -STEER_LIMIT, STEER_LIMIT))
    speed = float(np.clip(speed, 0.0, MAX_SPEED))
    left, right = ackermann_wheel_angles(steering)
    sim.setJointTargetPosition(handles["steer_left"], left)
    sim.setJointTargetPosition(handles["steer_right"], right)
    sim.setJointTargetVelocity(handles["motor_left"], speed)
    sim.setJointTargetVelocity(handles["motor_right"], speed)


class Mission:
    """Camera-driven Task 3A states and event latches."""

    def __init__(self):
        self.lanes = LaneVision()
        self.state = "FOLLOW"
        self.hospital_done = False
        self.stop_started = None
        self.stop_stable_frames = 0
        self.stop_complete_time = None
        self.speed_zone = False
        self.speed_sign_count = 0
        self.speed_sign_visible = False
        self.speed_sign_clear = 0
        self.parking_seen = 0
        self.parking_sign_visible = False
        self.parking_sign_clear = 0
        self.parking_sign_age = 10**6
        self.parking_marker_stable = 0
        self.target_offset = 0.0
        self.lane_change_committed = False
        self.lane_change_good = 0
        self.lane_change_direction = 0.0
        self.lane_change_using_target = False
        self.lane_change_frames = 0
        self.target_lane_white_side = None
        self.frame_index = 0
        self.lane_change_passed_baseline = 0
        self.lane_change_rearm_frame = 0
        self.obstacle_tracks = []
        self.obstacles_passed = 0
        self.obstacle_speed_cap = MAX_SPEED
        self.prev_gray = None
        self.prev_steering = 0.0
        self.last_speed = BASE_SPEED
        self.filtered_cte = None
        self.filtered_heading = None
        self.integral = 0.0
        self.last_wall_time = None
        self.lane_misses = 0
        self.last_pose = None
        self.marker_misses = 0
        self.parked_frames = 0
        # Task 2C's staged bay approach, kept per mission rather than in
        # module globals because Task 3 may be evaluated more than once in a
        # Python process.
        self.parking_motion = "CROSSING"
        self.parking_smoothed_steering = 0.0
        self.parking_stop_confirm = 0
        self.parking_target_angle = None
        self.parking_straighten_threshold = None
        self.parking_straighten_forward = None

    def _stationary(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = gray[int(0.14 * gray.shape[0]):int(0.68 * gray.shape[0]), :]
        if self.prev_gray is None:
            self.prev_gray = roi
            return False
        prev = self.prev_gray
        self.prev_gray = roi
        if prev.shape != roi.shape:
            return False
        shift, response = cv2.phaseCorrelate(prev.astype(np.float32),
                                             roi.astype(np.float32))
        return response > 0.10 and math.hypot(*shift) < 0.35

    def _lane_control(self, lane, speed_cap, now_wall, speed_scale=1.0):
        if not lane or "cte" not in lane:
            self.lane_misses += 1
            # Bridge only a few dropped camera fits at crawl speed. If the
            # markings remain unavailable, stop instead of driving blind.
            grace = (LANE_CHANGE_LOSS_GRACE_FRAMES
                     if self.state == "LANE_CHANGE" else LANE_LOSS_GRACE_FRAMES)
            if self.lane_misses <= grace:
                speed = min(speed_cap, RECOVERY_SPEED)
            else:
                speed = 0.0
            self.last_speed = speed
            return self.prev_steering, speed
        self.lane_misses = 0
        cte = float(lane["cte"]) + self.target_offset
        heading = float(lane["heading"])
        if self.filtered_cte is None:
            self.filtered_cte, self.filtered_heading = cte, heading
        else:
            self.filtered_cte += 0.40 * (cte - self.filtered_cte)
            self.filtered_heading += 0.40 * (heading - self.filtered_heading)
        dt = 0.0 if self.last_wall_time is None else float(np.clip(now_wall - self.last_wall_time, 0, 0.2))
        self.last_wall_time = now_wall
        cte_m = -self.filtered_cte * LANE_WIDTH_M
        self.integral = float(np.clip(self.integral + cte_m * dt, -0.10, 0.10))
        speed_mps = TASK1B_SPEED_MPS * max(
            0.30, self.last_speed / BASE_SPEED)
        request = (TASK1B_HEADING_WEIGHT * self.filtered_heading +
                   math.atan2(TASK1B_STANLEY_K * cte_m,
                              TASK1B_STANLEY_SOFT + speed_mps) +
                   TASK1B_INTEGRAL_K * self.integral)
        request = float(np.clip(request, -TASK1B_MAX_STEER,
                                TASK1B_MAX_STEER))
        steering = self.prev_steering + float(np.clip(request - self.prev_steering,
                                                       -TASK1B_MAX_STEER_STEP,
                                                       TASK1B_MAX_STEER_STEP))
        speed = BASE_SPEED / (1.0 + 1.2 * abs(steering))
        # Slow before a bend becomes a lane-loss event. The image tangent is
        # noisy at sharp corners, so cap speed progressively instead of
        # keeping cruise speed until the detector fails.
        curve_speed = BASE_SPEED * max(
            0.25, 1.0 - 0.80 * abs(self.filtered_heading))
        speed = min(speed, curve_speed)
        if self.state == "LANE_CHANGE":
            speed = max(LANE_CHANGE_MIN_SPEED, speed) * speed_scale
        else:
            speed = min(speed, BASE_SPEED * max(0.20,
                                                1.0 - 1.5 * abs(self.filtered_cte)))
            speed = max(BASE_SPEED * 0.20, speed) * speed_scale
        speed = min(float(speed), speed_cap)
        self.last_speed = speed
        return steering, speed

    def _consider_lane_change(self, lane, obstacles):
        self.obstacle_speed_cap = MAX_SPEED
        if not lane or "cte" not in lane or not obstacles:
            return
        h = IMG_H
        for obstacle in obstacles:
            x, y, bw, bh = obstacle["bbox"]
            # Use apparent size and lane-relative image position. React while
            # the obstacle is still distant enough to make a safe lane change.
            if bh < 14 or y > int(0.90 * h):
                continue
            # The bottom of a vehicle box is its road-contact estimate; using
            # the box centre exaggerates perspective error at distant cars.
            center_y = y + bh
            lane_center = self.lanes.center_at(lane, center_y)
            lane_width = self.lanes.width_at(lane, center_y)
            if lane_center is None or lane_width is None:
                continue
            lane_width = float(lane_width)
            obstacle_x = x + bw / 2.0
            effective_center = lane_center + self.target_offset * lane_width
            # A broad box clipped at a camera edge can have its centre close
            # enough to the lane to pass a centre-distance-only test. Require
            # a substantial part of the object footprint to lie between this
            # lane's fitted boundaries before it can brake or redirect us.
            lane_left = effective_center - 0.5 * lane_width
            lane_right = effective_center + 0.5 * lane_width
            overlap = max(0.0, min(x + bw, lane_right) - max(x, lane_left))
            overlap_fraction = overlap / max(float(bw), 1.0)
            same_lane = (lane_left <= obstacle_x <= lane_right and
                         overlap_fraction >= 0.40)
            if not same_lane:
                continue

            # If the adjacent lane is occupied at a similar image depth, hold
            # back instead of steering into another vehicle. The speed cap
            # tightens as the detected obstacle grows in the camera.
            proximity = float(np.clip((bh - 14.0) / 90.0, 0.0, 1.0))
            self.obstacle_speed_cap = min(
                self.obstacle_speed_cap,
                max(0.0, BASE_SPEED * (1.0 - 1.15 * proximity)))

            if self.lane_change_committed:
                continue
            direction = float(lane["white_side"])
            adjacent_center = effective_center + direction * lane_width
            adjacent_blocked = False
            for other in obstacles:
                ox, oy, ow, oh = other["bbox"]
                depth_tolerance = max(26.0, 1.25 * max(bh, oh))
                if abs((oy + oh) - center_y) > depth_tolerance:
                    continue
                if abs((ox + ow / 2.0) - adjacent_center) <= (
                        0.28 * lane_width + 0.5 * ow):
                    adjacent_blocked = True
                    break
            if adjacent_blocked:
                continue

            self.target_offset = 0.0
            self.lane_change_direction = direction
            self.lane_change_using_target = False
            self.target_lane_white_side = -int(direction)
            self.lane_change_committed = True
            self.lane_change_passed_baseline = self.obstacles_passed
            self.lane_change_good = 0
            self.lane_change_frames = 0
            self.state = "LANE_CHANGE"
            break

    def _track_obstacles(self, obstacles):
        matched = set()
        for obstacle in obstacles:
            cx, cy = obstacle["center"]
            bw, bh = obstacle["bbox"][2:]
            best_i, best_d = None, float("inf")
            for i, track in enumerate(self.obstacle_tracks):
                if i in matched:
                    continue
                scale = max(36.0, 0.12 * IMG_H, 0.7 * max(bh, track["h"]))
                distance = math.hypot((cx - track["x"]) / 1.4,
                                      cy - track["y"])
                if distance < scale and distance < best_d:
                    best_i, best_d = i, distance
            if best_i is None:
                self.obstacle_tracks.append({"x": cx, "y": cy, "w": bw,
                                             "h": bh, "missing": 0})
                matched.add(len(self.obstacle_tracks) - 1)
            else:
                track = self.obstacle_tracks[best_i]
                track.update({"x": cx, "y": cy, "w": bw, "h": bh,
                              "missing": 0})
                matched.add(best_i)
        survivors = []
        for i, track in enumerate(self.obstacle_tracks):
            if i not in matched:
                track["missing"] += 1
            if track["missing"] >= 30:
                self.obstacles_passed += 1
            else:
                survivors.append(track)
        self.obstacle_tracks = survivors

    def _reacquire_target_lane(self, frame):
        if self.lane_change_direction == 0.0:
            return None
        desired_white_side = -int(self.lane_change_direction)
        candidate = self.lanes.try_reacquire_lane(
            frame, desired_white_side=desired_white_side)
        if not candidate:
            return None
        self.lane_change_using_target = True
        self.target_lane_white_side = desired_white_side
        self.target_offset = 0.0
        self.lane_misses = 0
        self.filtered_cte = None
        self.filtered_heading = None
        self.integral = 0.0
        return candidate

    def step(self, sim, frame, lane, signs, obstacles, pose):
        self.frame_index += 1
        sim_time = float(sim.getSimulationTime())
        now_wall = time.monotonic()

        speed_sign = any(s["label"] == "Speed40" for s in signs)
        if speed_sign:
            self.speed_sign_clear = 0
            if not self.speed_sign_visible:
                self.speed_sign_count = min(2, self.speed_sign_count + 1)
                self.speed_zone = self.speed_sign_count == 1
                self.speed_sign_visible = True
        else:
            self.speed_sign_clear += 1
            if self.speed_sign_clear >= 20:
                self.speed_sign_visible = False

        parking_sign = any(s["label"] == "Parking" for s in signs)
        if parking_sign:
            self.parking_sign_age = 0
            self.parking_sign_clear = 0
            if not self.parking_sign_visible:
                self.parking_seen += 1
                self.parking_sign_visible = True
        else:
            self.parking_sign_age += 1
            self.parking_sign_clear += 1
            if self.parking_sign_clear >= 8:
                self.parking_sign_visible = False

        # Do not switch into the parking controller on a one-frame marker
        # sighting or a stale Parking sign. Wait until the sign was recently
        # seen and the marker pose is both close and stable in view.
        if (pose is not None and 0.15 <= pose["forward"] <= 1.0 and
                self.parking_sign_age <= 60):
            self.parking_marker_stable += 1
        else:
            self.parking_marker_stable = 0

        if (lane and "cte" in lane and
                self.target_lane_white_side is not None and
                int(lane.get("white_side", 0)) == self.target_lane_white_side):
            # The adjacent boundary pair is now visibly around the car. Drop
            # the virtual offset and steer to the measured target-lane centre.
            self.target_offset = 0.0
            self.lane_change_using_target = True

        self._track_obstacles(obstacles)
        if (self.state == "FOLLOW" and self.lane_change_committed and
                self.frame_index >= self.lane_change_rearm_frame and
                self.obstacles_passed > self.lane_change_passed_baseline):
            # A later obstacle may require another lane change. Re-arm only
            # after the obstacle that triggered the previous manoeuvre has
            # left the camera tracks and a cooldown has elapsed.
            self.lane_change_committed = False
        if self.state == "LANE_CHANGE":
            self.lane_change_frames += 1
            if self.lane_change_using_target:
                if not lane or "cte" not in lane:
                    candidate = self._reacquire_target_lane(frame)
                    if candidate:
                        lane = candidate
            else:
                # Keep tracking the original lane until the virtual centre
                # has shifted far enough that the requested adjacent lane is
                # visible. Failed probes restore the original lane tracker.
                should_probe = (self.lane_change_frames % 8 == 0 or
                                not lane or "cte" not in lane)
                candidate = (self._reacquire_target_lane(frame)
                             if should_probe else None)
                if candidate:
                    lane = candidate
                elif lane and "cte" in lane:
                    self.target_offset = float(np.clip(
                        self.target_offset + LANE_CHANGE_OFFSET_STEP *
                        self.lane_change_direction,
                        min(0.0, self.lane_change_direction),
                        max(0.0, self.lane_change_direction)))
        self._consider_lane_change(lane, obstacles)

        hospital = [s for s in signs if s["label"] == "Hospital"]
        if (not self.hospital_done and hospital and self.state == "FOLLOW"):
            # Apparent sign height provides a camera-distance cue. Trigger on
            # size and image position, never a world pose or elapsed travel time.
            _, sy, _, sh = max((s["bbox"] for s in hospital),
                               key=lambda b: b[3])
            if sh >= 0.075 * IMG_H and sy + sh >= 0.38 * IMG_H:
                self.state = "HOSPITAL_STOP"
                self.stop_started = None

        speed_cap = SPEED40_LIMIT if self.speed_zone else MAX_SPEED
        if self.state == "HOSPITAL_STOP":
            stable = self._stationary(frame)
            self.stop_stable_frames = self.stop_stable_frames + 1 if stable else 0
            if self.stop_started is None and self.stop_stable_frames >= 3:
                self.stop_started = sim_time
                self.stop_complete_time = sim_time + 5.0
            if self.stop_started is not None and sim_time - self.stop_started >= 5.0:
                self.hospital_done = True
                self.state = "FOLLOW"
            self.prev_steering = 0.0
            return 0.0, 0.0, self.state

        if self.state == "PARKING":
            if pose is None:
                # A single missing marker frame is expected during the bay
                # crossing. Continue with the last filtered command, as in
                # the updated Task 2C controller, but never declare parking
                # without a measured pose.
                parking_cues = [s for s in signs
                                if s["label"] == "Parking"]
                if parking_cues:
                    cue = max(parking_cues, key=lambda item: item["area"])
                    cue_x = float(cue["center"][0])
                    right_bearing = math.atan2(
                        cue_x - IMG_W / 2.0, FOCAL_PX)
                    steering = float(np.clip(-1.2 * right_bearing,
                                             -0.42, 0.42))
                    self.prev_steering = steering
                    return steering, min(0.24, speed_cap), self.state

                self.marker_misses += 1
                if self.last_pose is None:
                    return 0.0, 0.0, self.state
                # The updated Task 2C geometry places the bay midpoint at
                # 0.19 m forward.  If the marker disappears after the car has
                # already reached that zone, finish from the last measured
                # pose instead of driving through the bay indefinitely.
                if (self.marker_misses >= 8 and
                        self.last_pose.get("forward", 1.0) <= 0.22):
                    self.state = "DONE"
                    return 0.0, 0.0, self.state
                # Match the updated Task 2C behavior: a missing marker frame
                # is not a reason to freeze permanently.  Continue with the
                # filtered, bounded command until the marker is reacquired;
                # final completion still requires a fresh measured pose.
                return (self.parking_smoothed_steering,
                        min(0.8, speed_cap), self.state)
            else:
                self.last_pose = pose
                self.marker_misses = 0
            lateral = pose["lateral"]
            heading = pose["heading"]
            forward = pose["forward"]

            # Task 2C's midpoint is the evaluator's parking target. Confirm
            # several measured frames at or beyond it before ending the run.
            if forward <= 0.19:
                self.parking_stop_confirm += 1
                if self.parking_stop_confirm >= 3:
                    self.state = "DONE"
                    return 0.0, 0.0, self.state
                self.prev_steering = 0.0
                return 0.0, 0.0, self.state
            else:
                self.parking_stop_confirm = 0

            if self.parking_target_angle is None:
                # The Hard scene uses the farther bay geometry from Task 2C;
                # retain the lateral fallback for scenes whose bay is nearer.
                if lateral > 0.33:
                    self.parking_target_angle = math.radians(-39.5)
                    self.parking_straighten_threshold = 0.018
                    self.parking_straighten_forward = 0.38
                else:
                    self.parking_target_angle = math.radians(-24.0)
                    self.parking_straighten_threshold = 0.030
                    self.parking_straighten_forward = 0.42

            if forward <= 0.25:
                speed = 0.5
            elif forward <= 0.37:
                speed = 0.8
            elif forward <= 0.48:
                speed = 1.0
            else:
                speed = 1.2

            if (self.parking_motion == "CROSSING" and
                    (lateral <= self.parking_straighten_threshold or
                     forward <= self.parking_straighten_forward)):
                self.parking_motion = "STRAIGHTENING"

            if self.parking_motion == "CROSSING":
                raw_steer = 1.5 * (self.parking_target_angle - heading)
            else:
                raw_steer = -1.6 * heading - 4.0 * lateral
            raw_steer = float(np.clip(raw_steer,
                                      -STEER_LIMIT, STEER_LIMIT))
            self.parking_smoothed_steering = (
                0.70 * self.parking_smoothed_steering + 0.30 * raw_steer)
            self.prev_steering = self.parking_smoothed_steering
            return (float(self.parking_smoothed_steering),
                    min(float(speed), speed_cap), self.state)

        if self.state == "DONE":
            return 0.0, 0.0, self.state

        if (self.hospital_done and self.speed_sign_count >= 2 and
                self.obstacles_passed >= 3 and
                self.parking_marker_stable >= 3):
            self.state = "PARKING"
            self.last_pose = pose
            return 0.0, 0.0, self.state

        if self.state == "LANE_CHANGE" and lane and "cte" in lane:
            target_error = float(lane["cte"]) + self.target_offset
            if abs(target_error) < 0.18:
                self.lane_change_good += 1
                if self.lane_change_good >= 6:
                    if self.lane_change_using_target:
                        self.state = "FOLLOW"
                        self.lane_change_direction = 0.0
                        self.lane_change_rearm_frame = self.frame_index + 90
                    else:
                        if abs(self.target_offset) >= 0.85:
                            self.state = "FOLLOW"
                            self.lane_change_direction = 0.0
                            self.lane_change_rearm_frame = self.frame_index + 90
                        else:
                            self.lane_change_good = 0
            else:
                self.lane_change_good = 0

        speed_scale = 0.90 if self.state == "LANE_CHANGE" else 1.0
        steering, speed = self._lane_control(lane, speed_cap, now_wall, speed_scale)
        self.prev_steering = steering
        return steering, min(speed, speed_cap, self.obstacle_speed_cap), self.state


def _connect_and_run():
    sim, handles = _connect()
    try:
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters())
    except AttributeError:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters_create()
        detector = (dictionary, params)
    marker_pose = MarkerPose()
    mission = Mission()

    deadline = time.monotonic() + 120.0
    while sim.getSimulationState() != sim.simulation_advancing_running:
        if time.monotonic() >= deadline:
            print("Simulation did not start; evaluator must start it.")
            return 2
        time.sleep(0.05)

    frames = 0
    last_state = mission.state
    try:
        while sim.getSimulationState() == sim.simulation_advancing_running:
            frame = _read_frame(sim, handles)
            if frame is None:
                continue
            frames += 1
            lane = mission.lanes.detect(frame)
            signs = detect_signs(frame)
            obstacles = detect_obstacles(frame)
            if isinstance(detector, tuple):
                corners_all, ids, _ = cv2.aruco.detectMarkers(
                    frame, detector[0], parameters=detector[1])
                marker_corners = None
                if ids is not None:
                    idx = int(np.argmax([cv2.contourArea(c.reshape(-1, 1, 2))
                                         for c in corners_all]))
                    marker_corners = corners_all[idx].reshape(4, 2)
            else:
                marker_corners = _find_marker(detector, frame)
            pose = marker_pose.update(marker_corners, mission.prev_steering)
            steering, speed, state = mission.step(
                sim, frame, lane, signs, obstacles, pose)
            _drive(sim, handles, steering, speed)
            if state != last_state:
                marker_summary = ("none" if pose is None else
                                  "fwd=%.2f lat=%.2f hdg=%.2f" %
                                  (pose["forward"], pose["lateral"],
                                   pose["heading"]))
                print("state_transition frame=%d %s->%s hospital_done=%s "
                      "speed_signs=%d obstacles_passed=%d parking_seen=%d "
                      "marker=%s parking_stable=%d" %
                      (frames, last_state, state, mission.hospital_done,
                       mission.speed_sign_count, mission.obstacles_passed,
                       mission.parking_seen, marker_summary,
                       mission.parking_marker_stable), flush=True)
                if state == "PARKING":
                    # The transition is gated by three consecutive current
                    # marker poses.  Preserve that estimate at handoff; reset
                    # it here would turn a valid parking target into an
                    # artificial marker loss on the next frame.
                    mission.last_pose = pose
                    mission.marker_misses = 0
                    mission.parking_motion = "CROSSING"
                    mission.parking_smoothed_steering = 0.0
                    mission.parking_stop_confirm = 0
                    mission.parking_target_angle = None
                    mission.parking_straighten_threshold = None
                    mission.parking_straighten_forward = None
                last_state = state
            if state == "DONE":
                print("Task 3A parked; returning control to the evaluator.")
                break
            if frames <= 10 or frames % 50 == 0:
                lane_valid = isinstance(lane, dict) and "cte" in lane
                cte = float(lane["cte"]) if lane_valid else float("nan")
                heading = float(lane["heading"]) if lane_valid else float("nan")
                white_side = int(lane.get("white_side", 0)) if lane_valid else 0
                sign_labels = ",".join(s["label"] for s in signs) or "-"
                print("frames=%d state=%s lane=%s cte=%.3f heading=%.3f "
                      "white_side=%d offset=%.2f visible_signs=%s "
                      "steer=%.3f speed=%.2f obstacle_cap=%.2f "
                      "signs=%d obstacles=%d" %
                      (frames, state, "ok" if lane_valid else "lost", cte,
                       heading, white_side, mission.target_offset, sign_labels, steering,
                       speed, mission.obstacle_speed_cap, mission.speed_sign_count,
                       mission.obstacles_passed),
                      flush=True)
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as exc:
        print("Task 3A loop ended: %s: %s" % (type(exc).__name__, exc))
        raise
    finally:
        try:
            _drive(sim, handles, 0.0, 0.0)
        except Exception:
            pass
    print("Task 3A processed %d camera frames." % frames)
    return 0


def main():
    try:
        return _connect_and_run()
    except Exception as exc:
        print("Task 3A could not connect or initialize: %s: %s" %
              (type(exc).__name__, exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())
