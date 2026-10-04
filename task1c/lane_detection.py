'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 1C of Niti Vahan (NV) Theme.
*
*  Filename:         lane_detection.py
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

# Team ID:          NV_5761
# Author List:      Derish Abraham
# Filename:         lane_detection.py
# Functions:        detect_lane
# Global variables: None


####################### IMPORT MODULES #######################
import argparse
import json
import os

import cv2
import numpy as np
##############################################################

# The only three values "lane" is allowed to take.
LANE_LEFT = "left"
LANE_RIGHT = "right"
LANE_UNKNOWN = "unknown"
VALID_LANES = (LANE_LEFT, LANE_RIGHT, LANE_UNKNOWN)


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

def detect_lane(frame):
    '''
    Purpose:
    ---
    Detect the lane in a single frame and report where the centre of the lane
    is, and which of the two lanes the vehicle is currently in.

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]
        A single BGR frame read from the video, of shape (height, width, 3).

    Returns:
    ---
    `result` :  [ dict ]
        {
            "center_x" : int,   x-pixel of the lane centre in this frame,
                                or -1 if the lane could not be found
            "lane"     : str,   "left", "right" or "unknown"
        }

    Example call:
    ---
    result = detect_lane(frame)

    COORDINATE SYSTEM:
    ---
    `center_x` is an absolute pixel column in the frame AS RECEIVED - the
    dataset's own resolution, 640x480. It is compared against a ground truth
    measured in those pixels, so it only means anything in them.

    You may resize, crop or warp all you like inside this function, but scale
    the answer back before returning it. A centre found in a 320x240 copy is
    half the value it should be, and a centre read off a bird's-eye view is in
    warped coordinates, not frame ones - map the point back through the inverse
    of your transform. Do not re-encode or resize the clip files themselves.

    NOTE:
    ---
    This function must ONLY compute and return the result.
    Do not call cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from
    inside it. All visualisation and debugging output belongs outside this
    function - see draw_overlay() and process_video() below.
    '''

    height, width = frame.shape[:2]
    center_x = -1
    lane = LANE_UNKNOWN

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    yellow_mask = (
        (hsv[:, :, 0] >= 15)
        & (hsv[:, :, 0] <= 40)
        & (hsv[:, :, 1] >= 100)
        & (hsv[:, :, 2] >= 100)
    )
    white_mask = (
        (gray >= 180)
        & (hsv[:, :, 1] <= 80)
        & (hsv[:, :, 2] >= 180)
    )

    y_start = int(0.52 * height)
    y_end = int(0.86 * height)
    reference_y = int(0.62 * height)
    frame_center = width / 2.0

    def fit_marking(mask, side=None):
        observations = []
        for y in range(y_start, y_end, 2):
            xs = np.flatnonzero(mask[y])
            if xs.size == 0:
                continue

            runs = np.split(xs, np.flatnonzero(np.diff(xs) > 2) + 1)
            runs = [run for run in runs if run.size >= 3]
            if side is None:
                if runs:
                    selected = max(runs, key=lambda run: run.size)
                else:
                    continue
            else:
                eligible = [
                    run for run in runs
                    if (float(np.mean(run)) - frame_center) * side > 0
                ]
                if not eligible:
                    continue
                selected = min(
                    eligible,
                    key=lambda run: abs(float(np.mean(run)) - frame_center),
                )
            observations.append((y, float(np.mean(selected))))

        if len(observations) < 5:
            return None

        ys = np.asarray([point[0] for point in observations], dtype=np.float64)
        xs = np.asarray([point[1] for point in observations], dtype=np.float64)
        degree = 2 if len(observations) >= 8 else 1
        keep = np.ones(len(observations), dtype=bool)
        for _ in range(3):
            coefficients = np.polyfit(ys[keep], xs[keep], degree)
            residuals = xs - np.polyval(coefficients, ys)
            median_residual = np.median(residuals[keep])
            mad = np.median(np.abs(residuals[keep] - median_residual))
            limit = max(8.0, 3.0 * 1.4826 * mad)
            updated = np.abs(residuals - median_residual) <= limit
            if updated.sum() < degree + 1 or np.array_equal(updated, keep):
                break
            keep = updated

        coefficients = np.polyfit(ys[keep], xs[keep], degree)
        return float(np.polyval(coefficients, reference_y))

    yellow_x = fit_marking(yellow_mask)
    if yellow_x is not None:
        white_side = -1 if yellow_x > frame_center else 1
        white_x = fit_marking(white_mask, white_side)
        if white_x is not None:
            lane_width = abs(white_x - yellow_x)
            markings_bracket_vehicle = (
                (yellow_x - frame_center) * (white_x - frame_center) < 0
            )
            if lane_width >= 0.15 * width and markings_bracket_vehicle:
                center_x = int(round((yellow_x + white_x) / 2.0))
                if white_x > frame_center + 0.015 * width:
                    lane = LANE_LEFT
                elif white_x < frame_center - 0.015 * width:
                    lane = LANE_RIGHT

    return {"center_x": center_x, "lane": lane}


# ------------------------------------------------------------------
# Add any helper functions and global variables you need below this
# comment, and keep them ABOVE the "END OF YOUR IMPLEMENTATION" line.
# They must be called from detect_lane() - the evaluation script only
# ever calls that one function. List them in the file header too.
# ------------------------------------------------------------------


##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

def validate_result(result, frame_index):
    '''
    Purpose:
    ---
    Check that detect_lane() returned the expected structure and normalise it,
    so that a malformed return is reported here instead of silently scoring
    zero during evaluation.

    Input Arguments:
    ---
    `result` :          [ object ]      whatever detect_lane() returned
    `frame_index` :     [ int ]         index of the frame, used in error messages

    Returns:
    ---
    `clean` :           [ dict ]        {"center_x": int, "lane": str}
    '''
    where = "detect_lane() on frame {}".format(frame_index)

    if not isinstance(result, dict):
        raise TypeError("{} must return a dict, got {}".format(where, type(result).__name__))

    missing = {"center_x", "lane"} - set(result.keys())
    if missing:
        raise ValueError("{} is missing the key(s): {}".format(where, ", ".join(sorted(missing))))

    center_x = result["center_x"]
    if isinstance(center_x, bool) or not isinstance(center_x, (int, float, np.integer, np.floating)):
        raise TypeError("{} returned center_x of type {}, expected a number".format(
            where, type(center_x).__name__))
    center_x = int(round(float(center_x)))

    lane = result["lane"]
    if not isinstance(lane, str):
        raise TypeError("{} returned lane of type {}, expected a string".format(
            where, type(lane).__name__))
    lane = lane.strip().lower()
    if lane not in VALID_LANES:
        raise ValueError("{} returned lane = '{}', expected one of {}".format(
            where, result["lane"], ", ".join(VALID_LANES)))

    return {"center_x": center_x, "lane": lane}


def draw_overlay(frame, result):
    '''
    Purpose:
    ---
    Draw the detected lane centre and lane label on a copy of the frame.
    This is where display code belongs - never inside detect_lane().

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]   the frame that was passed to detect_lane()
    `result` :  [ dict ]            the validated result for that frame

    Returns:
    ---
    `canvas` :  [ numpy.ndarray ]   a copy of the frame with the overlay drawn
    '''
    canvas = frame.copy()
    height, width = canvas.shape[:2]

    # frame centre, for reference - roughly where the vehicle is pointing
    cv2.line(canvas, (width // 2, height), (width // 2, height - 40), (128, 128, 128), 1)

    center_x = result["center_x"]
    if 0 <= center_x < width:
        cv2.line(canvas, (center_x, height), (center_x, height // 2), (0, 0, 255), 2)
        cv2.circle(canvas, (center_x, height - 10), 5, (0, 0, 255), -1)

    label = "lane: {}   center_x: {}".format(result["lane"], center_x)
    cv2.putText(canvas, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return canvas


def process_video(video_path, show=False):
    '''
    Purpose:
    ---
    Read a video frame by frame, hand each frame to detect_lane() and collect
    the results.

    Input Arguments:
    ---
    `video_path` :  [ str ]     path to the video file
    `show` :        [ bool ]    if True, display the overlay while processing

    Returns:
    ---
    `results` :     [ list ]    one dict per frame:
                                {"frame": int, "center_x": int, "lane": str}
    '''
    if not os.path.isfile(video_path):
        raise FileNotFoundError("no such video file: {}".format(video_path))

    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        raise IOError("OpenCV could not open the video: {}".format(video_path))

    window = "Task 1C - {}".format(os.path.basename(video_path))
    results = []
    frame_index = 0

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break

            # A copy is passed in, so anything drawn inside detect_lane() cannot
            # corrupt the frame used for display.
            result = validate_result(detect_lane(frame.copy()), frame_index)
            results.append({"frame": frame_index, **result})

            if show:
                cv2.imshow(window, draw_overlay(frame, result))
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break

            frame_index += 1
    finally:
        capture.release()
        if show:
            cv2.destroyAllWindows()

    if not results:
        raise IOError("no frames could be read from: {}".format(video_path))

    return results


def summarise(video_path, results):
    '''
    Purpose:
    ---
    Print a one-line-per-video summary, so you can see at a glance whether the
    detector is returning anything sensible.

    Input Arguments:
    ---
    `video_path` :  [ str ]     path to the video that was processed
    `results` :     [ list ]    output of process_video()

    Returns:
    ---
    None
    '''
    total = len(results)
    counts = {lane: 0 for lane in VALID_LANES}
    for entry in results:
        counts[entry["lane"]] += 1
    not_found = sum(1 for entry in results if entry["center_x"] < 0)

    print("{:<28} {:>5} frames | left {:>5} | right {:>5} | unknown {:>5} | no centre {:>5}".format(
        os.path.basename(video_path), total,
        counts[LANE_LEFT], counts[LANE_RIGHT], counts[LANE_UNKNOWN], not_found))


def expand_videos(paths):
    '''
    Purpose:
    ---
    Turn the command-line arguments into a list of video files, accepting a
    FOLDER as well as individual files.

    A folder is the portable way to say "all the clips": Windows shells do not
    expand `public/*.mp4` the way bash does - cmd and PowerShell hand the
    pattern through verbatim and the script would look for a file literally
    named "*.mp4". `python lane_detection.py public` behaves the same on every
    platform.

    Input Arguments:
    ---
    `paths` :   [ list ]    the raw command-line arguments

    Returns:
    ---
    `videos` :  [ list ]    paths to individual video files, folders expanded
    '''
    videos = []
    for raw in paths:
        if os.path.isdir(raw):
            found = sorted(f for f in os.listdir(raw) if f.lower().endswith(".mp4"))
            if not found:
                raise FileNotFoundError("no .mp4 files in the folder: {}".format(raw))
            videos.extend(os.path.join(raw, f) for f in found)
        else:
            videos.append(raw)
    return videos


def main():
    parser = argparse.ArgumentParser(
        description="Task 1C - run your lane detector over one or more videos.")
    parser.add_argument("videos", nargs="+",
                        help="video file(s), or a folder holding them "
                             "(e.g. 'public')")
    parser.add_argument("--show", action="store_true",
                        help="display the detection overlay while processing (press q to stop)")
    parser.add_argument("--out", metavar="FILE",
                        help="write the per-frame results to this JSON file")
    args = parser.parse_args()

    all_results = {}
    for video_path in expand_videos(args.videos):
        results = process_video(video_path, show=args.show)
        summarise(video_path, results)
        all_results[os.path.basename(video_path)] = results

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(all_results, handle, indent=2)
        print("\nresults written to {}".format(args.out))


if __name__ == "__main__":
    main()
