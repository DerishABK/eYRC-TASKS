'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 2B of Niti Vahan (NV) Theme.
*
*  Filename:         sign_obstacle_detection.py
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

# Team ID:          5761
# Author List:      Derish Abraham,Joshua Varghese, Reibin Chacko Thomas, Delvin Varghese
# Filename:         sign_obstacle_detection.py
# Functions:        detect_signs, detect_obstacle, match_blue_sign
# Global variables: None


####################### IMPORT MODULES #######################
import cv2
import numpy as np
##############################################################

# The only sign labels detect_signs() is allowed to return.
SIGN_LABELS = ("School", "Speed40", "Parking", "Hospital")


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

import os

def detect_signs(image):
    '''
    Purpose:
    ---
    Detect every road sign visible in a single image, and report what each
    one is and how large it appears. An image can hold zero, one, or more
    than one sign at once.

    Input Arguments:
    ---
    `image` :   [ numpy.ndarray ]
        A single BGR image, of shape (height, width, 3).

    Returns:
    ---
    `detections` :  [ list of dict ]
        Zero, one, or many entries - one per sign found in the image:
        [
            {
                "label" : str,   one of SIGN_LABELS
                "area"  : int,   that sign's area in pixels
            },
            ...
        ]

    Example call:
    ---
    detections = detect_signs(image)

    NOTE:
    ---
    This function must ONLY compute and return the result. Do not call
    cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from inside it -
    reading images and reporting results is the evaluator's job, not this
    function's.
    '''

    detections = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 1. Red signs (School and Speed40)
    red1 = cv2.inRange(hsv, (0, 60, 60), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 60, 60), (180, 255, 255))
    red_mask = red1 | red2

    red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in red_cnts:
        area = cv2.contourArea(c)
        if area < 400:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = w / float(h)
        if not (0.70 <= aspect <= 1.35):
            continue

        # Signs are hollow rings with a light center (obstacles are solid red)
        fill = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(fill, [c - [x, y]], -1, 255, -1)
        red_pts = np.count_nonzero((red_mask[y:y + h, x:x + w] > 0) & (fill > 0))
        if (red_pts / area) > 0.65:
            continue

        peri = cv2.arcLength(c, True)
        if peri == 0:
            continue
        circ = 4 * np.pi * area / (peri * peri)
        poly = cv2.approxPolyDP(c, 0.04 * peri, True)

        # 3 vertices -> triangle (School), rounded -> circle (Speed40)
        label = "School" if (len(poly) == 3 or circ < 0.65) else "Speed40"
        detections.append({"label": label, "area": int(round(area))})

    # 2. Blue signs (Parking and Hospital)
    blue_mask = cv2.inRange(hsv, (95, 70, 70), (130, 255, 255))
    blue_cnts, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in blue_cnts:
        area = cv2.contourArea(c)
        if area < 400:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = w / float(h)
        if not (0.75 <= aspect <= 1.30):
            continue
        if (area / float(w * h)) < 0.65:
            continue
        peri = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
        if circ < 0.60:
            continue

        label = check_blue_symbol(image[y:y + h, x:x + w])
        if label in SIGN_LABELS:
            detections.append({"label": label, "area": int(round(area))})

    return detections


def detect_obstacle(image):
    '''
    Purpose:
    ---
    Detect whether the obstacle is visible in a single image, and how large
    it appears.

    Input Arguments:
    ---
    `image` :   [ numpy.ndarray ]
        A single BGR image, of shape (height, width, 3).

    Returns:
    ---
    `result` :  [ dict ]
        {
            "present" : bool,   True if the obstacle is visible
            "area"    : int,    its area in pixels, 0 if not present
        }

    Example call:
    ---
    result = detect_obstacle(image)

    NOTE:
    ---
    This function must ONLY compute and return the result. Do not call
    cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from inside it -
    reading images and reporting results is the evaluator's job, not this
    function's.
    '''

    present = False
    area = 0

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red1 = cv2.inRange(hsv, (0, 60, 60), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 60, 60), (180, 255, 255))
    red_mask = red1 | red2

    # Close small internal gaps
    kernel = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    max_area = 0

    for c in cnts:
        c_area = cv2.contourArea(c)
        if c_area < 500:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = w / float(h)
        if not (0.70 <= aspect <= 1.35):
            continue

        # Obstacle is a solid red cube on the road, signs are hollow borders
        fill = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(fill, [c - [x, y]], -1, 255, -1)
        red_pts = np.count_nonzero((red_mask[y:y + h, x:x + w] > 0) & (fill > 0))
        if (red_pts / c_area) < 0.75:
            continue

        # Reject thin cross shapes on buildings (box face fills > 68% of bounding box)
        if (c_area / float(w * h)) < 0.68:
            continue

        peri = cv2.arcLength(c, True)
        circ = 4 * np.pi * c_area / (peri * peri) if peri > 0 else 0
        if circ < 0.55:
            continue

        if c_area > max_area:
            max_area = c_area
            best_cnt = c

    if best_cnt is not None:
        present = True
        area = int(round(max_area))

    return {"present": present, "area": area}


# ------------------------------------------------------------------
# Add any helper functions and global variables you need below this
# comment, and keep them ABOVE the "END OF YOUR IMPLEMENTATION" line.
# They must be called from detect_signs() or detect_obstacle() - the
# evaluation script only ever calls those two functions. List them in
# the file header too.
# ------------------------------------------------------------------

def check_blue_symbol(crop):
    '''Tell Parking ('P') and Hospital ('H') signs apart.'''
    h, w = crop.shape[:2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] < 100) & (hsv[:, :, 2] > 140)).astype(np.uint8)

    # Focus on the center where the letter is (avoid sign edges)
    pad_y, pad_x = int(0.18 * h), int(0.18 * w)
    roi = white[pad_y:h - pad_y, pad_x:w - pad_x]
    if roi.size == 0 or roi.mean() < 0.15:
        return None  # No valid white letter found

    # Compare bottom-left vs bottom-right fill:
    # 'H' has two full vertical stems (bottom-right is filled)
    # 'P' has only one left stem (bottom-right is empty)
    rh, rw = roi.shape
    bl = roi[rh // 2:, :rw // 2].mean()
    br = roi[rh // 2:, rw // 2:].mean()
    ratio = br / (bl + 1e-5)

    label = "Parking" if ratio < 0.60 else "Hospital"

    # Optional cross-check with reference images if provided
    ref_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reference_images")
    p_ref_file = os.path.join(ref_folder, "P.jpg")
    h_ref_file = os.path.join(ref_folder, "Hospital.png")
    if os.path.isfile(p_ref_file) and os.path.isfile(h_ref_file):
        try:
            p_ref = cv2.Canny(cv2.imread(p_ref_file, 0), 50, 150)
            h_ref = cv2.Canny(cv2.imread(h_ref_file, 0), 50, 150)
            xp, yp, wp, hp = cv2.boundingRect(p_ref)
            xh, yh, wh, hh = cv2.boundingRect(h_ref)
            t_p = cv2.resize(p_ref[yp:yp + hp, xp:xp + wp], (40, 40))
            t_h = cv2.resize(h_ref[yh:yh + hh, xh:xh + wh], (40, 40))

            edges = cv2.Canny(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 50, 150)
            b = max(2, int(min(h, w) * 0.1))
            edges[:b, :] = edges[-b:, :] = edges[:, :b] = edges[:, -b:] = 0
            cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
                if bw > 4 and bh > 4:
                    sub = cv2.resize(edges[by:by + bh, bx:bx + bw], (40, 40))
                    sp = cv2.matchTemplate(sub, t_p, cv2.TM_CCOEFF_NORMED)[0][0]
                    sh = cv2.matchTemplate(sub, t_h, cv2.TM_CCOEFF_NORMED)[0][0]
                    if sp > sh and label == "Parking":
                        return "Parking"
                    elif sh > sp and label == "Hospital":
                        return "Hospital"
        except Exception:
            pass

    return label


##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

def validate_signs(detections, image_name):
    '''
    Purpose:
    ---
    Check that detect_signs() returned the expected structure and normalise
    it, so a malformed return is reported here instead of silently scoring
    zero during evaluation.

    Input Arguments:
    ---
    `detections` :  [ object ]     whatever detect_signs() returned
    `image_name` :  [ str ]        name of the image, used in error messages

    Returns:
    ---
    `clean` :       [ list of dict ]   [{"label": str, "area": int}, ...]
    '''
    where = "detect_signs() on {}".format(image_name)

    if not isinstance(detections, list):
        raise TypeError("{} must return a list, got {}".format(where, type(detections).__name__))

    clean = []
    for index, entry in enumerate(detections):
        entry_where = "{}, entry {}".format(where, index)

        if not isinstance(entry, dict):
            raise TypeError("{} must be a dict, got {}".format(entry_where, type(entry).__name__))

        missing = {"label", "area"} - set(entry.keys())
        if missing:
            raise ValueError("{} is missing the key(s): {}".format(entry_where, ", ".join(sorted(missing))))

        label = entry["label"]
        if label not in SIGN_LABELS:
            raise ValueError("{} returned label = '{}', expected one of {}".format(
                entry_where, label, ", ".join(SIGN_LABELS)))

        area = _validate_area(entry["area"], entry_where)
        clean.append({"label": label, "area": area})

    return clean


def validate_obstacle(result, image_name):
    '''
    Purpose:
    ---
    Check that detect_obstacle() returned the expected structure and
    normalise it.

    Input Arguments:
    ---
    `result` :      [ object ]     whatever detect_obstacle() returned
    `image_name` :  [ str ]        name of the image, used in error messages

    Returns:
    ---
    `clean` :       [ dict ]       {"present": bool, "area": int}
    '''
    where = "detect_obstacle() on {}".format(image_name)

    if not isinstance(result, dict):
        raise TypeError("{} must return a dict, got {}".format(where, type(result).__name__))

    missing = {"present", "area"} - set(result.keys())
    if missing:
        raise ValueError("{} is missing the key(s): {}".format(where, ", ".join(sorted(missing))))

    present = result["present"]
    if not isinstance(present, (bool, np.bool_)):
        raise TypeError("{} returned present of type {}, expected a bool".format(
            where, type(present).__name__))
    present = bool(present)

    area = _validate_area(result["area"], where)
    if not present and area != 0:
        raise ValueError("{} returned present=False but area={}, expected 0".format(where, area))

    return {"present": present, "area": area}


def _validate_area(area, where):
    if isinstance(area, bool) or not isinstance(area, (int, float, np.integer, np.floating)):
        raise TypeError("{} returned area of type {}, expected a number".format(
            where, type(area).__name__))
    area = int(round(float(area)))
    if area < 0:
        raise ValueError("{} returned a negative area: {}".format(where, area))
    return area
