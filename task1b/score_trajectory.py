"""
score_trajectory.py
Scores a trajectory.csv against the Task 1B rubric:
  - Convergence  (18 pts): |y - target_y| < 0.02 m for the last 3 s of each segment
  - Settling time ( 6 pts): first time |y - target_y| < 0.02 m and stays there
  - Ride quality  ( 6 pts): max overshoot past target_y
"""

import csv
import sys
import os

BAND       = 0.02   # m
SETTLE_MAX = 45.0   # s - zero marks
SETTLE_MIN = 25.0   # s - full marks
OVERSHOOT_FULL = 0.02  # m - full marks
OVERSHOOT_NONE = 0.10  # m - zero marks

def load(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows

def score_segment(rows, t_start, t_end, target_y, label):
    seg = [r for r in rows if t_start <= r["t"] < t_end]
    if not seg:
        print(f"  {label}: no data")
        return 0, 0, 0

    # Convergence: last 3 s all within band
    last3 = [r for r in seg if r["t"] >= t_end - 3.0]
    converged = all(abs(r["y"] - target_y) < BAND for r in last3)
    conv_pts = 18 if converged else 0

    # Settling time: first entry + stays in band
    settle_t = None
    for i, r in enumerate(seg):
        if abs(r["y"] - target_y) < BAND:
            if all(abs(s["y"] - target_y) < BAND for s in seg[i:]):
                settle_t = r["t"] - t_start
                break

    if settle_t is None:
        settle_pts = 0
        settle_str = "never settled"
    elif settle_t <= SETTLE_MIN:
        settle_pts = 6
        settle_str = f"{settle_t:.1f} s (full marks)"
    elif settle_t >= SETTLE_MAX:
        settle_pts = 0
        settle_str = f"{settle_t:.1f} s (zero marks)"
    else:
        frac = (SETTLE_MAX - settle_t) / (SETTLE_MAX - SETTLE_MIN)
        settle_pts = round(6 * frac, 2)
        settle_str = f"{settle_t:.1f} s ({settle_pts}/6)"

    # Ride quality: overshoot = excursion PAST target_y (not initial approach).
    # Find first time vehicle enters the 0.02 m band, then measure max excursion
    # beyond target_y after that point.
    entry_idx = None
    for i, r in enumerate(seg):
        if abs(r["y"] - target_y) < BAND:
            entry_idx = i
            break

    if entry_idx is None:
        # Never reached target - use max excursion from starting side
        overshoot = 0.0
        overshoot_str = "never reached target"
    else:
        post = seg[entry_idx:]
        # overshoot: how far the vehicle went past target_y on the far side
        # approaching from left (y increasing): overshoot = max(y - target_y - BAND, 0)
        # approaching from right (y decreasing): overshoot = max(target_y - y - BAND, 0)
        # Simpler: max |y - target_y| minus BAND after entry, clamped to 0
        overshoot = max(max(abs(r["y"] - target_y) - BAND, 0.0) for r in post)
        overshoot_str = f"{overshoot*100:.1f} cm past ±{BAND*100:.0f}cm band"

    if overshoot <= 0.0:
        rq_pts = 6
        overshoot_str = overshoot_str if entry_idx is None else f"0.0 cm (within band)"
    elif overshoot >= (OVERSHOOT_NONE - BAND):
        rq_pts = 0
    else:
        frac = ((OVERSHOOT_NONE - BAND) - overshoot) / (OVERSHOOT_NONE - BAND - 0.0)
        rq_pts = round(6 * frac, 2)

    print(f"  {label}  target_y={target_y:.2f} m  window=[{t_start:.0f}s,{t_end:.0f}s]")
    print(f"    Convergence : {'YES' if converged else 'NO '}  {conv_pts}/18")
    print(f"    Settling    : {settle_str}  -> {settle_pts}/6")
    print(f"    Ride quality: {overshoot_str}  -> {rq_pts}/6")
    return conv_pts, settle_pts, rq_pts


def score(path, segments=None):
    print(f"\n{'='*55}")
    print(f"  Scoring: {os.path.basename(path)}")
    print(f"{'='*55}")
    rows = load(path)

    if segments is None:
        segments = [
            (15.0, 60.0, 0.2, "Seg1 L->R"),
            (60.0, 95.0, 0.0, "Seg2 R->L"),
            (95.0, 120.0, 0.2, "Seg3 L->R"),
        ]

    total_conv = total_settle = total_rq = 0
    n = len(segments)
    for t0, t1, ty, lbl in segments:
        c, s, r = score_segment(rows, t0, t1, ty, lbl)
        total_conv   += c
        total_settle += s
        total_rq     += r
        print()

    print(f"  --- Score averaged over {n} lane change(s) ---")
    print(f"  Convergence  : {total_conv/n:.1f} / 18")
    print(f"  Settling     : {total_settle/n:.2f} / 6")
    print(f"  Ride quality : {total_rq/n:.2f} / 6")
    total = total_conv/n + total_settle/n + total_rq/n
    print(f"  TOTAL        : {total:.1f} / 30")
    print()


if __name__ == "__main__":
    files = sys.argv[1:] if len(sys.argv) > 1 else ["trajectory.csv"]
    for f in files:
        if os.path.exists(f):
            score(f)
        else:
            print(f"File not found: {f}")
