# Task 3A Hard-Mode Execution & Resolution Plan

**Status (Updated 12 November 2026):** Critical failure diagnosed and prioritized.
In the latest run, the car came to a dead stop (`speed=0.00`, `obstacle_cap=0.00`) directly in front of the second obstacle because `obstacle_speed_cap` reached 0 while waiting for `lane_change_rearm_frame` cooldown (200 frames). In the earlier run, the car timed out in `PARKING` because it was wandering inside the bay without locking on or finishing.

---

## 1. Anatomy of Failures in `lastrunresult.md`

### Failure A: Deadlock Freeze in front of Obstacle (`speed=0.00`)
- **Observed:** At frame 212, car changes lane for obstacle 1, completes at frame 268, setting `lane_change_rearm_frame = 268 + 200 = 468`.
- At frame 400+, the car approaches Obstacle 2. Because `frame < 468`, `self.lane_change_committed` remains True / rearm is blocked.
- Meanwhile, lines 1159-1162 continuously scale `self.obstacle_speed_cap = min(obstacle_speed_cap, max(0.0, BASE_SPEED * (1.0 - 1.15 * proximity)))`. As the car gets closer, proximity hits 1.0, pulling `obstacle_speed_cap` to `0.00`.
- The car halts dead on the track in `FOLLOW` mode, unable to trigger a lane change or move forward.

### Failure B: Hospital Halt Triggered Too Late / False Detection
- In previous run, `FOLLOW -> HOSPITAL_STOP` was triggered at frame 1781 after 2 speed signs!
- The Hospital sign was in view in the background earlier, but wasn't recognized or the distance condition `sy + sh >= 0.38 * IMG_H` was only met at a strange viewing angle.
- Once stopped, it stayed stopped for too few frames or did not register properly.

### Failure C: Parking Drift & Bay Timeout (420s timeout)
- Car reached parking at frame 2697, but wandered around for 400+ frames (`PARKING` state frames 2700-3100) until the evaluator 420s hard timeout killed it.
- Root cause:
  1. The vehicle was turning with `raw_steer = 1.5 * (parking_target_angle - heading)`, drifting to `cte = -1.01`, losing marker sight (`marker_misses`), and getting stuck in the sign-based fallback `min(0.24, speed_cap)` or missing the bay target alignment completely.
  2. The alignment criteria `lat <= 0.06` and `hdg <= 12 deg` was never satisfied because the approach trajectory from the second lane was poorly conditioned.

---

## 2. Hard-Fix Concrete Action Plan

### Step 1: Fix Obstacle Speed Cap & Lane Change Re-arm (Prevent Deadlock)
1. **Never drop speed cap to 0.0 in `FOLLOW` if adjacent lane is clear:**
   - If `proximity` is high and an obstacle is in our lane, `self.obstacle_speed_cap` should have a floor of `0.6 rad/s` (or immediately force a lane change if not blocked), NEVER 0.0 unless the road is completely blocked in BOTH lanes.
2. **Dynamic Lane Change Triggering:**
   - Instead of an arbitrary 200-frame blind lockout (`lane_change_rearm_frame`), clear `lane_change_committed = False` as soon as the current obstacle is no longer blocking our path or has passed behind the camera horizon, OR if a NEW obstacle is detected within 1.5m in our current lane.
3. **Smooth Lane Transition:**
   - Don't linger: when changing lanes, command a decisive heading towards the target lane center, relock markings, and stabilize cruise speed at 2.0 rad/s (or 1.2 in Speed 40).

### Step 2: Accurate Hospital Pickup Detection & 5s Sim-Time Hold
1. **Sign Identification:** Ensure Hospital sign (blue square with 'H') is cleanly differentiated from Parking ('P') and Speed40.
2. **Pickup Point Alignment:**
   - The Hospital pickup point is beside the Hospital sign. Trigger stop when Hospital sign bounding box reaches the designated stop threshold.
   - Start 5.0 second timer using `sim.getSimulationTime()` only when linear wheel velocity is 0 / stationary confirmed.
   - Hold zero velocity until `sim.getSimulationTime() >= stop_complete_time`.
   - Set `hospital_done = True` and ignore Hospital signs for the rest of the lap.

### Step 3: Speed 40 Zone State Machine
1. Latch: First Speed 40 sign detected -> `speed_zone = True` (speed capped strictly at 1.2 rad/s).
2. Second Speed 40 sign detected -> `speed_zone = False` (speed restored up to 2.5–3.0 rad/s).
3. Ensure no over-speeding even for a single frame (evaluator deducts -1 mark per second over limit).

### Step 4: Overhaul Parking Bay Trajectory & Alignment (Task 2C Port)
1. **Plausible Entry Corridor:**
   - When `hospital_done` and `speed_sign_count >= 2`, activate parking seeker.
   - When ArUco marker is spotted, use pure geometry:
     - Stage 1 (Entry/Crossing): Steer towards bay centerline at `forward > 0.40m`. Target entry angle ~ -25° to -30°.
     - Stage 2 (Straighten & Dock): At `forward <= 0.38m`, aggressively steer to align `heading -> 0.0` and `lateral -> 0.0`.
     - Stage 3 (Stop): At `forward <= 0.19m`, apply brakes. If `forward <= 0.20m` and marker is temporarily lost or vehicle has stopped, declare `DONE`.
2. **Eliminate 420s Wandering:**
   - Cap maximum parking search time: once inside bay (`forward < 0.35m`), vehicle drives straight at 0.3 rad/s until midpoint (0.19m) and stops.

---

## 3. Step-by-Step Execution Checklist

- [ ] **Phase 1: Code Updates in `task3.py`**
  - Implement unfreezable obstacle avoidance & rearm logic.
  - Fix Hospital stop trigger distance and precise 5.0s sim timer.
  - Streamline ArUco parking controller (staged crossing -> straight docking -> stop at 0.19m).
- [ ] **Phase 2: Compilation & Validation Checks**
  - Run `python3 -m py_compile task3.py` to guarantee zero syntax or import errors.
- [ ] **Phase 3: Test on Simulator**
  - Test run with `./Ubuntu_eval` on `Task_3_Hard.ttt`.
  - Monitor terminal logs for continuous movement (no stalls), proper 5s halt, both Speed 40 signs, 3 obstacle passes, and clean parking `DONE`.
- [ ] **Phase 4: Packaging**
  - Ensure `NV_Task3_run.csv` is generated with 0 errors and within 180s.
  - Package into `NV_Task3.zip`.
