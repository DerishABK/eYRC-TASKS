# Task 3A Blockers and Resolution Plan

## Current status

**Parking fix validated on Hard.** The fresh run completed normally with `PARKING->DONE` at frame 1388. The final marker estimate was `forward=0.23 m`, `lateral=0.02 m`, `heading=-0.04 rad`; evaluator returned code 0 after 276 seconds of simulation. The run generated a CSV matching the current script. The existing ZIP was not refreshed and must not be submitted.

## What the runs established

- Hospital stop, both Speed 40 signs, and all three obstacle passes completed in the observed runs.
- The early crossing handoff is necessary. A prior controller switched to straightening at 0.95 m and skipped the crossing phase.
- With the Task 2C crossing angle and 0.38 m straightening handoff, one run reached `forward=0.38 m`, `lateral=0.02 m`, `heading=-0.05 rad`; the user saw the vehicle inside the bay.
- A different randomized approach lost the marker earlier, around `forward=0.48 m`, `lateral=0.47 m`, heading `-0.69 rad`. The 0.24 rad/s, 30-frame fallback braked before it crossed the bay edge; the user screenshot showed the vehicle outside.
- The yellow-outline detector has not detected the bay in the recorded runs. It must not be treated as a dependable primary signal yet.
- The latest run exposed a confirmation bug: marker-loss handling reset the stop counter each frame, so the car stopped but never reached `DONE`. Removing that reset let the same early-stop gate confirm parking after three frames.
- The accepted parking gate stopped at `forward=0.23 m`, `lateral=0.02 m`, `heading=-0.04 rad` in the bay patch. Hospital stop, both Speed 40 signs, and three obstacle passes were also observed in this run.
- The run took 276 seconds, 96 seconds beyond the three-minute time threshold. Parking works; the run still has a time deduction to address before maximizing score.

## Current candidate changes

1. Match Task 2C's staged turn: right bay crossing at -39.5 degrees until 0.38 m or the lateral handoff; moderate offset uses -24 degrees and 0.42 m.
2. Begin the stop 6 cm before Task 2C's bay midpoint (0.25 m from the marker line) to account for actuator coast. Require lateral error at most 5 cm and heading error at most 15 degrees, matching the submission's small parking patch.
3. If ArUco disappears, estimate only the motion caused by commanded wheel speed and continue the staged maneuver toward the early stop gate. Use the camera bay-outline estimate if it becomes available. Never read simulator poses.
4. Bound marker-loss continuation to 450 frames; brake if camera or estimate cannot safely reach the stop gate.

## Remaining work

- Keep this tested source unchanged while reviewing the generated CSV and exact evaluator score.
- Improve the 276-second run time without exceeding the 3.0/1.2 rad/s limits or regressing the validated parking stop.
- Rebuild the ZIP only after the final source and matching evaluator CSV are selected; the existing ZIP predates this run.

## Submission gate

- A complete Hard run reaches `DONE` while parked in the scored patch.
- Hospital, speed-limit, obstacle, and road-position criteria pass.
- `task3.py` remains unchanged after the accepted run.
- Final ZIP contains exactly `task3.py` and the evaluator-written `NV_Task3_run.csv`, and passes `unzip -t`.
