# P008.1 — checkpoint-guided learning (in development)

This is a **development branch**, not a hardware-validated release. P008 v1 and P007b.2 remain untouched.

## Goal
Teach an independent learned policy to complete MK7 races using **coarse direction toward the next valid checkpoint**, local kart state, and real progress rewards. No replay of demonstrations, CPU path following, or scripted steering.

## Already implemented in this branch
- Reward evaluator distinguishes current progress movement from historical maximum progress. Backward current progress is penalized within a bounded range, while positive reward is only paid for newly achieved progress. Lap terminal reward remains intact.

## Unverified blockers before enabling checkpoint compass
1. Identify the actual race checkpoint array/segment geometry and next-checkpoint index in USA Rev2. Earlier progress telemetry alone does **not** establish a safe address for target geometry.
2. Validate checkpoint boundary endpoints and kart-relative bearing against manual driving on hardware. Show INVALID/UNAVAILABLE on any failed read, invalid coordinate, out-of-range index, or ambiguity; **never synthesize bearings**.
3. Determine how optional checkpoints, lap wrap, rescue, and intersecting course geometry behave.
4. Locate Lakitu's wrong-way HUD/game-state flag separately. Until verified, retain only the independently validated progress/reverse flags; do not label those as Lakitu.
5. Validate KCL collision-query interface before enabling radar rays.

## Planned next patch
- Read-only checkpoint-geometry diagnostic and CSV capture, with no synthetic steering.
- Semi-Markov action persistence with bounded command watchdog refresh, reward aggregation, and appropriate duration discounting; do not extend command TTL without continuing physical override checks.
- Shorter start sequence without deleting runtime safety checks.
- Explicit model version migration or reset when feature definitions change.

## Test gate
Build success is not proof of correct runtime addresses. First hardware test should be stationary/manual with the new compass **read-only**; then compare bearings before and after a checkpoint crossing. Only enable checkpoint-conditioned TRAIN after geometry and checkpoint sequencing are verified.
