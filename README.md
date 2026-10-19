# Collie Kart P007b.2 — Endpoint-aware stopping and true route progress

**Compiled source is experimental; real-hardware validation is still required.** Based on successful P007b.1 autonomous attempt #001 using the user's manual `demo_003.csv`. Preserve P006.4 (four control verification), P007a.1 (validated manual recorder) and P007b.1 (first successful AI hardware drive).

## Why this change
- Hardware observed: P007b.1 closely reproduced the path then crept beyond the demonstrated stop and steered sharply left.
- Attempt CSV: came to ~6.2 game units from destination at 9.66s, speed ~0.39, but old index-based completion condition remained at waypoint 17/22. At 9.86s was only 1.5 units away, then steered to stick=1 and missed finish. Old 82% route completion was misreported due to stationary trailing waypoints.
- The AI already learned and issued 718 synthetic input frames without read/write failures. This release addresses **terminal recognition**, not a failed input hook.

## What changed
1. **Arc-length route progress:** matches projection to forward route segments, computes cumulative distance fraction, and displays progress based on traveled path distance, not total number of waypoints.
2. **Stationary endpoint detection:** only when the demonstration's last recorded speed is near zero; gate on >=80% route progress and distance from the actual endpoint. Finishes when within **8 game-coordinate units and speed <=0.6**; stops all synthetic inputs before publishing another action.
3. **Low-speed final approach:** from within 85 units of the endpoint at >=80% progress, automatically reduces target speed as distance shrinks; chooses brake or neutral rather than oscillating acceleration/brake at the stop. Steering recenters inside the final 28 units. Avoids braking below 0.35 speed because B may become reverse.
4. **Overshoot cutoff:** if final approach was activated but the kart moves >110 units away near route end, automatically stops with a specific failure reason instead of chasing a ghost waypoint.
5. **Detailed diagnostics:** last and closest endpoint distances, time of closest approach, final speed, final-approach frame count, plus per-decision route progress, distance-to-end, and desired speed. **Retains all six independent attempt records and SAVE ALL CSVs.**

## Suggested first hardware check
1. Back up validated P007b.1 and P006.4. Power off New Nintendo 3DS and copy `CollieKart.3gx` to `SD:/luma/plugins/0004000000030800/`.
2. Start **MK7 USA Rev2 offline Toad Circuit Time Trial only**. READY ON. Load `demo_003.csv` (if necessary rename/copy as latest `demo_###.csv`), TRAIN model.
3. Install pass-through native hook; manually verify normal controls, 30+ player callbacks, 10+ physical A frames, 0 read/write faults. Complete safety dry-run and a separate B-aborted run.
4. Return stationary to approximately the original start. Use initial **12-second maximum**. Queue AI attempt, close CTRPF, unpause MK7 early in countdown, release controls to neutral. Keep the 3DS supervised.
5. After attempt, view AI reports and check: stop reason `FINISHED ROUTE`, 100% route if destination actually reached, final endpoint distance, last speed, native frames, zero faults. **No claim of physical standstill solely from stopping input**: momentum can still cause a little coast after disengagement.
6. Save individual attempt CSVs through SAVE ALL. Can retain six attempts in RAM before clearing saved batch. Share at least the first CSV and report screenshot for review.
7. If steering behaves unexpectedly, kart runs past endpoint, game slows/freezes, or controller release fails, end session and restore P007b.1 or P006.4.

## Development limits and scoring honesty
- Travel heading is still displacement-based, not calibrated kart-facing yaw.
- No automatic game mode/revision verifier beyond MK7 USA title ID; manual offline Time Trial restriction is mandatory.
- The stop is defined as arriving slowly and **releasing AI control**, not guaranteeing the virtual kart is immobile. The first attempt after this patch must validate coast/stop behavior.
- A higher route-progress percentage from the corrected metric is not proof of model improvement. Compare geometric error, closest endpoint, final speed and human-observed behavior across runs.
- Scores still reflect a simplified path-error/progress objective. Never label this as a guaranteed full-lap capability.
