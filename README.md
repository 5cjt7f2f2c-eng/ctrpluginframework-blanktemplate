# Collie Kart P007b — First on-device learning + six retained attempts

**P007b HAS NOT BEEN TESTED ON NEW 3DS HARDWARE.** Successful compilation alone does not verify autonomous driving, controller shutdown, learning, or performance. Preserve the tested P006.4 and read-only P007a.1 binaries. No online races.

## What is implemented
- Load the latest `/CollieKart/demo_###.csv` saved by P007a.1, or make a fresh manual demonstration.
- Fit a four-weight regression on-device: predict recorded steering from the angle to a future sampled path point and observed speed. This is a *small supervised learner*, not a large neural network.
- A hybrid reference-path matcher searches only forward waypoint segments, compares live position and movement direction against recorded path, predicts controls and corrects heading deviations. Model decisions use live position, not exact elapsed-time button replay.
- Native input callback carries bounded commands, renewed every `~65ms`. Each action expires independently after 180ms if UI processing stalls. Original physical controller executes first. A/B/START or Circle Pad movement immediately revokes artificial authority at the native hook.
- Per-action controller read/write faults, menu opening, READY-off, invalid inputs, stale position, route error above 95 coordinate units, time horizon, and physical override stop the AI.
- Six detailed AI attempts stored together in RAM. Each has independent decision trace, route error, inputs, model parameters, score, delivery counts and stopping reason.
- Session report compares all attempts, and `SAVE ALL ATTEMPTS` writes separate uniquely numbered `ai_attempt_###.csv` files to SD. The saved batch can then be cleared while keeping the loaded demo and best learned gain.
- After each valid attempt the system evaluates completion and mean route error, keeps the best performing correction parameter set and proposes a nearby variation for the next attempt. This is a constrained empirical parameter search, **not proof of improving driving** until hardware outcomes are compared.
- Record and analysis remain on the physical 3DS. No SD writes while AI is driving.

## Critical hardware testing gate (start here — do not immediately run an AI attempt)
1. **Back up P006.4 and P007a.1**, power off the New 3DS, install `CollieKart.3gx` under `SD:/luma/plugins/0004000000030800/` and start **offline Toad Circuit Time Trial only** on MK7 USA Rev2.
2. Enable READY. Before input injection, verify **6 LOAD latest demonstration CSV**, then **7 TRAIN model on 3DS**. If model training reports insufficient examples, DO NOT force autonomous operation.
3. Install **8 native PASS-THROUGH hook**. Drive manually on a safe part of the track. Confirm manual inputs and menu **9 diagnostics**: 30+ native callbacks, 10+ physical A frames, zero faults. Report unexpected slowdown or crashes.
4. Perform **10 five-second SAFETY dry run**, resume MK7 early and let it FINISH. Repeat and press physical B to ABORT. Check both dry-run/B-abort flags YES/YES.
5. **STOP, send photos** of model and native calibration diagnostics. This separates hardware read-only validation from first sustained experimental write. The first AI autonomous attempt is not yet hardware validated. Only after calibration review, reset stationary near the original demonstration's start (within 65 game-coordinate units) and continue under supervision.
6. Choose **11 START supervised AI attempt**, close CTRPF, unpause game early in five-second countdown, release A/B/START and center Circle Pad for final 350ms. It will fail open on timeout/override or re-opened menu. **Do not leave the console unattended.** If anything behaves unusually, close MK7 and disable 3GX loader.
7. After attempt finishes, open **14 AI attempt reports**. Each trial is retained. Press **15 SAVE ALL attempt CSV files** after playing (never while driving), copy them from `SD:/CollieKart/` to analyze. Run up to 6 attempts before having to save and clear the batch. Use **16 CLEAR saved attempt batch** to continue with same trained policy in another session.
8. **12 CHANGE max AI duration** cycles bounded horizons 12 / 30 / 75 / 120 seconds; start with 12 seconds only.

## Diagnostic columns
Each saved attempt has a metadata row with attempt number, stop reason, steering correction gain, score, percentage of demo route completed, mean and maximum deviation, native controller delivery totals, model version and weights. The detailed timeline includes timestamp, observed position/speed, matched waypoint, target point, route error, heading error, policy-predicted steering, actual commanded steering/buttons/stick, and cumulative successfully injected native callback frames.

## Honest limitations
- **No reliable true vehicle-facing yaw** has yet been calibrated. Travel heading comes from position deltas (inaccurate when stationary/reversing/drifting).
- Only USA Title ID and candidate telemetry are verified by software; version and offline mode are **not actively enforced**. User must restrict to MK7 USA Rev2 offline Time Trial.
- Model may fail to generalize outside demonstration. Output of an on-device training routine does not imply route replication succeeds.
- If the hook freezes completely, no plugin code can guarantee escape. Restore P006.4 if any issue.
- An attempt's higher score is evidence within these metrics only; it does not prove the policy learnt a robust driving skill.
