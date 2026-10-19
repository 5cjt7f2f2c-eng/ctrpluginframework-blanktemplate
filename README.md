# Collie Kart P006.4 — Four-Control Hardware Validation

**UNTESTED on hardware. NOT a learning AI.** Based on the successful P006.3b: three synthetic A frames, brief movement, 12 natural-input release frames, zero read/write failures, controls restored. Preserve that branch and compiled plugin.

## Test matrix
| Menu | Action | Native inputs | Max game-frame callbacks | Deadline |
| --- | --- | --- | ---: | ---: |
| 4 | Throttle | A | 18 | 700 ms |
| 5 | Steer left | A + stick X=2 (center=7) | 36 | 1100 ms |
| 6 | Steer right | A + stick X=12 (center=7) | 36 | 1100 ms |
| 7 | Reverse | B, stick unchanged | 90 | 2300 ms |

At 60 FPS these correspond to approximately 0.30, 0.60, 0.60 and 1.50 seconds. Real timeouts may end them early. Reverse requires a standstill; MK7 B means brake at speed and reverse at/near zero speed. Left/right include limited throttle to make turning visible.

## Safety design
- Opt-in hook installation; always runs MK7's original controller processing before optional game-frame changes.
- Each command is one shot, with hard frame and real-time budgets; each of the four actions can be **attempted once per MK7 session**, and only after safety rehearsals. Attempt bitset in diagnostics indicates attempted, **not passed**.
- Before every action, kart must be stationary and on the ground, input-neutral, with validated telemetry, 5-second countdown, B-abort rehearsal and completed dry-run.
- Physical A/B/START or off-center stick (outside 6..8) cancels a synthetic command. Cancel cannot undo an in-progress memory write, nor guarantee no residual momentum.
- Left/right use `KDPadDataOnFrame::m_stick_x` at +2; MK7's 0..14 x representation was cross-checked against mk7re's `tools/mk7re/dynamic/driver.c`. The controller pointer and exact behavior still require hardware verification. 
- Release is verified by observing 3+ native input callbacks with no A/B and a centered physical stick. The original function refreshes the game-frame fields.
- No online race use. Only test offline Toad Circuit Time Trial. A real game freeze cannot be resolved by a software abort.
- Both source and binary of P006.3b remain preserved. If behavior becomes abnormal, stop, disable the plugin loader and revert to P006.3b.

## Hardware protocol
1. Back up tested P006.3b and fully power off the 3DS before replacing the `.3gx`. Enter MK7 USA revision 2, offline Toad Circuit Time Trial.
2. Toggle READY, menu **1: Install PASS-THROUGH hook**. Drive normally for a few seconds: A, left/right circle pad, B. If any unexpected behavior, stop.
3. Open **2: Native hook diagnostics**. Require hook YES, 1 signature, 30+ player callbacks, 10+ physical-A frames, zero read/write failures.
4. Park stopped and grounded. Run **3: 5s SAFETY dry run**, close CTRPF/unpause early, release all controls and center stick well before 0. Wait until FINISHED.
5. Run a second dry run and physically press B during the countdown. Check diagnostics show **Dry-run/physical B: YES/YES**.
6. **STOP AND SHARE DIAGNOSTICS FIRST.** Once reviewed, select one action (4-7), starting with THROTTLE. Close menu/unpause game early; release buttons and center Circle Pad before the countdown ends. The action runs automatically and stops after its bounded input frames. Do not intervene unless abort is needed.
7. Record which action was selected, whether kart moved in expected direction, whether it stopped receiving artificial input, and whether manual controls still work. Open menu 2 for frame count, attempt mask, release count and faults; send a photo. A successful memory write alone is insufficient to declare movement verified.
8. Repeat a new action only after reviewing the prior result. Each action can be attempted once per process. P006.3b remains the fallback.
9. After all four are independently verified on hardware, consider designing the learning subsystem (observation/action/reward dataset, policy interface, training loop, fallback and safe action envelopes). Do not assume reinforcement learning is possible on the console just because input tests work.

## Experimental limitations
- Exact ROM revision and offline Time Trial mode are not automatically established by title-ID detection. Check manually.
- Steering is analog and may produce a smaller yaw change than expected; don't call it failed solely based on subjective movement. Correlate with telemetry and verify with an independent test if ambiguous.
- P006.4 logs signed speed snapshot at arming and after the observation window; sparse telemetry may miss peaks and negative reverse speed could be represented unexpectedly. Human observation remains essential.
- Success requires real-world outcome **and** native diagnostics. Memory modifications, CI compilation, or an action attempt mask do not prove physical input worked.
