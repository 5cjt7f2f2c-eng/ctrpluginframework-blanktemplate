# Collie Kart P007a.1 — Menu-exit startup fix and capture diagnostics

**Hardware UNTESTED.** P007a's first console session produced 0 samples, 9 consecutive rejected samples, then fault-stop. Actual observed callback gap was 68 ms with no gaps >150ms. This does NOT prove the root cause. The original code incorrectly treated every failed check—including the game's paused input pad—as a memory-read fault.

## Fixes
- **Five-second post-menu grace**, starting when CTRPF closes, with on-screen `DEMO IN 5s` countdown. Resume MK7 with A/START EARLY in this window; drive only after countdown finishes.
- After grace, pause flag remains a WAIT condition, not a read error. Recorder waits for game-pad unpause. Ten seconds of persistent paused state after grace is a stop/fault with label.
- Every individual read and range check reports a named failure source. Diagnostics show first and most recent failure, number of pause-waits, total memory errors and consecutive errors.
- Initial transient reads are tolerated for up to 2.5s after grace; persistent failed reads still end capture safely, preserving RAM.
- No control injection, no AI policy, no unattended driving. Original P006.4 and P007a preserved.

## Strict first hardware test
1. Back up P006.4, power off the New 3DS, replace `CollieKart.3gx` under `SD:/luma/plugins/0004000000030800/`. Start offline MK7 USA Rev2 Time Trial, Toad Circuit.
2. Toggle READY. Select **1 START manual demonstration**. Close CTRPF, **resume MK7 immediately using A or START during the visible 5-second countdown**, and release buttons as needed. The HUD should show `DEMO IN Ns`.
3. Once countdown ends, drive normally for 5–10 seconds. No synthetic inputs are active.
4. Open CTRPF, select **2 STOP & ANALYZE**, then **3 DEMO diagnostics**. Photograph both the top and scrolled bottom of the diagnostic panel.
5. If valid samples > 0, inspect sampling rate, input counts, pause wait count and anomalies. Then save via **4 SAVE demo CSV**, send `SD:/CollieKart/demo_###.csv` if possible.
6. If 0 valid samples, **do not repeat**. Share first/last failing field, total read errors, pause waits and fault state. This identifies the actual failure stage.
7. If the game slows, freezes or behaves strangely, stop and restore P006.4. We have not hardware-validated this new build.

## Remaining limitations
- The five-second countdown exists to allow returning to gameplay; it is not evidence the memory addresses are correct.
- Actual driving actions during the countdown are intentionally not recorded.
- The 20Hz sample target is not guaranteed; only valid samples and effective rate can establish it.
- Travel heading is derived from successive positions, not the kart's true forward yaw.
- No new learning or control capability in this debugging build.
