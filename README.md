# Collie Kart P007b.1 — AI startup scheduler fix

**Experimental and untested on 3DS hardware.** Built after user hardware test of P007b showed `Trained YES, v2, 88 examples, 0 attempts, 0 synthetic frames, current IDLE`.

## Definite defect corrected
P007b invoked `AiStep()` only within a legacy telemetry condition that did **not** include `AI_COUNTDOWN` or `AI_RUNNING`. With optional telemetry, recorders and traces off, AI countdown never progressed. P007b.1 makes AI state itself activate the OSD scheduling block.

## Added diagnostics
- On-screen `AI READY IN 5s` countdown and `AI DRIVING Ns` while active.
- Report shows queued / started / denied counts, precise last startup step, most recent stop reason, queue denial category, neutral button/stick reset count.
- The countdown cannot wait indefinitely for neutral controls; after 12 seconds it stops with a named reason.
- Final startup checks distinguish no player observation, failed speed read, bad pad pointer, pause flag read failure, MK7 still paused, and cart moving during arming.
- Zero attempts remains expected if a test aborts **before** AI has issued a decision; unlike P007b, the reason is now visible.

## Hardware test protocol: only begin supervised AI attempts after the original safety preflight
1. Back up P006.4 and P007a.1, power off New 3DS, copy `CollieKart.3gx` to `SD:/luma/plugins/0004000000030800/`.
2. MK7 USA Rev2 **offline Toad Circuit Time Trial**, READY on. Load demonstration CSV, TRAIN on console.
3. Install PASS-THROUGH hook, verify 30+ local-player input callbacks, 10+ physical A input frames, **0 read/write errors**, normal manual controls.
4. Complete 5s dry run with no injection. Repeat and physically B-abort. Confirm both flags YES/YES.
5. Select `START supervised AI attempt` **while stopped near the recorded starting location**. Close CTRPF and unpause the game early in the visible 5s countdown. Release A/B/START, center Circle Pad before countdown ends.
6. For first hardware AI run, keep the default **12-second limit**. Do not leave the console unattended. Any physical input or menu reopening should cancel synthetic control. If frozen, exit MK7 and disable loader.
7. Open `AI attempt reports / startup diagnostics`. Send photos of the whole report; especially queued / started / denied, phase, last startup step, last stop reason, native synthetic frames.
8. If a run started, save the full per-attempt CSV using `SAVE ALL`; up to 6 attempts are kept in RAM and preserved separately before clearing. **Do not assume learning has succeeded based only on synthetic frame count.**

## Limits
The new scheduler and report compiled in CI but have not been verified on hardware. The model remains a small on-device supervised regression, not a proven self-driving agent. No new control behavior was added other than running the previously implemented controller in the appropriate state; the native actuator still uses a 180ms command expiry. P006.4 is the hardware-verified fallback.
