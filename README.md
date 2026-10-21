# Collie Kart P008 — Independent Learning v1 (EXPERIMENTAL)

**Target:** MK7 USA Rev 2 on a physical New Nintendo 3DS, offline Time Trial only. **Compiled via GitHub Actions; autonomous hardware testing is not yet performed.**

This branch preserves the entire P007b.2 route-guided AI and controller safety system and adds a **distinct route-free reinforcement-learning controller**.

## What works in source

- Driving mode switch: MANUAL -> GUIDED -> TRAIN -> EVALUATE -> MANUAL (default at boot GUIDED).
- GUIDED runs the prior P007b.2 model and recorded demo. TRAIN and EVALUATE do **not** consult human demonstration waypoints or Nintendo CPU waypoints.
- Nine bounded discrete action choices with existing CTRPF native `KDPadInputer` passthrough hook: A straight/gentle L/R/hard L/R, coast, B straight/L/R. The plugin makes no unverified native collision calls.
- Online semi-gradient SARSA(0) action-value updates in TRAIN; no exploration or model updates in EVALUATE. Hashed, bounded state features; world coordinates excluded from the policy to avoid automatic coordinate lookup.
- Current live observation candidates: speed, speed vector (for sideslip), kart-facing vector at Vehicle+0xF44, yaw strength, current road collision class, wall-contact flag, race/lap progress and mode.
- Reward from legitimate max progress and real lap transitions, with time cost, wall-contact penalty, reverse-direction penalty and prolonged-stagnation penalty. Evaluation uses the same reward but leaves policy unchanged.
- Six independent flight-recording attempts with CSV export `/CollieKart/ind_attempt_###.csv`, policy update counters and exploration flags.
- Versioned, checksummed and non-overwriting `/CollieKart/ind_model_###.csv` policy snapshots. Save after stopping and reload newest valid snapshot. Policy resets in RAM only.
- Native physical override, five-second release/countdown, pause rejection, command expiry, input-fault abort, fixed runtime cap and exact observed lap-transition stopping.

## NOT working/proven yet

- **360-degree KCL collision raycasts are NOT implemented.** Twelve-direction radar fields are explicitly invalid; no dummy distance readings are supplied. Collision-manager pointer is read-only and does not give us lookahead by itself.
- New facing, sideslip, surface, wall and collision-manager reads are **research candidates**. The physical 3DS needs to validate their values; invalid facing causes independent driving to refuse/stop.
- Compilation success is **not** verification of memory safety, framerate, controller behavior, sensor fidelity, policy effectiveness, a completed autonomous lap, or learning improvement.
- Without look-ahead, the policy has limited ability to anticipate corners. This release validates the end-to-end *learning mechanism* and behavior, not general driving competency.
- No automatic SD saving while the kart moves; use explicit SAVE model menu after each finished training batch.

## Recommended first hardware test

1. **Back up your working P007b.2 plugin** and the SD card. Copy the new `CollieKart.3gx` to `SD:/luma/plugins/0004000000030800/CollieKart.3gx` while the console is powered off.
2. Enter MK7 USA Rev 2 **offline Toad Circuit Time Trial**. Enable READY. Install native PASS-THROUGH hook. Confirm manual A and steering are unchanged, the native diagnostics show zero read/write faults, and redo dry-run plus physical B-abort preflight for this build.
3. Run **P008: READ-ONLY sensor preflight** while at rest and after manually sampling normal asphalt and grass/wall areas. Check facing VALID (and its direction), sensible speed, road surface and wall flags. Expect `RADAR UNAVAILABLE`. **If facing is invalid or data is implausible, STOP and report; do not attempt AI driving.**
4. P008 mode starts GUIDED. Select **P008: Cycle driver MODE** *once* to TRAIN. Position kart stationary at a normal Toad Circuit start. No demonstration needs to be loaded or recorded.
5. Select **P008: START independent attempt**, close CTRPF, resume MK7, release buttons and center Circle Pad before the five-second neutral check ends. The first independent attempt has a **12-second** maximum; supervise continuously. Physical control aborts artificial commands.
6. After the attempt, open **P008: Independent learning report**. Check startup, actual native frames, policy update count, exploration rate, final progress, reward and failure reason. Save attempt CSVs, and separately **SAVE learned model** to preserve policy state.
7. Attempt another TRAIN run from approximately the same start and compare traces. Then cycle mode to EVALUATE to disable exploration and policy updates, and assess reproducibility. Evidence of better driving requires multiple comparable trials, not merely changed weights.
8. If the game slows, glitches, crashes, or releases controls incorrectly, stop, power off and restore your validated P007b.2 plugin.

## Design philosophy

Train a **learned action policy** from the agent's own sensor observations and consequent reward. Do not hard-code Nintendo CPU racing routes, human sample waypoints, ideal steering instructions, or future checkpoint coordinates into the independent controller. Route progress is a **training grade**, not an instruction.

## Sources

- [MK7-Memory VehicleMove](https://github.com/mk7re/MK7-Memory/blob/master/template/Kart/Vehicle/VehicleMove.hpp)
- [MK7-Memory LapRankChecker](https://github.com/mk7re/MK7-Memory/blob/master/template/RaceSys/LapRankChecker.hpp)
- [MK7 emulator collision/vehicle script](https://github.com/mk7re/MK7-Emu-Scripts/blob/main/Vehicle.py)
- [Native collision manager pointer documented](https://github.com/mk7re/MK7-Memory/blob/master/template/Object/CharacterEngine.hpp)
- [Independent KCL collision query reference, not integrated](https://github.com/PabloMK7/YAMKC_3DS/blob/main/source/KCollisionServer.cpp)
