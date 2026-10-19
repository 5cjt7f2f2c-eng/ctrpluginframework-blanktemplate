# Collie Kart P007a — read-only demonstration recorder

**NOT YET TESTED ON NEW 3DS.** Based on the hardware-successful P006.4 four-control test branch. Original P006.4 is unchanged and remains the rollback build.

## Objective
Teach Collie Kart to OBSERVE a driver's motion and input choices before attempting an imitation controller. P007a does **not** imitate or learn a driving policy, use synthetic controller inputs, or install a native hook. No training yet. It records, summarizes, and exports a reference dataset **entirely on the New Nintendo 3DS**.

## How data is acquired
- Existing USA Rev2 `Probe()` finds/validates player kart and vehicle objects at the original **2Hz** cadence; P007a caches a kart data address from that validated chain.
- A separate **50ms target** loop reads only cached position coordinates, speed and player pad's current game-frame A/B button bits and horizontal analog-stick byte (0..14, neutral=7).
- The recorder observes the actual local-player controller pad through `VehicleControl::m_player_pad` at +0xE0; MK7 menu presses are excluded while CTRPF is open.
- Movement bearing is estimated via `atan2(delta_z, delta_x)` ONLY when the kart has moved far enough between readings. **This is travel heading, not proven vehicle facing yaw** (unreliable at low speeds, while reversing, drifting or sliding).
- Time is derived from timestamps of actual valid samples, not from a presumed FPS. A sampling target is not a guaranteed achieved rate.
- Cache invalidation, implausible position jumps and repeated read failures cause the recorder to **preserve buffered samples and stop**, rather than assume an invalid pointer is usable.
- At most 4,800 manual samples (nominal 4 minutes at 20Hz) in RAM. **No SD writes during gameplay.**
- On stop, analyze in RAM: total approximate path length, endpoint displacement, manual A/B/steering counts, sample gaps, motion-bearing availability and a reference trajectory of up to 960 waypoints, downsampled to about 4Hz. The final endpoint is always retained.
- On explicit SAVE, export to `SD:/CollieKart/demo_001.csv`, then increment to the next available name. File contains each timestamped raw observation and a `reference_waypoint` flag. RAM is retained if saving fails.

## Safe first test (no AI driving)
1. Back up your successful `P006.4` plugin. Power off the 3DS and install `CollieKart.3gx` to `SD:/luma/plugins/0004000000030800/`.
2. Boot **MK7 USA revision 2**, enter an **offline Toad Circuit Time Trial**. Check overlay says `CK P007a | INACTIVE`. Toggle READY.
3. Select `1: START manual demonstration`. Close CTRPF, then **drive a very short 5–10 second straight or gentle curve** using real A/Circle Pad/B.
4. Open the plugin menu and choose `2: STOP & ANALYZE demonstration`.
5. Select `3: DEMO diagnostics`; take a photograph showing sample count, actual effective Hz, heading validity, path distance, any anomalies, and button/stick counts.
6. Select `4: SAVE demo CSV to SD`. Copy `SD:/CollieKart/demo_###.csv` to the chat if possible; send the diagnostics photo regardless.
7. `5: DISCARD demo in RAM` allows a new demonstration, but intentionally clears only the unsaved RAM buffer. Save first if you want to keep it.
8. On issues such as zero samples, low sample frequency, unnatural position discontinuities, crashes or game slowdown, **stop and restore P006.4**. Do not proceed toward automated control yet.

## Important limitations and next gate
- The fast snapshot access pattern and target rate require real-hardware validation. Do not claim the recorder is reliable solely from a successful compile.
- Heading from displacement is an approximation; **true facing orientation is not yet instrumented**. A later feedback controller will require accurate orientation and scene-transition handling.
- Track geometry, collision responses, a trained model and autonomous control do not exist in this branch.
- We will only develop P007b's on-device learning and supervised adaptive reproduction after the recorder proves accurate, fast and stable with a real `demo_###.csv`.
