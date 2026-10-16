# P006.1 — MK7 Native Input Pipeline Examination

**This is a separate, opt-in READ-ONLY investigation build.** P006's unverified HID pulse menu option is inherited but **DO NOT USE IT** during this examination. The only new capability is sampling MK7's input data at ~10 Hz; the tested P006 branch and P005 fallback are unchanged.

## Purpose

P006's 80ms experimental HID pulse made three injection calls, while 43 follow-up reads reported no acceleration and no movement. But manual physical A produced seven out of seven matches at VehicleControl::m_controls bit0. To locate the missing input, this build samples in parallel:

- CTRPF's physical A state
- MK7 `KDPlayerPad::m_data_on_frame.m_buttons` (low bit of the u32 at `KDPad + 0x1C`)
- MK7 raw physical-button snapshot at `KDPad + 0x2C`
- MK7 active add-in at `KDPad + 0x18`, type at `+0x3C`, pause at `+0x3D`
- `VehicleControl::m_controls` bit0 and `m_stick_x` (Vehicle +0xFC / +0x100)

All the new accesses are **reads only** based on [mk7re/MK7-Memory](https://github.com/mk7re/MK7-Memory) and [mk7re/MK7-Emu-Scripts/KDPad.py](https://github.com/mk7re/MK7-Emu-Scripts/blob/master/KDPad.py). These pointers and states are not validated on the user's physical MK7 install yet.

## On-3DS test (NO artificial driving)

1. Back up working P006 `CollieKart.3gx` on iPhone. Download the successful **Collie-Kart-P0061-3GX** artifact and replace `SD:/luma/plugins/0004000000030800/CollieKart.3gx` after powering down the 3DS.
2. Launch MK7, choose Toad Circuit Time Trial, and enable READY.
3. In the CTRPF menu select **Toggle READ-ONLY input trace**, then **Start recording**. DO NOT select **Queue REAL A pulse (EXPERIMENT)**.
4. Close the plugin menu. Resume MK7 if its ordinary pause screen appears. Drive with physical A held about five seconds, release it and coast or stop briefly. Turn left/right with the Circle Pad for a few seconds.
5. Open CTRPF, choose **Input pipeline diagnostics** and take a photo.
6. Choose **Stop & Save CSV**, power off and transfer the next `SD:/CollieKart/session_NNN.csv` to your iPhone, then send it here.

Look for `Physical A polls`, `KDPad A matches`, `Vehicle A matches`, and `All-three matches`. These should rise together when the structures are read correctly, but the actual value is the purpose of this test.

The CSV appends `trace_valid,physical_a,kd_buttons_stick,kd_raw_buttons,kd_addin,kd_type,kd_paused,vehicle_controls,vehicle_steer_x,pad_polls,pad_fails,physical_a_polls,kd_a_matches,vehicle_a_matches,full_chain_matches`.

If we see invalid pointers, freezes, or a slowdown, switch OFF the trace and restore prior P006/P005. **No native input writing will be attempted in P006.1.**

---

# Collie Kart Portable — P006 First Contact

**EXPERIMENTAL / DEVELOPMENT BUILD. BACK UP WORKING P005 BEFORE TESTING.**

P006 inherits P001's bottom-left HUD, P002's live telemetry, P003's manual SD flight recorder, P004's provisional rewards and driving modes, and P005's fused boost-panel detection and brake-linked diagnostics.

P006 adds **independent performance profiling** and a **gated single-use, local A-button injection experiment**. The real input test is unverified on the user's actual New 3DS. **It is not a learned or autonomous driver**, does not inject analog steering, and never writes MK7 process memory.

## Important discovery: CTRPluginFramework's InjectKey

In the framework source, `Controller::InjectKey(Key::A)` temporarily ORs the A-bit into **eight 3DS HID shared-memory buffers**. The API does **not** provide a confirmed release function and documentation says injection may fail depending on game timing. Therefore P006 does **not** provide repeated or sustained auto-driving controls. It only provides a **single 80ms maximum injection burst**, explicitly requested by the user after two dry-run checks. Afterward it stops injecting and relies on the HID subsystem to refresh the shared buffers. **Actual successful input and release MUST be tested on the console.** A freeze or lingering input is possible.

There is no independent hardware watchdog: a game/plugin freeze can prevent the callback from releasing or responding. Physical B/START aborts are checked only while callbacks run. If something malfunctions, restart MK7, turn off 3GX Plugin Loader, and restore P005. **Never assume a software deadman switch can recover a completely frozen plugin.**

## Installation

- Back up your currently working `SD:/luma/plugins/0004000000030800/CollieKart.3gx` (P005) to a safe place on your iPhone.
- After GitHub Actions has compiled successfully, download `Collie-Kart-P006-3GX`. Extract and rename `CTRPluginFramework-BlankTemplate.3gx` to `CollieKart.3gx`.
- Power off your New 3DS and replace the plugin under `SD:/luma/plugins/0004000000030800/` (confirmed USA MK7 title).
- Check that the bottom-left indicator reads `CK P006 | INACTIVE`. The P001 HUD anchor has not moved.

## Test A — performance only (NO injected inputs)

P006 begins entirely disarmed. **Do this first.**

1. Enter Toad Circuit Time Trial, activate READY, then enable **Toggle performance profiler**. Leave recording and live telemetry OFF and drive part of a lap; open **Profiler & control diagnostics** and note callback counts and max gaps.
2. Toggle live telemetry ON and drive the same section. Compare callback counts / gaps and perceived FPS.
3. Repeat with **Start recording**, then **Stop & Save CSV**. P006 appends profiler and control columns to the same incrementing `SD:/CollieKart/session_NNN.csv` format.
4. Send the CSV and say **where** whole-game slowdown occurred (e.g. checkpoint or nearby track landmark) in each mode. The callback cadence is a **proxy** and does not directly measure actual MK7 simulation FPS.

All ordinary P005 features should still function. If anything freezes or acts suspicious, restore P005 and skip all input experiments.

## Test B — SAFETY dry run (NO injected inputs)

1. Stay on **Toad Circuit Time Trial**, on level ground, stopped, with READY ON and candidate telemetry valid.
2. Choose **Queue DRY RUN (no input)**, close the plugin menu, and release A/B/START. **P006 now waits for the menu to close, gives the menu's A press 400ms to clear, then requires 150ms of neutral controls before starting the five-second countdown.** The plugin should finish without moving the kart. It displays `RACCOON IN 5s`, etc., as the countdown proceeds. A/START are allowed early to dismiss MK7's own pause menu; release them well before the countdown reaches zero. The final 800ms is a neutral-input guard, and at least 300ms of neutral input is required at the deadline. This unlocks the first dry-run criterion.
3. Queue **ANOTHER** dry run, close the menu, resume MK7 if its own pause screen appears, and press **physical B** during the five-second countdown to test the abort. The countdown now permits A/START early for unpausing; keep B reserved for emergency abort. Check **Profiler & control diagnostics**. It should report `ABORTED` with `B/START physical override`, and `Physical abort verified: YES`.
4. The real experiment stays locked until **both** a successful dry run AND a physical-abort dry run are observed within this game session. The lock resets when the game/plugin restarts.

**Warning:** Mario Kart uses B for brake/reverse; press it only when stationary in the dry-run test. If the menu key state does not clear within 3.5 seconds, P006 now aborts safely with `A/B/START remained held after menu` rather than attempting artificial input. CTRPF menu opening also cancels a queued or active test.

## Test C — optional single real A-key experiment

**Only after Tests A and B pass and the 3DS behaves normally. This test is fully opt-in.**

1. Use Time Trial, stationary and grounded; keep the physical A button released.
2. Choose `Queue REAL A pulse (EXPERIMENT)` from CTRPF, close the menu, and release the buttons to satisfy the new release interlock.
3. After the five-second countdown, the plugin requests a **single short HID A-key burst** (at most 80 ms), then ceases all injections. It also passively observes MK7's candidate acceleration-control bit for about 1.3 seconds after the burst. You can press physical B or START during countdown/pulse to abort if callbacks continue.
4. Check whether the kart begins accelerating and, most importantly, **whether it stops accelerating after the pulse**. Record the outcome even if no movement happens.
5. Use `Profiler & control diagnostics` to inspect `HID injection calls`, `Control phase`, and `MK7 control bits`. If it does not release naturally, stop the game and disable Plugin Loader; do not repeat.
6. Even if it works, **P006 is not approved for longer driving or steering**. Hardware release, timing, performance, and independent recovery remain open issues.

The real experiment can be queued again only by explicitly selecting it anew. **If the pulse appears to do nothing, don't increase duration yet:** inspect the new `pulse_control_read_count`, `pulse_accel_bit_seen`, `pulse_speed_change`, and `pad_pause_candidate` diagnostics to distinguish a paused game or missed HID acceptance from a silent injector. There is no self-arming or automatic repetition, and no learning algorithm.

## New logging fields

These columns are appended after the P005 CSV fields:

`callback_gap_ms,callback_work_ms,probe_read_ms,profiler_callbacks,callback_gaps_over_50ms,control_phase,hid_injection_calls,control_aborts,mk7_controls_valid,mk7_controls_raw,callback_work_us,probe_read_us,environment_read_us,pulse_control_read_count,pulse_accel_bit_seen,pulse_speed_change,manual_a_control_matches,pad_pause_known,pad_pause_candidate`

- `callback_gap_ms` — gap between plugin bottom-screen OSD callbacks in milliseconds when profiler is enabled and menu closed. **Not guaranteed game FPS.**
- `callback_work_ms` — duration of our OSD callback including HUD drawing, rounded to 1ms resolution.
- `callback_work_us`, `probe_read_us`, `environment_read_us` — higher-resolution measurements (microseconds) from the system tick counter, helpful when reads are shorter than 1 ms.
- `probe_read_ms` — time spent executing a complete MK7 telemetry Probe (not game frame duration).
- `profiler_callbacks`, `callback_gaps_over_50ms` — cumulative diagnostic counts since the profiler toggle.
- `control_phase` — DISARMED, WAIT_CLOSE, COUNTDOWN, PULSE, FINISHED, ABORTED.
- `hid_injection_calls` — number of explicit `Controller::InjectKey(A)` calls, **not proof of MK7 accepting the input**.
- `control_aborts` — number of software aborts.
- `mk7_controls_valid`, `mk7_controls_raw` — read-only candidate `VehicleControl::m_controls` at vehicle + `0xFC`, for cross-checking injection.
- `pulse_control_read_count` — read-only control-bit samples taken during/after the A burst; `pulse_accel_bit_seen` records whether bit0 ever became active; `pulse_speed_change` reports if speed exceeded the test's threshold. None independently guarantees the injector caused movement.
- `manual_a_control_matches` — how often physical A and MK7 control bit0 were observed together during normal driving; a calibration aid.
- `pad_pause_known`, `pad_pause_candidate` — MK7 `KDPad.m_is_paused` research candidate. This is logged as diagnostic evidence only; it is NOT a verified game-pause detector or safety interlock. Based on [mk7re/MK7-Memory](https://github.com/mk7re/MK7-Memory/blob/master/template/Kart/Vehicle/VehicleControl.hpp); hardware behavior unknown.

The existing P005 reward weights and all original CSV columns are unchanged.

## Hard limits and caveats

- Real injection only after a dry run + physical B/START abort has passed.
- User must be READY and on MK7 USA, with valid stationary grounded telemetry.
- 400ms post-menu grace and 150ms stable-neutral interlock, then five-second countdown; 80ms **maximum request duration**; callbacks with a gap over 250ms abort. Holding physical A during countdown also aborts.
- B or START triggers abort if detected; reopening CTRPF menu also aborts; READY OFF aborts.
- The plugin stops requesting injections after the pulse. **It cannot currently guarantee clearing the shared HID buffers or overriding a frozen callback.**
- The first pulse intentionally has very short duration, and therefore **may not be noticed by the game**; that is a diagnostic result, not a failure to be hidden.
- Pulse event logs require recording to be enabled separately; the profiler and on-screen diagnostic counters operate independently of recording.
- No new control pathways are active by default. No MK7 game-memory writes, no automated steering, no model training.

P006 remains a **draft** until physical profiling and input-release tests are complete.
