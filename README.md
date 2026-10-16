# Collie Kart Portable P004 — Environment & Reward Probe

**Experimental hardware build. Back up working P003 before installation.**

## Already demonstrated

- P001 UI toggles, menu and bottom-left indicator run in MK7 on New 3DS.
- P002 speed, lap and checkpoint progress responded during Time Trials without noticeable slowdown.
- P003 wrote its first actual SD CSV, transferred back for analysis. Its existing data path is preserved.
- P004 is **new, compiled software**, but its new environmental flags, events and rewards have **not yet been tested** on the console.

## P004 additions

- **Driving modes** (read-only candidates): `GROUND`, `UNDERWATER`, `AIRBORNE`, `GLIDING`, `UNKNOWN`.
- **Independent pad activation counters:** glider pad via `VehicleMove::StatusFlags.glider_pad`; normal boost pad via `VehicleMove::BoostFlags.boost_pad`. These counters detect *rising edges*, not guaranteed physical collisions. The game may keep a boost flag active across adjacent pads.
- **Glider transitions**: open/deployed, close on ground, close while airborne, and `wing_force_close` rising signal. **The program does NOT interpret force-close as a proven manual brake action, and applies no brake/glider penalty yet.** Input causality still needs verification.
- **Mode durations:** separate cumulative durations in milliseconds for grounded, underwater, airborne (without glider) and gliding. Pauses/long gaps are excluded.
- **Surface & boost research:** raw KCL type from `VehicleMove + 0xD14` and dash type from `VehicleMove + 0xF98` are logged for calibration. There is **no off-road punishment yet** because drivable and penalizing terrain categories are not verified.
- **Passive reward calculation:** new max completion delta times 100, elapsed-time cost of -0.1/second, and candidate +20 for a forward lap transition. Progress jumps above 0.10/sample are rejected. No automatic rewards for boost/glider-pad contact, preventing pad farming.
- All state events are counted using lightweight frame-level polling. Larger telemetry snapshots and CSV rows remain at the existing ~1–2/s interval.
- Add `Environment & rewards` menu entry, extend diagnostics and append CSV columns. P001 bottom-left HUD placement is unchanged.
- **No autonomous steering, learning, game-memory edits or automatic SD writes.** Saves happen only when `Stop & Save CSV` is explicitly selected.

## Install

1. **Back up P003's working CollieKart.3gx on your iPhone.**
2. Download and extract a **successful** GitHub Actions artifact named `Collie-Kart-P004-3GX`.
3. Rename compiled `CTRPluginFramework-BlankTemplate.3gx` to `CollieKart.3gx`.
4. Shut down the 3DS and replace `SD:/luma/plugins/0004000000030800/CollieKart.3gx` (the MK7 USA title ID confirmed by FBI). Avoid other MK7 plugins during test.
5. Launch MK7 and ensure `CK P004 | INACTIVE` appears.

## Calibration runs — do not merge P004 before testing

For each test, turn on **READY**, enter a Time Trial, **Start recording**, perform the exercise, then **Stop & Save CSV**. Each run creates a new `SD:/CollieKart/session_NNN.csv`; previous sessions are not intentionally overwritten.

1. **Ordinary driving on Toad Circuit:** verify mode stays GROUND, speed and checkpoint behave as in P003. Observe candidate reward rising with new max completion and decreasing slightly while stopped.
2. **Normal glide:** drive over Toad Circuit's blue glider pad and glide to the natural landing. Expect a glider-pad activation, deployment, gliding duration and landing closure. Inspect raw flags if any counter differs.
3. **Manual early closure:** repeat the glide but deliberately press the brake once while safely airborne. Check force-close signal, midair closure and gliding duration. **A midair closure isn't automatically a manual brake event.** No penalty is applied.
4. **Ordinary jump:** use an ordinary ramp without glider activation. Expect AIRBORNE, not GLIDING. If you see UNKNOWN, use raw air/ground frame counts to debug.
5. **Regular boost pad:** cross an ordinary boost pad and check it increments boost-pad activations without incrementing glider-pad activations.
6. **Underwater:** later test a suitable underwater section (e.g., Cheep Cheep Lagoon) to check the `in_water` flag and time counter.

Send the CSV(s) or photographs of **Environment & rewards** diagnostics; report freezes or frame drops. If unstable, restore P003.

## CSV additions

P004 appends these columns to P003's existing telemetry CSV:
`mode,mode_valid,status_flags,boost_flags,air_frames,ground_frames,surface_code,dash_type,glider_pad_activations,boost_pad_activations,glider_deployments,force_close_signals,midair_glider_closures,landing_glider_closures,ground_ms,underwater_ms,airborne_ms,gliding_ms,reward_progress,reward_time,reward_lap,reward_total,reward_cumulative`.

- Invalid environmental reads become `UNKNOWN` with `mode_valid=0`.
- Raw flags are hex; `4294967295` means an unknown optional surface/dash code.
- Counts accumulate over the recording session and may lag by up to the CSV snapshot interval.
- Reward values are experimental **scores**, not validated training rewards. No learning occurs in P004.
- The original P003 size cap remains **1,800 rows**. When full, Stop & Save; no automatic saving or in-race disk writes.
- The game version's memory layout and these newly decoded bits are based on [mk7re/MK7-Memory](https://github.com/mk7re/MK7-Memory/blob/master/template/Kart/Vehicle/VehicleMove.hpp) and [mk7re/MK7-Emu-Scripts](https://github.com/mk7re/MK7-Emu-Scripts) and must be verified on physical 3DS.

## Recovery

A native plugin can freeze even if it never writes to the game. Disable Plugin Loader, restore your P003 backup, and report any issue. Existing CSVs remain in `SD:/CollieKart/`.
