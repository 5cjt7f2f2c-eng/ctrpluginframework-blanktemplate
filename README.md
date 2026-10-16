# Collie Kart Portable — P005 Sensor Fusion & Passive Evaluation

**New P005 build: NOT YET TESTED on physical New 3DS. Back up working P004 first.**

## Why P005?

On the actual New 3DS, P004 successfully recorded two glider deployments, one natural landing, one midair glider closure, and plausible progression rewards. It showed occasional mild slowdown without stutter or freezing.

One ordinary Toad Circuit boost pad was **missed by the P004 boost counter**:
- P004 recorded a `dash_type=6` (`DASH_PANEL`) at ~27.8 seconds on lap 1.
- `boost_flags` stayed `0x00000000` across the entire recording.
- The old `boost_pad_activations` counter stayed at zero.

The real evidence is in the user's P004 CSV (held privately by the user, not added to this public repository).

## New systems

1. **Dual-source boost detection.** Count a normal boost panel if either `boost_flags bit 0x2` rises **or** `dash_type` enters value `6`. Keep both counters, `dash_panel_edges` and `boost_flag_edges`, so we can audit the combined event count. Deduplicate overlapping detections using a **400 ms** cooldown. This is a working hypothesis and may combine two extremely close consecutive pads.
2. **Lower-overhead sensor scheduling.** Full pointer/position/speed telemetry targets **500 ms** between reads (~2Hz); lightweight mode/boost/glider reads target **100 ms** (~10Hz). Both use elapsed time from `osGetTime()` rather than assuming MK7 renders at 60 FPS. Times are best-effort under frame drops or pauses; the recorder never promises exact sampling.
3. **Raw evidence retained.** P003/P004 columns remain unchanged and in order, including raw status flags, boost flags, dash type, modes, candidate rewards, and all cumulative event counts. Additional diagnostic columns are appended.
4. **Reward coefficients unchanged.** Forward newly attained maximum completion = `100 * delta`; time cost = `-0.1 * seconds` for intervals up to 3s; forward one-lap transition = `+20` if progress delta is plausible. No boost/glider points and **no brake-closure penalty yet**.
5. **Passive brake correlation.** Sample CTRPF's read-only B-button state at the 100 ms environment interval. Log B-held status, times braking while gliding, and midair closure events with a B reading within the past 350 ms. These are only **suspected correlations**, not causal proof. No game input is modified.
6. **Performance counters.** Add environment poll counts, read-failure counts, measured poll-to-poll gap, individual read duration and maximum read duration, plus actual CSV-sample gap. This lets us assess slowdown quantitatively.
7. **Prevent menu miscounts.** Environment snapshots taken while CTRPF's pause/menu is open update the visible values without crediting pad events, mode time, or glider closures. Toggling the live overlay while recording no longer discards a cached state.

## What P005 does NOT do

No driving controls, button injection, machine learning, active-race verification, or automatic saving. `Stop & Save CSV` is still the only writer and only touches the `SD:/CollieKart/` directory. Values and flag meanings are based on [mk7re/MK7-Memory](https://github.com/mk7re/MK7-Memory/blob/master/template/Kart/Vehicle/VehicleMove.hpp) and [mk7re/MK7-Emu-Scripts](https://github.com/mk7re/MK7-Emu-Scripts); new interpretations require field tests.

## Installation

1. Back up the **working P004 CollieKart.3gx** elsewhere (iPhone Files is fine).
2. Download the successful **Collie-Kart-P005-3GX** GitHub Actions artifact; extract the `.3gx` and rename it to `CollieKart.3gx`.
3. Power off New 3DS before removing the microSD; replace `SD:/luma/plugins/0004000000030800/CollieKart.3gx`.
4. Boot MK7 USA, enable the 3GX Plugin Loader if needed; expect `CK P005 | INACTIVE` in the same approved bottom-left location.

## Field-test plan

Record separate **30–90 second solo Time Trials**, each with READY enabled. Choose `Start recording`, drive, then use `Stop & Save CSV`. Copies are numbered `SD:/CollieKart/session_NNN.csv`; no previous files are intentionally overwritten.

1. **Baseline and performance:** Toad Circuit ordinary driving with optional live HUD. Check for slowdown, freezing, video issues. Observe the environment poll count, read failures, measured gap, and max read time. Check that CSV `sample_gap_ms` is closer to 500ms than P004's ~1.26s.
2. **Normal boost pad on lap 1:** cross the same pad. Expect `dash_panel_edges` > 0 and fused `boost_pad_activations` > 0; raw `boost_flag_edges` may remain zero. No double counting from repeated snapshots.
3. **Normal Toad Circuit glider flight:** hit the blue glide pad, glide, land. Confirm glider-pad activation and natural closure.
4. **Controlled early glider close:** activate glide pad, then press B to brake once while airborne. Check `brake_held`, `brake_while_gliding_polls`, and `brake_linked_midair_closures` alongside the older `midair_glider_closures`. Lack of a linked event doesn't prove no braking—100ms polling can miss brief inputs.
5. **Normal ramp without a glide pad:** ensure the ordinary airborne state isn't classified as GLIDING.
6. **Underwater later:** test a course such as Cheep Cheep Lagoon for the `UNDERWATER` state and duration.

Please send the CSV files (and any useful diagnostics screenshots) and mention any unexpected frame-rate changes.

## CSV format additions

P005 *appends* these columns after all P004 fields:
`dash_panel_edges,boost_flag_edges,brake_held,brake_while_gliding_polls,brake_linked_midair_closures,environment_polls,environment_read_failures,environment_poll_gap_ms,environment_read_ms,environment_max_read_ms,sample_gap_ms`.

CSV rewards remain passive experimental scores, **not training results**. As before, 1,800 rows fit in the in-memory buffer. Unknown modes are recorded without inventing certainty. If the plugin crashes, restore P004; CSVs remain on SD.

## Limits

At 10Hz, a very short boost/brake pulse may still be missed. We can increase the polling rate only if hardware measurements justify it. Mode classification, B-button state, and glider closure causality need validation; don't automatically punish glider closure yet. The current running environment is not a visual road-perception system.
