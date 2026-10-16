# Collie Kart Portable — P002 (experimental telemetry)

P001 has been verified to load on a real New 3DS running USA Mario Kart 7, draw the bottom-left status line, toggle READY/INACTIVE, show diagnostics, and continue working during a Time Trial.

P002 preserves its approved indicator position, with no autonomous controls, learning, game-memory writes, or file writes. It adds opt-in *read-only* telemetry from an **unverified hypothesis** based on [mk7re/MK7-Emu-Scripts](https://github.com/mk7re/MK7-Emu-Scripts), specifically common.py, Vehicle.py, and LapRankChecker.py. Those scripts were tested in Citra, **not on actual 3DS hardware**.

## What to test

1. **First back up your working P001 .3gx!** P002 is experimental native code and could freeze the game.
2. Download the successful "Collie-Kart-P002-3GX" GitHub Actions artifact, extract the compiled file, rename to CollieKart.3gx, and replace the P001 file in the verified USA folder: SD:/luma/plugins/0004000000030800/CollieKart.3gx. Do not install alongside other MK7 plugins.
3. Launch MK7. Confirm "CK P002 | INACTIVE" appears at exactly the same bottom-left location as P001.
4. Select -> plugin menu -> "Read telemetry snapshot" while on the **main menu**. Note the diagnostic stage; absence of a race is expected.
5. Enter a solo Time Trial (Toad Circuit). Before moving, open "Read telemetry snapshot" again. Record any lap, checkpoint, progress, position and speed values; unknown/unavailable is a useful result.
6. If values appear reasonable, set READY and enable "Toggle live telemetry". When the menu closes, an optional yellow line appears directly above the status label. "OBS?" explicitly means **unverified observations**. Drive a short distance and assess if checkpoint, completion and speed change appropriately.
7. Test returning to MK7 menus and exiting the game. Observe flicker/frame-rate impact and report unexpected crashes. If unstable, disable Plugin Loader, remove P002 and restore P001 from your backup.

## Important limits

- Currently only USA telemetry addresses are attempted; other MK7 regional copies retain the display but their telemetry reads are disabled.
- The static pointer hypothesis comes from the USA rev1/rev2 Citra scripts and **may not fit MK7 2.4.0 running on this console**.
- The probe uses Process::ReadXX APIs and pointer/range validation. These reduce risk but cannot guarantee stability.
- Incomplete or nonsensical results should be reported, **not interpreted as real telemetry**.
- P002 deliberately does **not** claim to detect an active race yet, does not perform AI training, and does not write any files to SD. We'll add those only after physical validation.
- Keep the plugin on a separate P002 development branch; do not merge until tested.
