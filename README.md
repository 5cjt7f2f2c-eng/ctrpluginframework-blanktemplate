# Collie Kart Portable — P003 Flight Recorder

**Development build. Back up your known-working P002 .3gx before testing.**

P001 passed its bottom-screen indicator, toggle, diagnostics, and Time Trial tests on real New 3DS hardware. P002 also showed live lap/checkpoint/progress/speed readings that the tester reported changed appropriately during Time Trial, with no stuttering or other unusual behavior.

P003 keeps the same approved indicator position, game-specific checks, and telemetry. It adds a **manual, opt-in flight recorder**. It never takes control of the kart or writes to MK7 game memory.

## How recording works

1. Set **READY** in the plugin menu.
2. In Mario Kart 7 **Time Trial**, select **Start recording** from the CTRPF menu.
3. Close the menu and drive. The plugin buffers readings in RAM around **twice per second** while you play. Disk writes do not happen in the racing loop.
4. When finished, reopen the plugin menu and choose **Stop & Save CSV**.
5. The plugin creates **SD:/CollieKart/session_001.csv**, then session_002.csv, and so on. It will not intentionally overwrite a prior session.
6. Remove the microSD after properly powering off the console, or retrieve the CSV with FTPD on a local Wi-Fi network. Open the CSV in your iPhone's Files app / spreadsheet viewer or share it with ChatGPT for analysis.

**Save is manual.** Turning off READY stops recording but retains the buffered session. If the console crashes or powers off before **Stop & Save CSV**, unsaved samples are lost.

### Recording limits

- Buffer capacity: **1,800 samples** (about **15 minutes** at 2 Hz).
- When full, recording automatically stops; **BUFFER FULL** appears. Choose Stop & Save CSV to retain the collected session.
- Invalid readings are preserved as rows with `valid=0` and a failure stage, rather than silently masquerading as true telemetry.
- Timestamps are milliseconds since recording started, including time spent in the plugin menu. They are not guaranteed to correspond exactly to in-game elapsed time.
- CSV columns: `elapsed_ms,valid,lap,checkpoint,completion,max_completion,pos_x,pos_y,pos_z,speed_valid,speed_game_units,probe_stage`.
- Speed uses **internal game units**, not automatically km/h or mph.
- Only the **USA** MK7 memory layout is currently supported. The program cannot yet automatically identify a genuine Time Trial; only start recording while inside one.
- Stop & Save performs file operations while the CTRPF menu is open. If saving fails, an incomplete file may remain and samples are kept in RAM for a retry, provided the plugin remains running.

## Installation

1. Download the successful **Collie-Kart-P003-3GX** artifact and unzip it on your iPhone.
2. **Back up P002's working CollieKart.3gx** somewhere safe, preferably outside the SD card.
3. Rename the new `CTRPluginFramework-BlankTemplate.3gx` to `CollieKart.3gx`.
4. Shut down MK7 and power off the 3DS. Replace the plugin in:
   `SD:/luma/plugins/0004000000030800/CollieKart.3gx` (USA MK7, Title ID previously confirmed by FBI).
5. Launch MK7 with the 3GX Plugin Loader enabled. The plugin should show `CK P003 | INACTIVE` at the familiar bottom-left position.
6. First verify the P001/P002 toggles and telemetry. Record a **short 30–60 second drive** before attempting longer logs.
7. After Stop & Save, confirm the CSV is actually present and contains plausible changing values. No successful save has been verified on physical hardware yet.

## Recovery

In case of crash, unusual FPS behavior, or save errors: disable Plugin Loader and restore your **P002** plugin backup; then report what happened. Do not merge the development branch until actual P003 save/reload tests pass.

### Source notes

Telemetry was adapted from [mk7re/MK7-Emu-Scripts](https://github.com/mk7re/MK7-Emu-Scripts), whose published version explicitly says it was tested in Citra, not physical 3DS hardware. The user has confirmed plausible responding values from P002 on their actual console, but edge cases and calibration remain unverified.

CTRPluginFramework's `Directory` and `File` SD-card APIs are used to keep file operations outside the game-rendering loop. The only filesystem writes are under `/CollieKart` when **Stop & Save CSV** is explicitly selected.
