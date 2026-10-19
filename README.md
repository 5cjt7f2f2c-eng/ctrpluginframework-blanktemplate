# Collie Kart Portable P006.3a — pass-through safety preflight

**HARDWARE STATUS: NOT YET TESTED. ACTUATOR HARD-DISABLED.**

This branch is deliberately narrower than the original P006.3. It retains all prior telemetry and safety rehearsals, but the native hook invokes the game's original `KDPadInputer::calcInput` and then performs **read-only, checked** inspection of the local player's input. The three-frame A actuator is inaccessible **and the native hook contains no memory writes**. This is a first-stage preflight for New Nintendo 3DS / Mario Kart 7 USA revision 2.

## What changed in P006.3a
- Source is based on `collie-kart-p0063-frame-hook-first-contact`; stable P006.2 remains unchanged.
- The hook is **PASS-THROUGH ONLY**, never substitutes a button. The old P006.3 experimental pulse code is not invoked by the menu, and the hook contains no button writes.
- Pad and frame observations now use `Process::Read32/Read8/Read16` checked reads rather than unchecked raw pointer dereferences. These checks reduce risk but cannot absolutely prevent a memory race.
- Callback-visible enable, pad, and count state uses C++ atomics.
- Turning READY off disarms the experiment and **attempts to disable the installed hook**. Failure is explicitly reported. Abnormal behavior still requires quitting MK7 and disabling Luma3DS 3GX Plugin Loader.
- Signature scanning validates mapped text pages (including boundary crossings), then reads mapped instructions directly for speed. Zero/multiple matches or unmapped pages prevent installation.
- Diagnostics report native callback counts and rejected reads.
- Cloud build targets `Collie-Kart-P0063a-3GX`; compilation **does not** establish real hardware compatibility.

## Conservative first hardware sequence (offline Time Trial ONLY)
1. Back up the tested P006.2 plugin on your iPhone/SD card **before** changing the installed 3GX. Do not delete the backup.
2. With the console fully powered off, install the compiled P006.3a `.3gx` at `SD:/luma/plugins/0004000000030800/CollieKart.3gx`.
3. Boot MK7, enter **offline Toad Circuit Time Trial**, enable READY, and open the CTRPF menu.
4. Choose **1: Install PASS-THROUGH hook**. If the game refuses with 0 or multiple signatures, STOP; do not retry with guessed addresses.
5. Only if installation succeeds, resume play and manually hold A for a few seconds, steer, and press B. Ensure everything behaves exactly as before. Do **not** attempt any synthetic acceleration.
6. Read **2: Native hook diagnostics**; record player callback frames, physical-A frames and read failures. Unexpected behavior, slowdown or anomalous reads: STOP and revert to P006.2.
7. Switch READY off. The plugin should unhook and return to the normal native input path. If removal fails, stop and restart with plugin loader disabled.
8. Send diagnostic screenshots to review. The next milestone, P006.3b, would separately reintroduce a synchronised, guarded actuator only after pass-through is confirmed.

## Important limitations
- The Title ID check establishes MK7 **USA**, not an exact revision number; the external calcInput signature has not been verified on this exact console. A unique signature does not prove hook ABI safety.
- **No in-game software can rescue a fully frozen title.** Power off, disable the Luma3DS plugin loader, and restore P006.2 if needed.
- Install only for offline Time Trial research, not online races.
- P006.2's successful test does not constitute P006.3a hardware validation.

The original P006.3 research branch is retained for future reference and is **not** the recommended first hardware test build.
