# Collie Kart P006.3b — guarded native A first-contact

**BUILD MILESTONE ONLY UNTIL HARDWARE VERIFIED.** The P006.3a pass-through hook already passed New 3DS Mario Kart 7 USA revision 2 testing (522 player frames, 178 physical A frames, no read failures, normal driving and READY-off behavior). **The P006.3b artificial A write is NEW and untested.**

## Safety changes
- Original `KDPadInputer::calcInput` always executes first, preserving physical controls when the one-shot is not armed.
- One-shot synthetic A may affect **at most three game-frame inputs**, with an independent **350 ms** expiry and no steering, HID or other button changes.
- The input hook observes physical A/B/START; real B/START take priority. Holding physical A instead of the synthetic test cancels the substitution.
- A checked `Process::Write16` performs the only in-frame mutation. Original MK7 processing refreshes physical data on subsequent calls. Cancel cannot reverse an already in-flight write.
- `std::atomic` and compare-exchange distinguish OFF, ARMED, DONE, ABORT and WRITING, reducing late-write races.
- Requires 30 native player frames, 10 physical A frames, zero read/write errors, a completed 5-second dry run, and a **separate physical B abort rehearsal**. Cannot attempt artificial input more than **once per MK7 process session**.
- Turning READY off disarms the experiment and attempts to uninstall the hook; errors require quitting the title.
- P006.3a and P006.2 remain untouched as backups.

## Mandatory first hardware session — rehearsals only
1. Save both known-good P006.3a and P006.2 `.3gx` backups on your iPhone/SD card. Fully power off 3DS before replacing the installed plugin with P006.3b.
2. Start **OFFLINE Toad Circuit Time Trial only** (the build cannot independently verify Time Trial/online status). Check `CK P006.3b | INACTIVE`, then select READY.
3. Choose **1: Install PASS-THROUGH hook**; drive manually with A, steering, B. Confirm the kart drives normally as in P006.3a.
4. Open **2: Native hook diagnostics**. Require Hook installed YES, exactly 1 signature match, at least 30 player input frames and 10 physical-A frames, and zero read/write failures. Otherwise stop.
5. Park on a flat, grounded road. Select **3: Queue 5s SAFETY dry run**. Close CTRPF; unpause MK7 early with A/START, then release controls well before the end. Wait for FINISHED. No artificial input occurs.
6. Repeat **3: Queue 5s SAFETY dry run**, and press physical B during countdown to abort it. Diagnostics should report **Dry-run/physical B: YES/YES**.
7. **STOP HERE AND SEND A DIAGNOSTIC PHOTO.** Do not run experimental menu option 4 until the rehearsals are reviewed. This separates safety validation from first write.

## Later optional one-shot, only after rehearsal review
1. Reconfirm normal controls, then park stationary and grounded on Toad Circuit.
2. Select **4: Queue ONE-SHOT 3-frame A pulse**. Close CTRPF, unpause MK7 early in the five-second countdown and release A/B/START early. Do not manually hold A through the final seconds.
3. It may briefly accelerate, or may produce no visible movement. Synthetic input ends after at most 3 hooked player frames or 350ms. Physical B/START aborts if observed by game hook; B/START can also abort via the slower on-screen callback.
4. Inspect **2: Native hook diagnostics**: Native A frames (0–3), pass-through release frames, control state, and read/write failures. Hardware movement and control recovery must be observed separately; counters alone do not prove MK7's downstream vehicle used the synthetic input.
5. Do not repeat a test in the same session. If anything becomes abnormal, stop, power-cycle and, if necessary, disable Luma3DS 3GX Plugin Loader and restore P006.3a.

## Limitations
- The title ID checks USA but **not exact game revision or offline race mode**. Only test offline.
- **No software abort can rescue an entirely frozen game.** A concurrent cancel cannot undo one write in progress, nor can stopping A retroactively remove the kart's momentum.
- A successful CI build validates compilation only, never real-hardware acceleration.
- Do not extend the pulse, add steering, or enable any driving AI until the results have been reviewed.
