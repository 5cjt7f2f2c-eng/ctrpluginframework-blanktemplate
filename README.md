# Collie Kart Portable P006.3 — Native 3-frame acceleration test

**HIGHLY EXPERIMENTAL. Code compiles, but there has been NO real New 3DS test of this native frame hook.**
Keep a backup of the stable P006.2 3GX. If MK7 crashes, freezes, stops responding to normal inputs or accelerates unexpectedly: quit MK7 (or power off the console), disable Luma3DS 3GX Plugin Loader, and restore P006.2. The in-plugin emergency abort *cannot* recover a frozen game.

## Research and implementation

This approach is **not** the original P006 HID Injector and **not** a speculative direct write into `KDPadSetter`. Two sources inform it:

- [mk7re's simulated player](https://github.com/mk7re/mk7-llm-research/blob/master/docs/tooling/emulator/simulated-player.md), tested in an **EUR2 emulator**: it hooks `KDPadInputer::calcInput`, lets the game read physical inputs, then replaces the current frame's button/stick data. When its driving mode is disabled, the original game input passes through.
- [MK7-Revolution's `KDPadInputer::calcInput` hook](https://github.com/Anto726/MK7-Revolution/blob/master/src/base/hooks/System/KDPadInputer/System_KDPadInputer_calcInput.cpp) and [signature](https://github.com/Anto726/MK7-Revolution/blob/master/src/base/pointers.cpp): evidence for locating and intercepting the relevant game function without a fixed EUR address.
- [CTRPluginFramework hook API](https://github.com/Tekito-256/CTRPluginFramework/blob/master/Library/include/CTRPluginFramework/System/Hook.hpp): provides MITM and original-function handling.

The current 3GX attempts to locate the **unique 16-byte instruction signature** inside the running MK7 USA executable. If it finds zero or multiple occurrences, it refuses to install a hook. The original input function runs on every intercepted call. **Only** if the actual local player `KDPad` pointer matches, gameplay is not marked paused, and the user has explicitly armed an experimental pulse does the hook temporarily set bit 0 (A) in `KDPadDataOnFrame::m_buttons` for up to **three frame callbacks**. It leaves steering and all other buttons unchanged.

This may or may not be compatible with this real USA New 3DS. A uniquely matching signature does not *guarantee* the actual hook is safe. The signature is an external project result and has not yet been validated on this console. Hooking modifies an instruction in the running game's text memory, unlike P006.2.

**No HID key injection. No changes to vehicle physics, speed or reward system. No machine-learning controls or automatic steering.**

## Installation

1. Save the P006.2 working `CollieKart.3gx` backup onto the iPhone.
2. Download the successful **Collie-Kart-P0063-3GX** workflow artifact (compiled but hardware-UNVERIFIED).
3. Extract and rename `CTRPluginFramework-BlankTemplate.3gx` to `CollieKart.3gx`, then replace `SD:/luma/plugins/0004000000030800/CollieKart.3gx` with your 3DS powered OFF.
4. Start MK7 USA Time Trial on Toad Circuit. Verify bottom-left `CK P006.3 | INACTIVE`, then toggle READY.

## Test 1 — PASS-THROUGH hook only

**DO THIS FIRST. No artificial input is used in this test.**

1. On Toad Circuit Time Trial, stationary, READY on, open CTRPF.
2. Select **1: Install PASS-THROUGH hook**. If the signature reports 0 or >1 matches, **STOP**. This build cannot safely locate the correct USA function; send the result.
3. If hook installation succeeds, close the menu and resume MK7 normally. Drive using physical A for 3–5 seconds, steer left/right and press B briefly. Nothing should feel different from P006.2.
4. Choose **2: Native hook diagnostics**. Require **Hook installed: YES**, many **Player input frames**, and **Physical-A hook frames** at least 10. If callbacks remain 0 or the game responds incorrectly, **STOP**, do not proceed to an artificial pulse.
5. If anything slows, crashes, or feels strange, restore P006.2 and send the diagnostic screen. Do not assume a successful cloud build verifies a game hook on hardware.

## Test 2 — two harmless safety rehearsals

1. Park on a flat, grounded Time Trial road with READY on.
2. Select **3: Queue 5s SAFETY dry run**, close CTRPF and exit MK7's own pause menu early in the countdown using A or START.
3. Release A/START before the final second. The dry run must reach FINISHED with **no artificial input**.
4. Repeat **3: Queue 5s SAFETY dry run**, resume MK7, then tap physical **B** during the countdown. Inspect **2: Native hook diagnostics** or **Profiler & control diagnostics**: physical-B abort verified.
5. All these conditions must remain true **within one MK7 session**: 30 native player-hook callbacks, 10 manual-A hook callbacks, successful dry run, and separate physical B abort.

## Test 3 — optional native A pulse, ONLY after tests 1 and 2 pass

1. Record the hook diagnostic screen first. **Do not test if physical controls behaved unusually.**
2. Stationary on the ground in Toad Circuit Time Trial, READY enabled. Keep the physical A and B buttons released.
3. Select **4: Queue NATIVE 3-frame A pulse**, accept the warning, close CTRPF, and resume MK7 early during the familiar **five-second countdown**.
4. Release A/START before the final second. The callback will **attempt three A-input game frames at most**, with an independent **350ms deadline**. After the third call the callback returns to **normal physical input on subsequent frames**, without restoring any persistent button override. **Real hardware release is not yet verified.**
5. Physical B can abort while the game-frame callback is running; the normal CTRPF menu-based abort also works if the plugin remains responsive. If the game freezes neither software abort is guaranteed.
6. Watch whether the kart starts moving **and whether it stops accelerating after the pulse**. Keep the first attempt to a single trial; do not repeat if anything seems wrong.
7. Inspect **2: Native hook diagnostics** for **Native A frames** (target 3) and **Pass-through release frames** (at least 3). These counters prove the hook wrote input and subsequently observed original unpressed input; they do *not* alone prove the game's vehicle consumed each artificial frame.
8. If you have enabled **Start recording**, use **Stop & Save CSV** to save the new `SD:/CollieKart/session_NNN.csv` and share it, along with any observations. Recording is opt-in.

### How to interpret results

- **Signature missing or nonunique:** no hook installed; revise USA signature research.
- **Pass-through hook breaks manual driving:** stop, revert to P006.2; hook ABI/timing/location is incompatible.
- **3 injected frames, no movement:** may be too short, ignored downstream, or mismatched hook timing. Don't increase pulse automatically.
- **3 injected frames + movement, but no normal-release frames:** stop; inspect safety and restore fallback.
- **3 injected frames + movement + normal physical control afterward:** first promising native-control result. Still validate stability and emergency override separately before adding steering or learning.

## Safety limits

- Off by default; hook installation requires an explicit menu action.
- A `Controller::InjectKey` HID pulse is not used anywhere in this build.
- Unique USA executable signature requirement; never hardcode the EUR2 address.
- Pass-through physical A must be observed >=10 frames before the real test is unlocked.
- Five-second countdown, neutral-input guard, dry-run completion, physical-B dry-run abort.
- A substitution is ONLY local player A, maximum 3 game callbacks, expiration deadline 350ms. No sustained steering or automatic repetition.
- Physical B cancels a pending/active experiment while callbacks run; turning READY off and reopening CTRPF also disarms it.
- Hook is disabled when the plugin exits normally; crashes/frozen callbacks may prevent this.
- Human must supervise the hardware test. Current hook, release and game input behavior remain **UNVERIFIED on real hardware**.

## New flight-recorder columns

P006.3 appends these to P006.2 CSVs:
`native_hook_address,native_hook_frames,native_manual_a_frames,native_injected_frames,native_release_frames,native_pulse_state`.

The meaning of `native_pulse_state` is: 0 disabled, 1 armed, 2 three A-frame writes finished, 3 aborted. These are plugin states, not direct game physics.

The known P006.2 performance was smooth according to the latest hardware report. This P006.3 has additional hook overhead and must be checked for slowdown separately.

### Version backups
Original P006.2: branch `collie-kart-p0062-native-control-preflight`.
P006.3 development: branch `collie-kart-p0063-frame-hook-first-contact`.
