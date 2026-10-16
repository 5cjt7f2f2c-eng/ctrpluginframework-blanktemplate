# Collie Kart P006.2 — Native Acceleration Preflight

> **Status: development-only READ-ONLY hardware probe, not the active native acceleration test.**
> The actual input actuator is intentionally not armed because MK7's KDPadSetter activation/restoration semantics are still unknown. We will not risk an uncontrolled held accelerator by guessing.

## Why this version exists

- P006 (old HID method) made 3 experimental HID-A injection calls, but MK7's vehicle control bit remained off and the kart did not accelerate.
- P006.1 hardware testing observed **43/43 physical-A → KDPad-A → VehicleControl-A matches**, and the user observed Circle Pad steering responding as expected. That validates the observable pathway, *not the ability to write to it*.
- Reverse-engineered MK7 structures suggest `KDPad::m_pad_add_base` (offset `+0x14`), `m_pad_add_idx` (`+0x18`), `KDPadAddBase::m_prev_pad_add_base` (`+0x0C`) and `KDPadSetter` candidate fields at `+0x24` buttons, `+0x28` steering, `+0x30` disable and `+0x3C` update.

These fields come from [mk7re/MK7-Memory](https://github.com/mk7re/MK7-Memory), and **the setter fields are not proven to belong to a particular linked object on physical hardware**. An active pad mask containing `0x10` is not enough to identify the setter's memory location, switching sequence, or restoration mechanism.

## New P006.2 functions

1. **Toggle NATIVE setter preflight** — opt-in **read-only** sampling of the pad add-in chain at most 5 times/sec. Traverses at most four nodes and validates that every node backlinks to the same player pad and its current data-on-frame. Failed or cyclic chains are marked invalid.
2. **Native setter diagnostics** — reports chain validity, add-in mask, number of chain nodes, number of merely *plausible* setter-like nodes, candidate button values and update flags, and count of structurally valid samples. `plausibleSetterNodes` **does not identify** a confirmed setter pointer.
3. **Queue 5s SAFETY dry run** — pre-existing P006 countdown/release-interlock rehearsal with **no input injection or memory writes**. Physical B aborts after countdown begins. A/START can resume MK7 early in the five-second countdown.
4. **ABORT safety dry run** — cancels the queued test.
5. Existing P006.1 **Toggle READ-ONLY input trace**, **Input pipeline diagnostics**, **Start recording**, and **Stop & Save CSV** remain.

P006.2 also **hard-disables the HID injection call** inherited from P006. No real actuator menu is exposed.

## Installation (New 3DS, MK7 USA)

1. Back up your successfully tested P006.1 `SD:/luma/plugins/0004000000030800/CollieKart.3gx` to your iPhone.
2. Download the successful **Collie-Kart-P0062-3GX** Actions artifact and extract it.
3. Rename `CTRPluginFramework-BlankTemplate.3gx` to `CollieKart.3gx`, and replace the plugin only after powering off the console.
4. Launch MK7; check bottom-left `CK P006.2 | INACTIVE`, toggle READY.

## Hardware test — safe, no artificial accelerator

1. Go to **Toad Circuit Time Trial**.
2. From CTRPF enable **Toggle NATIVE setter preflight**. Also enable **Toggle READ-ONLY input trace** if desired.
3. Select **Start recording**.
4. Close the menu and resume the game. Drive normally, holding **physical A** for about five seconds and steering left and right; release the accelerator and stop.
5. Open **Native setter diagnostics**. Photograph the whole panel.
6. Choose **Stop & Save CSV**. Transfer the new `SD:/CollieKart/session_NNN.csv` to your iPhone and share it here.
7. Optional: when stationary on a grounded surface, queue the **5s SAFETY dry run**, close menus/resume game, and verify it finishes harmlessly. Repeat and press physical B during countdown to verify abort. There is no real pulse option.

**Stop and restore P006.1 if FPS worsens, the game becomes unstable, or the probe reports obviously corrupted pointers.**

## New CSV columns

Appended after the P006.1 fields:
`setter_read_valid,setter_chain_valid,setter_mask,setter_chain_length,setter_candidates,setter_node0_addr,setter_node0_buttons,setter_node0_update,setter_node1_addr,setter_node1_buttons,setter_node1_update,setter_node2_addr,setter_node2_buttons,setter_node2_update,setter_node3_addr,setter_node3_buttons,setter_node3_update,setter_polls,setter_failures,setter_structural_matches`

These are **diagnostic candidates**. Do not use addresses or names as proof that a safe controller setter has been found.

## Criteria before any native acceleration command

- The same live player pad and candidate setter must be identified across multiple real driving samples.
- We must establish which MK7 processing phase consumes the setter fields; a display callback may be too late/early.
- A native actuation method must include a verified neutral-input/release path and immediate return to physical controller authority; merely stopping writes is **not** enough.
- We need bounded countdown, human authorization, reliable physical override, valid grounded/stationary Time Trial checks, timing watchdog, and a recovery method for failure.
- The actual first short acceleration pulse should be **separately approved and field-tested**, not enabled by this read-only diagnostic run.

The successful cloud build validates compilation only. All new pointer-chain interpretations need real New 3DS testing before use.
