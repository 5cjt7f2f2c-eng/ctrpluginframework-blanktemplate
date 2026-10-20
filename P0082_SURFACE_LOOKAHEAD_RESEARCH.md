# P008.2 S1/S2 — read-only live surface + look-ahead research (2026-10-20)

## Grounded findings

- USA Rev2 current collision-info pointer **address** is `0x0065C584` in `mk7re/MK7-Emu-Scripts/common.py`. This is a pointer **slot**, not the collision-info object. Citra reference, **unverified on physical New 3DS**.
- `VehicleCurrentCollisionInfo.py` reads the dereferenced object at offsets `+0x00` (u32 collisionFlags), `+0x04` (u16 rawCollisionFlags), `+0x08` (s32 unknown), `+0x0C` (float unknown). **Never assign semantic meaning to unknown fields**.
- MK7-specific `MAIN_KCL_TYPES` enumeration in `common.py` names `ROAD=0`, `DIRT=5`, `HDIRT=6`, `DASH=8`, `ITROAD=10`, `RESQ=11`, `WALL=16`, `OUTF=18`, `ITWALL=19`, `LWALL=20`, `BWALL=21`, `VALLEY=25`, `TRIGGER=26`, `ZONE=31`. Base type extraction `raw & 0x1F` is consistent with KCL layout; verify against live data.
- Existing Toad Circuit KCL reconstruction material inventory: 5,557 bounded triangle candidates, 14 distinct base types. Counts: ROAD 882, DIRT 505, HDIRT 69, DASH 4, ITROAD 141, RESQ 196, WALL 1598, OUTF 224, ITWALL 158, LWALL 72, BWALL 1174, VALLEY 35, TRIGGER 106, ZONE 393. Counts describe **mesh**, not active ground contacts.
- Existing 42 checkpoint midpoints overlap collision projections; 14 positions have multiple vertical hits. 2D-only probing is unsafe.
- `mk7re/MK7-Memory/template/Field/Entry/CheckPoint.hpp` documents checkpoint sector edges and forward vector; checkpoint sectors are **not verified road edges**.

## S1 + S2 implementation gates

1. **Contact-only prototype**: validate pointer slot via existing safe-read infrastructure; validate dereferenced pointer range/alignment and all reads. Log raw flags, `raw&31`, position, speed, race state, and unknown/no-contact explicitly. Do not alter controller or collision memory.
2. **Native look-ahead research**: find verified USA Rev2 KDGndCol collision query address, ABI, object pointer, and required thread/context. Until all are verified, **do not call arbitrary game functions**.
3. **Safe fallback**: if native query cannot be validated, evaluate a **read-only** bounded spatial query over *already loaded* KCL only after resolving runtime mesh pointer and spatial index, with vertical ray intersections. Cap query count and time; invalid/ambiguous returns UNKNOWN. Never label checkpoint sectors as physical boundaries.
4. **Diagnostic output**: CSV for contact and 5 probe rays (near/far left/right and forward), each with validity, raw flag, base type, distance, hit Y and elapsed microseconds. Mark unsupported probes UNKNOWN rather than fabricate results.
5. **First hardware run**: manual driving only, on-road → dirt → on-road, then approach boundary without crossing. Verify surface IDs, latency, geometry height, sensor cadence, pointer safety, and zero controller interference. Do not enable learning or acceleration.
6. **Release gate**: both contact and look-ahead samples match observed transitions and elevation in repeatable runs; all failures fail closed. If look-ahead remains unknown, ship S1 alone for hardware validation rather than claim S2 works.

## Architecture

The game owns its loaded course data. We are exposing *read-only perception*, **not importing a duplicate map into Mario Kart 7**. Offline KCL decoding is for reverse-engineering, unit tests, and verification.

## Sources

- https://github.com/mk7re/MK7-Emu-Scripts/blob/main/common.py
- https://github.com/mk7re/MK7-Emu-Scripts/blob/main/VehicleCurrentCollisionInfo.py
- https://github.com/mk7re/MK7-Memory/blob/master/template/Field/Entry/CheckPoint.hpp

## 2026-10-20 KDGndCol interface investigation

The public `mk7re/MK7-Memory` repository documents `KDGndCol::CheckIF` (0x10 bytes): `m_manager` at +0x0 inherited from `NarrowIF`, `m_collision_result` at +0x4, `m_hit_info` at +0x8, and `m_block_cache` at +0xC. The comment explicitly identifies `KDGndCol::Manager::checkSphere` as the producer of `m_collision_result`. `HitInfoImpl` (0xB0 bytes) stores `m_collision_flags` at +0xA4 and `m_collision_result` at +0xA8, plus ground-normal vector at +0x28. These are **structure layouts**, not verified physical-memory addresses or an executable ABI.

The available indexed public sources do **not** provide a callable USA Rev2 address, signature, calling convention, lifetime rules, or a proven safe way to construct `CheckIF` and `HitInfo` for an independent query. Therefore the native query **is not yet authorized for hardware testing**. The successful physical S1 ROAD/DIRT reads validate the existing contact sensor, not remote surface look-ahead.

Research path: obtain USA Rev2 disassembly or symbol/map material; locate call sites to `Manager::checkSphere`, recover arguments and collision mask, inspect construction/ownership of `CheckIF`/`HitInfo`, confirm loaded manager pointer, and run bounded **read-only pointer-only** diagnostics before any call. Alternative is a validated loaded-KCL spatial query using height-aware ray intersections, without calling game functions.

References:
- https://github.com/mk7re/MK7-Memory/blob/master/template/KDGndCol/CheckIF.hpp
- https://github.com/mk7re/MK7-Memory/blob/master/template/KDGndCol/HitInfoImpl.hpp
- https://github.com/mk7re/MK7-Memory/blob/master/template/KDGndCol/NarrowIF.hpp
