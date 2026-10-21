# P008.2 Final Discovery Pass → Functional Vision Implementation

Date: 2026-10-21. Target: MK7 USA Rev2, physical New 3DS, Toad Circuit.

## Decision
Prioritize an **independent, bounded, read-only KCL geometry query** over calling an unidentified `KDGndCol::Manager::checkSphere` ABI. Keep the native function route as an optional research branch, NOT a prerequisite to first vision.

## Evidence
- S1 contact surface: hardware verified (ROAD/DIRT observed).
- S2 manager chain: hardware verified.
- S3 manager first four words: hardware verified, stable.
- S4 six candidate table words: hardware verified, function identities unverified.
- S5 48-sample audit: user supplied CSV, prior analysis reports 48 samples, 100% chain/contact reads; check original CSV when writing parser.
- Public `MK7-Memory` gives CheckIF and HitInfo layouts, but no verified USA Rev2 checkSphere address/signature.
- KCL collision primitives encode vertex, face/edge normals, prism height, attributes, octree. MK7 PolygonMesh::parseModel reconstruction documented at https://mkwiiki.org/wiki/KCL
- Reference MK7 headers: https://github.com/mk7re/MK7-Memory
- Portable multi-version KCL reader: https://github.com/KillzXGaming/KCollisionLibrary

## Batch the *remaining feasible discovery checks* into one pass
Capture one numbered report with **raw evidence and actual per-check state**:
1. Live kart world x/y/z, normalized forward vector, speed, grounded state, checkpoint/lap; reject nonfinite/implausible values.
2. Contact KCL raw type/flags alongside each coordinate sample; compare road/dirt transition locations.
3. Manager chain, table and auxiliary candidate pointer stability, readable permissions, bounded memory fingerprint. No function calls.
4. Candidate loaded course geometry: identify course resource pointer through verified structures or resource loader; check plausible KCL header/sections, bounds, counts, index ranges, endianness and format variant. **No unbounded memory scans**.
5. If loaded KCL inaccessible: explicitly report `LOADED_KCL_UNKNOWN` and allow offline course asset supplied from user's lawful game dump as implementation alternative. Never pretend success.
6. Collect timed samples on straight, curve, road edge and elevation change; report min/max/median sample interval, read failures and per-field changes.
7. If geometry is verified: run a tiny **read-only offline-equivalent intersection self-test** on fixed points with conservative Y constraints; compare center contact classification to ground truth, mark mismatches and ambiguity.
8. Output a machine-readable CSV with test state PASS/FAIL/UNKNOWN/BLOCKED/NOT_RUN, prerequisites, exact addresses/values, error counts, and acceptance criteria. Preserve previous S1-S5 results as baseline, but distinguish imported evidence from this run.
9. Strict manual-driving-only, max samples/duration, no memory writes, no controller injection, no unknown game calls, fail closed.

## Implementation after final discovery pass
A. Implement tested KCL parser and height-aware downward ray/triangle intersection (prism decoding; reject degenerate geometry). Use coarse spatial acceleration, preferably the KCL octree; cap work per frame.
B. Build local samples at heading-relative offsets (left/center/right) and distances (near/medium/far), considering current kart height, slopes, ramps, bridges, walls and overlapping surfaces.
C. Label surface class (road, offroad, wall, hazard, unknown), hit distance, ground height, confidence, sample age and validity. Never label unknown as road.
D. Cycle near→medium→near→far (configurable), budget raycasts per tick; throttle/skip under high load. Log timing and failure reasons.
E. Validate on hardware across road/grass, curve, edges, elevation; compare sensor prediction with subsequent contact samples and manually observed ground truth.
F. Only after perception passes, connect *valid* sensor features to independent SARSA policy. Keep legacy manual and READY-off controls intact; gate AI if confidence low.
G. Regression tests for parser malformed input, no hit, multi-level geometry, boundary, performance, menu pause, READY-off and abort.

## Release gate
No claim of functional vision until actual ahead-of-kart hit points, type and distance are measured on hardware and validated against repeated observed transitions. Successful compilation is not hardware validation.

## Practical blocker
No verified physical MK7 loaded-KCL pointer, and no verified callable checkSphere ABI. Resolve via one batched pointer/resource investigation OR use validated extracted course KCL data as the first sensor backend. Do not require indefinite diagnostics if the latter works.
