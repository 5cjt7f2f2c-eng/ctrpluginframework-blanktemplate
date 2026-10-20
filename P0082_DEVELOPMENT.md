# P008.2 Track-aware learning

## Target
Build a genuinely learned MK7 driver with the full drivable track corridor (left/right boundaries), legal checkpoint topology, intended travel direction and look-ahead curvature. Demonstration data may warm-start the policy but may not secretly steer during autonomous evaluation.

## Current P008.2a code (NOT full P008.2)
- Branch forked from P008.1; native input, safety watchdog, telemetry and held actions preserved.
- Read-only KartInfo checkpoint travel-heading is now included in four additional policy features (18 total). This is **not** a next-checkpoint bearing or full track map.
- Random exploration favors the six forward/neutral actions. Learned reverse actions remain possible.
- Model header is now COLLIE_P0082A_V1,9,256 to prevent silent loading of incompatible P008 weights. Existing snapshots remain intact on SD.
- Visible menu/build marker CK82-1020A.

## Still required before claiming corridor-aware driving
1. Resolve and validate the Field::MapdataCheckPointAccessor pointer chain on **MK7 USA Rev 2**; confirm checkpoint entry counts, pointers, left/right sector geometry, and successor links read-only. Abort geometry use on any validation failure.
2. Checkpoint sectors alone may not match actual drivable road edges. Validate against collision data or preprocessed map; represent branching, overpasses, glider segments, shortcuts and out-of-bounds.
3. Compute corridor-relative lateral offset, width, intended tangent and multiple look-ahead curvature samples. Avoid assuming nearest XY corridor is current corridor; constrain by checkpoint order and section.
4. Add optional P007 demonstration-derived **state-to-action** supervised warm start (NOT absolute-coordinate replay). Record schema, validate data, keep guided P007 separate from independent evaluation.
5. Improve action selection and reward; explicitly track failure reasons. Preserve physical override and watchdog.
6. Compile, then verify heading changes through manual cornering and geometry against the actual track on hardware before autonomous steering uses it.

## Release gate
P008.2a is an exploratory stepping-stone, not a completed map-aware learner. Do not claim full corridor data, imitation pretraining, cross-track generalization or hardware validation until implemented and tested.
