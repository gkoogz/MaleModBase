# Ordered mechanics and independent presentation

The Witcher modular adapter previously waited for reference skin, target mesh,
lighting preparation and publication before admitting another physics sample.
The numerical solver itself was not the dominant cost. The October 20 pipeline
separates these responsibilities without changing the integration clock.

## Shared API

`Session::StepMechanical` retains the original control mapping, fixed-step
solver, garment impulse accumulator, shape authoring and rest-length feedback.
`ReadMechanical` exports guides and independent lobe state without reference
vertices, attachment metrics or a geometry-derived nozzle. A full `Read`
rejects a stale surface. Consumers cannot request reference collar samples on
this path.

`StepRetainedSkin` evaluates the original reference skin, including its final
collar corrections, and omits only unused body/collar export. Those corrections
affect retained skin donors, so skipping them is not equivalent. The rejected
shortcut is not part of this implementation.

The adapter sends identical ordered inputs to numerical and retained-skin
processes. The skin process can combine pending publications, advancing each
intermediate input mechanically and evaluating skin on the last input. It
never drops elapsed time, control changes, contacts or cumulative impulses.
Queue admission is atomic across both streams; capacity exhaustion causes a
visible retry. Runtime resets replace both processes together.

## Geometry and rendering

`TaskArena` provides bounded reusable CPU workers. Independent vertex/face work
can run concurrently; reductions retain original face order. `OrderedNormalPlan`
owns validated topology and deterministic adjacency. Prepared material sections,
endpoint derivatives, sparse skin-relief contributors and control transforms
avoid repeated invariant calculations. None changes native topology, UV aliases,
weights, lineage or the shared waist.

The Witcher adapter independently coalesces mesh snapshots and displays current
controls on prepared geometry. Expensive shape authoring waits for a short quiet
interval during repeated size edits; angle edits bypass that debounce. Final
exact authoring replaces bounded interim deformation.

## Evidence and limits

Current private evidence lives under
`D:/MaleMod/Work/witcher/runtime-rearchitecture`.

- 240 ordered samples match the preceding worker's numerical state exactly,
  including moving contacts, all states, controls and cumulative impulses.
- The same test preserves all 7,218 retained-skin donor rows exactly.
- Six final packed native vertex buffers match the full-reference baseline.
- The split-service test reaches the same final numerical state in both streams;
  numerical work averaged 4.96 ms and advanced ahead of skin preparation.
- Separate native timing distinguishes numerical work, retained-skin batches,
  mesh preparation, display publication and acknowledged control changes.

These are independent checks, not blanket native acceptance. A historical
Wolverine log is not a matched benchmark. The attachment, campaign and supported
LOD gates remain required before installation. An earlier corrupted native
triangle index was guarded but its writer remains unidentified.

## Adoption

Base owns the APIs, math, deterministic workers and regression tests. Witcher
owns wire operations, process queues, scene integration and native rendering.
The adapter pins numerical implementation independently from the historical
character binding revision; existing assets must not be relabelled.

Wolverine and future spokes can adopt the pinned shared geometry/arena APIs
after their own native parity tests. Wolverine's installed runtime remains
authoritative until explicit adoption; this work does not modify it.
