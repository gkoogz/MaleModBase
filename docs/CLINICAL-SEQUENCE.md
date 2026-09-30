# Portable clinical ejaculation sequence

The existing Wolverine teaching sequence now has an engine-independent CPU
implementation in `include/malemod/clinical`. It contains the original timing
envelopes, preliminary-flow approximation, main viscous stream solver, mesh
construction, passive-flow gate, deposition model and audio cue times. The
extraction preserves the existing modeled behavior rather than inventing a new
physiological sequence. Eight generated modules have source and output hashes
in `provenance/clinical.json`.

## Adapter API

Include `<malemod/clinical/session.hpp>` and allocate one `Session` per character.
Set `session.fluid.settings`, a reproducible variation seed, and collision
callbacks before calling `Begin`. Use `Advance(elapsedSeconds, nozzlePose,
milliseconds)` each frame. The original 120 Hz fluid step and 100 ms accepted
frame advance remain, including their handling of render hitches.

The adapter supplies an actual mesh outlet position, direction, inherited
actor-root velocity and collision receivers in one consistent simulation space.
The source solver applies gravity along negative Z. Convert target-engine axes
and units at the boundary. Never mix camera-relative coordinates with world
collision positions. Current settings remain uncalibrated source model units;
volumes and timing are source teaching-model parameters, not medical estimates.

Read `session.timeline.Get()` to drive the character's deformation/control
mapping. Read `session.fluid.mesh.vertices/indices` for renderable CPU geometry.
The opaque index boundary and separate surface samples remain available.
The engine translates materials, uploads buffers and handles visibility.

`collisionQuery.callback` returns `hit`, `miss` or `deferred`. An exhausted
collision budget must return `deferred` so the next query includes the entire
unchecked crossing. The callback supplies hit point, normal, receiver kind,
section, stable triangle/source IDs and barycentric coordinates. Each character
has its own callback; lambdas can capture the adapter's character context.

`deposits.project` and `deposits.resolve` similarly accept capturing callbacks.
They project footprints to receivers and resolve moving receiver anchors.
`deposits.marks` exposes CPU density, coverage/normal pixels and anchor samples;
the adapter owns textures and draw submission. Share immutable `SplatBakes`
data, while keeping each model's marks and projection budget independent.

Load the original bake with
`session.deposits.splatBakes.LoadFile("legacy/wolverine/src/runtime/splat_bakes.bin")`.
The loader accepts the exact SPK1 dimensions/length, explicitly decodes
little-endian values, and preserves valid data if a subsequent load fails.
`Load(bytes)` also supports an engine's asset loader without filesystem access.

Set `session.audio.playPhase` to receive the original phase cues (zero-based
phases 0 and 1 at 2.5 and 7 seconds). Audio file selection, playback and stopping
belong to the adapter. No private patient recordings were transferred. The
source player and its WAV validation remain archived for a Windows adapter.

`Session::Cancel` clears the session and its deposition marks and disables
further audio cues. The adapter stops any audio playback it initiated. The
lower-level `Fluid` also exposes the original `BeginPassive`,
`TriggerPassiveClear`, `Advance` and `PassiveThrobGate` interfaces for idle-flow
integration independent of the active demonstration timeline.

## Reproduction and verification

```powershell
python tools/extract_clinical.py
python tools/extract_clinical.py --check
cmake -S . -B build -DBUILD_TESTING=ON -DMALEMOD_BUILD_EXAMPLES=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

`tests/clinical_test.cpp` compiles the original CPU implementation alongside the
portable extraction. It compares timing, particle/node positions, meshes,
volume, receiver impacts, deferred-budget behavior and passive-flow parity. It separately tests
interleaved characters, callback isolation, baked-data parsing, deposition
conservation, moving receiver anchors, expiry and one-time audio cues.
`examples/clinical_session.cpp` demonstrates the adapter interface.

Wolverine's D3D9/PhysX hooks, mesh outlet discovery, character skin deformation,
shader code, audio devices and game UI remain engine responsibilities. The CPU
sequence is migrated; the installed Wolverine build and Witcher integration
have not been changed or tested against this library.

