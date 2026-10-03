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


## Phase presentation and terminal receiver drain (2026-10-03)

`FluidImpact::phase` records `LiquidPhase::clear` for both preliminary events
and passive flow, and `opaque` for the four main emissions. `emissionTime`
records a main node's birth time, rather than the later receiver callback time.
The numerical emission, node, pressure, timing and mesh expressions are unchanged.
`StainMark` and `DepositVertex` retain this phase; coincident clear and opaque
footprints remain separate fields. Adapters must use this phase when choosing
material opacity. A white vertex color alone does not establish transparency.

`Session::DepositImpacts(now)` adds valid impacts and records cumulative phase
volumes/counts. `finalPumpDepositedVolume` and `finalPumpDepositedImpacts` count
only opaque material born at or after the final event's 11.5-second start.
Earlier material landing later cannot satisfy this gate. The passive adapter
path must call this method too. Timeline completion ends deformation and cues;
it does not clear particles whose receiver callbacks have not completed.
`Session::Advance` drains existing live liquid after the timeline ends, without
new emissions or cues. Explicit cancellation or a character change still clears
liquid, fields and these ledgers.

Wolverine and every spoke adoption: retain current settings and callbacks,
replace direct session `deposits.Add` calls with `DepositImpacts`, advance while
`timeline.active || fluid.Live()`, and remove forced `fluid.Clear` at timeline
completion. Begin passive flow only after an earlier main flow has drained.
Wolverine's native renderer must retain its main/preliminary materials and use
`DepositVertex::phase` for phase-specific receiver presentation. Validate the
existing source replay and actual final-pump ground contact separately before
installing this revision. Immutable legacy source remains unchanged.

Offline gates compare all existing source replay fields at 15/30/60/120 FPS and
an 800-ms hitch, verify clear-only preliminary deposition, final-born opaque
receiver volume, separate coincident phase fields, and a deferred receiver
completed after the 20-second timeline without further emission or cues.
These gates do not certify game terrain or native shader opacity.
