# Complete evaluated source surface

For the full Witcher phase, explicit collision calibration, process-transport
contract and C++ target collar adoption, read FULL-RUNTIME-ADOPTION.md. Optional
measured contacts preserve the uncalibrated reference branch. Process-parallel
state relocation has a failed shape gate and must not replace the verified core.

## Default baseline with mechanical bindings

`Output` now returns the same-session 12-point guide, rest guide, lobe centers,
anchors, radii and axes, root direction and ten raphe bend multipliers. This
avoids inferring a physics rest shape from rendered vertex centroids.
`tools/export_default_baseline.py build/<new-name>` runs the verified Win32
session at state 2 and all seventeen UI values 50, for 120 source frames. It
exports geometry, topology IDs and mechanical data with hashes together. Its
pose uses source reference thigh capsules and zero gait input, not live capture.
The existing measured source-to-character scale must remain explicit; changing
the graft boundary width must not silently enlarge the default anatomy.

Witcher consumes this export for its fixed baseline. Wolverine and future
spokes can adopt the same output contract after replay and native validation;
this addition does not modify Wolverine's installed runtime or source snapshot.

The static `derived__final_reference_positions` export is a posed authoring
reference, not Wolverine's evaluated neutral anatomy. Both have 17,528 vertices
and the same 35,000 triangles, but the evaluated neutral differs by RMS 24.593439
and maximum 44.696598 source units. The existing Witcher ten-joint size transport
cannot reproduce the final vertex construction. The 07:54 user screenshot marks
0.4.23 as improved but still failing shape quality.

## Shared implementation

`surface::Session` in `include/malemod/surface/runtime.hpp` evaluates the complete
source numerical pipeline. It owns a worker thread; mutable globals and local
caches are isolated with thread-local storage. Immutable source tables are shared.
Requests are synchronous and serialized per session. Another character has its
own session and state. The public API returns positions, normals, tangents, UVs,
source IDs, original final indices, both coupled body sections and measured dimensions.
Packed source buffers are internal reference working storage; native game buffer
layouts, skeleton palettes, coordinate calibration and uploads remain adapter work.

The kernel is generated from hash-checked source declaration spans in
`tools/data/surface-closure.json`. The immutable snapshot is a build provenance
input. It is not compiled directly as a portable library and is not edited.
The generated 54 MB include remains in ignored build output. The shared target
needs C++17, SSE2 and Eigen, with no game, Direct3D, PhysX or audio SDK.

The evaluator includes prepared morphology and fairing, original rest-frame
history, root regularization, logical shaft, suspension/contact and lobe fitting,
R14 glans construction, rounded/pouch/raphe support, pelvic attachment, final
neck topology, the active UnifiedCollar solve and final lighting reconstruction.
ResetPelvicAttachmentBody runs before evaluation, as in the original ApplyShape.
No game capture is needed to seed it: source rest/body tables supply the input.

State plus all 17 sliders use `malemod_base.controls.ORDER`. Optional calibrated
thigh endpoints and numerical pitch/yaw forces are explicit caller inputs.
An absent thigh input uses Wolverine's source reference pose. It does not observe
another game's legs. Animation sequences, ambient pulses, fluid, audio, world
collision queries and engine motion filtering are excluded from this feature.
Native adapters must construct their own bitangent handedness and skin bindings;
the portable output does not promise unchanged source packed W/skin/color bytes.

## Build and verify

The TLS session recipe requires strict arithmetic. The process-isolated recipe
uses source-global storage, Wolverine's `/fp:precise` arithmetic and its bounded
parallel geometry dispatcher. These are separate verified recipes. Combining
process-global storage with `/fp:strict` failed the scrotum-100 gate; disabling
optimization also failed it. Matching the original arithmetic repaired that
case without changing the geometry, constraints or tolerance.

```powershell
cmake -S . -B build/surface-cmake -A Win32 -DMALEMOD_BUILD_SURFACE_RUNTIME=ON
cmake --build build/surface-cmake --config Release --target surface_runtime_cli surface_session_test
ctest --test-dir build/surface-cmake -C Release -R surface_session --output-on-failure
python tools/extract_surface_runtime.py --verify-provenance
```

`tests/wolverine_surface_oracle.cpp` is a separate reference-only target that
includes the original runtime and needs its original graphics SDK. It calls no
GPU or game and can use `-` as its source-table seed. `capture_surface_controls.py`
records isolated endpoint cases; `verify_surface_runtime.py` compares complete
session outputs, original topology/UVs, lighting XYZ and both body sections.
The reference target's SDK dependency does not apply to the shared target.
The local verification was built with MSVC directly and through CMake's Win32
Release target. The latter builds the static library and passes the session
isolation test. CMake was provisioned only into ignored local build dependencies.
Other compilers have not passed parity gates.

## Evidence and gates

- 37 shape cases and 33 physics cases: maximum anatomy position error 0.00000763
  source units. Both body sections, exact topology/UVs and lighting gates pass.
- 180-frame trace: changing preferences, all three physics states, motion forces
  and moving thigh contacts; 3,155,040 anatomy vertex comparisons. Maximum anatomy
  error 0.00001527; maximum coupled body error 0.00000763 source units.
- Simultaneous sessions retain byte-identical independent positions; invalid
  controls are rejected without changing state and the worker remains usable.
- A 64-bit build compiles but **fails** the 0.0001 source-unit parity gate
  (neutral maximum error 0.020826891). CMake blocks that architecture until repaired.
  The original full runtime is 32-bit; its PhysX ABI also blocks direct 64-bit
  reference compilation. Neither failure is waived.

These are source/offline results. No full-surface Witcher bridge was installed.
The existing 0.4.31 joint preview remains installed. Native buffer correspondence,
target fitting, protected body boundaries, upload synchronization, loader behavior,
performance and observed gameplay are still required.

## Every-spoke adoption

Wolverine can consume the verified 32-bit session output through its adapter,
retaining its observed skin/material/draw conventions and supplying actual pose
inputs. Do not replace its working installed runtime until native parity passes.
Witcher must replace joint approximation with an implemented full-vertex output
path, consume exact source/edge lineage, preserve Geralt's existing body joins,
and pin this Base revision. Its 64-bit process cannot directly link this currently
verified 32-bit library: an explicitly tested service bridge or repaired 64-bit
arithmetic is required. Future spokes follow the same source, binding, native,
installed and observed gates. A menu or a two-mesh blend does not establish them.

## Source metric lifecycle and process recipe

Output now includes `collarMetric`: the actual root, axis, up, radius and length
passed to UnifiedCollar::Build, plus a generation counter. The extraction recipe
observes that call; it does not mirror or replace source invalidation rules.
Ordinary motion changes the live guide and fresh target displacements while the
support metric remains cached. Shape/state changes rebuild it under the source
policy. Adapters must use this support frame for their target collar metric,
while retaining the moving guide for actual anatomy and skeleton transport.
Wire version 3 carries both frames and rejects older packet versions.

The parallel Win32 recipe permits one Session lifetime per process. A reset
requires a new worker process because source-global caches survive destruction.
It preserves sequential constraints, reductions and collision publication;
only the original independent geometry work is dispatched in parallel.
Wolverine remains unchanged. Its future adapter can adopt either verified
recipe with source/native/gameplay gates; every other spoke uses the same Base
contracts, without independently copied solver or cache rules.

```powershell
cmake -S . -B build/surface-process-verified-cmake -A Win32 -DMALEMOD_BUILD_SURFACE_RUNTIME=ON -DMALEMOD_SURFACE_PROCESS_ISOLATED=ON
cmake --build build/surface-process-verified-cmake --config Release --target surface_runtime_cli surface_session_test surface_wire_test
ctest --test-dir build/surface-process-verified-cmake -C Release -R "surface_session|surface_wire" --output-on-failure
```
