> Implementation update: see [PERFORMANCE-IMPLEMENTATION.md](PERFORMANCE-IMPLEMENTATION.md). The user subsequently authorized 32 rays at the same full update rate. The audit below records the earlier investigation.

# Full-quality runtime performance audit - October 8, 2026

## Governing requirement

The user explicitly rejects a 30 Hz cloth-solve compromise. Optimize throughput
and latency while preserving motion, geometry, contact behavior, lighting and
controls. Keep current render-rate garment evaluation and the existing 240 Hz
anatomy integrator with all 24 constraint iterations. Lower cadence, fewer solver
iterations, approximate reciprocal math, frozen hidden physics and reduced mesh
resolution are not the starting plan. A change of representation must prove
equivalent behavior, rather than assume interpolation conceals a loss.

This is an architectural investigation with an offline numerical experiment.
It does not install new runtime code or certify the outstanding attachment gate.

## Current machine and evidence

Observed hardware: Ryzen 7 5800X, eight cores / sixteen logical processors,
68,614,627,328 bytes physical RAM (about 64 GiB), Radeon RX 7900 XT. The Wolverine
executable is PE i386 and already Large Address Aware. A larger GPU or more RAM
does not automatically shorten sequential CPU dependencies or make a 32-bit
process a 64-bit process. No saturation, thermal or clock diagnosis was made.

The selected developer DLL is
`9e42b4d47c36b972945c4f991e3708cfee78638d75633f0bf6eb6ff1fe98e5a5`.
Its human launcher receipt reports October 8, 08:30:46 America/New_York, classic
D3D9, 1280x960, without sealed input/capture/pause controls. Its current DLL hash
matches. The October 8 user says this revision is mostly good but sluggish.

The human-play log is more representative than the earlier successful-pose
replay. At the last reported cumulative garment sample:

- 11,041 attempts; 1,077 successful full wraps; 9,964 transported fallbacks.
- 9,633 attempts lacked a complete fallback collision certificate.
- 90.25% used fallback; continuous drawing is not the same as a converged solve.
- Garment Draw mean 23.6496 ms, maximum 80.2480 ms; donor Update mean .8165 ms.
- Repeated late diagnostics say `Sewn edge clearance did not converge`.

Latest 300-call human-play CPU windows:

| Inclusive scope | Mean ms | Peak ms |
| --- | ---: | ---: |
| Overlay/update, including physics and surface | 42.128 | 79.744 |
| Anatomy physics | 18.679 | 29.559 |
| Gameplay anatomy surface, including children below | 23.144 | 57.464 |
| Rounded surface | 11.253 | 17.351 |
| Anatomical pouch skin, within rounded surface | 4.804 | 9.677 |
| Structural skin, within rounded surface | 3.318 | 5.144 |
| Collar solve | 2.033 | 4.547 |
| Render mesh reconstruction | 1.070 | 8.918 |

These scopes are nested wall times; do not sum this table. The anatomical
"pouch surface" label refers to anatomy skin, not the white garment. Garment
cumulative timing uses a different window. None is a GPU duration or a measured
end-to-end FPS value. Older sealed matrix averages include extreme controls and
rebuilds: physics 24.111 ms, gameplay surface 45.482 ms, collar peak 159.223 ms.
Those do not establish steady default-control cost.

Private evidence lives under `build/performance-20261008/holistic/`: audit JSON
and a log snapshot SHA256
`79bb8efd5bb5d96bf5f98d3e5aeefc2cd148a0f65f1e517d789d563d25c24f50`.
Do not commit the private log. Preserve exact build, controls and workload with
future measurements. The published CPU replay had zero fallbacks; its results
must not be generalized to this fallback-heavy play session.

## What the current architecture actually does

Wolverine's present/end-scene path advances anatomy physics and reconstructs
anatomy. `ApplyShape` locks the game's vertex buffer, performs the complete
`EvaluateAnatomy` pipeline while holding that lock, and then unlocks it. Later
draw hooks capture current body section palettes. The Meridian adapter then
skins garment donors, generates collision supports, fits the seam, reseeds the
whole chart, solves triangle clearance, rebuilds normals and uploads/draws.

The cloth is currently a kinematic taut-surface reconstruction (`clothDynamics=0`),
not the earlier time-integrated fabric worker. The selected candidate bypasses
the earlier JockstrapCpuWorker update path. Merely enabling that existing worker
would select a different solver and behavior.

Some optimization is already present: `geometry_pass.h` has a bounded four-worker
PPL scheduler, 16 batches, a serial threshold of 512 items and propagated MXCSR;
several skin passes use SSE. Rest-shape caching, precomputed UV data, prepared
bend constraints, cached collar factorization, XYZ sparse solve reuse, cached
plane IDs and a dynamic DISCARD garment vertex buffer also exist. Audit and
extend these facilities; do not propose them as entirely absent.

The still expensive work includes sequential contact constraints, in-place
surface guards, repeated chart reconstruction and fallback clearance. The
dependency order prevents all phases from running simultaneously. Physics also
does more fixed substeps per rendered frame when frames get slower, up to the
existing cap; measure per-substep cost and simulated time, not just cost per frame.

## Priority 1: repair the expensive failure cycle

The current common human-play path is full seam fitting which fails, followed
by surface transport and a separate bounded contact-repair pass. Seam fitting
can run twelve passes; fallback repair can run four full face/solid passes.
Fallback repair lacks the full solver's persistent separating-plane cache.
This is both a performance and a mathematical robustness concern.

Build a private regression bank from the actual failing controls/motion. Count
attempted passes, face/solid tests, planes examined and failure reason. Repair
the chart/seam feasibility problem, then preserve valid chart state across
frames. Use the previous solution as an initial guess and validate it against
the current geometry every frame. Unchanged constraints need inexpensive
validation; only changed/violated regions need correction. Every accepted
display triangle must still receive current-pose contact validation.

Do not merely suppress exceptions, assume an uncertified fallback is correct,
freeze the pouch, skip checks on a timer, or make a failing test "pass" by moving
the authored seam arbitrarily. Dynamic chart singularities and truly infeasible
constraints need explicit classification. Preserve the ordered meridians and
sewn boundary. Cache validity must depend on pose, shape, topology and contact
generation, not just whether a previous frame was successful.

## Priority 2: remove work without changing the answer

**Collision arithmetic and candidate rejection.** Reuse signed distances within
a face/plane test. Maintain certificates for successful and fallback paths.
Use conservative interval/bounding tests to reject solids that cannot intersect
a triangle or meridian sector; nine solids do not automatically justify a complex
general-purpose BVH. Cache only under explicit geometry generations. Additional
support evaluations may be eliminated if their inputs are identical; body bases
can rotate inside the mechanical solve, so a once-per-frame radius cache would
be incorrect there.

**Measured experiment.** An isolated x86 /O2 variant reuses the same three plane
distances already calculated before the correction-cost loop. It retains every
triangle, pass, predicate and arithmetic operation used to derive each distance.
Three alternating paired runs, 24 private poses, one warmup sweep and ten
measured sweeps per run gave:

| Mean across paired runs | Baseline ms | Reuse ms | Reduction |
| --- | ---: | ---: | ---: |
| Clearance | 9.4478 | 8.5427 | 9.6% |
| Entire measured post-skinning block | 10.9753 | 10.0739 | 8.2% |

All emitted positions and rendered vertex attributes were byte-identical across
all six runs (SHA256 `fc723f2f7cf94e6354df93fae4b0eb62aabcbec2702d7a6b0a3d1deae35fc645`).
This is proof on that corpus, not every possible input, new in-game FPS, or a fix
for the fallback-heavy case. The experiment is not installed or adopted by Base.
Wolverine's `tools/iteration/experiment_meridian_distances.py` reproduces it from
the exact-build profile; raw geometry and executable outputs stay ignored.

**Dependency-aware surface evaluation.** Build a graph of which final render
vertices, seam donors, pressure supports, collision controls, fluid contacts,
normals and tangents consume each intermediate surface. Compute the reverse
dependency closure for the active garment and render passes. Fully hidden
display-only detail can be omitted, while all mechanical and attachment inputs
remain full fidelity. A hidden final triangle is not proof its upstream vertex
is unused. Rounded/refined skin and its normal rebuilds are substantial targets.

**Cache and combine preparation.** Deduplicate garment donor triangles and bone
blends; preclassify lobe controls; prepare inverse transforms once per pose;
reuse allocated workspaces instead of constructing vectors through hot loops.
Audit repeated packing/unpacking and normal reconstruction between graft,
refined, rounded and final render meshes. Defer a normal/tangent operation only
after proving no intermediate consumer uses it. Existing rest/factor caches
must remain correctly keyed; don't weaken invalidation to hide control spikes.

## Priority 3: use multiple cores with explicit ownership

Use bounded, persistent workers for SDK-free numeric work. Capture one coherent
pose/control snapshot on the engine thread. Within a frame, schedule independent
chunks, wait at genuine dependency boundaries, and publish one complete result.
This accelerates the current frame without lowering cadence or silently adding
one-frame stale-pose latency. The palette source currently arrives through draw
hooks, so overlap is limited until all required sections are captured coherently.

Good candidates: independent donor transforms, per-vertex skin evaluation,
per-face contact tests against immutable iteration positions, and per-face normal
generation followed by ordered per-vertex gathering. The meridian clearance loop
already accumulates maximum corrections before applying them, so chunk-local
corrections can be reduced with deterministic maxima; preserve failure ordering,
certificate IDs and finite-value handling. Sum reductions need fixed contributor
order for bit parity. Test 1/2/4 workers and task sizes with the game competing
for cores. Avoid a thread per ray, repeated scheduler attachment for tiny work,
nested pools, false sharing and holding locks across a solve.

Poor candidates for naive parallel loops: the small Gauss-Seidel anatomy
constraint chain, in-place triangle safety guards and sparse triangular solves.
Their iteration order is part of the current answer. Graph coloring, Jacobi
reformulation or another solver can be researched, but require separate behavior
and convergence proof. Do not describe them as automatically equivalent.

Keep D3D9 calls on their owning thread. Microsoft documents significant overhead
from making the entire D3D9 device multithread-safe. CPU parallelism does not
require that flag. [D3D threading guidance](https://learn.microsoft.com/en-us/windows/win32/direct3d11/overviews-direct3d-11-render-multi-thread-differences)

Witcher already has a service thread, an isolated 32-bit source worker, versioned
transport and render publication. It illustrates why a worker alone is not a
throughput fix: the service queue holds eight requests and a slow source can
produce old results. Measure request age/queue depth and source/read/encode/
target/render costs independently. Repair the common numeric hot paths in Base;
do not duplicate another solver in the Witcher adapter. The current Witcher
working tree also contains uninstalled service/cadence edits.

## Graphics ownership, GPU use and occlusion

**Short buffer ownership.** Move numerical evaluation into owned CPU arrays.
Acquire/read engine inputs briefly, release engine buffers, compute, then upload
completed geometry to adapter-owned dynamic buffers. Bind both body resources
from the same published seam state. A static/in-use buffer lock can wait for the
GPU, and holding it while solving impairs overlap. Measure Lock/Unlock wait time
before attributing the inclusive surface timer entirely to math. Never apply
DISCARD to the engine's shared buffer without preserving all its contents.
The garment's dynamic DISCARD upload already follows the intended pattern.
[D3D9 buffer guidance](https://learn.microsoft.com/en-us/windows/win32/direct3d9/performance-optimizations)

**Occlusion.** Covered anatomy already omits 31,998 of 35,000 final triangles in
the relevant garment draw, retaining 3,002 collar triangles. Blue guides are
not drawn. The remaining opportunity is selective *evaluation*, not simply
turning off those same draw calls again. Frustum/occlusion culling may omit
display work that contributes to no color, depth, shadow or reflection pass;
retain simulation and collision state. Occlusion query results are delayed;
waiting for them can stall the pipeline. Use conservative bounds and asynchronous
availability, never a query-result wait on the render thread. Pouch containment
must be established before treating the underlying surface as wholly invisible.
[D3D9 query behavior](https://learn.microsoft.com/en-us/windows/win32/direct3d9/queries)

**GPU compute.** Consider large regular skinning/normal/contact batches only
after measuring transfer and synchronization cost. Wolverine renders through
D3D9; the existing fluid subsystem's D3D11 compute path copies to a staging
resource and maps it for CPU readback. Dispatching a cloth solve and immediately
reading it back could replace CPU work with a GPU wait. A useful GPU design
keeps most data/results resident and measures a proven compatible presentation
path. Device sharing, loss/reset and 32-bit integration are adapter concerns.
The RX 7900 XT is an opportunity, not evidence that an unmeasured round trip wins.

## Language, compiler and low-level choices

**Rust:** no whole-mod rewrite for speed at this stage. The current hot path is
optimized native C++ doing excessive/repeated work. Rust does not by itself
remove that work. It could improve ownership and concurrency safety for a future
Base core, but the D3D hooks/PhysX boundary and source arithmetic still require
care. A gradual experiment would use a small stable C ABI, i686 for Wolverine,
explicit layout/lifetimes and no unwinding across the boundary. Benchmark the
same algorithm/data/compiler settings first. Correctly structured C++ can provide
the same snapshot and worker architecture.
[Rust FFI contracts](https://doc.rust-lang.org/nomicon/ffi.html)

**SIMD and memory:** extend measured hot loops with structure-of-arrays or small
AoSoA blocks, contiguous plane data, reusable scratch arenas, prepared division
constants and sensible alignment. Existing SSE code must be accounted for.
AVX2 dispatch is a benchmark candidate, not a guaranteed gain; use a fallback
for other hosts and preserve floating-point environments on worker threads.
Avoid multiplying transformed donor work, false sharing or large temporary
copies in the name of "parallelism".

**Compiler:** /O2 is already enabled. Compare targeted inlining, link-time
optimization, profile-guided optimization and alternative compiler codegen on
the same replay corpus. Preserve the current FP contract. Global /fp:fast,
FMA contraction or approximate reciprocal changes can alter contact predicates
and attachment output. Base's historical 64-bit parity failure is another reason
to isolate and verify numerical changes rather than assume a port is identical.
[MSVC floating-point contract](https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior)

**Allocation and diagnostics:** instrument allocator activity before pooling
everything. Preallocate scratch vectors for known topology. Log numeric counters
into a bounded buffer and flush outside critical work; the current logger opens
and closes a file per message. Existing diagnostics are throttled, so this is
secondary unless a failure storm appears. Shader reflection and material setup
have caches already; measure misses and resource recreation before rewriting them.

## Environment and a trustworthy measurement plan

The rooms remain stock jungle1/q002 worlds with added controlled studios. They
are useful visual fixtures, not empty-engine benchmarks. Their source recipes
are preserved, but stripped world/gameplay systems would require engine-specific
authoring and new grounding/lifecycle proof. Even-v3 uses sky fill plus four
shadowless directional fills: measure native light-pass count and GPU cost;
maintain the requested even appearance if equivalent cheaper lighting is possible.
Do not attribute CPU fallback cost to those lights without evidence.

Separate ordinary human classic-D3D9 measurements from invisible D3D9Ex sealed
tests, their capture stalls and command polling. Disable scheduled captures during
timed intervals; record resolution, VSync/presentation mode, controls, movement,
fluid activity and current source/runtime hashes. Record game baseline, anatomy
only and garment in matched runs. Do not compare a static naked pose with a moving
garment and call the delta garment overhead.

Measure CPU sampling and context switches (Windows Performance Recorder is
available locally), exact per-frame phase spans, fixed-substep counts, allocation
counts, queue age, solver attempts/iterations, fallback reason, Lock wait and
asynchronous GPU query timing. CPU call timing is not GPU execution timing.
Keep first-load/shape-change spikes separate from steady idle and movement;
report median/p95/p99, not only cumulative means. No whole-machine ETW trace was
captured in this investigation, and no power/driver/affinity settings were changed.
[Microsoft profiling guidance](https://learn.microsoft.com/en-us/windows/win32/direct3d9/accurately-profiling-direct3d-api-calls)

## Adoption order and acceptance

1. Reproduce the human fallback workload and add attributable per-frame timing.
2. Repair seam/chart continuation and remove redundant exact arithmetic; certify
   both successful and fallback paths against unchanged geometry.
3. Split engine-buffer ownership from computation; derive surface dependency
   masks and shared caches with explicit invalidation.
4. Extend the existing bounded worker/SIMD pipeline to independent hot loops;
   measure current-frame latency and deterministic output.
5. Re-evaluate GPU placement and engine/render overhead after CPU bottlenecks move.

For a proposed 60 FPS target the whole game has 16.67 ms; a preliminary engineering
budget is at most 4 ms p95 added mod critical-path work in the normal default
case. That is a target to validate, not a forecast from current measurements.
Larger-control states and clinical scenes need separately stated budgets.

Require identical output where arithmetic/order are unchanged; otherwise strict
numerical tolerances plus the full native attachment, lighting, movement,
visibility and recovery regressions. Preserve both body resources, all supported
LODs, original-edge donors, UV aliases and pressure support. Test long motion,
control changes, pause/resume, device resets and campaign transitions. Keep exact
rollback. A faster invalid shape or stale pose fails acceptance.

Base owns numerical state, geometry dependency graphs, contact algorithms and
portable job kernels. Wolverine/Witcher own snapshots, native graphics, input,
transport and profiling hooks. Publish versioned state/output contracts and pin
the same tested Base revision in each adopting adapter. Current findings and
experimental scripts are local pending changes; no commit, push or install was
performed by this audit.
