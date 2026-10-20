# Jockstrap regression audit - October 10, 2026

The walking algorithm accumulated global collision validation, projection,
retries and recurring smoothing. This audit identifies fixes; no runtime code
or game installation changed, and no new attachment acceptance is claimed.

## Verified evidence

Installed DLL SHA-256 remains
`ab1f8d8e028a1239474465aa5f98faedea45860e1547aff21ad4db8d4087a1a7`.
This matches the handoff's source `2d144e80` / Base `389f6c04` build. The hash was
checked again; source attribution uses that existing receipt, not a new build.

The retail `C:/Games/X-Men Origins Wolverine/Binaries/WolverineRuntime.log`
read this turn has SHA-256
`5332bbd81113410e722b295c9de9feec0dc46c6a9a2cbd0a8e902c354c55dc7a`.
At frame162 it reports 121 garment attempts: **88 wraps / 33 follows**, zero
transported/uncertified attempts in that sample. Garment Draw averages
**425.2613 ms**, peak **920.1850 ms**; first wrap took **584.4248 ms**. Donor
Update averages **0.4434 ms**. The wrap fraction is **72.73%**.

These are existing log samples read today, not a newly launched benchmark,
campaign FPS or GPU timings. The log identifies menu-tank evaluation with
Overall86 / Width100 and state2; fluid work is also enabled. Compare identical
controls and scenes in future benchmarks, not these numbers against older
default-pose room results as though the workload were identical.

## Regression chain

1. Wolverine commit `2b5e829` added `if(!wrapped) FairMeridianReversals(...,4)`.
   This includes already-certified follower frames. The fairer rebuilds triangle
   adjacency, sweeps two directions, tries five movement fractions per reversal
   and tests incident triangles against solid planes. It is much more expensive
   than neighbor averaging. Current source `9fac680` skips certified followers,
   but that candidate failed native acceptance and is not installed. The fairer
   is a confirmed contributor, not a measured explanation of the whole425ms.
2. Commit `6ac0ee9` increased the recipe from32x24 to64x40: **1,504 -> 5,056
   collision triangles**, a **3.36x** increase. Full garment rendering is4,279
   vertices /8,368 triangles; trim and collision face counts are different.
3. `WalkCertifiedTautEnvelope` can reconstruct the cover/walk **six times**.
   Each attempt computes full clearance, runs up to **24** contact passes,
   computes final clearance and checks displacement. The adapter adds another
   physical-space certificate and can enter another fallback walk/refit.
4. `TautTriangleClearance` scans every plane for each face/solid pair, then
   evaluates exact support in both triangle-normal directions. `OvoidSupport`
   uses24 double-precision bisection iterations with square roots. An unculled
   scan of5,056 faces against two ovoids can perform **485,376 bisection
   iterations**, besides other work. Installed Base lacks the later current-pose
   bounding-box rejection. Current source has it; prior trials show it is not
   sufficient by itself.
5. The latest retail sample rebuilds on most attempts, so the nominal cheap
   follower path does not dominate. Prior moving-room trials also exposed
   sewn-edge angular reversal and infeasible seam fitting. Increasing budgets
   or mesh density treats symptoms of an unstable surface parameterization.

The earlier48-interval curved walking refinement is separate: its recorded
16-pose paired replay increased5.63073 to5.78195ms (~2.7%), with equal fallback
counts. See MERIDIAN-CURVATURE.md. That evidence does not support blaming the
original walking refinement alone for the catastrophic regression.

## Additional code findings

**Repeated failed work in one frame is possible.** `prepared=renderFrameSerial`
is set only after geometry succeeds and uploads. Several draw hooks call Draw.
Rejection leaves prepared/drawn guards unset; later eligible hooks can rerun the
same failing geometry. This is present in installed and current source. Its
frequency was not measured this turn; the121-attempt sample above reports no
rejected attempts and does not demonstrate this amplification there.

**Draw averages exclude failures.** Timing is accumulated only after successful
draw submission. Early returns and exceptions are omitted. Count attempts by
unique coherent input generation, and time rejected work too. Do not interpret
attempt counters as unique frames or successful-only means as total cost.

**Some expensive measurements are unnecessary on success.** The initial seed
gap in WalkCertifiedTautEnvelope is used only after refinement failure, to choose
the next padding. Compute it lazily against the unchanged seed when another
attempt is needed. The final full-gap scan is used only as a threshold predicate:
use early-exit separating-plane certification, retaining the full minimum scan
for diagnostics. Preserve nonfinite, degenerate and tolerance behavior explicitly.

## Fix priorities

| Priority | Fix | Required safeguard |
| --- | --- | --- |
| First | Time all attempts, rejection reasons, retry counts, support queries and p95/p99 | Separate menu/gameplay and match controls |
| First | Cache rejection per actor/frame/topology/controls/coherent palette and pose generation | Allow a new attempt when previously missing palettes arrive or actual inputs change |
| First | Test existing follower-fairing skip, fixed-seam early rejection and current-pose bounds together | Source presence is not native acceptance |
| First | Lazy seed-gap evaluation and threshold-only final certificate | Compare accepted geometry and rejection decisions; retain finite checks |
| Next | Persist topology adjacency and separating-plane hints; build bounds once per pose | Revalidate against current geometry; invalidate by topology/control generation |
| Next | Repair violated face neighborhoods and expand to incident faces when vertices move | Whole displayed surface remains certified |
| Next | Distinguish seam infeasibility, chart degeneracy, contact failure and buffer failure | Retry padding only when it can fix the failure |
| Structural | Stable material-space sewn order and tailored front/side/gusset patches | Shared bindings across boundaries; no angular reordering of a sewn seam |
| Structural | Coarse fitting cage with smooth display tessellation | Conservatively certify reconstructed display triangles, seam and UV aliases |

Keep the walk as a local fitting/contact operator in the structural redesign.
Transport a stable patch cage with current body/anatomy deformation and repair
local contact changes; reserve global reconstruction for actual validity or
control changes. Construct smooth, consistently sampled paths before meshing
instead of introducing spikes and repeatedly repairing them afterward.
This is a design proposal requiring native proof, not a promise that another
single-chart follower will work. Existing single-chart fast trials failed.

Do not simply revert to32 rays, widen padding, drop certification, lower cadence
or paint the anatomy as fabric. Recorded native trials of those fast shapes
showed flaps, missing pouches, exposed gaps or oversized panels. They remain
rejected. A low-single-digit-ms total garment preparation budget is a useful
engineering target, not a measured result or promised speedup.

## Holistic considerations and verification

Historical human-play profiling also found expensive anatomy physics and rounded
surface work. Slow frames increase fixed substeps up to the cap, amplifying
stalls. Profile per-substep cost and hidden display dependency closures after
the garment bottleneck. Preserve mechanics, seam donors and pressure supports
even when skin is hidden. Existing SSE and bounded worker parallelism already
exist; a language rewrite or an unbounded worker does not resolve excess work.

Taut refinement updates vertices in place: naive parallel face correction changes
its algorithm. Immutable certificate tests are a safer parallel target after
eliminating redundant work. Graphics calls remain in the engine adapter.

Base owns numerical/contact/patch algorithms and regression cases. Wolverine
owns coherent-input guards, timers, native bindings and buffers; consume a tested
Base pin and record other-spoke adoption separately. Validate failing high-size
moving histories as well as successful poses, transitions and seam failures.
Exact-work changes need output/rejection comparison. Shape changes require
front/side/oblique native views, motion, coupled controls/states, both body
resources, LODs and campaign checks. Retain installed runtime and rollback until
a replacement passes. No fresh native or full attachment gate ran for this audit.

Source references: Base `meridian_clearance.hpp` (fairer), `meridian_rig.hpp`
(ovoid support), `taut_contact.hpp` (retry/refinement/full gap scans),
`meridian_follow.hpp` (certification/refit); Wolverine
`src/runtime/meridian_adapter.h` (Draw and preparation guards),
`src/runtime/meridian_recipe.h` (counts), and `d3d9_proxy.cpp` (hook callers).

No runtime source/settings/game files changed. Existing unrelated dirty work is
preserved. After fetching Wolverine origin, main is68 commits ahead/zero behind.
Base locally reports41 ahead/zero behind its existing tracking ref; Base was
not fetched this turn. No commit or push was made.
