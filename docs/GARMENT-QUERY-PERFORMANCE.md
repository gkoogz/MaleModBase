# Exact garment query optimization evidence

This checkpoint is offline source/native geometry verification. It is not a
clean pinned delivery receipt, acceptable frame cadence, an installation, or
observed gameplay. All anatomy triangles and the complete fine cloth remain.

## Shared query precomputation

A cloth triangle caches its edges, bounds and plane once per BVH query. The
original SAT normal expression is retained separately from the closest-plane
normal to preserve the original floating arithmetic. Current SDK-free tests
compare 120,000 original and precomputed pruning decisions across explicit
units, translations and degenerate cases. Eight actual coupled source frames
in `build/cloth-face-query-adapter4/comparison8.json` preserve full cloth binary
positions, normals, tangent, UV, donor lineage and topology, complete source
wire6 bytes, and every non-timing field. Frozen cloth time improves from about
425–699 ms to 311–466 ms. This remains far above a playable frame budget.

## Certified outside point queries

The moving-surface certificate first proves that a point is outside the full
closed volume. Membership-only callers then use its positive distance lower
bound. Positive-clearance callers query the original physical triangles within
`max(requestedClearance, margin + 1e-5) + 1e-8`, beyond every existing positive
projection/friction action threshold. Inside, boundary and indeterminate
membership continue through the original exact path. The extra guard prevents
a bounded no-hit radius from entering a projection branch. No physical contact
or separating normal velocity is discarded.

`coverageMargin` is conservative certified clearance. A smaller positive
certificate or no-hit radius is not an exact global minimum and cannot claim a
larger clearance. The frozen guard proof reports the changed lower bound.
Independent finite-plane tests verify exhaustive/bounded contact points and
actual action decisions under rotation, translation and three source-unit
scales. Physical impulse conservation and rendered-gradient tests also pass.

Eight actual coupled frames in
`build/cloth-point-guard-adapter7/comparison8.json` retain exact complete cloth
binary and source wire6 bytes, contact impulses and aggregate moments, material
strain and actual source shape. Time is about 267–386 ms on that frozen case.

### Nearest shared-edge donor tie

The guard proof does **not** have identical per-source-vertex reaction records.
`build/cloth-reaction-trace-adapter8/reaction-difference.json` records the exact
forensic trace. On publication 8 both paths record 1,619 contact events with
identical measured contact points and physical impulses. A nearest shared-edge
tie selects native triangle 7480 versus 6917 on edge 3695–3698. One original
unused corner receives barycentric weight `0x1.78p-55`; its tiny nonzero moment
passes the existing publication threshold, yielding 150 versus 149 unique
vertex records. No collector event was lost or zeroed. Publication 7 has another
adjacent-triangle tie (7014 versus 7017). Maximum measured per-vertex moment
component difference is 3.9898639947466563e-17 in explicit source units; impulse
differences are about 6e-19. Aggregate physical results and actual source bytes
remain exact in this fixture.

This narrow floating tie is disclosed, not relabeled exact record parity. It
does not authorize donor truncation, zeroing tiny weights, broader approximate
contacts, or relaxing complete cloth/source byte comparison. Final current
geometry and all required coupled/native delivery matrices remain separate
acceptance gates.

## Current full solver cost diagnosis

A later frozen current-shape run in `build/cloth-current-cost-adapter10` keeps
3,671 physical nodes, 17,183 material constraints, 4,495 rendered vertices and
8,752 complete faces. Two actual coupled frames pass contact and material
publication. Cloth costs approximately 374 and 438 ms. The coarse profile records
about 165/220 ms of nested nearest-surface queries, 38 ms of initial full-input
collider updates, 78 ms of substep Input/interpolated collider updates, and
24/26 ms of twelve material passes. Actual ray classification takes only 2/6 ms.
Rendered evaluation count is 377,326/379,456; active manifold time is under 2 ms
on these early frames. Nested timers must not be summed as independent costs.

The initial full-input collider refit is overwritten before queries when a
frame has positive substeps. A frozen guarded-removal diagnostic retains that
update for zero-step/pause frames. Its six retained contact-bearing frames keep
complete cloth bytes, all reported physical fields and input/rest files exact;
only conservative coverage bounds change. Cloth time improves from
385–450 ms to 318–420 ms. Both same-shape variants reject frame 5 on the material
gate while contacts pass (fine strain 1.07013), so this is **not** an accepted
eight-frame adoption and the guarded removal has not been promoted. The failed
prefix and exact input remain in
`build/cloth-initial-update-adapter11/diagnostic-prefix.json`.

## Private actor-origin coordinate formulation

`build/cloth-actor-frame-adapter14/proof.json` binds a private historical-chart
experiment. Physical/closed BVHs receive the complete measured samples in an
interpolated actor-origin frame; cloth queries and returned contact points are
translated explicitly to/from the final solver origin. Original world/native
samples remain available for obstacle velocities and source consumption.
Active physical pairs and reaction barycentrics use the same prepared triangles.
Actor-local unchanged endpoints remain unchanged during interpolation; changed
native points still interpolate. No triangle, donor, gradient or anatomy is
removed, and no contact action threshold is loosened.

Independent exhaustive point/finite-face oracles compare physical contacts,
closed membership, measured world moments and virtual work across rotations,
large translations and three explicit unit scales (3,600 point plus 3,600 face
cases). Maximum normalized physical distance/contact-point difference is
8.19e-13; maximum source-scaled world moment difference is 1.40e-12.

Eight actual frozen historical-chart feedback frames pass contact/material and
preserve complete lineage/UV/topology/materials. Cloth times are 196–323 ms
versus the earlier 267–386 ms baseline. All body vertices no longer appear to
move solely because the actor trajectory is interpolated under a final origin.
However, iterative contact branches amplify altered floating arithmetic: cloth
position component differences reach .00311256 source units and normal
components .00405709; source floats and final wire6 bytes differ. Contact
impulses also differ, while measured equal/opposite linear conservation remains
at floating precision. This **fails exact-byte adoption**. The code remains in
ignored frozen build storage, with no live source change. A separately reviewed
coordinate formulation and complete current source/native quality matrix would
be required to adopt it. Cadence still exceeds a playable budget.

## Private ordered parallel query experiment

The historical candidate19 snapshot has full measured source contacts and zero
sheet crossings, but rejected hems. It is a numerical performance fixture,
never final garment geometry acceptance. No parallel-query implementation was
installed or promoted into shared source.

BodyCollider point/face queries read immutable collider data; each worker owns
its memo copy and inherits the caller's floating environment. Four workers are
retained for one Update. Coarse point queries are computed speculatively, then
consumed in the original anatomical/body/node order. A ticket whose exact
physical query point changed is discarded and recomputed, including the body
query after an anatomical correction. Only current tickets commit memo state.
No simultaneous contact projection, changed contact order or surface reduction
is used.

`build/cloth-parallel-point-adapter16/proof8.json` binds all frozen headers,
executables, actual source library, full rendered cloth bytes and final wire6
source bytes for eight actual source-feedback frames. All bytes and non-timing
physical fields match the serial baseline exactly. Reaction records are active
in six frames (3/64/107/131/119/105); 54 genuinely stale body tickets take the
serial requery path. Coarse stages decrease from 49-110 ms to 23-44 ms, while
total cloth time decreases only from 536-1021 ms to 522-973 ms. This remains far
above the active frame budget. Current sewn-nullspace integration and safe
worker exception propagation are still required before any live adoption.

Finite-face experiments retain original serial corner membership queries and
certificate updates. Only physical/capped closest-face prefixes run in bounded
worker chunks, with exact current corner/dependency checks before consumption.
`build/cloth-parallel-face-stamp-adapter18/proof2.json` binds two-frame full
cloth/source byte equality for both variants. Chunks of 64 are slower
(555/982 ms versus point-only 524/898 ms). Chunks of 256 and exact positions of
all actual translational/ribbon/transport support nodes produce 526/896 ms,
which establishes no useful additional speed improvement. Extra rendering,
memo copying and synchronization consume savings from mostly already-certified
clear faces. These variants remain private rejected performance experiments;
optional longer face replays were stopped.
