# Sewn curvature investigation, October 11

The requested Witcher saddle/ridge repair is **not complete**. No candidate from
this investigation is accepted, installed, or enabled in a normal adapter build.

## Confirmed numerical defects

The previous graft curvature energy used `L*P`, then squared its individual
rows. `P` eliminates original-edge seam constraints, but this ordering leaves
the two sides' boundary fluxes separate. On a flat joined interface their
equal and opposite contributions should cancel before curvature is penalized.

The opt-in `GraftDomain::coupledCurvature` instead uses `P^T*L*P`, with lumped
mass from `P^T*area`, before forming the squared curvature term. Protected
physical boundary rows are excluded from absolute surface curvature penalties.
Protected positions and original-edge interpolation remain hard constraints.

The opt-in operator also keeps signed cotangents. Clipping each cotangent to
[0,20] loses linear precision on obtuse triangles; the assembled signed FEM
stiffness retains its positive-semidefinite energy. The historical arithmetic
remains the default, including its existing cotangent clipping.

`graft_sewn_curvature_test` uses two unequal-width strips on a tilted plane,
with duplicated seam vertices and original-edge constraints. The historical
solver moves the plane by 0.0573148 fixture units; the joined operator's maximum
movement is 6.353e-17. These are synthetic units, not game millimetres.
The existing orientation/donor/infeasible-constraint regression also passes.

Four real Geralt-domain fixtures, spanning both LODs and two frame/radius
combinations, reproduce the pre-change solver byte for byte with the option
disabled. Positions, recruitment weights, and five updated-frame weight sets
per fixture agree. This establishes default replay for those fixtures only.

## Geometry results and rejected approaches

Private evidence and scripts are in
`D:/MaleModBuilds/witcher-ridge-20261011`.

- Rebuilding normals directly from geometry retains the ridge. It is not just
  a normal-map or shading artifact (`normal-isolation.png`).
- Joined curvature improves the oblique transition but does not eliminate the
  remaining saddle (`coupled-comparison.png`, `a25-signed.png`).
- A full joined solution moves lobe-owned geometry by about 49 mm. It fails
  surrounding-anatomy preservation (`a25-joined-full.png`).
- Holding the lower hemisphere fixed preserves that region but leaves a
  constricted transition (`a25-ventral.png`). Hard semantic lobe constraints
  create new folds (`a25-roles-full.png`, `a25-majority-full.png`).
- A measured upper-rim continuation eliminates the extra local height dip,
  but produces a ledge, especially in LOD1 (`bounded-lod-review.png`). Bounding
  its biharmonic displacement stops overshoot but does not cure that ledge.
- Solving all three coordinates under those constraints distorts the nearby
  pelvis (`a25-joint.png`). Reject it.
- A posed-metric solve guarding all anatomy triangles fails signed-area
  convergence; its output must not be used. The failed face was
  19890/19889/19819, ratio -0.032178.
- Disabling coarse source diffusion or moving it before the pose operation,
  combined with the joined adapter experiment, retains or worsens creasing
  (`source-coupled-comparison.png`). Neither ablation is promoted.

The mathematical correction is verified; it is not proof that these two
operator defects account for the entire visible saddle. The coupled treatment
of shape, moving attachment geometry, and adjacent surface derivatives remains
unresolved. Do not call a smaller dip a completed repair, relax orientation
guards, or promote one of these images as a clean result.

## Scope and adoption

The shared correction is in Base. No new adapter algorithm, dependency pin,
binding, topology, UV lineage, donor identity, game asset, or installed runtime
was changed during this follow-up. The earlier guarded angle-preview source
remains disabled in normal builds. All new geometry experiments are private.

Full `tools/verify.py` still stops at the existing motion-header extraction
mismatch. That unrelated header was not regenerated. Standalone numerical
tests and direct graft provenance hash verification are recorded separately.
There is no new grey-room, campaign, motion, performance, complete-control,
native visual acceptance, install, or rollback proof.

Witcher, Wolverine, and future spokes can consume the shared opt-in through a
pinned Base revision only after their own attachment gate passes. Do not copy
the experimental source into a separately maintained adapter fork. Preserve
the accepted runtime and rollback while the visual repair remains unresolved.
