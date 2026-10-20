# Angle-driven collar release

2026-10-11. Implemented numerical components and a private Witcher preview.
Not accepted for installation: a visible oblique attachment crease remains.

## Mechanism

`collar_arc_release.hpp` supplies bounded C2 upper/lower responses to signed
angle, with zero response at the authored 30-degree neutral pose. The upper
response reaches one at -25 degrees; the lower response reaches one at 85.
The exact slider-to-degree mapping remains an adapter/control-contract concern.

Material arcs must follow source lineage. Classifying them by height relative
to a fixed root incorrectly changes classification along a curved shaft.
The preview instead takes upper/lower components from immutable source normals,
excludes independently controlled tissue, and interpolates the fields through
the existing source refinement and adapter donor maps for both LODs.

`GraftDomain::surfaceFairingRetention` optionally attenuates individual curvature
rows. Empty retains the old metric; all-one retention reproduces it exactly.
These are immutable plan inputs: changing angle-dependent retention requires
a rebuilt plan, not just `UpdateFrame`. It does not alter source positions,
seam donors, UV aliases or topology.

The adapter preview releases both its pelvis-to-module blend and local absolute
surface fairing on the compressed material arc. The ordinary reference solve
provides exact prescribed positions for inactive tissue. A separate constraint
projection uses that posed reference to prevent inverted attachment triangles.
It retains the original-edge constraints throughout. The increased iteration
budget belongs only to this projection, not to the ordinary body solve.

## Evidence and rejected alternatives

Private evidence and reproducible scripts are in
`D:/MaleModBuilds/witcher-ownership-20261011`.

- Direct source diffusion release reduced the previously traced Angle25
  polygon's smoothing drop from 33.644 mm to approximately 0.010 mm. This is
  a stage measurement, not a final-surface acceptance result.
- Extending that release into source pose/root-follow changed neighbouring
  tissue. Those source variants are rejected. The current guarded adapter
  preview uses the original source output.
- Position-based arc weights and unguarded release produced spikes/folds.
  The material-mapped, guarded preview removes those authored face inversions.
- `run_arc_guarded.py` covers reported-shape angles 20,25,30,35,40,50 and both
  LODs. `audit_arc_guarded.py` measures body, lower-lobe, waist, protected
  boundaries, and signed triangle area against the original baseline.
- The first Angle30 projection exhausted 128 iterations; the dedicated
  projection budget was increased to 4096. This requires performance review.
- Forcing full original projected triangle area produced ridges and changed
  the upper shape; that variant is rejected. Positive-up-only reconciliation
  also produced visible ripples and is rejected.
- The scalar-law unit test and the shared sewn-surface test pass. The latter
  checks all-one replay, released contraction, invalid weights, protected
  positions, and original-edge constraints.

The upper depression is reduced substantially in these offline native-buffer
renders, but the remaining oblique crease prevents visual acceptance. Exact
collar circumference across the control matrix has not been demonstrated.
No game package, Base pin or installed runtime was changed. No native room,
animation, campaign, lower-angle branch, comprehensive shape/state sweep, or
runtime performance pass was performed. The full attachment gate is false.

## Adoption boundary

Keep the angle law and weighted fairing in Base. Before any adapter adoption,
resolve the crease, serialize/version the material arc bindings with geometry,
measure circumference and intermediate slider continuity, and complete the
native attachment gate. Witcher, Wolverine and future spokes must pin the same
reviewed Base revision and retain their own measured boundary bindings. The
private preview files are evidence, not an independently maintained runtime.
