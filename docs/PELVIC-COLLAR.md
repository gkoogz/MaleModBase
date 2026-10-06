# Shared pelvic recruitment and exact seam constraints

## Measured target attachment repair (October 6)

`RootTransition` evaluates one stable C2 root field around the measured opening
and preserves complete morphology outside its radius-dependent neighborhood.
Its radial support expands with effective root radius. Ordered tissue rings
receive a quintic falloff whose width grows with dilation; they are not collapsed
onto a common barrel radius. First and second derivatives fade at the outer
support. The inferior sector has less radial expansion beside the thighs and
receives forward recruitment instead. A concentric anterior
loft recruits the upper, lateral and lower annulus, fading into the abdomen;
it does not lift the waist into a separate shield. Character body recruitment
must use that character's measured opening, including both sides of existing
body-resource joins. Geralt does not copy Wolverine's absolute pelvic field.
`GraftDomain::orientationTriangles` optionally identifies pelvic triangles that
must retain positive rest-frame projected area. Projection acts on independent
masters, preserving exact original-edge elimination and protected/prescribed
rows; an infeasible field is rejected. Leave the list empty for the historical
solver, including freely rotating anatomy triangles.

Geralt adopts this through two coupled solves: recruit/fair the body against the
stable root field, then prescribe those body masters while fitting independent
anatomy motion back to the same sewn edge. The adapter supplies measured body
membership, opening and native edge/UV aliases. Shared geometry and constraints
remain here. The first broad body prescription and an all-body orientation
restriction were rejected; neither is a deployed solution.

For an already smooth measured body field, set the optional
`GraftDomain::preserveTargetDifferential` flag. The screened solve then fairs
deviations from that field, preserving its differential coordinates around
prescribed waist rows. Fairing displacement toward zero while prescribing the
waist creates an artificial shelf there. This opt-in does not alter historical
anatomy fitting; the empty/default path retains its replay. Original-edge
constraints, protected boundaries and signed-area projection remain active.

`GraftPlan(domain).ProjectDisplacement` caches just the constraint space for an
actual cooked surface, without a support frame or smoothing factorization. An
adapter may use this after transporting authored positions to native packing,
to retain native body orientation on needle-like subdivision triangles. Supply
original edge donors, exact geometric aliases and fixed resource joins; exclude
freely rotating anatomy faces. Valid positions pass through unchanged. This is
an additional native geometry check, not permission to exempt tiny collar faces
from the gate or hide their diagnostics.

Shared tests exercise folds, exact donors, repeatability, infeasible rejection
and transition unit/rotation covariance. They also sample radial contact onset
for abrupt slope changes and require lateral annulus participation. The user's
October 6 side-profile correction requires reviewing the entire lower abdomen
into the shaft and the lateral thigh junction, including meridian sections;
absence of a dark seam alone does not establish a sufficiently progressive ramp.
Geralt's prior 278 native-buffer exports pass
the local collar orientation and seam checks, with separate waist/LOD tests.
These are offline evidence; native grey-room and campaign visual acceptance
must be recorded by the adapter before installation is accepted.

Adoption: canonical Wolverine must explicitly pin and test this measured-target
path before consuming it; its authoritative runtime remains unchanged. Future
spokes provide their actual opening and body membership and run the same
attachment gate, with their native seam/lighting contracts in the adapter.

Feature `surface.pelvic-collar`, algorithm revision 1, lives in Base.
See `modules/pelvic-collar.json` for its versioned contract and
`provenance/collar.json` for source hashes and omissions.

## What Wolverine actually does

The active last stage is `UnifiedCollar::Apply` in
`legacy/wolverine/src/runtime/unified_collar_solver.h`, called at the end of
`anatomy_surface.h`. Do not port only the older `ApplyPelvisCollar` ramp in
`d3d9_proxy.cpp`: that would miss the final coupled correction.

The existing runtime first constructs the moving attachment and authored
pelvic shape corrections. `pelvic_attachment.h` blends radius-keyed correction
fields, preserves body/attachment seam deltas, and uses the shared conservative
triangle-area limiter. It rebuilds the affected normals and tangents. The final
unified solver then processes the combined pelvic surface and attachment.

1. A smootherstep growth value follows the **effective proximal radius**:
   `(radius - 2.9) / 4.72`, in uncalibrated source model units.
2. The recruited region grows outward with radius. Its radial envelope is
   `radius * 1.55 + 2`, with a smooth outer fade, and its axial reach grows from
   5 by an upper-sector-dependent amount. These are source numbers, not meters
   or Witcher-native dimensions.
3. The mask includes upper/lower sector and inter-thigh/ventral exclusions.
   It is an asymmetric pelvic annulus, not unrestricted spherical scaling.
   Blindly recruiting every concentric ring would move protected lower tissue
   into the thighs and erase the intended proximal profile.
4. A shared barrel radius and the actual ventral guide define radial targets.
   Growth moves the surrounding body and proximal attachment together, with a
   separate guard on backward displacement in the lower arc.
5. The final energy blends clamped cotangent curvature, a biharmonic term and
   screened target attraction. Vertices outside the supported region are fixed.
6. Fine seam vertices are eliminated into their original body-edge donors:
   `slave = (1-u)*endpointA + u*endpointB`. They are hard constraints, not a soft
   proximity penalty. UV-split render aliases receive the same solved position.
7. Normals/tangents are rebuilt on final geometry; smoothing, winding and UV
   aliases matter as much as eliminating positional cracks.

The snapshot contains 34 fine seam constraints and a 36,872-vertex combined
collar domain. `geometry.npz` preserves the associated topology, source lineage,
packed aliases and interpolation weights. Pressure-field support points in
the reference bank must also be retained, including untriangulated points.
No source tables or the authoritative Wolverine installation were changed.

## Implemented in Base

`include/malemod/surface/collar_field.hpp` contains the exact active-source
scalar metric row, extracted by `tools/extract_collar.py`. It compiles with
standard C++17 and no game/graphics SDK. It expects source-equivalent calibrated
coordinates and finite positive dimensions; adapters validate their inputs.

`malemod_base/collar.py` supplies the source-derived offline evaluator:

- `CollarFrame`: measured authoring frame/dimensions and explicit length scale.
- `recruitment`: the growing field with the original directional guards.
- `radial_targets`: source barrel/guide target law; guide samples are explicit.
- `CollarPlan`: per-character sparse operator, cached factorization, exact edge
  elimination and fixed-exterior solve. It preserves supplied topology.

The sparse energy is `8*(C*P)' D (C*P) + 2*P' C P + P' S P`, matching the source
operator. The offline implementation uses SciPy's sparse LU rather than the
original optimized Eigen/SIMD LDLT traversal. Internal calculations normalize
by the explicitly supplied measured scale, so porting to meters does not change
the source metric's balance. Scale 1 is appropriate for source-unit fixtures;
it is not an inferred Witcher calibration.

This evaluator supports offline fits/bakes and numerical inspection. It is not
a live WitcherScript plugin, a body-opening generator, a replacement Raphe
guide, or full Wolverine runtime parity. The original sequence's cached-metric
policy and eight-frame phase correction, live contact/pose inputs and packed
lighting uploads remain adapter/integration work.

## Verification

- 1,024 C++ active-source metric cases match the portable scalar kernel exactly.
- Python's field/screen coefficients match those native float fixtures within
  declared float/double tolerances.
- A separate dense reduced solve agrees with the sparse evaluator.
- Fixed exterior vertices remain unchanged and the hard interpolated seam
  remains exact under expansion.
- Radius recruitment, measured unit-scale equivalence, instance isolation,
  invalid seam/scale rejection and all 34 source donor constraints are checked.
- Imported source and geometry provenance remain verified.

These fixtures establish source-expression and offline geometry checks. They
do not establish the complete posed-body parity of the original game solver;
the live full-body capture bank is not part of the extraction.

```powershell
python tools/extract_collar.py --check
python -m unittest discover -s tests -p test_collar.py -v
python tools/verify.py
```

The CMake `collar_test` target also emits the native fixture CSV when supplied
an output filename. Keep the generated fixture and its provenance together.

## Radiating changes back to spokes

All size/recruitment laws, seam equations, shared guides and numerical fixes
belong here. Version the feature algorithm and geometry/bindings together.
A spoke pins the Base revision and records each verification level separately.

| Spoke | Current status | Adoption path |
| --- | --- | --- |
| Wolverine | Existing native final collar solver is authoritative | Reuse its observed topology/donors; replace shared math behind its buffer adapter; compare default, maximum radius, bent and sequence poses before adopting |
| Witcher | Bare stock body selected; default underwear override installed for testing | Measure frame/scale; create a body opening and fitted seam binding; author enough local support loops; export native skin/morph assets or implement a supported deformation bridge; test poses and lighting |
| Future games | No binding assumed | Observe the native body/rig; satisfy the same feature contract; pin Base and run the shared fixtures plus target-specific tests |

Geralt's bare lower-body LOD0 has only 951 vertices. A very large root expansion
may require local refinement before fitting; adding support rings must preserve
stock skin weights, material/UV seams, original-edge donor lineage and each LOD.
Do not copy Wolverine's packed vertex IDs, pressure distances or palette slots
into Geralt. Do not attach a separate floating mesh and describe it as welded.
An adapter's supported runtime path must be demonstrated before promising live
continuous expansion.
