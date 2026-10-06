# Shared pelvic recruitment and exact seam constraints

## Local side/ventral root refinement (October 6)

Keep the previous exterior support; do not widen it into the legs. A compact
radial quintic adds .06*radius on the sides with sixth-power angular localization,
and .14*radius below. It ends two neutral radii beyond the measured opening.
Lower forward projection returns toward the pelvis by up to .40 in this same
window. The original surface law is unchanged outside it and at neutral size.
The native Geralt adapter opts into .10 orientationAreaTargetRatio; shared
default .02 retains historical output. This projection target gives additional
headroom without changing donor/boundary constraints. Full native-buffer tests
pass, but no native gameplay or user visual acceptance is claimed. The final
leg audit records up to .605 mm additional safety projection below .82 m; it
is not exact unchanged geometry. See HANDOFF for paths and rejected trials.


## Current preview: local continuous surface fairing

This supersedes the historical forward-1.7 candidate described below. The
current bell has section flare 1.12, width .81*r0 + 2.18*dilation, and forward
projection 1.3*dilation. The aim is a modest wider transition across both sides,
with the original pelvis outside its support preserved.

GraftDomain offers optional surfaceFairingDistance/Strength, both default zero.
It caches a local curvature metric on actual positions, in addition to the
displacement metric, preserving exact original-edge donors and fixed exterior
rows. The metric also enters the RHS for rest positions; smoothing displacement
alone would preserve an existing crease. Adapter unit calibration is required.
Shared lighting can align cooked frames to final geometric normals and diffuse
a locally weighted normal field while preserving per-UV tangents/handedness.
This shading operation cannot repair a gap or fold; orientation and positional
gates remain separate. A flat normal fixture is not proof of native C1 shape.

Current Geralt preview passes offline seam/orientation/body tests, not native
acceptance. Stronger smoothing and explicit tangent-ring prescription were
visually rejected. Extreme upward angles remain unresolved. Wolverine is not
automatically modified; each spoke must pin the shared implementation and pass
its own geometry, runtime and observed visual gates.


Current uncommitted review candidate: support width is
`0.8*openingRadius + 2.6*dilation`, and forward projection is `1.7*dilation`
through the same smooth bell. Its outer boundary is unchanged. Shared tests
check monotone forward falloff and independence from shaft rest angle. This
candidate is not installed or visually accepted.


## Measured target attachment repair (October 6)

`RootTransition` starts from the measured rest pelvis and opening. It caches
an immutable smooth angular opening section, so an irregular body cutout is
not treated as the source character's nominal circular radius. A monotone
Hermite bell enlarges the local section toward the shaft radius, then returns
to the original pelvic position, tangent and curvature at its outer support.
The small positive inner radial slope keeps adjacent rings distinct. The local
anterior bell replaces the broad upper loft that raised a shelf at the waist.
Neutral and exterior pelvis are exact rest inputs. Shaft motion is blended
separately outside the sewn neighborhood. All dimensions use the supplied
calibration; the adapter owns the observed opening and stable opening frame.
Character body recruitment includes both sides of native resource joins.
`GraftDomain::orientationTriangles` optionally identifies pelvic triangles that
must retain positive rest-frame projected area. Projection acts on independent
masters, preserving exact original-edge elimination and protected/prescribed
rows; an infeasible field is rejected. Leave the list empty for the historical
solver, including freely rotating anatomy triangles.

Geralt uses a constraint-only projection for the measured body field, then
prescribes those body masters while fitting independent anatomy motion back
to the same sewn edge. The broader preview also preserves the anatomy target
differential, so fairing displacement against fixed body rows does not create
a collar lip. The adapter supplies measured body
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
