# Independent garment geometry acceptance

External tissue clearance does not establish garment consistency. A pouch may
clear all native anatomy/body faces yet contain crossed material lanes, folded
cells or open sewn junctions. Keep these checks separate from dynamic material
strain, reaction balance, native optical coverage and observed gameplay.

The shared tools are `tools/audit_surface_intersections.cpp`,
`malemod_base/garment_geometry_audit.py` and `tools/audit_garment_matrix.py`.
They require no game or graphics SDK. Adapters export their actual input, mesh,
typed material layout and source receipts. They do not keep separate copies of
the geometry auditor.

## Matrix and immutable thresholds

The final source matrix contains 72 actual exports:

- Geralt and Wolverine;
- Overall 25, 50, 75 and 100;
- Angle 1, 50 and 100 (the actual UI minimum is 1 in both adapters);
- all three mechanical states 0, 1 and 2.

Each typed pouch must have zero proper triangle crossings, zero positive-area
coplanar overlaps, zero degenerate triangles and consistent shared-edge winding.
Legitimate point/edge contact is reported separately. Shared-vertex pairs are
excluded only when their intersection is legitimate contact; a shared vertex
does not excuse positive overlap. The triangle tolerance is the actual pouch
diagonal times `1e-9`; the area threshold is diagonal squared times `1e-12`.

Maximum rear strap and side hem section turns must remain strictly below `pi/6`.
Current sewn
point mismatch against an authored finite rest offset must not exceed one
quarter of the normalized waistband thickness. Do not adjust these limits to
accept a failing case. Cell aspects, folds and raw junction gaps are
preserved as diagnostics in addition to the acceptance fields.

## Authored finite seam offsets

A sewn ribbon has nonzero width and thickness. Demanding coincident centerlines
can destroy its geometry. Conversely, capturing a gap from a fitted mesh and
declaring that gap to be its rest offset cannot prove a closed joint.

The layout supplies `measuredCircumference`, `bandThicknessNormalized` and
`authoredJoints`. A joint contains a name, two weighted mesh bindings `a`/`b`, an
authored world-coordinate `restOffset`, and
`offsetProvenance: "authored-thickness-contract"`. Each binding has up to four
actual vertex indices and nonnegative weights summing to one. Required names
are `hem-0-top`, `hem-1-top`, `hem-0-bottom`, `hem-1-bottom`,
`strap-0-bottom` and `strap-1-bottom`. The attachment pairs must represent the
actual respective sheet edges, band/hem ends, and strap cap/underside patches.

The auditor measures `length((a - b) - restOffset) / circumference`, retaining
the raw gap separately. Missing authored endpoint correspondences remain
uncertified. The current shared fitter publishes these references under
`jointRevision: 1`: hem endpoint offsets are authored from finite hem thickness
before contact fitting; underside strap cap centers use the declared weighted
sheet endpoints with zero rest offset. Historical captured fit gaps do not meet
this contract.

`geometry_export.hpp` provides the shared SDK-free `WriteOBJ` and `WriteLayout`
serializers. OBJ positions, UVs, normals, index correspondence and actual
material enums are preserved at double round-trip precision. The layout writer
exports the authoritative joint declarations without recapturing offsets from
the final mesh. Invalid finite-value/weight/index contracts and stream write
failures are rejected. `geometry_export_test` checks this source-export contract;
actual character fit still needs independent measurements below.

## Schema 2 receipt

Each case binds absolute artifact paths and SHA-256 values for `input`,
`sourceExportReceipt`, `samplerBinding`, `meshOBJ`, `layout`,
`baseHeaderManifest` and `auditTool`. The export receipt must also declare the
exact family/size/angle/state and bind its actual input/output/header artifacts.
No case may be relabeled after its sampling. The top-level header manifest
hashes every current shared garment header, names the Base commit, and declares
the clean pin. The auditor independently checks Git HEAD, cleanliness and
complete current header equality; a copied `cleanPin` flag is insufficient.

The independent fixture proof binds the crossing executable/source and finite
offset closure test/auditor sources. Analytic tests distinguish real crossing,
positive-area coplanar overlap, legitimate contact and separation; seam tests
retain a valid thickness offset while rejecting an added gap and captured-gap
authorization. These tests do not establish actual-character fit.

Missing actual exports are `PENDING`, unsuccessful exports are `FAILED`, and
measured geometry failures are `REJECTED`. Only the complete clean-pin 72-case
matrix may set `allCasesPassed: true`. A source receipt is not an installed-game
or native-rendering receipt. Development renders must identify source rest vs
dynamic output and must not describe exposed side openings as fully covered.

## Final adapter delivery

`validate_geometry_receipt` independently rechecks the clean source pin,
complete garment header manifest, artifact hashes and exact sampled UI states.
It recomputes the shared geometry measurements and reruns the fixture-bound
intersection executable for all 72 cases, rather than trusting copied PASS
flags. Witcher's `record_extended_delivery` and its `--verify` operation require
the canonical receipt at `Base/build/garment-geometry-<pin7>/proof.json`.
Incomplete development matrices remain diagnostic artifacts and cannot satisfy
the delivery gate.

## October 4 independent development checkpoint

The private candidate13 snapshot replays both actual retained characters at
Overall 25/50/75/100, State 2 and Angle 50. All eight sheet crossing, coplanar
overlap, degeneracy and winding counts are zero. All six authored joint
correspondences are certified, with maximum normalized residual below
`4e-16`; rear strap and side hem turns remain below 18 degrees. These are
eight static source cases, not the clean-pin 72-case gate or moving cloth.

The visual/coverage audit still rejects the current garment as finished.
Independent side rays through complete actual anatomy/body/cloth show broad
openings: cloth covers only about 18--49% of sampled visible anatomical surface
area for Geralt and 35--44% for Wolverine. Native body and anatomical self
occlusion are removed before this face-centroid measurement. This is a
diagnostic sample, not fragment-complete coverage or native optical proof, but
it demonstrates that the tight, mostly closed modesty requirement is unmet.
The current 37/48-column band also retains visible polygon edge turns around
the rear. Render omission of underlying anatomy must never be treated as
coverage success. Keep these risks separate from successful sheet/contour
intersection checks.

The subsequent candidate14 latitude-width experiment is also preserved as
rejected: all eight sampled hem section turns increase to approximately
63--74 degrees, above the frozen 30-degree limit. At default size its wider
sheet covers the glans but nearly none of the exposed-side lobe; total side
coverage remains inadequate. Positive rest contact/crossing/offset checks do
not override these failures. Do not replace the final matrix with either
development snapshot.

Candidate15's stereographic/harmonic side patch increases sampled side
coverage to about 90--100%, but is rejected for material crossings and sharply
reversing hems. A separate frozen-source precontact diagnostic finds these
crossings already present immediately after chart initialization (at default
size: 112 proper crossings for Geralt, 141 for Wolverine). Complete external
skin clearance does not certify an injective material chart. The new 96-column
finite-thickness band also has six outer/inner/cap crossings at Geralt100's
bottom rows, columns 91/92; other seven band cases pass that face audit.
Preserve these diagnostic failures independently of improved modesty coverage.

`malemod_base.garment_coverage` holds the shared diagnostic side-ray and
area-weighted visibility math; native capture decoding and source rendering
remain in adapters. Inputs use the caller's measured Frame and normalized waist
circumference. Full native body and anatomy self-occlusion are checked before
testing cloth coverage. Analytic fixtures distinguish cloth in front from cloth
behind, overlap, separation, tilted ray intersections, both windings and native
body occlusion. There is no invented modesty acceptance threshold in this
module. Its face-centroid sample diagnoses broad side openings without proving
complete fragment coverage, optical opacity or moving-cloth behavior.

Candidate16 distinguishes orientation from usable material quality. Its raw
initialized eight sheets have zero proper crossings and coplanar overlaps
using the exact shared corner diagonals, but near-singular triangles reach
aspect ratios approximately 528,000--3,590,000 before contact. Subsequent
physical corrections introduce crossings in six of eight cases, and most side
hems still exceed the frozen turn limit. Seven complete 96-column bands pass
self-consistency; Geralt100 has 33 outer-row/cap crossings after local clearance.
Source side coverage remains approximately 86--100%, yet these geometry
failures prohibit adoption. Frozen precontact and final diagnostics are
preserved separately; no runtime or native optical acceptance is claimed.

Candidate17 improves the initialized material quality: all eight raw sheets
have zero crossings/overlaps/degenerates and maximum triangle aspect ratios
of approximately 107--201. Later rest-contact processing still introduces
crossings in six cases and the upper hem reverses in seven, so the full
garment remains rejected. The later candidate18 band-only construction uses
one coherent geometric thickness director for each angular column. All eight
complete 2,688-face bands pass self-intersection/overlap/degeneracy tests both
before and after the unchanged local physical clearance corrections, including
the previously failing largest Geralt case. These band-only diagnostics do
not accept the rejected pouch chart or moving fabric.

Candidate19's rest-only radial clearance and collar transition produce zero
sheet crossings, overlaps, degenerates and winding conflicts in all eight
full source cases, while complete physical contact and authoritative seam
residual checks pass. This is progress rather than acceptance: maximum hem
turns still reach about 36--70 degrees and severe upper-collar folds remain.
Sampled source side coverage is about 87--100%. Covered front/side/rear and
varied-size images are explicitly labeled source-rest diagnostics; dynamic
support and native optical/material performance remain separate gates.

Candidate20 frees the interior side hems during measured chart construction
and adds angular bending-energy descent with exact local spherical orientation
and triangle-quality checks. All eight raw/full sheets remain free of crossings,
overlap, degeneracy and winding conflicts; physical contact and authored seam
checks pass. However all eight hems still exceed 30 degrees, and canonical
Overall25 retains one rear strap kink at approximately 60 degrees.
The same failures occur in the actual sheet sideBoundary centerline: the
small outward finite hem offset does not cause these kinks. The comparison
receipt retains every measured sheet and tube turn for candidate19/20.

Candidate21 adds rest-only outward radial membrane/edge fairing with complete
physical contact after every sweep. All eight contact/intersection/author-seam
checks pass. Geralt25 and50 now meet the frozen hem turn limit (maximum27/28
degrees); six other cases still exceed it, and canonical25 retains one failing
rear strap. Some sheet fold counts increase. The complete garment remains
rejected, while the improved small/default Geralt cases are preserved rather
than overwritten. Covered source previews and exact varied-size receipts
remain separate from native and dynamic acceptance.

Candidate22 uses signed rest-only radial membrane fairing with the same strict
unilateral native-contact corrections. All eight measured source cases now
pass the frozen side-hem turn limit, full contact, sheet intersection and
authored seam checks. The canonical25 terminal rear-strap kink remains:
approximately 59.7 degrees at penultimate section91 of93. Position remeasurement
confirms that strap was identical in candidate19/20/22, correcting an earlier
regression attribution to20. Residual interior sheet folds also need visual
quality review. Sampled side coverage remains approximately86--99%; this
does not replace fragment or native optical proof.

Candidate23 adds natural-boundary ribbon bending for every nonendpoint section,
including the formerly excluded penultimate section. All eight retained actual
State2/Angle50 cases now pass full contact, sheet intersection/overlap/degeneracy,
winding, authoritative seams and the frozen hem/strap turn limits. Maximum
hem turns are 26.01 degrees and straps 13.11 degrees. Coupled contact can move a
joined endpoint together with its authored partner; fixed endpoints inside the
bending helper do not imply byte-identical final fitted positions. This is an
eight-case source-rest checkpoint, not the final 72-state or gameplay proof.

Candidate24's additional lower-side guide fullness remains rejected. Seven
cases pass complete contact, but canonical100 fails at the inward corners of
both side hems near sections14--18. The worst body-face clearance is .00073380
circumference against the unchanged .0024 requirement; the anatomy-side worst
deficit is .00020150 circumference. The diagnostic executable reproduces the
original frozen mesh and layout byte-for-byte and records exact native triangle
IDs and closest-point barycentrics. Its small Geralt coverage improvement does
not excuse failed physical clearance. Candidate23 is preserved as the prior
source-rest checkpoint; all24 artifacts remain available as rejected evidence.

Candidate25 strengthens that lower-side guide but is also rejected. All eight
sheets remain free of crossings, overlaps, degenerates and winding conflicts;
authored seams and straps pass. Geralt's four cases retain complete native
clearance and passing hems. Canonical75/100 fail clearance and exceed the hem
turn limit at 31.42/31.78 degrees. Exact frozen diagnostic reproduction shows
mid-hem inward corners penetrating the required margin; canonical100 also
has two failing actual sheet side-boundary nodes at rows 12/13. Thus changing
only tube directors cannot be assumed to fix the largest case. The largest
Geralt sampled coverage improves to 89.5/88.1% in the two lateral views, while
the complete covered clinical source preview still shows a lower triangular
opening. This diagnostic coverage does not authorize the failing canonical
geometry. The final 72-state, dynamic and native-render checks remain pending.

The complete garment audit is broader than the historical sheet-only gate.
`audit_surface_intersections --all-pairs` retains every proper crossing and
positive-area coplanar overlap pair without changing tolerances or excluding
sewn components. Exact component classification of candidate23 finds no
individual band/sheet/strap/hem self-crossings, but numerous crossings between
the sheet and its finite hems, band and strap origins, and sewn junctions.
These require explicit construction and sewing semantics; they are not
blanket exemptions or proof of complete garment self-consistency. Candidate26's
shading-normal profile rebuild additionally introduces same-hem self-crossings
and fails multiple hem turns. Its exact 16 donor records and UVs are preserved
across all eight inputs, so that failure is geometric rather than ancestry loss.

Candidate27's free conditioned boundary contact clears all eight native inputs,
but fails some hem turns and retains same-hem crossings. Candidate28 fixes the
original spherical area floor, improving the largest canonical hem, yet seven
contact cases pass and canonical100 fails at sheet face 5889 against native
anatomy face 507. The vertices themselves pass: the face clearance deficit is
approximately .00008947 circumference. Exact frozen face diagnostics preserve
this between-sample failure. Some actual hem bend radii are smaller than the
finite hem halfwidth; a passing centerline angle alone cannot certify a smooth,
nonintersecting thick ribbon. All these snapshots remain source diagnostics.

Candidate29 discovers nearby original body-triangle pressure supports before
constructing the tension hull. It never converts them to additional hard cloth
pins. Five of eight actual inputs hit the unchanged eight-pass nonconvergence
gate and produce no garment export; their status is FAILED. The three exported
cases pass native contact and sheet self-consistency, but two canonical cases
have hem turns exceeding 100 degrees and same-hem crossings. An exact diagnostic
records monotone newly discovered native body vertex IDs for every pass. Two
canonical support walks converge at passes 6/7, while some others still discover
new supports at pass 7. This distinguishes a finite discovery horizon from
repeated-ID cycling without authorizing larger unchecked bounds or accepting
the resulting folded hems. All evidence remains separate from runtime proof.

Candidate30 uses one measured physical arc-length row map for both initial
side guides. Actual row spacing improves substantially and no individual
garment component has proper self-crossings across the eight retained neutral
inputs. One canonical75 native contact case still fails. Candidate31 combines
that row map with conditioned free-boundary contact and hem bending: all eight
retained State2/Angle50 cases pass complete native clearance, sheet consistency,
six authored seams and the frozen hem/strap turn limits. Maximum hems are 22.35
degrees and straps 13.11. Cross-component sewn intersections are still reported
without exemptions, and residual interior folds need visual review. Covered
varied-size source renders retain all native tissue behind white garment,
using a common measured circumference display scale within each character.

These neutral fixtures do not establish the full UI contract. Eight fresh
actual Overall100 extreme inputs (both characters, angles 1/100, states 0/2)
fail restored23/30 initialization or contact. Failure-only chart diagnostics
show 1694--2904 interior material faces violating the planar chart orientation
gate in seven cases; the remaining 30 case collapses the rest chart radius.
Exact controls, native input/export hashes and full frozen header hashes are
preserved. No source pose is relabeled as a different UI state or accepted by
substituting old neutral captures. All 72 actual states require their own fit
and independent geometry validation before final acceptance.

Candidate32 tests a measured sewn-root chart origin and original pressure-hull
centroid pole fallback. It is rejected: all eight fresh extreme inputs still
fail before export; all four retained canonical neutral cases fail contact and
three develop almost reversing hems and same-hem crossings. Geralt neutral
clearance passes and triangle aspect improves, but actual lateral coverage
falls to roughly 60--73%, substantially enlarging the exposed side region.
Neither improved aspect nor source-only SDK covariance can accept this change.
Exact source 32 geometry, full component classifications, coverage receipts and
actual extreme failures are preserved separately from the prior 31 checkpoint.

Candidate33 restores the 31 origin/pole and increases lower-side fullness. The
largest Geralt's sampled coverage improves to 89.3/88.0%, yet the complete
garment remains rejected: canonical25 cannot unfold 21 lower-strip material
faces, small Geralt's hem exceeds 51 degrees and canonical100 loses native
clearance. All exported components remain individually free of crossings;
the usual sewn-component interactions stay explicitly reported. Fresh 33
extreme cases also fail and remain diagnostic rather than relabeled success.
Exact failed exports are shown as FAILED in coverage/geometry matrices, with
no fabricated mesh or borrowed preview substituting for them.

## Candidate 34 endpoint-preserving lower fullness (rejected)

The explicit source refinement preserves the candidate 31 seam endpoint slopes
and exports all eight retained neutral inputs, but does not satisfy both ports.
Canonical Overall 25 has 16 proper sheet crossings and seven same-hem crossings;
its first hem turns 71.52 degrees at the penultimate section 31. Canonical
Overall 100 fails complete native clearance and its hems turn 49.09 and 35.58
degrees. All four Geralt cases pass native clearance and the frozen route limits
(maximum hem turn 19.58 degrees); their covered source appearance does not
authorize the failing canonical construction. Largest Geralt centroid-side
coverage is 88.9 and 87.4 percent, with a visible lower triangular opening.
All other seven sheet ranges have zero proper crossings/positive overlaps and
all eight authoritative seam residuals pass. Full component-pair receipts retain
every sewn interaction without blanket exemptions. Candidate 31 remains the
latest passing eight-case neutral checkpoint; actual extreme UI, dynamic cloth
self-contact and observed gameplay remain separate incomplete gates.

## Candidate 35 controlled numerical refinement (rejected)

One frozen source binary records an explicit bounded sideCoverageExtra in its
material layout and per-trial receipts; this is a construction diagnostic, not
a newly exposed user slider. Both tested refinements fail the canonical small
case: 0.15 creates one proper sheet crossing, seven same-hem crossings and a
74.44-degree edge turn; 0.30 fails source chart initialization. The other three
risk inputs (both ports at Overall 100 and Geralt at 25) pass the rest/contact
gates at both values, which does not authorize adopting either refinement.
The zero setting also fails exact historical candidate 31 parity: regrouping
its floating point multiplication changes the iterative material fit, including
a 70.57-degree canonical small hem. All eight zero-setting OBJ hashes differ
from the frozen candidate 31 exports. Preserve the exact baseline expression
before applying any conditional nonzero refinement. Full measurements and
input/header/sampler hashes remain in candidate35-controlled/receipt.json and
baseline31-byte-comparison.json. No remaining four-size or native gameplay
acceptance is inferred from the bounded risk trials.

## Candidate 36 historical parity restored; nonzero refinement rejected

The zero-refinement expression now preserves candidate 31's literal arithmetic
association. All eight independently exported zero-setting OBJ files are byte
identical to their frozen candidate 31 counterparts, and all corresponding rest
gates pass. This is stronger than comparing default zero with explicit zero
inside a changed implementation.

Controlled risk cases remain rejected at both nonzero values. At 0.15, canonical
Overall 25 has one proper sheet crossing (row 31 column 0 against row 30 column
0), six same-hem crossings and a 124.78-degree penultimate edge turn. The
penultimate centerline overshoots its sewn bottom cap, reversing its last
segment. At 0.30 that input fails initialization. Both ports' largest cases and
the small Geralt case pass their rest gates at both values. No remaining four
cases or new render matrix were run because neither risk subset passes. Exact
artifacts and arguments are bound by candidate36-controlled/receipt.json and
its baseline-first-receipt.json; source31 remains the all-eight neutral shape.

## Candidate 37 measured pattern activation (neutral source checkpoint)

A shared measured pattern rule applies the lower-middle fullness only when the
pouch's projected forward reach requires it relative to its actual broad top
seam width. Below aspect 1.25 it preserves the literal historical baseline;
smooth activation reaches the requested maximum at aspect 1.50. This is shared
geometry, not a game-, character-, Overall- or UI-state special case. Layout
exports separately record requested maximum, measured aspect and effective
refinement.

All eight retained State 2 / Angle 50 source cases pass native clearance, sheet
crossing/overlap/degeneracy/winding, authored finite seams and frozen route-turn
limits. Complete-mesh pair enumeration reports no individual-component self
intersections. Maximum hems are 22.47 degrees, maximum straps 13.12 degrees.
Canonical Overall 25 and 50 activate zero and their complete OBJ exports remain
byte identical to candidate 31. Canonical 75/100 and all four Geralt inputs
activate 0.30; their actual aspects are 1.5546/1.9796 and 1.7313-1.7665.

Largest Geralt lateral centroid-ray coverage is 88.1 and 86.6 percent, versus
87.1 and 85.8 in candidate 31. This modest change is not proof of complete
modesty or fragment coverage. Full native anatomy/body remain in the covered
neutral source comparison and varied-size renders. Sewn cross-component
intersections still need explicit construction certification; actual extreme
UI geometry, persistent motion, native optics and observed game results remain
independent incomplete gates. All source/layout/header/sampler artifacts are
bound in candidate37-actual-growth-activation.json; covered-source-comparison31-37
uses the same camera and scale, and covered-varied-source-candidate37 uses a
common circumference-normalized display scale within each port.

### Bounded stronger-maximum study after 37

Using the same frozen candidate 36 binary and explicit parameters, the largest
canonical input passes at 0.45 but fails native clearance and the frozen hem
turn limit at 0.60 (49.12 degrees). No other 0.60 cases were run. All six
noncompact neutral inputs pass their rest gates at explicit 0.45, including
the remaining Geralt 25/50/75 and canonical 75 cases. This is separately bound
in candidate36-controlled/stronger-noncompact-receipt.json, not relabeled as
integrated candidate 37 geometry. Largest Geralt centroid coverage becomes
88.4 and 87.1 percent, only a modest gain over the integrated 37 maximum of
0.30 (88.1 and 86.6 percent). The same-camera covered source comparison retains
the complete gray native anatomy/body and shows the actual remaining lower
opening. No threshold was relaxed and no broader control matrix or gameplay
claim follows from this bounded study. Candidate 37 source stays unchanged.

## Candidate 38 frozen integrated neutral checkpoint

The requested maximum is now 0.45, with the same measured aspect activation
and exact compact-panel baseline association used by 37. Fresh integrated
exports pass all eight retained neutral State 2 / Angle 50 cases: complete
native clearance, proper sheet crossings/positive-area overlaps/degeneracy/
winding, six authored finite seams and both frozen route-turn limits. Complete
all-pair mesh enumeration finds no individual-component self crossings.
Maximum hem turn is 22.3444 degrees and maximum strap turn is 13.1126 degrees.
The two compact canonical cases retain byte-identical candidate 31 OBJ files;
the six extended panels record effective refinement 0.45.

Largest Geralt visible-area centroid coverage is 88.4/87.1 percent for the two
lateral views, compared with 87.1/85.8 in 31. Near-side lobe coverage is only
78.79/74.52 percent (31: 75.47/71.53), so the remaining lower opening must not
be described as completely closed or modesty-certified. The covered comparison
and varied-size PNGs retain the complete native body/anatomy in neutral gray;
source lighting remains separate from native optical opacity.

The orchestrator returned exit code 1 after all eight per-case native sampler
exit-code-zero receipts had been written. Its aggregate was recovered from
those exact existing receipts without rerunning source fits; the unexpected
wrapper exit cause is unknown and explicitly recorded. Independent geometry
audits completed successfully afterward. Hash-bound measurements are in
candidate38-actual-growth-activation.json, candidate38-actual-geometry-audit-
summary.json and candidate38-complete-cloth-intersection-classification.json.
The exact same-camera comparison is covered-source-comparison31-38/
covered-comparison.png; all-size source views are covered-varied-source-
candidate38/covered-varied.png. Existing 31/37 and rejected trial artifacts
remain unchanged. This is a neutral source checkpoint; actual extreme UI
geometry, certified cross-component sewing, persistent cloth motion, native
optics and gameplay remain incomplete independent gates.
