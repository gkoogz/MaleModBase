# Measured pelvic entry ramp — preview, not installed

## Further natural integration: Candidate13

Candidate13 supersedes Candidate11 as the current source preview. Shared
DynamicMount uses a compact .24 radial margin (previously .18), retaining the
.08 extra underside margin. The same constants drive the measured aperture
and ramp tangents. The measured surface constraint releases with
`(1-t)^3 * (1+3*t)`: bending begins earlier, while the actual outer-rim residual
has zero position/slope/curvature and the inner correction vanishes through
its second derivative. This reduces the late, concentrated turn of Candidate11.
The retained original pelvic vertices, shared waist and original seam donors
remain exact. No normal-only repair or retained-body lifting was reintroduced.

Candidate12 established this geometry. Its eight size/angle replays show an
outer perimeter increase of about 4-5%. The fitted maximum-size anchor moves
up by .6051 source units and forward by .14072; the default-size anchor is
unchanged. Level-angle shaft circumference is exact; tilted rim perimeter
changes by up to 1.84% through the existing clearance solve, with no reduction.
These are measured changes, not a claim that every attachment dimension is
unchanged. See candidate-12/geometry-comparison.json in the private evidence.

The motion test exposed an expensive full-body ray search for every generated
vertex when rebuilding after angle changes. Candidate13 indexes forward body
triangles in a projected spatial chart. The original exact projection,
barycentric tolerance and original-face tie order remain authoritative. Tests
cover layered surfaces, exact original edges, rotations and coplanar ties.
Live reposing retains cached measured planes and the existing numerical solver.

Private evidence: D:/MaleModBuilds/modular-anatomy-20261015/root-seam-20261019/candidate-13.
Source and binary snapshots are in changed-sources.json and binary-hashes.json.
DLL: d5541ecacccb9f5ba6eb655bc25ca39c2cf313ff9360076e362af31dbcaa4b1c.
Probe: 3785387715388bfd29540f4a8a8cb09b43dc0ac865946175d21fe510ce51d41e.
Eight C++ tests and 40 prior-pose replays pass. All 40 exported geometry and
native attribute sets are byte-identical to Candidate12 after spatial indexing.
The separate replay runs show median rebuild cost 4739 to 2222 ms; they ran
alongside other work and are not an FPS benchmark. No installation, new
dependency pin or commit has been made. The adapter builds this explicit dirty
Base diagnostic; other spokes must adopt a tested shared commit through their
pins and independently review native attachment integrity. Material charts and
maps are unchanged, so the previous close-view material boundary is not claimed
eliminated by this geometric refinement.

Seventy-two Candidate13 native views were captured and inspected: 24 skin/shape
views, 36 angle/pause sequence views and 12 pause/angle/resumed-walking views.
Both body resources drew, with completed LOD mask 5. The root transition is
rounder in the sampled views; no new root gap or folded ramp was visible. Shaft
and lobe motion continued, and the resumed walk changed actor position. Angle
rebuilds remain costly: median 1246 ms in the angle sequence versus 2630 ms in
Candidate12. Cached resumed-walk evaluations measured 74.735 ms. This is not a
real-time performance acceptance or an assertion of equal workload/FPS.

All three exact owned sessions were closed. All 31 protected and 53 installed
files were verified exact; only engine-reordered input.settings lines needed
restoration after checking identical section/line multisets. See
preview-review.json, projection-equivalence.json, the three *-evidence.json
files and preservation reports. The fresh full native mechanical-state/scale/
LOD matrix and campaign check remain unperformed for Candidate13. The full
attachment gate is false. These are preview renders before installation.

## Candidate11 checkpoint (superseded preview)

The user identified the root ramp/pelvis boundary as a mesh problem. Earlier
acceptance of the installed seam is superseded. Installed DLL remains
fd95ca61331a983788208480b9ecd372b16d61d24751a3274eaaf64c240b0935; exact rollback remains
TheWitcher3MaleMod/local/native-backup-modular-20261019-114157.

## Current implementation

Candidate11 constructs the connector against measured original pelvic triangles.
BodySurfacePoint supplies an actual surface point and triangle plane;
RampEntryPoint releases that constraint with a quintic transition across the
connector. The correction and its first two derivatives vanish at the shaft
boundary. This changes vertex positions, not merely shader normals. Both
boundaries, shaft circumference and retained pelvic geometry are preserved.
There are 24 longitudinal intervals instead of 12.

The angle constraint now measures radius perpendicular to the shaft axis.
Previously the pelvis-plane constraint mistook the axial component of a tilted
shaft tangent for radius loss and flattened it. Live reposing uses the same
entry rule and cached planes without per-frame body raycasts. The Witcher
adapter carries endpoint shading normals through the geometric transition.
Textures, numerical worker, suspension geometry and material package are unchanged.

Shared geometry and tests belong in Base; native packing, captured poses and
REDengine evidence stay in the adapter. Other spokes must adopt a tested Base
commit through their pins and native gates. This preview upgrades no other game.
No commit, new dependency pin or installation was made. Preserve unrelated dirty
work in both repositories.

## Evidence and limits

Private evidence:
D:/MaleModBuilds/modular-anatomy-20261015/root-seam-20261019/candidate-11.
changed-sources.json and source-* preserve exact changed runtime/test files.
DLL: 96b29905a0c0e80093be96d1bad1e5a8ee375e4b2b0ed71510b69eb0026706c9.
Probe: 439268fc1b803900cd65adb31541db7a81e4dd55c019f847b1e2ebd61bb88816.

- Eight shared C++ tests pass, including measured-plane entry, exact retained
  body/module boundaries, circumference preservation and tangent release.
- Eight preliminary pose replays pass manifoldness, projected winding, unchanged
  original body and independent-motion assertions. The additional 32-pose audit also passes; exact cases and assertions are
  recorded in audit-full/audit.json. These are prior captured-pose replays,
  not 32 newly observed native cases.
- Twenty-four native grey-room views cover angle controls 15/25/50/90, Overall
  and Width 50/100, hang extremes and additional shape combinations. Both body
  resources drew; completed LOD mask was 5. This is not all-state/all-LOD coverage.
- All 24 views were inspected. The joining shape improves without the earlier
  rippled pelvic patch. A skin/material boundary remains discernible in close
  views; do not call the entire seam eliminated or the attachment gate passed.
  Candidate11 has not had its own motion/resume/distance/campaign matrix.
- Owned native session sealed-session-20261019-153558 was closed and captures
  archived. All 31 protected files were restored/verified exactly. The engine
  only reordered input.settings; section/line multisets were identical.
- Repository-wide provenance verification stopped on a pre-existing CRLF/LF-only
  motion_filter.hpp extraction mismatch. Candidate07/provenance-diagnosis.json
  records byte hashes and normalized equality. Do not claim that verifier passed
  or modify unrelated provenance to hide the failure.

Native views are review artifacts, not installation approval. Retain the accepted
runtime and exact rollback until full attachment validation and visual review.

## Rejected approaches

Normal-only work did not resolve the requested geometry. The custom dual-UV shader
rendered white/metal skin and is not deployable. Candidate03–08 lifted retained
pelvic triangles into a curved patch. Untextured review exposed ripples and severe
local triangle-plane differences despite zero projected face reversals.
Candidate08 retriangulation reduced the median discrepancy but did not eliminate
artifacts. These approaches are rejected and no longer active. Their 96-view
native runs must not count as Candidate11 validation.

Candidate09 constrained measured-plane release too narrowly. Candidate10 spread
it across the connector. Candidate11 adds longitudinal sampling and consistent
endpoint shading. Skin maps remain unchanged on this preview path. Earlier
trials remain private evidence only.
