# October 14: endpoint-constrained collar native preview

The adult anatomical teaching context remains in PROJECT-CONTEXT.md. This
supersedes the October 13 statement that the authored repair is unimplemented.

## Implementation

The saddle combines upper-root displacement blending, surface fairing and
adapter displacement attenuation. Moving the pivot or releasing the waist
alone did not repair the resulting surface. The current candidate transports
the authored final sewn-patch repair in its measured material frame, preserving
the opening, original-edge donors, protected tissue, UV aliases and body parts.

`attachment_corrective.hpp` distinguishes evaluated endpoints from continuous
activation paths. It can invoke a repair callback for an invalid endpoint and
independently validates the result. `endpoint_constraints.hpp` eliminates nested
seam donors into independent coordinates; preserves local area, adjacent-face
and native-body orientation; bounds steps; and solves coupled active constraints
together when sequential projection cycles. Failure returns the baseline.

`section_clearance.hpp` measures the upper clearance deficit against the
extruded shaft section. This supplements radius-based activation: size 75 at
angle 15 needs full correction; the previous size-only fade supplied about 41%
and retained the notch. Neutral-size and angle-50 baseline behavior stay separate.
This is measured geometry, not clinical validation. Witcher owns MMSEWN02 loading,
controls, resource layouts, calibration and rendering. Native tests used the
installed source worker; rejected stock-anchor/source-worker experiments are off.

## Evidence

Private root: `D:/MaleModBuilds/witcher-anchor-native-20261013`.
Licensed geometry, captures, packages and user data remain outside Git.

* `sewn-matrix-1791959721972536800`: 63 source cases / 126 LOD results;
  zero new fold pairs/collapsed faces, exact protected/excluded rows, maximum
  donor error 4.914e-13 source units. Offline native-C++ evaluation only.
* `sewn-extra-final`: 12 additional maximum-size cases at angles 20/30/40/45
  across three states; both LODs accepted full correction.
* Standalone C++ corrective tests and 31 relevant Python tests passed.
* Full-source AddressSanitizer build passed composed max-size angle-15 output
  and two 48-pose sweeps with concurrent LOD evaluation. The sweep does not
  compose every render buffer. One earlier noninstrumented offline probe access
  violation was not reproduced or conclusively explained; do not claim it fixed.
* Witcher `sealed-session-20261014-025215`: 57 native views / 19 cases;
  angles 10/15/20/25/35/50/70/90, minimum/default/intermediate/maximum sizes,
  mixed width, three states, walking. Calibrated side/front/oblique cameras.
  Both body resources and both LOD buffers recorded; movement 4.10845 m.
  This is not independent visual observation of every rendered LOD or frame.
* Median source/target evaluations 45.73/61.42 ms; recorded presentation interval
  median 116.41 ms is not FPS. No performance-parity claim.
* Session closed, 24 original save files verified, audio restored, room files
  removed. Normal installation, dependency pin and exact rollback retained.

DLL SHA256: `f9274f42ce0845b61801924240452ba9d96ee1162d0edce9c63734e3686a0741`.
Field SHA256: `2758de0522e4dcf5ff49f4b32f38e96e801839d790e5e8700bbf81185cfe4082`.
Recorder `baseCommit` identifies the binding/source-worker pin, not a committed
revision of the dirty experimental headers.

## Visual status and adoption

Reviewed all three native contact sheets and full-resolution side captures at
15 and 25. The deep open cleft is closed in the reviewed raised-angle views,
including intermediate-size views. A narrow upper crease/ridge remains visible.
Do not call the attachment completely smooth or the full gate passed. New
campaign checks, all control combinations, independent visual LOD tests and
performance parity have not passed. Present actual renders before replacement.

The zero-new-fold statistic does not imply a globally fold-free surface. A
follow-up absolute adjacency measurement on size 75 / angle 15 / LOD0 found
inherited sharp seam pairs, including faces 767/14832 (cosine -0.9331) and
392/14773 (-0.8466). These were not protected by the original-acute-pair test.
Future work must address the seam's tangent continuity explicitly and retain
absolute seam-angle evidence; another global smoothing strength change or a
zero-new-fold report alone cannot establish visual acceptance.

Private `seam-tangent-fit` then tested body-plane tangent targets with fixed
boundary circumference on size 75 / angle 15. Two-ring constrained continuation
completed all eight stages on both LODs (223 editable rows each). Its actual
mesh side render still retains the upper crease, so this experiment was not
adopted into the artifact or runtime. Completion of its optimizer is not visual
success. The 57-view native candidate and its recorded hashes remain current.

The candidate is a private playable diagnostic preview, not a normal install or
release. Preserve accepted runtime/rollback. Finish the visual/campaign gates,
isolate shared API/tests in a clean Base revision, deliberately pin it in Witcher
with matching geometry, and package the hash-bound field. Wolverine and every
future spoke consume the same API through tested Base pins, supplying their own
measured character data. Do not copy Geralt's calibration to another character.
