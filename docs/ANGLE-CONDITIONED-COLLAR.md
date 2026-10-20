# Angle-conditioned collar preview

October 11, 2026: experimental, offline only. This is not an accepted attachment
repair and must not be installed or released as one.

The traced moderate-angle depression has two adapter contributors: fixed
material seam distances are blended over a growing posed-radius reach, and
absolute surface fairing contracts the recruited dorsal opening. At Angle25,
the selected triangle's adapter blend lowers its centroid 41.046 mm and the
final solve another 6.200 mm. The source raphe stage contributes zero there.

`include/malemod/surface/angle_conditioned_collar.hpp` supplies shared material
weights and a smooth corrective response. The Witcher preview additionally
solves with the recruited dorsal original-edge donors prescribed and with the
module handoff measured against the neutral opening. It takes only the positive
upward residual, weighted by the immutable dorsal material sector. The original
edge constraints and body orientation constraints are projected afterward.
This avoids applying the alternate solution's lateral contraction or forward
displacement directly. Constraint projection can still introduce small lateral
adjustments; this is not proof of circumference preservation in every case.

The existing 1..100 reference Angle control maps its lower half to
`degrees = -80 + 110*(ui-1)/49`. Upward response is quintic smoothstep of
`-degrees/25`, clamped to [0,1]. Horizontal/downward response is exactly zero;
full response begins at 25 degrees upward. The requested control drives this
response rather than an oscillating physics guide. Expansion response uses
the same neutral-radius interval as the existing recruitment law, avoiding a
large correction on neutral and smaller anatomy.

## Verification and limitations

Private reproducible evidence is in
`D:/MaleModBuilds/witcher-angle-response-20261011`:

- Unit tests cover bounds, continuity, slider mapping, material-sector
  preservation, and rest-space handoff endpoints. Standalone test passes.
- The canonical adapter evaluator compiles against the uninstrumented pinned
  Base graft source plus the new shared header. All 39 resulting native-buffer
  files equal the reviewed prototype byte for byte.
- Seven reported-shape angles (10,20,25,30,35,40,50), plus six state/size cases
  at Angle25, cover both LODs. Protected boundaries and shared waist are exact.
  Angle40 and Angle50 outputs equal the baseline. Maximum float-render seam
  gap is 1.139e-7 metres. No body faces reverse relative to baseline.
- Front/side/oblique offline renders show a reduced moderate-angle dip.
  An oblique transition ridge and a separate steep-angle source crease remain.
  The intermediate 75/75 case needs further LOD review: constraint projection
  changes horizontal positions by up to 1.386 mm in LOD1. Do not describe the
  preview as fully preserving circumference or resolving every saddle.
- The full Base CMake configuration is blocked by the existing unrelated
  motion-header extraction mismatch. That header was not regenerated.
- No native grey-room, animation, campaign, exhaustive controls, performance,
  clean install, or rollback run was performed. Attachment acceptance is false.

## Adoption

Witcher integration is guarded by `MALEMOD_ANGLE_CONDITIONED_COLLAR`; normal
builds do not enable it. The existing Base pin and installed files are unchanged.
The test overlay header is a hashed copy for this preview, not a maintained fork.
Before promotion, resolve the residual visual defects, commit the shared change,
pin it in the adapter with versioned binding provenance, and complete the native
attachment gate. Preserve the currently accepted runtime and exact rollback.

Wolverine and future spokes must consume the same shared implementation through
a Base revision, retain their own observed donors/calibration, and run their
native attachment matrix. Do not copy Geralt's vertex IDs or maintain a second
algorithm in a spoke.
