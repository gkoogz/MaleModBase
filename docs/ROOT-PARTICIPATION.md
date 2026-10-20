# Preserve simulated root rotation

The October 20 Witcher report described a rigid proximal quarter while the
distal shaft moved. An actual-worker replay confirms the mapping defect:
source root rotations reached 1.15 to 6.52 degrees across six state/size cases,
but the driven root remained at exactly zero degrees in every case.

`AnchorRootPhysics` replaced the root basis with the rest basis. Later,
`SocketMaterialState` replaced the complete first frame again. The bend-radius
limiter then started from that fixed tangent, distributing motion downstream.
The numerical solver was supplying root rotation; presentation discarded it.

The correction anchors position while retaining orientation. Socket resampling
rebases the whole live centerline at the attachment instead of replacing one
frame, avoiding a first-segment kink. The radial ramp's moving endpoint uses the
actual resampled socket tangent. Its body edge, original pelvis and rest shape
remain fixed. Section diameter, rest lengths and independent suspended-body
frames remain enforced. No numerical worker or force setting changes.

Regression exercises positive/negative rotation, three radii and three socket
stations: root rotation reaches the first free interval, circumference is
preserved, socket center stays anchored and suspended frames do not move.
Existing curved-section continuity, rest-shape and compression checks remain.

This is shared geometry behavior. Witcher adopts the pinned headers and performs
native motion/attachment review separately. Wolverine and future spokes require
their own adoption and native checks; their installed runtimes are unchanged.
Source tests alone are not installation or native appearance acceptance.
