# Longitudinal underside contour

The requested repair concerns the profile silhouette of the ventral raphe,
with the distal taper into the frenulum retained. It is separate from the
accepted hanging-sac repair and does not change its authored assets.

## Diagnosis and implementation

Captured zero-motion material geometry already contains a midshaft ventral
bulge: the maximum-size sample deepens from approximately 9.3 to 10.2 source
units before tapering distally. The same contour appears after undoing root
rotation at upward, level and downward angles. It is therefore present before
the modular physics transport. The retained source pipeline includes axial
support, tube-envelope and local triangle-backoff passes; their combined final
surface is the input, not a new oscillation introduced by the live rod.

`surface/raphe_profile.hpp` corrects the immutable material shape after source
transfer and before it is bound to the moving rod. It measures proximal ventral
crest depth from shaft-owned vertices, builds a rounded transverse crest at that
depth, and blends toward it only over the underside. The shift is bounded by
0.2 section radii. Quintic longitudinal/angular weights protect the socket,
dorsal and lateral surface, and restore the source's distal taper. Crown-owned
vertices, including the authored frenulum, remain exact. Suspended tissue is
excluded by semantic ownership; its connecting loft follows the adjusted shaft
endpoints using the existing tangent construction.

The first candidate reversed four/five small material triangles at the two
shortest-length settings. It was rejected before installation. The final
algorithm computes the first positive root of each triangle's oriented-area
quadratic along the correction. A single common amplitude preserves at least
25 percent of original oriented area with a small safety margin. This avoids
introducing ripples through independent per-vertex backoff. The focused test
includes an explicitly inverting correction and an unchanged safe correction.

Coordinates come from the current material guide, not world height. The
correction runs only during rest-morphology preparation. The existing physics
subsequently transports the corrected shape, so there is no force opposing
motion, no live straightening, and no extra per-frame profile solve. Geometry,
UV donors, native binding, pelvis, numerical worker and scrotal authoring remain
the accepted versions. This does not remove natural curvature during bending.

## Verification and adoption

The focused C++ suite in `tests/modular` covers a deliberately wavy crest,
protected root/dorsal/lobe/crown vertices, unchanged axial/lateral coordinates,
and rotation/translation/scale invariance. Material transfer and independent
motion regressions run alongside it. Native replay must additionally verify
the measured profile, original pelvis, collar faces and native bindings.

Witcher consumes the shared header through its modular dependency pin. It needs
a new native DLL but retains the accepted geometry/transfer bundle and worker.
Other spokes can adopt the same material-space correction only after providing
their measured shaft/crown ownership and guide frames, pinning Base, and passing
their native attachment gate; their installed runtimes are not changed here.

Private diagnostics and candidate evidence are under
`D:/MaleModBuilds/modular-anatomy-20261015/raphe-profile-20261018`.
The bounded replacement's evidence is in its `bounded` subdirectory; the parent
retains the rejected first candidate and its two detected profile regressions.
Native review and installation status are recorded in the adapter report;
unit tests and offline plots alone are not installation acceptance.
