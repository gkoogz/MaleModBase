# Follow the existing anatomy physics

The normal pouch update is kinematic skinning of a cached, successful taut wrap.
It reads the current anatomy pose every rendered frame. There is no separate
cloth simulation, timestep, lag filter or 30 Hz throttle. Walking remains the
rebuild operation when a binding or contact configuration stops being valid.

`include/malemod/garments/meridian_follow.hpp` is SDK-free. `SurfaceFollower`
stores each successful surface point in the sewn-boundary frame and its four
strongest measured rig frames. Inverse normalized distance gives the rig weights;
a cubic row-based blend retains the body attachment near the seam. Frames carry
orientation and size as well as position. Every evaluation starts from the
remembered material coordinates, preventing accumulated drift. Seam offsets
follow the current sewn boundary; the current collision-supported pole is exact.

Every frame checks whole triangles against current convex support hulls. Cached
plane indices are hints, never proof from an old pose. A missed hint searches
current planes. Minor contact changes get at most two projection passes and a
0.12 displacement limit in the existing adapter coordinate units. These passes
keep seam and pole fixed, reject collapsed or inverted correction triangles and
publish only a fully certified result. They do not walk paths. A failure asks the
adapter to rebuild. Gross frame scale changes also request rebuilding; the
adapter must explicitly invalidate bindings for preference and scene changes.

The Wolverine prototype supplies seven circular-section frames and two current
testicle frames from its existing physics/bone pose. Geometry binding revision5,
32 longitudes, 24 rows, fixed trim, UV aliases and normal/material update cadence
are preserved. Old interior donor skinning is skipped while cached bindings are
active, except during private raw capture. Numerical collision guides remain
unrendered. The previous continuity/repair path remains available for poses that
the full walker already rejected; an uncertified fallback is still reported as
such and is not a successful contact certificate.

Tests cover rigid rotation/translation, immediate independent rig translation
and orientation, exact seam/pole, 500 repeated evaluations without drift, scale
invalidation, stale-plane rejection, chord penetration, collapsed triangles,
bounded contact projection and transactional projection failure. Shared
clearance/runtime/continuity regressions also remain required.

Private CPU replays extract the exact selected build's post-skinning chart block.
Synthetic interpolation between historical captured poses is separately marked;
it is neither recorded intermediate motion nor native gameplay evidence. This
experiment's native and installation status belongs in HANDOFF, not inferred
from compilation or CPU timing. The complete attachment/campaign/LOD gate
remains mandatory and incomplete until observed. Cache deformation is not a
general cloth self-contact solver or a guarantee of geodesic tautness at every
intermediate pose.

This shared algorithm is the adoption path for every spoke. Future adapters
consume a tested committed Base pin plus versioned measured binding data and
provide coherent current-pose frames, invalidation and native verification.
The private Wolverine builder hashes its named dirty overlay separately from
the historical Base pin; this is not a release dependency update. Witcher
remains paused pending REDkit and its own measured/native integration.
