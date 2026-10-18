# Material-bound scrotal skin relief

The October 18 request is a neutral anatomical surface refinement: irregular
skin folds and a shallow longitudinal web beside the raphe, retaining the
complete shaft, independent suspension, rear attachment expansion and low-Hang
retraction. References inform appearance only, not anatomical validation.

`skin_relief.hpp` owns the SDK-free display-surface operation. `modular_asset.hpp`
derives the domain from existing lobe owners and scrotal connecting-skin lofts.
The shaft, pelvic ramp and body are excluded. A geodesic quintic fade reaches
zero at the shaft boundary; the lower retained-skin/loft join is continuous.
Topology, UV aliases, donor lineage and mechanical owners are unchanged.

Features use immutable material coordinates in the measured lobe/shaft frame.
Irregular transverse wrinkles transition to broader longitudinal neck folds.
A shallow tapered fold beside the central raphe fades into the same skin.
Local edge stretch, normalized by lobe scale, controls gathering and relaxation.
There is no time noise, smoothing of physics, new collision body or rigid ridge.
Broad web relief uses current geometric directions. Fine wrinkles use a bounded
normal field, preserving detail without corrugating thin cut triangles. A continuous
oriented-area bound protects broad relief without per-vertex clipping or discrete
amplitude jumps. Nearly coincident cut donors sample a continuous material-space
normal/strain field; the underlying surface and topology are never smoothed.
This local check is not a global self-intersection certificate.

The Witcher adapter transports its retained surface-normal detail through the
relief normal change and preserves existing texture/tangent/skin bindings.
The same operation is called for rest construction and cached live posing.
Worker forces, integration, cadence and rest/Hang placement are unchanged.

Adoption: pin the shared headers together, use actual semantic lobe and loft
owners, and propagate geometric normal changes in each renderer. Wolverine and
other spokes retain their installed runtime until their own native gate passes.
Do not copy this implementation into an adapter fork.

Verification status: seven focused C++ tests pass (material anchoring, scale,
rotation, gathering, protected boundary and orientation included). Native and
installation evidence will be recorded separately after observed review.
