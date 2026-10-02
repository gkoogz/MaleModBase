# Volume-bearing secondary bodies

`physics/rigid_cluster.hpp` adds a portable shape-matching body for a secondary
rig. The off-axis supports are actual numerical material points with mass,
history and velocity. Their accepted covariance supplies a proper 3D rotation,
including axial roll. A centerline tangent alone cannot supply this state.

The Horn quaternion fit uses the previous accepted quaternion and a positive
spectral shift; projection preserves the cluster centroid and every internal
pair distance. Supports must span three dimensions and have equal point masses.
The calling solver integrates them with its other particles, exchanges contact
and attachment reactions, fits after constraints, and reconstructs velocities
from the accepted positions. No engine or graphics SDK is required.

This is a new reduced numerical backend, inspired by the reference's material
core and generalized mass. It is not a byte-identical extraction of Wolverine's
complete pressure/suspension solver or full surface evaluator. Its provenance
and the 540-frame, three-axis rotation/shape regression are recorded separately.

Witcher supplies supports measured from its approved evaluated glans, retains
the original four distal station masses across eleven supports, couples the
body to the flexible shaft, and recovers the existing distal render joints from
one accepted material transform. The old shaft bindings remain the reference;
no .78 head-weight cutoff or redistributed shaft render knots is added.

The source suspended-skin side partition is separately exposed by
`motion_binding.source_lobe_partition`: source Smooth01 with
`max(.20, max(.85,2.70*BallShapeScale)*.13)`. Caller lateral coordinates must be
converted to the source length scale. This preserves a connecting web while
avoiding blending both independently moving lobe interiors through a wide
target-unit envelope.

Adoption: future spokes can use measured volumetric supports with their native
rigs. Wolverine can opt into this alternative secondary backend only after its
own native/gameplay comparison; its existing source, imported snapshot and
installed runtime are unchanged. The source surface evaluator remains the
reference for a complete deformation port. Numerical tests do not establish
collision coverage or live performance in any engine.
