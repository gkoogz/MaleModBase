# Root section motion continuity

The October 17 walking report identifies a step between the body attachment and
the moving shaft. The retained material surface used Hermite center positions
but independent SLERP orientation on each guide interval. Position continuity
did not imply continuity of the surface differential: angular velocity changed
at every guide knot and jumped from zero at the fixed root. Section radius
amplified this into longitudinal compression on one side.

`MaterialSection` now uses normalized quaternion Hermite interpolation with a
shared arc-length derivative at each interior knot and zero angular derivative
at the fixed root and rigid crown. It retains an orthonormal frame and therefore
does not blend or shrink section radii. Guide positions, numerical dynamics,
collar geometry, neutral pelvis, and independent suspension are unchanged.
The rest and live material maps use the same interpolant, preserving rest shape.

The regression in `modular_motion_test.cpp` evaluates left/right derivatives of
the surface around sixteen radial samples at the root and ten guide junctions,
in addition to existing independent ownership, section diameter and crown tests.
Four previously captured native poses replay without inverted collar triangles,
nonmanifold additions, or changes to the original pelvis. These are offline
checks; native visual review and installation are recorded separately in the
Witcher adapter. Other spokes can adopt this shared header through their own
pinned modular dependency and must run their native attachment gate.
