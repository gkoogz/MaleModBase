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

The first native trial still showed a raised base. Tracing the final root rim
found clearance moved material as much as 0.711 guide intervals beyond its
assigned station. `DynamicMount` now binds the final cleared material once per
shape rebuild and reuses those bindings during motion. Rest geometry remains
identical; the live surface follows the frames at its actual location. The
native replay probe checks both properties. A separate tangent-plane experiment
produced inverted ramp triangles and was rejected before native installation.

Rebinding alone was also rejected: native steep-angle poses moved part of the
attached rim with a free guide and folded the ramp. The final construction clips
the material centerline at the frontmost complete socket circumference, then
resamples twelve sections from that physical anchor through the tip. The entire
attached rim is constrained to its rest shape. The first free interval moves
with zero initial angular strain and the radius bound below. Generated neck
directions and independently suspended objects retain their original reference
frames. Native temporal presentation uses the same final material frames and
ownership. Replay assertions require an unchanged zero-motion surface and an
unchanged attached rim under live motion, not merely welded positions.

The live rod reconstruction also enforces a finite bend radius. Fixed segment
length alone permitted the first segment to turn too sharply for its diameter.
Additional live turn is limited to `0.4 * segmentLength / sectionRadius`, with
half that interval at the clamped root. Authored rest curvature is transported
unchanged; the first free interval still moves, all segment lengths remain
fixed, and independent suspended frames are untouched. This is a material
presentation constraint, not a change to the pinned numerical worker or a claim
of full self-contact simulation. Unit tests cover length, turn bounds, active
root motion, independent suspension and exact rest preservation.
