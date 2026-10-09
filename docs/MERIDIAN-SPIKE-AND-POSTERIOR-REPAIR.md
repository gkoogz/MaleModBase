# Pouch shape and posterior contact repair

Shared math remains in Base; adapters supply current collision frames and
calibrated source units. Wolverine adopts the tested revision through its lock.
Future spokes must supply their own anatomical anterior frame and capsule/skin
support extents, rather than copying Wolverine's calibration or skeleton.
Witcher adoption remains deferred.

The 32 longitude paths retain the 24 render rows. Curvature-weighted sampling
now shares normalized axial stations across neighboring rays. Each ray is
evaluated on its original taut envelope. This prevents neighboring triangles
from bridging unrelated heights around a lobe and requiring isolated large
radial corrections. Corrections are re-tautened before the next full-triangle
certificate. Required lifts spread to nearby rows and longitudes before the
next certificate instead of moving an isolated vertex. Final local reversal fairing accepts only moves that preserve
incident triangle separation and orientation. Sewn outline, pole, fixed trim,
UV aliases, topology, full-rate following and anatomy solver cadence remain.

`physics/anterior_capsule.hpp` supplies a one-sided anterior capsule envelope,
bounded recovery and relative normal-velocity projection. Wolverine evaluates
it against its existing measured thigh/pelvis calibration with current ovoid
support extents. Rear capsule contact/friction is relinquished to this guard.
Recovery shifts position history by the same amount, avoiding a projection
launch impulse; only entering relative normal velocity is removed. Free
tangential motion, suspension, paired contacts and shaft mechanics continue.

Regression cases include an isolated path return, fixed anchors, conservative
whole-face clearance, sub-row oblate detail, size-scaled capsule queries,
crouch/stand capsule sweeps, bounded corrections and preserved tangent/outward
velocity. Run the five meridian/anterior source tests with assertions enabled.
Private native poses/captures are evidence only and are excluded from Git.

Current private sixteen-pose replay reduced worst local-return measure from
4.04348 to 0.99914 source units (75.3 percent), with no outer failure. This is
not proof that every sharp shape is eliminated or that game FPS improves.
Full-solver forced-trap replay remains rearward with the guard disabled and
returns both supports forward with it enabled. Synthetic crouch capsules and
an injected trapped state are not observed native crouch-to-stand gameplay.
Native attachment/motion, all-max controls and campaign checks are required
before runtime acceptance; full attachment acceptance is not claimed here.
