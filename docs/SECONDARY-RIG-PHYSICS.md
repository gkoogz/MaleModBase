# Secondary motion on a fixed authored rig

The default-baseline revision exports the actual source mechanical guide with
the evaluated surface. `SampleGuide` and Python `sample_mechanical_guide` use
the source C1 Hermite interpolation, including its kinematic first interval.
Witcher now uses twelve physics stations independently of eight shaft skin
joints, the source zero-curvature bend target, uniform source segment lengths
and the exact exported raphe multipliers. The older curved-rest kernel remains
available for genuinely authored curved metrics, not for this reset baseline.

Contract 1 adds SDK-free kernels in `include/malemod/physics/rig_kernels.hpp`.
They adapt the active source solver's distance, Kelvin-Voigt bend, tension-only
suspension, transverse material shear, and contact recovery to point generalized
masses. A curved rest metric preserves an adapter's measured authored guide.
`ovoid_support.hpp` extracts the source tapered ovoid support using float Newton
arithmetic; `tools/extract_ovoid_support.py --check` verifies that extraction.

These kernels are **not full coupled solver parity**. They omit angular contact
effective mass, suspension arm torque, the Hermite guide reaction Jacobian,
pressure deformation, dynamic collar and full source surface skinning. Shared
coefficients come from `malemod_base.physics_controls`, with measured source to
target length conversion. Positions, radii and acceleration scale together;
XPBD compliance stays unchanged for the same inverse masses and timestep.

The Witcher adapter translates these pinned function bodies into WitcherScript,
samples actual pelvis/thigh bones and publishes unit-scale joint transforms.
Its calibration, fixed timestep, contacts, native binding and observed gameplay
are separate gates. `secondary_test` checks source support agreement, rest-bend
equivalence, suspension limits, symmetric reaction, contact recovery, length
conversion and a 600-step moving rod/lobe fixture. This does not measure the
WitcherScript runtime cost or establish gameplay success.

## Adoption

The pelvis-relative extension supplies `FrameAcceleration`, `FilterMotion` and
`IntegrateRelative`. Relative velocity damping is invariant under constant
world translation. Measured frame translation and angular acceleration, plus
centrifugal and Coriolis terms, supply inertia in local coordinates. An adapter
must rotate gravity and colliders into that same space and publish local points
directly, including render frames without a physics substep. Input smoothing,
acceleration limits and measured frame timing are explicit adapter calibration,
not source replay parity. No rest geometry, mass, bend or suspension coefficient
changes are required. Regression cases cover translation invariance, signed
acceleration response, angular terms and rotation covariance.

Wolverine's canonical runtime and install remain unchanged by this work, per the
user's October 1 clarification. It may later consume these headers in its adapter
after source replay and gameplay validation, with its existing full angular and
surface solver retained. Future spokes can consume the same versioned kernels
and coefficients with their own observed binding data. No automatic adoption or
copy into the immutable reference snapshot is authorized.
