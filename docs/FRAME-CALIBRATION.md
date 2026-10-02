# Animated frame calibration and source motion

`surface::CoordinateCalibration::SkinDeltaToSource` conjugates an affine native
model-space skin delta through the complete measured source-to-target map.
For `f(p) = targetRoot + scale * basis * (p - sourceRoot)`, its result is
`inverse(f) * nativeDelta * f`. Rotating/scaling translation alone omits the
different model origins and produces artificial velocity and acceleration.
The input is a skin delta (current model-space bone pose times its native
inverse bind), not a bone pose or actor world transform. The adapter owns those
observed matrices. Row-major affine matrices and explicit point/vector/length
operations prevent accidental translation of normals or radii.

`motion::Tracker` is generated from the active immutable Wolverine function
`TrackCharacterMotionMatrices`. The numerical expressions are unchanged. The
clock is supplied in unsigned 32-bit milliseconds, diagnostics are removed and
state is per instance. Nonfinite inputs fail before state changes. The original
minimum interval, long-gap/teleport reset, warmup, velocity/acceleration/angular
filters, clamps and quiet threshold are preserved, including timer wrap.
`TrackPoseOnly` provides the identical force law while marking unavailable thigh
skin matrices as unavailable. It does not supply invented character contacts.

Run `python tools/extract_motion_filter.py` to verify the generated header and
provenance and recreate the ignored original-function oracle. CMake test
configuration runs this verification automatically. x86 and x64 tests compare
2,000 complete filter states bit for bit with the original function, exercise
400 force-only states, and verify 500 calibrated affine transforms. The binding
regression also passes. These are source and offline results, not live output.

## Adoption

Witcher must pin this committed Base revision, consume the measured native
inverse bind and actor-local pose, then pass the calibrated delta to this
filter. Character collision envelopes remain separate measured adapter data;
the unavailable Geralt pelvis envelope must not be replaced with Wolverine's
radius. The current read-only probe has measured animated pelvis/thigh poses,
but this filter has not yet driven Witcher's live solver or vertices.

Wolverine can later replace its existing function with this generated filter
using its present skin matrices and clock, retaining original numerical behavior
and engine adapters. That adoption requires native replay and gameplay gates
and is not performed here. Every future spoke uses the same shared filter and
full coordinate mapping with independently measured character inputs.
