"""Shared dimensionless classic jockstrap route steering.

Adapters supply observed axes, waist samples and measured under-glute targets.
This moves the side origin gently anterior and preserves the measured glute
crease. Dropping the route below that crease produces a thigh loop rather than
a supporting glute strap. No skeleton or source-to-SI conversion is invented.
"""
import numpy as np

CLASSIC_ROUTE_VERSION = 3
ANTERIOR_HIP_FRACTION = .12
LOWER_GLUTE_FRACTION = 0.


def classic_route_targets(waist_candidates, start, crease, medial,
                          forward_axis, superior_axis):
    contour = np.asarray(waist_candidates, dtype=float)
    targets = np.asarray([start, crease, medial], dtype=float).copy()
    forward = np.asarray(forward_axis, dtype=float)
    superior = np.asarray(superior_axis, dtype=float)
    if contour.ndim != 2 or contour.shape[1] != 3 or len(contour)<2 or not np.isfinite(contour).all() or not np.isfinite(targets).all():
        raise ValueError('Missing measured garment route calibration')
    if not np.isfinite(forward).all() or not np.isfinite(superior).all() or abs(np.linalg.norm(forward)-1)>1e-7 or abs(np.linalg.norm(superior)-1)>1e-7 or abs(forward@superior)>1e-7:
        raise ValueError('Garment route axes must be observed orthonormal directions')
    shift = ANTERIOR_HIP_FRACTION * np.ptp(contour@forward)
    measured_descent = (targets[0]-targets[1])@superior
    drop = LOWER_GLUTE_FRACTION * measured_descent
    if shift<=0 or measured_descent<=0:
        raise ValueError('Measured garment route does not span waist/glute')
    targets[0] += forward*shift
    targets[1] -= superior*drop
    targets[2] -= superior*(drop*.6)
    return targets, dict(version=CLASSIC_ROUTE_VERSION, anteriorHipFraction=ANTERIOR_HIP_FRACTION,
                         lowerGluteFraction=LOWER_GLUTE_FRACTION, measuredAnteriorShift=float(shift),
                         measuredCupDrop=float(drop), coordinateUnits='caller measured units')
