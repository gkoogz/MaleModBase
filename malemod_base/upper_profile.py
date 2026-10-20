"""Bounded upper-arc compression against a measured extruded section.

The caller supplies a calibrated longitudinal/body-up chart and actual surface
samples. This never widens the section or moves its lower half. A positive
vertical derivative avoids the collapse caused by clamping vertices to a plane.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator
from .attachment_fields import smoother


def compress_upper_profile(points, section_center, section_axis, upper_radius,
                           samples, upper_heights, begin, end, locked=None):
    """Return source-chart points (X forward, Y lateral, Z up).

    ``samples``/``upper_heights`` are a measured upper meridian, not guessed
    anatomical landmarks. Original-edge donors must lie before ``begin`` or be
    locked together by the caller. Mesh validation remains a separate gate.
    """
    points = np.asarray(points, float)
    center, axis = np.asarray(section_center, float), np.asarray(section_axis, float)
    samples, heights = np.asarray(samples, float), np.asarray(upper_heights, float)
    if (points.ndim != 2 or points.shape[1] != 3 or center.shape != (3,)
            or axis.shape != (3,) or samples.ndim != 1 or len(samples) < 2
            or heights.shape != samples.shape or not np.isfinite(points).all()
            or not np.isfinite(center).all() or not np.isfinite(axis).all()
            or not np.isfinite(samples).all() or not np.isfinite(heights).all()
            or not np.isfinite([upper_radius, begin, end]).all()
            or upper_radius <= 0 or axis[0] <= .1 or end <= begin
            or np.any(np.diff(samples) <= 0)):
        raise ValueError('Invalid measured upper profile')
    locked = np.zeros(len(points), bool) if locked is None else np.asarray(locked, bool)
    if locked.shape != (len(points),):
        raise ValueError('Invalid profile exclusions')
    axis_height = center[2]+(samples-center[0])*axis[2]/axis[0]
    ratios = np.clip((upper_radius/axis[0])/np.maximum(heights-axis_height, 1e-8), .7, 1.)
    scale = PchipInterpolator(samples, ratios)(np.clip(points[:, 0], samples[0], samples[-1]))
    weight = smoother((points[:, 0]-begin)/(.12*upper_radius))
    weight *= 1-smoother((points[:, 0]-(end-.5*upper_radius))/(.5*upper_radius))
    height = points[:, 2]-center[2]-(points[:, 0]-center[0])*axis[2]/axis[0]
    weight *= smoother(height/(.5*upper_radius))
    weight[locked] = 0
    result = points.copy()
    result[:, 2] -= weight*(1-scale)*height
    return result
