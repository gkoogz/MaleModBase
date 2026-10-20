"""Material ownership and bounded support for attachment pose corrections.

Adapters supply observed interpolation roles and hard exclusions. These laws
do not infer body parts, units, skeletons or game capabilities.
"""
import numpy as np


def smoother(values):
    t = np.clip(np.asarray(values, float), 0, 1)
    return t**3 * (10 + t * (-15 + 6 * t))


def attachment_strength(attachment, shaft, transition=.20):
    a, s = np.asarray(attachment, float), np.asarray(shaft, float)
    if (a.ndim != 1 or s.shape != a.shape or not np.isfinite(a).all()
            or not np.isfinite(s).all() or not np.isfinite(transition) or transition <= 0):
        raise ValueError('Invalid measured material ownership')
    return smoother((a - s + transition) / (2 * transition))


def frame_follow(flex, reach=.30):
    flex = np.asarray(flex, float)
    if flex.ndim != 1 or not np.isfinite(flex).all() or not np.isfinite(reach) or reach <= 0:
        raise ValueError('Invalid measured frame influence')
    return smoother(flex / reach)


def neighbour_support(faces, active, fixed, rings=2):
    faces, active, fixed = np.asarray(faces), np.asarray(active, bool), np.asarray(fixed, bool)
    if (faces.ndim != 2 or faces.shape[1] != 3 or not np.issubdtype(faces.dtype, np.integer)
            or active.ndim != 1 or fixed.shape != active.shape or not len(faces)
            or faces.min() < 0 or faces.max() >= len(active)
            or not isinstance(rings, int) or rings < 0):
        raise ValueError('Invalid attachment support topology')
    result = active.copy(); result[fixed] = False
    for _ in range(rings):
        result[faces[result[faces].any(axis=1)].ravel()] = True
        result[fixed] = False
    return result
