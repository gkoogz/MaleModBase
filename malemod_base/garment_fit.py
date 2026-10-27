"""Small, measured clearance corrections for a rebound garment shell.

Offsets are expressed in each target triangle's outward (edge, bitangent,
normal) frame. Move each connected shell component by one common normal
offset, preserving its layer separation and every tangential coordinate.
This is a rest-fit operation, independent of garment simulation.
"""
import numpy as np


def clear_rebound_shell(offsets, faces, first, last, minimum):
    values = np.asarray(offsets, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if (values.ndim != 2 or values.shape[1] != 3 or
            not np.isfinite(values).all() or not np.isfinite(minimum) or
            minimum < 0 or not 0 <= first < last <= len(values) or
            faces.ndim != 2 or faces.shape[1] != 3 or
            np.any(faces < 0) or np.any(faces >= len(values))):
        raise ValueError('Invalid rebound shell clearance inputs')
    parent = np.arange(last-first)

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for face in faces[(faces.min(1) >= first) & (faces.max(1) < last)]:
        a, b, c = face-first
        parent[root(b)] = root(a)
        parent[root(c)] = root(a)
    groups = {}
    for i in range(last-first):
        groups.setdefault(root(i), []).append(first+i)
    result = values.copy()
    shifts = []
    for ids in groups.values():
        shift = max(0., minimum-float(values[ids, 2].min()))
        result[ids, 2] += shift
        shifts.append(shift)
    return result, dict(components=len(groups), maximumNormalShift=max(shifts),
                        minimumClearance=float(result[first:last, 2].min()))
