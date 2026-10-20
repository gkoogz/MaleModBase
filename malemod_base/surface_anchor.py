"""Rest-surface anchor measurement, with original triangle provenance.

The adapter supplies a calibrated intended location and inward direction on
the unmodified body. This selects a surface point, not a bone or an anatomical
landmark. Reuse one canonical binding across detail levels; independently
averaging cut-loop vertices makes placement depend on tessellation.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class SurfaceAnchor:
    triangle: int
    donors: tuple
    weights: tuple
    distance: float

    def evaluate(self, positions):
        p = np.asarray(positions, dtype=float)
        if (p.ndim != 2 or p.shape[1] != 3 or max(self.donors) >= len(p)
                or not np.isfinite(p[list(self.donors)]).all()):
            raise ValueError('Anchor donor positions are unavailable')
        return np.asarray(self.weights) @ p[list(self.donors)]


def measure_anchor(positions, triangles, origin, inward, max_distance):
    """Nearest forward ray hit, retaining exact original triangle donors.

    max_distance uses the caller's units and prevents selecting a remote body
    surface. Direction magnitude does not affect the result. Mesh winding is
    irrelevant; collapsed and ray-parallel triangles cannot provide a hit.
    """
    p = np.asarray(positions, dtype=float)
    f = np.asarray(triangles)
    o = np.asarray(origin, dtype=float)
    direction = np.asarray(inward, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all()
            or f.ndim != 2 or f.shape[1] != 3 or not len(f)
            or not np.issubdtype(f.dtype, np.integer)
            or f.min() < 0 or f.max() >= len(p)
            or o.shape != (3,) or not np.isfinite(o).all()
            or direction.shape != (3,) or not np.isfinite(direction).all()
            or not np.isfinite(max_distance) or max_distance <= 0):
        raise ValueError('Invalid calibrated surface anchor inputs')
    length = np.linalg.norm(direction)
    if length == 0 or not np.isfinite(length):
        raise ValueError('Anchor direction is degenerate')
    direction = direction / length
    a = p[f[:, 0]]
    e1 = p[f[:, 1]] - a
    e2 = p[f[:, 2]] - a
    h = np.cross(direction, e2)
    determinant = np.einsum('ij,ij->i', e1, h)
    area = np.linalg.norm(np.cross(e1, e2), axis=1)
    usable = (area > 0) & (np.abs(determinant) > area * 1e-12)
    ids = np.flatnonzero(usable)
    if not len(ids):
        raise ValueError('Anchor ray does not intersect the supplied surface')
    q = o - a[ids]
    u = np.einsum('ij,ij->i', q, h[ids]) / determinant[ids]
    cross = np.cross(q, e1[ids])
    v = cross @ direction / determinant[ids]
    distance = np.einsum('ij,ij->i', e2[ids], cross) / determinant[ids]
    inside = ((u >= -1e-10) & (v >= -1e-10) & (u+v <= 1+1e-10)
              & (distance >= -max_distance*1e-12) & (distance <= max_distance))
    if not inside.any():
        raise ValueError('Anchor ray misses the bounded surface region')
    hits = np.flatnonzero(inside)
    k = hits[np.argmin(distance[hits])]
    face = int(ids[k])
    weights = np.maximum([1-u[k]-v[k], u[k], v[k]], 0)
    weights /= weights.sum()
    return SurfaceAnchor(face, tuple(map(int, f[face])), tuple(weights),
                         float(max(0, distance[k])))
