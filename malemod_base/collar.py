"""Offline coupled pelvic collar, derived from Wolverine's active UnifiedCollar.

All inputs use one measured module authoring frame (+X forward, +Y lateral,
+Z body up). `source_length_scale` is supplied explicitly by a character fit;
source units are not assumed to be meters. The scipy factorization is per plan.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CollarFrame:
    root: tuple
    axis: tuple
    up: tuple
    radius: float
    length: float
    source_length_scale: float

    def canonical(self):
        scale = float(self.source_length_scale)
        vectors = np.array([self.root, self.axis, self.up], dtype=np.float64)
        if vectors.shape != (3, 3) or not np.isfinite(vectors).all():
            raise ValueError('Expected finite root/axis/up vectors')
        if not np.isfinite([scale, self.radius, self.length]).all() or min(scale, self.radius, self.length) <= 0:
            raise ValueError('Measured scale, radius and length must be positive')
        axis, up = vectors[1], vectors[2]
        if (abs(np.linalg.norm(axis) - 1) > 1e-5 or abs(np.linalg.norm(up) - 1) > 1e-5 or
                abs(axis @ up) > 1e-5 or abs(axis[1]) > 1e-5 or abs(up[1]) > 1e-5):
            raise ValueError('Axis/up must be orthonormal in the module XZ plane')
        return vectors[0] / scale, axis, up, self.radius / scale, self.length / scale, scale


def smoother(value):
    value = np.clip(value, 0, 1)
    return value * value * value * (value * (value * 6 - 15) + 10)


def _points(value):
    value = np.asarray(value, dtype=np.float64)
    if value.ndim != 2 or value.shape[1] != 3 or not len(value) or not np.isfinite(value).all():
        raise ValueError('Expected finite Nx3 points')
    return value


def recruitment(points, frame):
    """Active-source expanding support field, including dorsal/ventral guards."""
    root, axis, up, radius, length, scale = frame.canonical()
    p = _points(points) / scale
    q = p - root
    s, y, z = q @ axis, q[:, 1], q @ up
    rho = np.sqrt(y * y + z * z)
    upper = (z / np.maximum(rho, 1e-8) + 1) * .5
    growth = smoother((radius - 2.9) / 4.72)
    reach = 5 + growth * (2.5 + 2.5 * upper)
    w = smoother((s + reach) / 3) * (1 - smoother((s / length - .12) / .26))
    w *= 1 - smoother((rho - (radius * 1.55 + 2)) / 3)
    ventral_reach = 4 + 4 * smoother((radius - 2.7) / 1.1)
    w *= smoother((p[:, 0] - (2 + ventral_reach * (1 - upper))) / 3)
    w *= smoother((p[:, 2] - (root[2] - radius * 2.1 - 2)) / 4)
    return w


def radial_targets(points, frame, mask, guide_centers, guide_radii):
    """Source barrel/ventral-guide target law. Guide data comes from the module.

    Caller provides one guide center (including Raphe depth) and unexpanded
    guide radius per point. This does not invent or replace the source guide.
    """
    root, axis, up, radius, length, scale = frame.canonical()
    original = _points(points)
    p = original / scale
    centers = _points(guide_centers) / scale
    radii = np.asarray(guide_radii, dtype=np.float64) / scale
    mask = np.asarray(mask, dtype=np.float64)
    if (centers.shape != p.shape or radii.shape != (len(p),) or mask.shape != radii.shape or
            not np.isfinite(radii).all() or not np.isfinite(mask).all() or np.any(radii < 0) or
            np.any((mask < 0) | (mask > 1))):
        raise ValueError('Guide/mask dimensions or values invalid')
    q = p - root
    s, y, z = q @ axis, q[:, 1], q @ up
    radial = np.sqrt(y * y + z * z)
    dy, dz = y / np.maximum(radial, 1e-8), z / np.maximum(radial, 1e-8)
    parameter = s / length
    tube_radius = radii * (2 - smoother((parameter - .04) / .34))
    cy, cz = (centers - root)[:, 1], (centers - root) @ up
    dot = dy * cy + dz * cz
    discriminant = tube_radius**2 - cy**2 - cz**2 + dot**2
    bottom = np.where((discriminant > 0) & (dot > 0), dot + np.sqrt(np.maximum(discriminant, 0)), 0)
    growth = smoother((radius - 2.9) / 4.72)
    barrel = radius * 1.025 + (.06 + .12 * growth) * radius * (1 - smoother((parameter + .03) / .26))
    gap = np.maximum(0, np.maximum(barrel, bottom) - radial) * mask
    gap[radial < 1e-8] = 0
    delta = (np.array([0, 1, 0]) * dy[:, None] + up * dz[:, None]) * gap[:, None]
    delta[:, 0] = np.maximum(delta[:, 0], delta[:, 0] * (1 - smoother((-dz - .05) / .60)))
    return original + delta * scale


class CollarPlan:
    """Per-character cached screened biharmonic solve with hard edge welds.

    Seam rows are (slave, original edge endpoint A, endpoint B, interpolation).
    A render adapter expands unique solved vertices to its UV-split aliases.
    Rebuild the plan for changed topology, pose metric or recruitment radius.
    """
    def __init__(self, points, triangles, seams, frame, locked_vertices=()):
        from scipy import sparse
        from scipy.sparse.linalg import splu

        _, _, _, _, _, self.scale = frame.canonical()
        self.before = _points(points).copy() / self.scale
        n = len(self.before)
        faces = np.asarray(triangles)
        if (faces.ndim != 2 or faces.shape[1] != 3 or not len(faces) or
                not np.issubdtype(faces.dtype, np.integer) or faces.min() < 0 or faces.max() >= n):
            raise ValueError('Expected valid integer triangle indices')
        self.seams = []
        slaves = set()
        for x, a, b, weight in seams:
            if (any(not isinstance(i, (int, np.integer)) or i < 0 or i >= n for i in (x, a, b)) or
                    x in slaves or len({x, a, b}) != 3 or not np.isfinite(weight) or not 0 <= weight <= 1):
                raise ValueError('Invalid seam donor/weight or duplicate slave')
            self.seams.append((int(x), int(a), int(b), float(weight)))
            slaves.add(x)
        if any(a in slaves or b in slaves for _, a, b, _ in self.seams):
            raise ValueError('Seam donors must be independent masters')
        locked = set(locked_vertices)
        if any(not isinstance(i, (int, np.integer)) or isinstance(i, bool) or
               i < 0 or i >= n or i in slaves for i in locked):
            raise ValueError('Locked vertices must be valid independent masters')
        self.masters = np.array([i for i in range(n) if i not in slaves], dtype=np.int32)
        master_of = {int(i): j for j, i in enumerate(self.masters)}
        rows, cols, weights = list(self.masters), list(range(len(self.masters))), [1.] * len(self.masters)
        for x, a, b, w in self.seams:
            self.before[x] = self.before[a] * (1 - w) + self.before[b] * w
            rows.extend([x, x]); cols.extend([master_of[a], master_of[b]]); weights.extend([1 - w, w])
        self.projection = sparse.coo_matrix((weights, (rows, cols)), shape=(n, len(self.masters))).tocsr()
        p = self.before[faces]
        twice = np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1)
        valid = twice >= 1e-14
        faces, p, twice = faces[valid], p[valid], twice[valid]
        area = np.zeros(n)
        row, col, entries = [], [], []
        for j in range(3):
            a, b = faces[:, (j + 1) % 3], faces[:, (j + 2) % 3]
            w = np.sum((p[:, (j + 1) % 3] - p[:, j]) * (p[:, (j + 2) % 3] - p[:, j]), axis=1) / (2 * twice)
            w = np.clip(w, 0, 20)
            for r, c, value in [(a, a, w), (b, b, w), (a, b, -w), (b, a, -w)]:
                row.extend(r); col.extend(c); entries.extend(value)
            np.add.at(area, faces[:, j], twice / 6)
        area = np.maximum(area, .005)
        self.mask = recruitment(self.before * self.scale, frame)
        self.screen = area * (2.5 + 2 * (1 - self.mask)**4)
        curvature = sparse.coo_matrix((entries, (row, col)), shape=(n, n)).tocsr()
        projection = self.projection
        cp = curvature @ projection
        self.metric = (cp.T @ sparse.diags(1 / area) @ cp * 8 +
                       projection.T @ curvature @ projection * 2 +
                       projection.T @ sparse.diags(self.screen) @ projection).tocsc()
        active = (self.mask[self.masters] > 1e-4) & ~np.isin(self.masters, list(locked))
        self.free = np.flatnonzero(active)
        self.fixed = np.flatnonzero(~active)
        self.boundary = self.metric[self.free][:, self.fixed]
        self._factor = splu(self.metric[self.free][:, self.free].tocsc()) if len(self.free) else None

    def solve(self, targets):
        target = _points(targets) / self.scale
        if target.shape != self.before.shape:
            raise ValueError('Target topology differs from the cached plan')
        solved = self.before[self.masters].copy()
        if self._factor is not None:
            rhs = (self.projection.T @ (target * self.screen[:, None]))[self.free]
            rhs -= self.boundary @ self.before[self.masters[self.fixed]]
            solved[self.free] = self._factor.solve(rhs)
        expanded = self.projection @ solved
        if not np.isfinite(expanded).all():
            raise RuntimeError('Collar solve returned nonfinite positions')
        return np.asarray(expanded) * self.scale
