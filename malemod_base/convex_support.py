"""Exact projection onto bounded, coplanar support halfspaces.

The unconstrained axial coordinate is preserved. Used by the offline attachment
fit to impose both clearance floors and a measured section envelope.
"""
import numpy as np


class PlanarHalfspaceProjector:
    """Closest points in planar halfspaces, including an open fillet wedge.

    Unlike a bounded section envelope, a surface junction has two unbounded
    outer rays. Project onto feasible supporting lines as well as their finite
    intersections; joining the intersections into a polygon would cap the rays.
    The orthogonal coordinate is preserved exactly up to roundoff.
    """
    def __init__(self, groups):
        rows = []
        for normals, offsets in groups:
            normals, offsets = np.asarray(normals, float), np.asarray(offsets, float)
            if (normals.ndim != 2 or normals.shape[1] != 3 or len(normals) < 3
                    or offsets.shape != (len(normals),)
                    or not np.isfinite(normals).all() or not np.isfinite(offsets).all()
                    or not np.allclose(np.linalg.norm(normals, axis=1), 1)):
                raise ValueError("Support requires finite offsets and unit normals")
            _, singular, basis = np.linalg.svd(normals, full_matrices=False)
            if singular[1] < 1e-8 or singular[2] > 1e-8*singular[0]:
                raise ValueError("Support normals must span one plane")
            tolerance = 1e-10*max(1., np.max(abs(offsets)))
            vertices = []
            for i in range(len(normals)):
                for j in range(i):
                    dot = normals[i]@normals[j]
                    if 1-dot*dot < 1e-10:
                        continue
                    weights = np.linalg.solve([[1, dot], [dot, 1]], offsets[[i, j]])
                    vertex = weights@normals[[i, j]]
                    if np.min(normals@vertex-offsets) >= -tolerance:
                        vertices.append(vertex)
            if not vertices:
                raise ValueError("Support halfspaces have no feasible intersection")
            rows.append((normals, offsets, np.asarray(vertices), basis[2], tolerance))
        self.rows = rows
        count = len(rows)
        planes = max((len(row[0]) for row in rows), default=0)
        corners = max((len(row[2]) for row in rows), default=0)
        self.normals = np.zeros((count, planes, 3))
        self.offsets = np.zeros((count, planes))
        self.plane_valid = np.zeros((count, planes), bool)
        self.vertices = np.zeros((count, corners, 3))
        self.corner_valid = np.zeros((count, corners), bool)
        self.axes = np.zeros((count, 3))
        self.tolerance = np.zeros(count)
        for i, (normals, offsets, vertices, axis, tolerance) in enumerate(rows):
            self.normals[i, :len(normals)] = normals
            self.offsets[i, :len(normals)] = offsets
            self.plane_valid[i, :len(normals)] = True
            self.vertices[i, :len(vertices)] = vertices
            self.corner_valid[i, :len(vertices)] = True
            self.axes[i] = axis
            self.tolerance[i] = tolerance

    def project(self, points):
        points = np.asarray(points, float)
        if points.shape != (len(self.rows), 3) or not np.isfinite(points).all():
            raise ValueError("Each support group requires one finite point")
        if not len(points):
            return points.copy()
        deficit = self.offsets-np.einsum('npc,nc->np', self.normals, points)
        inside = ((deficit <= 0) | ~self.plane_valid).all(1)
        projected = points[:, None]+deficit[:, :, None]*self.normals
        feasible = ((np.einsum('nqc,npc->nqp', self.normals, projected)
                     >= self.offsets[:, :, None]-self.tolerance[:, None, None])
                    | ~self.plane_valid[:, :, None]).all(1) & self.plane_valid
        axial = np.sum(points*self.axes, axis=1)[:, None]*self.axes
        candidates = np.concatenate([projected, self.vertices+axial[:, None]], axis=1)
        valid = np.concatenate([feasible, self.corner_valid], axis=1)
        distance = np.where(valid, np.sum((candidates-points[:, None])**2, axis=2), np.inf)
        result = candidates[np.arange(len(points)), distance.argmin(1)]
        return np.where(inside[:, None], points, result)


class PlanarSupportProjector:
    def __init__(self, groups):
        polygons, axes, all_normals, all_offsets = [], [], [], []
        for normals, offsets in groups:
            normals = np.asarray(normals, float)
            offsets = np.asarray(offsets, float)
            if (normals.ndim != 2 or normals.shape[1] != 3
                    or offsets.shape != (len(normals),)
                    or not np.isfinite(normals).all()
                    or not np.isfinite(offsets).all()
                    or not np.allclose(np.linalg.norm(normals, axis=1), 1)):
                raise ValueError("Support requires finite offsets and unit normals")
            _, singular, basis = np.linalg.svd(normals, full_matrices=False)
            if singular[1] < 1e-8 or singular[2] > 1e-8*singular[0]:
                raise ValueError("Envelope support normals must span one plane")
            tolerance = 1e-10*max(1., np.max(abs(offsets)))
            vertices = []
            for i in range(len(normals)):
                for j in range(i):
                    dot = normals[i]@normals[j]
                    if 1-dot*dot < 1e-10:
                        continue
                    weights = np.linalg.solve([[1, dot], [dot, 1]],
                                              offsets[[i, j]])
                    point = weights@normals[[i, j]]
                    if np.min(normals@point-offsets) < -tolerance:
                        continue
                    if not any(np.linalg.norm(point-v) < tolerance for v in vertices):
                        vertices.append(point)
            if not vertices:
                raise ValueError("Clearance floor conflicts with section envelope")
            vertices = np.asarray(vertices)
            center = vertices.mean(0)
            uv = (vertices-center)@basis[:2].T
            vertices = vertices[np.argsort(np.arctan2(uv[:, 1], uv[:, 0]))]
            # A bounded polygon has supporting directions around the full plane.
            # Reject unbounded intersections; their edge projection is different.
            angles = np.sort(np.arctan2(normals@basis[1], normals@basis[0]))
            if np.max(np.diff(np.r_[angles, angles[0]+2*np.pi])) >= np.pi-1e-10:
                raise ValueError("Section envelope must be bounded")
            polygons.append(vertices)
            axes.append(basis[2])
            all_normals.append(normals)
            all_offsets.append(offsets)
        count = len(polygons)
        max_edges = max(map(len, polygons), default=0)
        max_planes = max(map(len, all_normals), default=0)
        self.starts = np.zeros((count, max_edges, 3))
        self.edges = np.zeros_like(self.starts)
        self.normals = np.zeros((count, max_planes, 3))
        self.offsets = np.full((count, max_planes), -np.inf)
        self.axes = np.asarray(axes).reshape(-1, 3)
        for i, polygon in enumerate(polygons):
            self.starts[i] = polygon[0]
            self.starts[i, :len(polygon)] = polygon
            self.edges[i, :len(polygon)] = np.roll(polygon, -1, axis=0)-polygon
            self.normals[i, :len(all_normals[i])] = all_normals[i]
            self.offsets[i, :len(all_offsets[i])] = all_offsets[i]
        self.lengths2 = np.sum(self.edges*self.edges, axis=2)

    def project(self, points):
        points = np.asarray(points, float)
        inside = (np.einsum('npc,nc->np', self.normals, points) >= self.offsets).all(1)
        axial = np.sum(points*self.axes, axis=1)[:, None]*self.axes
        planar = points-axial
        delta = planar[:, None]-self.starts
        t = np.sum(delta*self.edges, axis=2)/np.maximum(self.lengths2, 1e-30)
        candidates = self.starts+np.clip(t, 0, 1)[:, :, None]*self.edges
        nearest = np.argmin(np.sum((candidates-planar[:, None])**2, axis=2), axis=1)
        projected = candidates[np.arange(len(points)), nearest]+axial
        return np.where(inside[:, None], points, projected)
