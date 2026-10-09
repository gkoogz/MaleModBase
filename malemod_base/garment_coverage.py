"""SDK-free diagnostic orthographic coverage on actual measured mesh inputs.

Caller transforms positions to its measured Frame and divides by actual waist
circumference. Rays look from positive lateral X; forward/up define their Y/Z
plane. This face-centroid sample diagnoses openings; it does not certify all
fragments, native opacity, self contact, dynamic modesty or cloth containment.
"""
from __future__ import annotations
import numpy as np

def _points(value):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or (not np.isfinite(result).all()):
        raise ValueError('Measured finite three-component positions required')
    return result

def _mesh(vertices, faces):
    vertices = _points(vertices)
    faces = np.asarray(faces)
    if faces.ndim != 2 or faces.shape[1] != 3 or (not np.issubdtype(faces.dtype, np.integer)):
        raise ValueError('Actual integral triangle topology required')
    if faces.size and (faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError('Triangle exceeds measured vertices')
    return (vertices, faces)

def ray_depth(vertices, faces, query):
    """Return frontmost measured lateral depth at projected queries; -inf means no triangle hit."""
    (vertices, faces) = _mesh(vertices, faces)
    query = _points(query)
    if not len(query):
        return np.empty(0)
    if not len(faces):
        return np.full(len(query), -np.inf)
    t = vertices[faces]
    lo = np.minimum(t[:, :, 1:].min(1).min(0), query[:, 1:].min(0))
    hi = np.maximum(t[:, :, 1:].max(1).max(0), query[:, 1:].max(0))
    step = (hi - lo) / 80
    step = np.maximum(step, 1e-10)
    lower = np.floor((t[:, :, 1:].min(1) - lo) / step).astype(int).clip(0, 79)
    upper = np.floor((t[:, :, 1:].max(1) - lo) / step).astype(int).clip(0, 79)
    bins = {}
    for (k, (a, b)) in enumerate(zip(lower, upper)):
        for x in range(a[0], b[0] + 1):
            for y in range(a[1], b[1] + 1):
                bins.setdefault(x * 80 + y, []).append(k)
    cells = np.floor((query[:, 1:] - lo) / step).astype(int).clip(0, 79)
    code = cells[:, 0] * 80 + cells[:, 1]
    result = np.full(len(query), -np.inf)
    for cell in np.unique(code):
        qids = np.flatnonzero(code == cell)
        tri = t[bins.get(int(cell), [])]
        if not len(tri):
            continue
        (a, b, c) = (tri[:, 0], tri[:, 1], tri[:, 2])
        u = b[:, 1:] - a[:, 1:]
        v = c[:, 1:] - a[:, 1:]
        det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        good = np.abs(det) > 1e-14
        (a, b, c, u, v, det) = [p[good] for p in (a, b, c, u, v, det)]
        if not len(a):
            continue
        for start in range(0, len(qids), 256):
            ids = qids[start:start + 256]
            w = query[ids, 1:, None].transpose(0, 2, 1) - a[None, :, 1:]
            wb = (w[:, :, 0] * v[None, :, 1] - w[:, :, 1] * v[None, :, 0]) / det
            wc = (u[None, :, 0] * w[:, :, 1] - u[None, :, 1] * w[:, :, 0]) / det
            inside = (wb >= -1e-09) & (wc >= -1e-09) & (wb + wc <= 1 + 1e-09)
            x = a[None, :, 0] + wb * (b[:, 0] - a[:, 0])[None, :] + wc * (c[:, 0] - a[:, 0])[None, :]
            result[ids] = np.max(np.where(inside, x, -np.inf), axis=1)
    return result

def measure_side_coverage(anatomy_vertices, anatomy_faces, body_vertices, body_faces, cloth_vertices, cloth_faces, *, side=1):
    """Area-weighted actual centroid rays with self/body occlusion removed.

    Positions must already share measured circumference-normalized Frame axes.
    Side is +1 or -1 along that Frame's lateral axis, never an inferred game
    axis. Masks follow source triangle order for authoritative region summaries.
    """
    if side not in (-1, 1):
        raise ValueError('Measured lateral side must be -1 or +1')
    (av, af) = _mesh(anatomy_vertices, anatomy_faces)
    (bv, bf) = _mesh(body_vertices, body_faces)
    (cv, cf) = _mesh(cloth_vertices, cloth_faces)
    if not len(af):
        raise ValueError('Complete anatomy triangles required')
    triangles = av[af]
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    area = np.linalg.norm(normal, axis=1) * 0.5
    query = triangles.mean(axis=1)

    def sided(points):
        copy = points.copy()
        copy[:, 0] *= side
        return copy
    query = sided(query)
    self_depth = ray_depth(sided(av), af, query)
    body_depth = ray_depth(sided(bv), bf, query)
    cloth_depth = ray_depth(sided(cv), cf, query)
    visible = (normal[:, 0] * side > 1e-12) & (self_depth <= query[:, 0] + 1e-07) & (body_depth <= query[:, 0] + 1e-07)
    covered = visible & (cloth_depth >= query[:, 0] - 1e-07)
    total = float(area[visible].sum())
    return dict(visible=visible, covered=covered, exposed=visible & ~covered, triangleArea=area, visibleArea=total, coveredFraction=None if not total else float(area[covered].sum() / total), centroidSamplingNotCompleteCoverageProof=True)


def radial_coverage(surface, triangles, queries, *, radial_axes, height_axis, tolerance=1e-7):
    p=np.asarray(surface,float);t=np.asarray(triangles,int);q=np.asarray(queries,float)
    if len(set((*radial_axes,height_axis)))!=3:raise ValueError('Distinct frame axes required')
    x,y=radial_axes;angles=np.arctan2(p[:,y],p[:,x]);radii=np.hypot(p[:,x],p[:,y])
    theta=np.arctan2(q[:,y],q[:,x]);radius=np.hypot(q[:,x],q[:,y]);covered=np.zeros(len(q),bool)
    for face in t:
        a=angles[face];a=a[0]+(a-a[0]+np.pi)%(2*np.pi)-np.pi;z=p[face,height_axis]
        h=(theta-a[0]+np.pi)%(2*np.pi)-np.pi+a[0]
        candidate=(~covered)&(q[:,height_axis]>=z.min()-tolerance)&(q[:,height_axis]<=z.max()+tolerance)&(h>=a.min()-tolerance)&(h<=a.max()+tolerance)
        ids=np.flatnonzero(candidate)
        if not len(ids):continue
        m=np.array([[a[1]-a[0],a[2]-a[0]],[z[1]-z[0],z[2]-z[0]]])
        if abs(np.linalg.det(m))<1e-12:continue
        w=np.linalg.solve(m,np.array([h[ids]-a[0],q[ids,height_axis]-z[0]]));b=np.vstack((1-w.sum(axis=0),w))
        inside=np.all(b>=-tolerance,axis=0)&(radius[ids]<=radii[face]@b+tolerance)
        covered[ids[inside]]=True
    return covered
