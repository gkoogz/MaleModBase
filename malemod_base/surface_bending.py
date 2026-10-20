"""Local bending checks on a joined surface, independent of its world pose.

For seam coverage, callers must split original edges at bound seam samples,
then weld their positional aliases in the diagnostic topology.
This detects new local folds; it does not certify global self-intersection.
"""
import numpy as np


def adjacent_faces(faces):
    faces = np.asarray(faces)
    if (faces.ndim != 2 or faces.shape[1] != 3 or not len(faces)
            or not np.issubdtype(faces.dtype, np.integer) or faces.min() < 0):
        raise ValueError('Invalid triangle indices')
    edges = np.sort(np.vstack([faces[:, [0, 1]], faces[:, [1, 2]],
                               faces[:, [2, 0]]]), axis=1)
    owners = np.tile(np.arange(len(faces)), 3)
    order = np.lexsort((edges[:, 1], edges[:, 0]))
    edges, owners = edges[order], owners[order]
    same = np.all(edges[:-1] == edges[1:], axis=1)
    if np.any(same[:-1] & same[1:]):
        raise ValueError('Nonmanifold edge')
    rows = np.flatnonzero(same)
    return np.column_stack([owners[rows], owners[rows + 1]])


def bending_report(reference, candidate, faces, *, area_floor=0.02,
                   original_cosine=0.5, folded_cosine=0.0):
    """Report newly folded neighbours without rejecting a rigidly rotated mesh.

    A new fold has previously acute neighbouring normals which now oppose
    each other. Area is measured intrinsically, not projected onto an old
    world-space normal: a valid large rotation must not count as inversion.
    Thresholds are explicit because this is evidence, not a universal gate.
    """
    reference, candidate = np.asarray(reference, float), np.asarray(candidate, float)
    faces = np.asarray(faces)
    pairs = adjacent_faces(faces)
    if (reference.ndim != 2 or reference.shape[1] != 3
            or candidate.shape != reference.shape or faces.max() >= len(reference)
            or not np.isfinite(reference).all() or not np.isfinite(candidate).all()
            or not np.isfinite([area_floor, original_cosine, folded_cosine]).all()
            or not 0 < area_floor <= 1
            or not -1 <= folded_cosine < original_cosine <= 1):
        raise ValueError('Invalid bending comparison')

    def measure(points):
        tri = points[faces]
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        area = np.linalg.norm(normal, axis=1)
        unit = np.divide(normal, area[:, None], out=np.zeros_like(normal),
                         where=area[:, None] > 0)
        cosine = np.sum(unit[pairs[:, 0]] * unit[pairs[:, 1]], axis=1)
        return area, cosine

    before_area, before_cosine = measure(reference)
    after_area, after_cosine = measure(candidate)
    if np.any(before_area == 0):
        raise ValueError('Degenerate reference triangle')
    ratios = after_area / before_area
    folded = (before_cosine > original_cosine) & (after_cosine < folded_cosine)
    return dict(new_fold_pairs=pairs[folded].tolist(),
                collapsed_faces=np.flatnonzero(ratios < area_floor).tolist(),
                minimum_area_ratio=float(ratios.min()),
                minimum_adjacent_cosine=float(after_cosine.min()) if len(pairs) else None)


def chord_report(reference, start, end, faces, area_floor=.02):
    """Check every point of a linear deformation interval, not just endpoints.

    Triangle normals are quadratic; area-squared and neighbour dot products
    are quartic. Bernstein bounds cheaply accept positive polynomials. For
    remaining polynomials, evaluate their endpoints and real stationary roots.
    """
    reference, start, end = map(lambda x: np.asarray(x, float), (reference, start, end))
    faces = np.asarray(faces, np.int64)
    if (reference.shape != start.shape or end.shape != start.shape
            or not np.isfinite(np.stack([reference, start, end])).all()
            or not 0 < area_floor <= 1):
        raise ValueError('Invalid deformation chord')
    pairs = adjacent_faces(faces)
    def normal(p):
        t = p[faces]
        return np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    rn = normal(reference); area_squared = np.sum(rn*rn, axis=1)
    keep = np.sum(rn[pairs[:, 0]]*rn[pairs[:, 1]], axis=1) > .5*np.sqrt(area_squared[pairs[:, 0]]*area_squared[pairs[:, 1]])
    pa, pb = pairs[keep].T
    tr = start[faces]; dt = (end-start)[faces]
    a, b = tr[:, 1]-tr[:, 0], tr[:, 2]-tr[:, 0]
    u, v = dt[:, 1]-dt[:, 0], dt[:, 2]-dt[:, 0]
    normals = np.stack([np.cross(a, b), np.cross(u, b)+np.cross(a, v), np.cross(u, v)], 1)
    def product(a, b):
        out = np.zeros((len(a), 5))
        for i in range(3):
            for j in range(3):
                out[:, i+j] += np.sum(a[:, i]*b[:, j], axis=1)
        return out
    areas = product(normals, normals); areas[:, 0] -= area_floor**2*area_squared
    cosines = product(normals[pa], normals[pb])
    choose = np.array([[1, 0, 0, 0, 0], [1, 1, 0, 0, 0], [1, 2, 1, 0, 0],
                       [1, 3, 3, 1, 0], [1, 4, 6, 4, 1]], float)
    convert = choose / np.array([1, 4, 6, 4, 1])[None, :]
    def failures(polynomials):
        bernstein = polynomials@convert.T
        tolerance = 128*np.finfo(float).eps*np.max(abs(bernstein), axis=1)
        suspect = np.flatnonzero(np.min(bernstein, axis=1) < -tolerance)
        bad = []
        for row in suspect:
            p = polynomials[row]
            roots = np.polynomial.polynomial.polyroots(p[1:]*np.arange(1, 5))
            ts = np.r_[0., 1., roots.real[(abs(roots.imag) < 1e-8) & (roots.real > 0) & (roots.real < 1)]]
            values = np.polynomial.polynomial.polyval(ts, p)
            where = int(np.argmin(values))
            if values[where] < -tolerance[row]:
                bad.append((int(row), float(ts[where])))
        return bad
    collapsed, folded = failures(areas), failures(cosines)
    rows = sorted({i for i, _ in collapsed} | {int(v) for i, _ in folded for v in (pa[i], pb[i])})
    fractions = [t for _, t in collapsed+folded]
    return dict(safe=not rows, faces=rows, fractions=fractions,
                collapsed_faces=[i for i, _ in collapsed],
                new_fold_pairs=[[int(pa[i]), int(pb[i])] for i, _ in folded])
