"""Constrained attachment authoring in exact original-edge donor space.

This is an offline authoring solver, not a runtime or visual acceptance gate.
It records the entire accepted path, and never labels a partial path complete.
The inequalities are normalized by their current coordinate gradients: tiny
native subdivision triangles must not dominate the optimizer's conditioning.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
from scipy.optimize import minimize
from .surface_bending import adjacent_faces, bending_report


@dataclass
class DeformationPath:
    positions: np.ndarray
    knots: list
    fractions: list
    complete: bool
    message: str
    report: dict


def repair_path(reference, target, faces, editable, seams=(), *, start=None,
                steps=40, max_iterations=100, area_floor=.08,
                cosine_floor=.05, repair_limit=3., progress=None):
    """Approach a target without new folds, preserving supplied edge donors.

    ``editable`` selects independent repair degrees of freedom. All target
    rows still follow the requested path; noneditable rows cannot be repaired.
    ``start`` may be an already verified point on another authoring path.
    Distances and ``repair_limit`` are in the caller's calibrated units.
    """
    ref, target = np.asarray(reference, float), np.asarray(target, float)
    faces = np.asarray(faces, np.int64)
    editable = np.asarray(editable, bool)
    if (ref.ndim != 2 or ref.shape[1] != 3 or target.shape != ref.shape
            or editable.shape != (len(ref),) or not np.isfinite(ref).all()
            or not np.isfinite(target).all() or steps < 1 or max_iterations < 1
            or not 0 < area_floor <= 1 or not 0 <= cosine_floor < .5
            or not np.isfinite(repair_limit) or repair_limit <= 0):
        raise ValueError('Invalid deformation-path domain')
    n = len(ref)
    previous = ref.copy() if start is None else np.asarray(start, float).copy()
    if previous.shape != ref.shape or not np.isfinite(previous).all():
        raise ValueError('Invalid starting geometry')
    relations = {}
    for slave, a, b, weight in seams:
        slave, a, b = int(slave), int(a), int(b)
        if (min(slave, a, b) < 0 or max(slave, a, b) >= n
                or slave in relations or slave in (a, b)
                or not np.isfinite(weight) or not 0 <= weight <= 1):
            raise ValueError('Invalid original-edge donor')
        relations[slave] = (a, b, float(weight))
    active = np.array([i for i in range(n) if editable[i] and i not in relations], int)
    lookup, cache, visiting = {v: i for i, v in enumerate(active)}, {}, set()

    def donors(i):
        if i in cache:
            return cache[i]
        if i in visiting:
            raise ValueError('Cyclic original-edge donors')
        if i not in relations:
            return {lookup[i]: 1.} if i in lookup else {}
        visiting.add(i)
        a, b, weight = relations[i]
        row = {}
        for parent, w in ((a, 1-weight), (b, weight)):
            for j, value in donors(parent).items():
                row[j] = row.get(j, 0) + w*value
        visiting.remove(i)
        cache[i] = row
        return row

    rr, cc, vv = [], [], []
    for i in range(n):
        for j, weight in donors(i).items():
            rr.append(i); cc.append(j); vv.append(weight)
    transform = sparse.csr_matrix((vv, (rr, cc)), shape=(n, len(active)))
    transform3 = sparse.kron(transform, sparse.eye(3)).tocsr()
    for positions in (ref, target, previous):
        for slave, (a, b, w) in relations.items():
            if np.max(abs(positions[slave]-(1-w)*positions[a]-w*positions[b])) > 1e-7:
                raise ValueError('Input violates an original-edge donor')
    initial_report = bending_report(ref, previous, faces, area_floor=.02)
    if initial_report['new_fold_pairs'] or initial_report['collapsed_faces']:
        raise ValueError('Starting geometry already fails the requested bounds')
    changed = np.asarray(abs(transform).sum(1)).ravel() > 0
    face_ids = np.flatnonzero(changed[faces].any(1))
    local_faces = faces[face_ids]
    tri = ref[faces]
    normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    area = np.linalg.norm(normals, axis=1)
    normals /= area[:, None]
    pairs = adjacent_faces(faces)
    keep = (np.sum(normals[pairs[:, 0]]*normals[pairs[:, 1]], axis=1) > .5)
    keep &= changed[faces[pairs]].any(axis=(1, 2))
    pa, pb = pairs[keep].T
    hessian = (transform.T@transform).toarray()+np.eye(len(active))*.01
    start = previous.copy()
    knots, fractions, message = [start.copy()], [0.], 'No steps attempted'

    def normal_gradients(triangles, direction):
        b = np.cross(triangles[:, 2]-triangles[:, 0], direction)
        c = np.cross(direction, triangles[:, 1]-triangles[:, 0])
        return np.stack([-b-c, b, c], 1)

    def jacobian(gradient, vertices):
        rows = np.repeat(np.arange(len(vertices)), vertices.shape[1]*3)
        cols = (3*vertices[:, :, None]+np.arange(3)).ravel()
        matrix = sparse.coo_matrix((gradient.ravel(), (rows, cols)),
                                   shape=(len(vertices), 3*n)).tocsr()
        return (matrix@transform3).toarray()

    for stage in range(1, steps+1):
        fraction = stage/steps
        base = start+fraction*(target-start)
        tr = previous[local_faces]
        axis = np.cross(tr[:, 1]-tr[:, 0], tr[:, 2]-tr[:, 0])
        axis /= np.linalg.norm(axis, axis=1)[:, None]

        def constraints(x, derivative=False):
            posed = base+transform@x.reshape(-1, 3)
            tr = posed[faces]
            cr = np.cross(tr[:, 1]-tr[:, 0], tr[:, 2]-tr[:, 0])
            aa = np.maximum(np.linalg.norm(cr, axis=1), 1e-30)
            nn = cr/aa[:, None]
            cosine = np.sum(nn[pa]*nn[pb], axis=1)
            signed = np.sum(cr[face_ids]*axis, axis=1)/area[face_ids]
            if not derivative:
                return np.r_[signed-area_floor, cosine-cosine_floor]
            g = normal_gradients(tr[face_ids], axis/area[face_ids, None])
            da = (nn[pb]-cosine[:, None]*nn[pa])/aa[pa, None]
            db = (nn[pa]-cosine[:, None]*nn[pb])/aa[pb, None]
            cg = np.concatenate([normal_gradients(tr[pa], da),
                                 normal_gradients(tr[pb], db)], axis=1)
            rows = [jacobian(g, local_faces), jacobian(cg, np.concatenate([faces[pa], faces[pb]], axis=1))]
            return np.vstack(rows)

        x0 = (previous[active]-base[active]).ravel()
        if len(active) and constraints(np.zeros_like(x0)).min(initial=0) >= 0:
            candidate, message = base, 'Target already satisfies local bounds'
        elif not len(active):
            candidate, message = base, 'No editable repair rows'
        else:
            row_scale = np.maximum(np.linalg.norm(constraints(x0, True), axis=1), 1.)
            def objective(x):
                xx = x.reshape(-1, 3)
                return .5*np.sum(xx*(hessian@xx))
            result = minimize(objective, x0, jac=lambda x: (hessian@x.reshape(-1, 3)).ravel(),
                method='SLSQP', bounds=[(-repair_limit, repair_limit)]*len(x0),
                constraints=[dict(type='ineq', fun=lambda x: constraints(x)/row_scale,
                                  jac=lambda x: constraints(x, True)/row_scale[:, None])],
                options=dict(maxiter=max_iterations, ftol=1e-10, disp=False))
            candidate = base+transform@result.x.reshape(-1, 3)
            message = str(result.message)
            residual = constraints(result.x)
            # The optimizer aims for an 8% area / positive-cosine margin.
            # Accept only measured geometry above the actual 2% / zero-fold
            # gate, even if a sliver prevents full optimizer convergence.
            if (residual[:len(face_ids)].min(initial=0) < .02-area_floor
                    or residual[len(face_ids):].min(initial=0) < .005-cosine_floor):
                break
        report = bending_report(ref, candidate, faces, area_floor=.02)
        if report['new_fold_pairs'] or report['collapsed_faces']:
            message = 'Whole-mesh bounds reject this step'
            break
        previous = candidate
        knots.append(candidate.copy()); fractions.append(fraction)
        if progress is not None:
            progress(fraction, candidate, message)
    report = bending_report(ref, previous, faces)
    return DeformationPath(previous, knots, fractions, fractions[-1] == 1., message, report)
