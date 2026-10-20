"""Boundary-preserving surface fit with explicit attachment support planes.

This offline solver keeps measured body/healthy rows fixed during the fit.
Triangle validity must be checked separately; convergence is not a visual gate.
"""
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import splu


def fit_surface(reference, triangles, boundary, fixed, plane_ids=(),
                plane_normals=(), plane_offsets=(), iterations=1200,
                tolerance=1e-5, screen=1e-4, seams=(),
                differential_positions=None, differential_mask=None,
                plane_equalities=None, support_mode="bounded"):
    if support_mode not in ("bounded", "halfspace"):
        raise ValueError("Unknown attachment support mode")
    reference = np.asarray(reference, float)
    boundary = np.asarray(boundary, float)
    triangles = np.asarray(triangles, int)
    fixed = np.asarray(fixed, bool).copy()
    n = len(reference)
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    edges = np.unique(np.sort(edges, axis=1), axis=0)
    rows, cols = np.r_[edges[:, 0], edges[:, 1]], np.r_[edges[:, 1], edges[:, 0]]
    adjacency = sparse.csr_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n))
    degree = np.asarray(adjacency.sum(1)).ravel()
    fixed |= degree == 0
    laplace = sparse.eye(n)-sparse.diags(1/np.maximum(degree, 1))@adjacency
    # Eliminate original-edge slaves, rather than fitting them independently
    # and trying to repair the join afterwards. Nested donors are flattened.
    relations = {int(v): (int(a), int(b), float(w)) for v, a, b, w in seams}
    masters = np.array([i for i in range(n) if i not in relations], dtype=int)
    lookup = {int(v): j for j, v in enumerate(masters)}
    cache = {}
    def donors(i, visiting=frozenset()):
        if i in cache:
            return cache[i]
        if i in visiting:
            raise ValueError("Cyclic seam donors")
        if i not in relations:
            return {lookup[i]: 1.}
        a, b, w = relations[i]
        result = {}
        for parent, weight in ((a, 1-w), (b, w)):
            for j, value in donors(parent, visiting | {i}).items():
                result[j] = result.get(j, 0.)+weight*value
        cache[i] = result
        return result
    rr, cc, vv = [], [], []
    for i in range(n):
        for j, w in donors(i).items():
            rr.append(i); cc.append(j); vv.append(w)
    binding = sparse.csr_matrix((vv, (rr, cc)), shape=(n, len(masters)))
    locked = fixed[masters]
    # A prescribed slave cannot introduce an unrepresented equality.
    for i in relations:
        if fixed[i] and any(not locked[j] for j in donors(i)):
            raise ValueError("A fixed seam slave requires fixed donors")
        if fixed[i] and not np.allclose(
                boundary[i], sum(w*boundary[masters[j]] for j, w in donors(i).items()),
                rtol=1e-12, atol=1e-12):
            raise ValueError("Prescribed seam and donor positions disagree")
    free = np.flatnonzero(~locked)
    screen = np.broadcast_to(np.asarray(screen, float), (n,))
    if not np.isfinite(screen).all() or np.any(screen <= 0):
        raise ValueError("Surface screen must be finite and positive")
    energy = (laplace.T@laplace+sparse.diags(screen)).tocsr()
    linear = screen[:, None]*reference
    if differential_positions is not None:
        positions = np.asarray(differential_positions, float)
        mask = np.asarray(differential_mask, float)
        if positions.shape != reference.shape or mask.shape != (n,):
            raise ValueError("Differential preservation must match surface topology")
        if not np.isfinite(mask).all() or np.any((mask < 0) | (mask > 1)):
            raise ValueError("Differential preservation weights must be in [0, 1]")
        linear += laplace.T@((laplace@positions)*mask[:, None])
    energy = (binding.T@energy@binding).tocsr()
    quadratic = energy[free][:, free].tocsc()
    rhs = (-energy[free][:, locked]@boundary[masters[locked]]
           +(binding.T@linear)[free])
    # Project positions, including interpolated seam samples, rather than only
    # independent masters. ADMM's linear step enforces the donor relation while
    # its local step enforces clearance on the actual visible surface samples.
    # Skipping a slave permits the chord between two clear donors to penetrate.
    active_binding = binding[:, free].tocsr()
    active_rows = np.flatnonzero(np.asarray(abs(active_binding).sum(1)).ravel() > 0)
    active_binding = active_binding[active_rows]
    fixed_offset = binding[active_rows][:, locked]@boundary[masters[locked]]
    inverse = np.full(n, -1, dtype=int)
    inverse[active_rows] = np.arange(len(active_rows))
    ids = np.asarray(plane_ids, int)
    normals = np.asarray(plane_normals, float).reshape(-1, 3)
    offsets = np.asarray(plane_offsets, float)
    equalities = (np.zeros(len(ids), bool) if plane_equalities is None
                  else np.asarray(plane_equalities, bool))
    if equalities.shape != (len(ids),):
        raise ValueError("Plane equality flags must match the support rows")
    if len(ids) != len(normals) or len(ids) != len(offsets):
        raise ValueError("Each support plane requires a row, normal and offset")
    if len(ids) and (np.any(inverse[ids] < 0) or not np.allclose(np.linalg.norm(normals, axis=1), 1)):
        raise ValueError("Planes must address movable rows with unit normals")
    local = inverse[ids]
    unique_ids, counts = np.unique(local, return_counts=True)
    single = np.isin(local, unique_ids[counts == 1])
    pairs = [np.flatnonzero(local == i) for i in unique_ids[counts == 2]]
    envelope_ids = unique_ids[counts > 2]
    envelope = None
    if len(envelope_ids):
        from .convex_support import PlanarSupportProjector, PlanarHalfspaceProjector
        groups = [np.flatnonzero(local == i) for i in envelope_ids]
        if any(equalities[g].any() for g in groups):
            raise ValueError("Section envelopes require inequality planes")
        projector = PlanarSupportProjector if support_mode == "bounded" else PlanarHalfspaceProjector
        envelope = projector([(normals[g], offsets[g]) for g in groups])
    for a, b in pairs:
        if 1-np.dot(normals[a], normals[b])**2 < 1e-10:
            raise ValueError("Paired support planes must be independent")
        if equalities[a] != equalities[b]:
            raise ValueError("Paired planes must both be equalities or inequalities")
    if not len(free):
        return binding@boundary[masters], dict(iterations=0, residual=0., free=0)
    rho = max(.01, float(np.median(screen[masters[free]])))
    mass = (active_binding.T@active_binding).tocsc()
    solver = splu((quadratic+rho*mass).tocsc())
    z = reference[active_rows].copy()
    dual = np.zeros_like(z)
    residual = np.inf
    for iteration in range(iterations):
        x = solver.solve(rhs+rho*(active_binding.T@(z-dual-fixed_offset)))
        positions = active_binding@x+fixed_offset
        previous = z
        relaxed = 1.6*positions-.6*previous
        z = relaxed+dual
        violation = offsets[single]-np.sum(z[local[single]]*normals[single], axis=1)
        z[local[single]] += np.where(equalities[single], violation, np.maximum(violation, 0))[:, None]*normals[single]
        for a, b in pairs:
            i = local[a]; point = z[i].copy()
            na, nb = normals[a], normals[b]
            va, vb = offsets[a]-point@na, offsets[b]-point@nb
            if equalities[a]:
                dot = na@nb
                weights = np.linalg.solve([[1, dot], [dot, 1]], [va, vb])
                z[i] = point+weights[0]*na+weights[1]*nb
                continue
            if max(va, vb) <= 0:
                continue
            pa = point+max(va, 0)*na
            pb = point+max(vb, 0)*nb
            if pa@nb >= offsets[b]:
                z[i] = pa
            elif pb@na >= offsets[a]:
                z[i] = pb
            else:
                dot = na@nb
                weights = np.linalg.solve([[1, dot], [dot, 1]], [va, vb])
                z[i] = point+weights[0]*na+weights[1]*nb
        if envelope is not None:
            z[envelope_ids] = envelope.project(z[envelope_ids])
        dual += relaxed-z
        primal = np.max(np.abs(positions-z), initial=0)
        dual_residual = np.max(np.abs(rho*(active_binding.T@(z-previous))), initial=0)
        residual = max(primal, dual_residual)
        clearance = offsets-np.sum(positions[local]*normals, axis=1)
        feasibility = np.max(np.where(equalities, abs(clearance), clearance), initial=0)
        if residual < tolerance and feasibility < min(tolerance, 1e-12):
            break
        if iteration % 50 == 49:
            next_rho = rho
            if primal > 10*dual_residual:
                next_rho = min(rho*2, 1e6)
            elif dual_residual > 10*primal:
                next_rho = max(rho/2, 1e-6)
            if next_rho != rho:
                dual *= rho/next_rho
                rho = next_rho
                solver = splu((quadratic+rho*mass).tocsc())
    if residual >= tolerance or feasibility >= min(tolerance, 1e-12):
        raise ValueError(f"Attachment fit has not converged: residual={residual}, primal={primal}, dual={dual_residual}, clearance={feasibility}")
    out = boundary[masters].copy()
    out[free] = x
    return binding@out, dict(iterations=iteration+1, residual=float(residual), free=len(free))
