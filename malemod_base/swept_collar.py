"""Measured shaft sections and triangle-level pelvic recruitment.

Positions are in one adapter-calibrated character frame. Section vertices retain
their original mesh-edge donors; no nominal radius or engine units are assumed.
"""
from dataclasses import dataclass
import numpy as np


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n <= 1e-12:
        raise ValueError("Degenerate section frame")
    return v / n


@dataclass
class Section:
    center: np.ndarray
    axis: np.ndarray
    side: np.ndarray
    up: np.ndarray
    polygon: np.ndarray
    donors: np.ndarray
    weights: np.ndarray

    def project(self, points):
        return (np.asarray(points) - self.center) @ np.array([self.side, self.up]).T

    def radii(self, phi):
        """First positive ray/polygon crossing of the actual section."""
        phi = np.asarray(phi)
        rays = np.column_stack([np.cos(phi.ravel()), np.sin(phi.ravel())])
        a = self.polygon
        edge = np.roll(a, -1, axis=0) - a
        cross = lambda x, y: x[..., 0]*y[..., 1] - x[..., 1]*y[..., 0]
        denominator = cross(rays[:, None], edge[None])
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = cross(a, edge)[None] / denominator
            t = cross(a[None], rays[:, None]) / denominator
        valid = (distance > 0) & (t >= -1e-9) & (t <= 1+1e-9)
        result = np.min(np.where(valid, distance, np.inf), axis=1)
        if not np.isfinite(result).all():
            raise ValueError("Measured section is not radial about its centroid")
        return result.reshape(phi.shape)


def measure_section(points, triangles, center, axis, up=(0, 0, 1), eligible=None):
    """Intersect real triangles with a plane; return the nearest closed contour."""
    points = np.asarray(points, dtype=float)
    triangles = np.asarray(triangles, dtype=np.int64)
    axis = unit(axis)
    up = np.asarray(up, dtype=float)
    up = unit(up-axis*np.dot(up, axis))
    side = unit(np.cross(up, axis))
    center = np.asarray(center, dtype=float)
    signed = (points-center) @ axis
    scale = max(np.linalg.norm(np.ptp(points, axis=0)), 1e-12)
    # Deterministic symbolic perturbation prevents a plane through a vertex from
    # duplicating branches. Edge interpolation still uses the same signed field.
    signed += scale * 1e-12
    chosen = np.ones(len(triangles), bool) if eligible is None else np.asarray(eligible, bool)
    chosen &= (signed[triangles].min(1) < 0) & (signed[triangles].max(1) > 0)
    edges, adjacency, weight = {}, {}, {}
    for face in triangles[chosen]:
        keys = []
        for x, y in zip(face, np.roll(face, -1)):
            if signed[x]*signed[y] < 0:
                a, b = sorted((int(x), int(y)))
                key = a, b
                t = signed[a]/(signed[a]-signed[b])
                edges[key] = points[a] + t*(points[b]-points[a])
                weight[key] = t
                keys.append(key)
        if len(keys) == 2:
            a, b = keys
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)
    unseen, loops = set(adjacency), []
    while unseen:
        start = min(unseen)
        current, visited = start, []
        while current in unseen:
            unseen.remove(current)
            visited.append(current)
            next_edges = [v for v in adjacency[current] if v in unseen]
            if not next_edges:
                break
            current = min(next_edges)
        if len(visited) > 3 and start in adjacency[visited[-1]]:
            loops.append(visited)
    if not loops:
        raise ValueError("No closed measured shaft section")
    loop = min(loops, key=lambda keys: np.linalg.norm(np.mean([edges[k] for k in keys], axis=0)-center))
    xyz = np.array([edges[k] for k in loop])
    polygon = (xyz-center) @ np.array([side, up]).T
    cross = polygon[:, 0]*np.roll(polygon[:, 1], -1)-polygon[:, 1]*np.roll(polygon[:, 0], -1)
    area = np.sum(cross)/2
    if abs(area) < scale*scale*1e-12:
        raise ValueError("Measured section has zero area")
    centroid = np.sum((polygon+np.roll(polygon, -1, axis=0))*cross[:, None], axis=0)/(6*area)
    center = center + centroid[0]*side + centroid[1]*up
    polygon -= centroid
    if area < 0:
        polygon = polygon[::-1]
        loop = loop[::-1]
    return Section(center, axis, side, up, polygon, np.array(loop), np.array([weight[k] for k in loop]))


def inside_polygon(points, polygon):
    points = np.asarray(points)
    x, y = points[..., 0, None], points[..., 1, None]
    a, b = polygon, np.roll(polygon, -1, axis=0)
    with np.errstate(divide='ignore', invalid='ignore'):
        crossing = ((a[:, 1] > y) != (b[:, 1] > y)) & (x < (b[:, 0]-a[:, 0])*(y-a[:, 1])/(b[:, 1]-a[:, 1])+a[:, 0])
    return np.logical_xor.reduce(crossing, axis=-1)


def triangle_overlap(projected, triangles, polygon):
    """Exact projected triangle/polygon overlap, including edge-only crossings."""
    q = np.asarray(projected)[triangles]
    result = inside_polygon(q, polygon).any(1)
    cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
    pa, pb = polygon, np.roll(polygon, -1, axis=0)
    for k in range(3):
        a, b = q[:, k], q[:, (k+1)%3]
        d = b-a
        den = cross(d[:, None], (pb-pa)[None])
        offset = pa[None]-a[:, None]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = cross(offset, (pb-pa)[None])/den
            u = cross(offset, d[:, None])/den
        result |= ((abs(den) > 1e-14) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)).any(1)
    # A polygon wholly inside a coarse triangle also intersects the surface.
    signs = np.stack([cross(q[:, (k+1)%3, None]-q[:, k, None], pa[None]-q[:, k, None]) for k in range(3)])
    result |= ((signs >= -1e-12).all(0) | (signs <= 1e-12).all(0)).any(1)
    return result


def connected_patch(triangles, eligible, seeds):
    """Keep only the footprint component connected to the observed body cut."""
    eligible = np.asarray(eligible, bool)
    vertex_faces = {}
    for i in np.flatnonzero(eligible):
        for v in triangles[i]:
            vertex_faces.setdefault(int(v), []).append(int(i))
    pending = [i for v in seeds for i in vertex_faces.get(int(v), [])]
    result = np.zeros(len(triangles), bool)
    while pending:
        i = pending.pop()
        if result[i]:
            continue
        result[i] = True
        for v in triangles[i]:
            pending.extend(j for j in vertex_faces[int(v)] if not result[j])
    return result


def radial_ramp(radius, original, opening, width):
    """Strictly increasing radial opening with C2 identity at its exterior.

    Integrating a positive derivative avoids collapsed inner rings when a large
    opening is recruited through a narrow ramp. Ordinary interpolation between
    the two radii need not be monotone under that amount of dilation.
    """
    from scipy.special import gammainc, gamma
    radius, original, opening, width = np.broadcast_arrays(radius, original, opening, width)
    opening = np.maximum(opening, original)
    exterior = opening+width
    length = exterior-original
    if np.any(original <= 0) or np.any(width <= 0):
        raise ValueError("Opening and ramp must have positive extent")
    average = width/length
    # Reserve half the available average spacing at the opening. A fixed .05
    # derivative packed recruited body rows into a narrow crease at large size.
    slope = .5*average
    lo, hi = np.zeros_like(average), np.full_like(average, 1e9)
    integral = lambda k: gamma(1/3)*gammainc(1/3, k)/(3*np.cbrt(k))
    for _ in range(48):
        k = (lo+hi)/2
        smaller = slope+(1-slope)*integral(k) > average
        lo = np.where(smaller, k, lo)
        hi = np.where(smaller, hi, k)
    k = (lo+hi)/2
    t = np.clip((radius-original)/length, 0, 1)
    cumulative = slope*t+(1-slope)*gamma(1/3)*(gammainc(1/3, k)-gammainc(1/3, k*(1-t)**3))/(3*np.cbrt(k))
    mapped = opening+length*cumulative
    mapped = np.where(radius < original, opening+slope*(radius-original), mapped)
    # The zero-dilation branch is an exact identity, including rounding.
    return np.where((radius >= exterior) | (opening == original), radius, mapped)


def circumscribed_opening(section, original_corners, inferior_allowance=0,
                          max_lateral_ratio=1.10):
    """Opening corners whose straight donor edges enclose the measured section.

    Putting coarse corners directly ON a round section leaves every edge chord
    inside it. Tangent support lines instead preserve the entire circumference,
    including all interpolated original-edge seam donors. Reject an excessively
    wide opening instead of hiding a poor corner distribution by enlarging it.
    """
    corners = np.asarray(original_corners, float)
    profile = np.asarray(section.polygon, float)
    if (corners.ndim != 2 or corners.shape[1] != 2 or len(corners) < 3
            or not np.isfinite(corners).all() or profile.ndim != 2
            or profile.shape[1] != 2 or len(profile) < 3
            or not np.isfinite(profile).all()):
        raise ValueError("Opening and measured section must be finite 2D polygons")
    if not np.isfinite(max_lateral_ratio) or max_lateral_ratio < 1:
        raise ValueError("Maximum lateral ratio must be finite and at least one")
    center = corners.mean(0)
    angles = np.arctan2(corners[:, 1]-center[1], corners[:, 0]-center[0])
    order = np.argsort(angles)
    angles = angles[order]
    end = np.r_[angles[1:], angles[0]+2*np.pi]
    normals = np.column_stack([np.cos((angles+end)/2), np.sin((angles+end)/2)])
    # Use existing donor edges for exact lateral and upper supporting planes.
    # Otherwise a sparse rim can protrude beyond the shaft at the intersection
    # of two oblique tangents, despite both tangents touching the right profile.
    # No vertices/edges are added and the cyclic donor correspondence is retained.
    for direction in ([-1., 0.], [1., 0.], [0., 1.]):
        closest = np.argmax(normals@direction)
        normals[closest] = direction
    offsets = np.max(section.polygon@normals.T, axis=0)
    if not np.isfinite(inferior_allowance) or inferior_allowance < 0:
        raise ValueError("Inferior allowance must be finite and nonnegative")
    # Support values must themselves describe a convex shape. A cubic ventral
    # weight is not a support function: near a pointed underside it reverses
    # consecutive tangent intersections and folds the original donor edges.
    # Minkowski-add a thin ellipse whose top is at zero. Its full height is the
    # requested underside allowance; its lateral extent is only a tenth of it.
    half_height = inferior_allowance*.5
    half_width = inferior_allowance*.1
    offsets += np.hypot(half_width*normals[:, 0], half_height*normals[:, 1])-half_height*normals[:, 1]
    result = []
    for i in range(len(corners)):
        system = np.array([normals[i-1], normals[i]])
        if abs(np.linalg.det(system)) < 1e-8:
            raise ValueError("Opening has coincident angular corners")
        result.append(np.linalg.solve(system, [offsets[i-1], offsets[i]]))
    result = np.array(result)
    measured_width = np.ptp(section.polygon[:, 0])
    if measured_width <= 0 or not np.isfinite(result).all():
        raise ValueError("Measured section must have finite positive width")
    if np.ptp(result[:, 0]) > measured_width*max_lateral_ratio:
        raise ValueError("Collar corner distribution exceeds measured width allowance")
    edges = np.roll(result, -1, axis=0)-result
    following = np.roll(edges, -1, axis=0)
    turns = edges[:, 0]*following[:, 1]-edges[:, 1]*following[:, 0]
    if (np.min(np.linalg.norm(edges, axis=1)) < 1e-10*measured_width
            or np.any(turns < -1e-12*measured_width**2)):
        raise ValueError("Collar support planes collapse or reverse a donor edge")
    return order, result


def radial_opening(section, original_corners, inferior_allowance=0):
    """Circumscribe the section while retaining every original corner ray.

    Edge containment is linear in inverse corner radii. Keeping the rays fixed
    lets the same monotone radial map move the rim and the surrounding body;
    independently moving rim corners can overtake the next body row.
    """
    from scipy.optimize import minimize, LinearConstraint, Bounds
    corners = np.asarray(original_corners, float)
    center = corners.mean(0)
    angles = np.arctan2(corners[:, 1]-center[1], corners[:, 0]-center[0])
    order = np.argsort(angles)
    angles = angles[order]
    rays = np.column_stack([np.cos(angles), np.sin(angles)])
    profile = section.polygon.copy()
    radius = np.linalg.norm(profile, axis=1)
    profile *= (1+inferior_allowance*np.maximum(0, -profile[:, 1]/radius)**3/radius)[:, None]
    relative = profile-center
    count, samples = len(corners), len(profile)
    matrix = np.zeros((count*samples, count))
    bound = np.zeros(count*samples)
    cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
    for i in range(count):
        j = (i+1)%count
        rows = slice(i*samples, (i+1)*samples)
        matrix[rows, i] = -cross(rays[j], relative)
        matrix[rows, j] = cross(rays[i], relative)
        bound[rows] = cross(rays[i], rays[j])
    measured = Section(section.center, section.axis, section.side, section.up, relative, None, None)
    desired = 1/measured.radii(angles)
    scale = np.max(np.linalg.norm(relative, axis=1))
    constraint = LinearConstraint(matrix, -np.inf, bound)
    result = minimize(lambda u: (.5*np.sum((u-desired)**2), u-desired),
                      np.full(count, .1/scale), jac=True, method='SLSQP',
                      constraints=[constraint], bounds=Bounds(.01/scale, desired),
                      options=dict(ftol=1e-12, maxiter=200))
    if not result.success or np.max(matrix@result.x-bound) > 1e-8:
        raise ValueError("Cannot circumscribe the measured section on the original donor rays")
    return order, center+rays/result.x[:, None]
