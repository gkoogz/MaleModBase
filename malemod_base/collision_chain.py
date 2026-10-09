"""Geometry-only inspection scaffold. No fabric solver or game dependencies."""
import numpy as np


def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError('Degenerate collision direction')
    return v / n


def basis(normal):
    n = unit(normal)
    seed = np.eye(3)[np.argmin(np.abs(n))]
    a = unit(np.cross(n, seed))
    return a, np.cross(n, a)


def circle(center, normal, radius, segments=64):
    if not np.isfinite(radius) or radius <= 0 or segments < 8:
        raise ValueError('Invalid collision circle')
    a, b = basis(normal)
    angle = np.arange(segments) * (2 * np.pi / segments)
    return np.asarray(center) + radius * (np.cos(angle)[:, None]*a + np.sin(angle)[:, None]*b)


def chain(centers, radii, segments=64, normals=None):
    """Eight shared circular sections, seven closed-sided tapered links.

    Joint planes bisect adjacent centerline directions. The shared circle and
    center are identical for both incident links; there are no capsule endcaps.
    """
    centers, radii = np.asarray(centers, float), np.asarray(radii, float)
    if centers.shape != (8, 3) or radii.shape != (8,) or not np.isfinite(centers).all():
        raise ValueError('Expected eight measured sections for seven links')
    directions = np.array([unit(v) for v in np.diff(centers, axis=0)])
    if normals is None:
        normals = [directions[0]] + [unit(a+b) for a, b in zip(directions[:-1], directions[1:])] + [directions[-1]]
    else:
        normals = np.asarray([unit(n) for n in normals])
        if normals.shape != (8,3):raise ValueError('Expected eight joint planes')
    rings = np.array([circle(c, n, r, segments) for c, n, r in zip(centers, normals, radii)])
    # Keep the angular frame continuous as the chain bends.
    for i in range(1, len(rings)):
        previous = rings[i-1]-centers[i-1]
        offsets = rings[i]-centers[i]
        shift = np.argmin([np.sum((previous-np.roll(offsets, s, axis=0))**2) for s in range(segments)])
        rings[i] = np.roll(rings[i], shift, axis=0)
    faces = []
    for i in range(7):
        for j in range(segments):
            a, b = i*segments+j, i*segments+(j+1)%segments
            faces.extend([[a, b, b+segments], [a, b+segments, a+segments]])
    return rings, np.asarray(faces, np.uint32), np.asarray(normals)


def ovoid(center, axes, radii, expansion=1.03, rows=24, segments=48):
    """Exact concentric source-physics tapered ovoid, uniformly enlarged.

    Source OvoidSupport uses radial factor (1 - .13*z)*sqrt(1-z*z).
    Axes are the physics model's three world-space basis vectors.
    """
    axes, radii = np.asarray(axes, float), np.asarray(radii, float)
    if axes.shape != (3, 3) or radii.shape != (3,) or np.min(radii) <= 0 or expansion <= 1:
        raise ValueError('Invalid measured ovoid')
    if not np.allclose(axes @ axes.T, np.eye(3), atol=2e-5):
        raise ValueError('Ovoid physics frame must be orthonormal')
    theta = np.arange(segments)*2*np.pi/segments
    vertices = []
    for phi in np.linspace(.001, np.pi-.001, rows+1):
        z = np.cos(phi)
        radial = (1-.13*z)*np.sin(phi)
        local = np.column_stack([radial*np.cos(theta), radial*np.sin(theta), np.full(segments, z)])
        vertices.extend(np.asarray(center)+(local*radii*expansion) @ axes)
    faces = []
    for i in range(rows):
        for j in range(segments):
            a, b = i*segments+j, i*segments+(j+1)%segments
            faces.extend([[a, b, b+segments], [a, b+segments, a+segments]])
    # Latitude runs north to south, so reverse side winding and cap both poles.
    faces = [f[::-1] for f in faces]
    north, south = len(vertices), len(vertices)+1
    vertices.extend([np.asarray(center)+axes[2]*radii[2]*expansion,np.asarray(center)-axes[2]*radii[2]*expansion])
    for j in range(segments):
        k=(j+1)%segments
        faces.extend([[north,j,k],[south,rows*segments+k,rows*segments+j]])
    return np.asarray(vertices), np.asarray(faces, np.uint32)


def closed_link(first, second, first_center, second_center):
    count=len(first)
    points=np.vstack([first,second,first_center,second_center])
    faces=[]
    for j in range(count):
        k=(j+1)%count
        faces.extend([[j,k,k+count],[j,k+count,j+count],
                      [2*count,k,j],[2*count+1,j+count,k+count]])
    return points,np.asarray(faces,np.uint32)


def edge_arc(edge, forward, lateral, degrees=80, count=81):
    """Angular span on the existing bottom edge; never flattens the waistband."""
    edge = np.asarray(edge, float)
    center = edge.mean(axis=0)
    angle = np.arctan2((edge-center) @ lateral, (edge-center) @ forward)
    order = np.argsort(angle)
    a, p = angle[order], edge[order]
    a = np.r_[a[-1]-2*np.pi, a, a[0]+2*np.pi]
    p = np.vstack([p[-1], p, p[0]])
    result = []
    for t in np.linspace(-degrees*np.pi/360, degrees*np.pi/360, count):
        k = np.searchsorted(a, t)-1
        # Intersect the radial half-plane with this actual edge segment.
        side = np.asarray(lateral)*np.cos(t)-np.asarray(forward)*np.sin(t)
        x, y = (p[k]-center) @ side, (p[k+1]-center) @ side
        result.append(p[k]+(p[k+1]-p[k])*(-x/(y-x)))
    return np.asarray(result)


def enclosing_tip(points, crown, crown_radius, old_tip, axis, segments=64, clearance=.08):
    """Advance the tip cap and widen it to contain a measured distal surface.

    The inscribed polygon margin also encloses the surface in the rendered
    tessellation, not just in the ideal circular frustum.
    """
    axis=unit(axis);points=np.asarray(points,float);crown=np.asarray(crown,float)
    t=(points-crown)@axis
    points=points[t>=-1e-8];t=t[t>=-1e-8]
    radial=np.linalg.norm(points-crown-t[:,None]*axis,axis=1)
    length=max(float((np.asarray(old_tip)-crown)@axis),float(t.max()))+clearance
    fraction=t/length;inscribed=np.cos(np.pi/segments)
    take=fraction>1e-6
    required=(radial[take]/inscribed-crown_radius*(1-fraction[take]))/fraction[take]
    radius=max(float(required.max()),clearance)*1.01
    allowed=(crown_radius*(1-fraction)+radius*fraction)*inscribed
    excess=float((radial-allowed).max())
    if excess>1e-7:raise ValueError('Distal surface cannot be enclosed with fixed crown circle')
    return crown+axis*length,radius,dict(vertices=len(points),maxRadialExcess=excess,
                                       capClearance=float(length-t.max()))


def hemisphere_dome(base_ring,base_center,apex,rows=24):
    """Closed half-ellipsoid retaining an exact circular rim and one apex.

    Equal axial height and rim radius gives a mathematical hemisphere. An
    independently prescribed apex requires axial scaling of that hemisphere.
    """
    ring=np.asarray(base_ring,float);center=np.asarray(base_center,float)
    apex=np.asarray(apex,float);axial=apex-center;height=np.linalg.norm(axial)
    axis=unit(axial);radial=ring-center
    radius=np.linalg.norm(radial,axis=1)
    if rows<4 or not np.allclose(radius,radius[0],atol=1e-8) or np.abs(radial@axis).max()>1e-8:
        raise ValueError('Dome rim must be a perfect circle normal to its apex axis')
    phi=np.linspace(0,np.pi/2,rows+1)[:-1]
    circles=center+np.sin(phi)[:,None,None]*axial+np.cos(phi)[:,None,None]*radial
    circles[0]=ring
    count=len(ring);vertices=np.vstack([circles.reshape(-1,3),apex,center])
    pole=rows*count;base=pole+1;faces=[]
    for i in range(rows-1):
        for j in range(count):
            a=i*count+j;b=i*count+(j+1)%count
            faces.extend([[a,b,b+count],[a,b+count,a+count]])
    for j in range(count):
        k=(j+1)%count
        faces.extend([[(rows-1)*count+j,(rows-1)*count+k,pole],[base,k,j]])
    return vertices,np.asarray(faces,np.uint32),dict(radius=float(radius[0]),height=float(height),
        axialScale=float(height/radius[0]),rimExact=True,apex=apex.tolist(),latitudeRows=rows)
