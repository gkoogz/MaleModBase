"""Static garment guide geometry on measured surfaces; no fabric simulation."""
import numpy as np


def bezier(controls, count):
    p = np.asarray(controls, float)
    if p.shape != (4, 3) or count < 2 or not np.isfinite(p).all():
        raise ValueError('Expected four finite guide controls')
    t = np.linspace(0, 1, count)[:, None]
    return (1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t*t*p[2]+t**3*p[3]


def project_surface(points, vertices, faces, vertex_normals, clearance=0.):
    """Exact closest triangle, barycentric donor and interpolated surface normal.

    Adapters select the measured body region. Clearance uses caller units.
    Returns projected points, normals, face indices and barycentric donors.
    """
    points, vertices = np.asarray(points, float), np.asarray(vertices, float)
    faces, vertex_normals = np.asarray(faces, int), np.asarray(vertex_normals, float)
    tri = vertices[faces]
    a, ab, ac = tri[:, 0], tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]
    aa, bb, cc = (ab*ab).sum(1), (ab*ac).sum(1), (ac*ac).sum(1)
    determinant = aa*cc-bb*bb
    valid = determinant > 1e-16
    denominator = np.where(valid, determinant, 1.)
    outputs, normals, ids, donors = [], [], [], []
    for point in points:
        ap = point-a
        d, e = (ap*ab).sum(1), (ap*ac).sum(1)
        v, w = (cc*d-bb*e)/denominator, (aa*e-bb*d)/denominator
        bary = np.column_stack([1-v-w, v, w])
        candidates = [(tri*bary[:, :, None]).sum(1)]
        weights = [bary]
        distances = [np.where(valid & (bary.min(1)>=0),
                              ((candidates[0]-point)**2).sum(1), np.inf)]
        for i, j in [(0, 1), (1, 2), (2, 0)]:
            edge = tri[:, j]-tri[:, i]
            t = np.clip(((point-tri[:, i])*edge).sum(1)/np.maximum((edge*edge).sum(1), 1e-20), 0, 1)
            q = tri[:, i]+t[:, None]*edge
            b = np.zeros((len(tri), 3)); b[:, i]=1-t; b[:, j]=t
            candidates.append(q); weights.append(b)
            distances.append(((q-point)**2).sum(1))
        kind, face = np.unravel_index(np.argmin(distances), (4, len(tri)))
        bary = weights[kind][face]
        normal = (vertex_normals[faces[face]]*bary[:, None]).sum(0)
        normal /= max(np.linalg.norm(normal), 1e-12)
        outputs.append(candidates[kind][face]+clearance*normal)
        normals.append(normal); ids.append(face); donors.append(bary)
    return np.asarray(outputs), np.asarray(normals), np.asarray(ids), np.asarray(donors)


def smooth_clearance_curve(points, lift, outward_axis, count=161, clearance_values=None):
    """One smooth cubic, fixed endpoints, handles fitted to a measured guide.

    Clearance raises handles rather than individual samples, avoiding dents
    and kinks from pointwise collider projection. Caller supplies measured
    outward direction and obstacle query; no game geometry is assumed.
    """
    points=np.asarray(points,float);axis=np.asarray(outward_axis,float)
    axis/=np.linalg.norm(axis)
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    t=distance/distance[-1]
    weights=np.column_stack([3*(1-t)**2*t,3*(1-t)*t*t])
    remainder=points-(1-t[:,None])**3*points[0]-t[:,None]**3*points[-1]
    handles=np.linalg.lstsq(weights,remainder,rcond=None)[0]
    controls=np.vstack([points[0],handles,points[-1]])
    if clearance_values is not None:
        from scipy.optimize import minimize
        initial=handles.copy()
        samples=np.linspace(0,distance[-1],count)
        reference=np.column_stack([np.interp(samples,distance,points[:,i]) for i in range(3)])
        span=max(float(np.ptp(points,axis=0).max()),1.)
        low=points.min(0)-span*.3;high=points.max(0)+span*.3
        seed=initial.copy()
        parameter=np.linspace(0,1,count);handle_weight=3*parameter*(1-parameter)
        for _ in range(12):
            candidate_curve=bezier(np.vstack([points[0],seed,points[-1]]),count)
            need=(lift(candidate_curve)-candidate_curve)@axis
            if need.max()<1e-5:break
            proposal=seed+axis*(float(np.max(need[1:-1]/handle_weight[1:-1]))*1.02)
            if (proposal<low).any() or (proposal>high).any():break
            seed=proposal
        def evaluate(values):return bezier(np.vstack([points[0],values.reshape(2,3),points[-1]]),count)
        def objective(values):
            curve=evaluate(values)
            return float(((curve-reference)**2).sum()/count+.02*((values.reshape(2,3)-initial)**2).sum())
        solved=minimize(objective,seed.ravel(),method='SLSQP',
            bounds=list(zip(np.tile(low,2),np.tile(high,2))),
            constraints=[dict(type='ineq',fun=lambda values:clearance_values(evaluate(values)))],
            options=dict(maxiter=150,ftol=1e-9))
        curve=evaluate(solved.x)
        minimum=float(np.min(clearance_values(curve)))
        if minimum<-1e-5:raise ValueError('Bounded smooth hem has no accepted clearance fit')
        controls=np.vstack([points[0],solved.x.reshape(2,3),points[-1]])
        return curve,dict(controls=controls.tolist(),iterations=int(solved.nit),
            endpointError=float(np.linalg.norm(curve[[0,-1]]-points[[0,-1]],axis=1).max()),
            minimumSignedClearance=minimum,boundedFit=True)
    parameter=np.linspace(0,1,count)
    handle_weight=3*parameter*(1-parameter)
    for iteration in range(32):
        curve=bezier(controls,count)
        raised=lift(curve)
        need=(raised-curve)@axis
        if max(need[0],need[-1])>1e-5:
            raise ValueError('Fixed guide endpoint lacks collider clearance')
        if need.max()<1e-5:
            return curve,dict(controls=controls.tolist(),iterations=iteration,
                endpointError=float(np.linalg.norm(curve[[0,-1]]-points[[0,-1]],axis=1).max()),
                maximumResidualLift=float(need.max()))
        amount=float(np.max(need[1:-1]/handle_weight[1:-1]))
        if amount>max(float(np.ptp(points,axis=0).max()),1.)*.3:
            raise ValueError('Guide needs bounded multi-axis clearance fitting')
        controls[1:3]+=axis*(amount*1.02)
    raise ValueError('Smooth fixed guide failed clearance convergence')


def fixed_ribbon(curve,width,thickness,normal_hint):
    """Closed fixed strap with a continuous rectangular cross-section frame."""
    curve=np.asarray(curve,float)
    if width<=0 or thickness<=0:raise ValueError('Invalid fixed strap dimensions')
    tangent=np.gradient(curve,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    hint=np.broadcast_to(np.asarray(normal_hint,float),curve.shape)
    normal=hint-(hint*tangent).sum(1)[:,None]*tangent
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    side=np.cross(normal,tangent)
    vertices=np.stack([curve-side*width/2,curve+side*width/2,
        curve+side*width/2-normal*thickness,curve-side*width/2-normal*thickness],axis=1)
    faces=[]
    for i in range(len(curve)-1):
        for j in range(4):
            a=i*4+j;b=i*4+(j+1)%4
            faces.extend([[a,b,b+4],[a,b+4,a+4]])
    faces.extend([[0,2,1],[0,3,2]])
    a=(len(curve)-1)*4;faces.extend([[a,a+1,a+2],[a,a+2,a+3]])
    return vertices.reshape(-1,3),np.asarray(faces,np.uint32)
