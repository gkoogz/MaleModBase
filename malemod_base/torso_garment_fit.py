"""SDK-free refit of a measured tubular garment over a measured torso.

Positions are in the supplied common reference frame, never assumed SI units.
The adapter retains topology, UV aliases, source IDs and skinning. Radial
projection preserves height and the source neckline/armhole vertex ordering.
Only outward movement is allowed; existing ease and folds remain intact.
"""
import numpy as np


def smooth_tubular_chart(positions, triangles, passes=30):
    """Relax internal angle/height backtracking while preserving cut edges.

    A wrinkled stock surface can fold over itself when projected directly onto
    a larger body. Smooth its cylinder chart first, with periodic angles and
    welded position aliases, then refit. UVs and original donor lineage remain
    adapter data; this does not weld render seams.
    """
    p=np.array(positions,dtype=float);ids={};groups=[];mapping=[]
    for i,q in enumerate(p):
        key=tuple(np.round(q,5))
        if key not in ids:ids[key]=len(groups);groups.append([])
        groups[ids[key]].append(i);mapping.append(ids[key])
    mapping=np.array(mapping);faces=mapping[np.asarray(triangles,dtype=np.int64)]
    edges={};neighbors=[set() for _ in groups]
    for a,b,c in faces:
        for x,y in ((a,b),(b,c),(c,a)):
            if x==y:continue
            edge=tuple(sorted((x,y)));edges[edge]=edges.get(edge,0)+1;neighbors[x].add(y);neighbors[y].add(x)
    boundary=set(x for edge,count in edges.items() if count==1 for x in edge)
    q=np.array([p[g].mean(0) for g in groups]);angle=np.arctan2(q[:,1],q[:,0]);height=q[:,2].copy();radius=np.linalg.norm(q[:,:2],axis=1)
    for _ in range(passes):
        da=np.zeros(len(q));dz=np.zeros(len(q))
        for i,ns in enumerate(neighbors):
            if i in boundary or not ns:continue
            ns=list(ns);diff=(angle[ns]-angle[i]+np.pi)%(2*np.pi)-np.pi
            da[i]=diff.mean()*.5;dz[i]=(height[ns].mean()-height[i])*.5
        angle+=da;height+=dz
    q=np.column_stack((radius*np.cos(angle),radius*np.sin(angle),height))
    return q[mapping]


def refine_triangles(positions, triangles):
    """One edge subdivision with explicit interpolation donors.

    The adapter interpolates UV and skin attributes using the returned source
    pairs. Separate render aliases stay separate; boundaries are not welded or
    moved. Original vertices retain their indices and winding is preserved.
    """
    positions=np.asarray(positions,dtype=float)
    triangles=np.asarray(triangles,dtype=np.int64)
    if positions.ndim!=2 or positions.shape[1]!=3 or not np.isfinite(positions).all():
        raise ValueError("Expected finite Nx3 positions")
    if triangles.ndim!=2 or triangles.shape[1]!=3 or np.any(triangles<0) or np.any(triangles>=len(positions)):
        raise ValueError("Invalid garment topology")
    out=list(positions);donors=[(i,i) for i in range(len(out))];edges={};faces=[]
    def midpoint(a,b):
        edge=tuple(sorted((int(a),int(b))))
        if edge not in edges:
            edges[edge]=len(out);out.append((positions[a]+positions[b])*.5);donors.append(edge)
        return edges[edge]
    for a,b,c in triangles:
        ab,bc,ca=midpoint(a,b),midpoint(b,c),midpoint(c,a)
        faces.extend(((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)))
    return np.array(out),np.array(faces),np.array(donors)


def _bounded_smooth_field(coefficients, targets, shape, regularization):
    """Smooth least-energy iterate with a feasible bounded-field certificate.

    Unlike repeated raise-only filtering, dual updates can remove excess
    expansion. The uniformly feasible field bounds every knot. If the capped
    dual solve has residual error, a minimal convex mix with that witness
    restores all inequalities; no post-clamp clearance is assumed.
    """
    a=np.asarray(coefficients,float);b=np.asarray(targets,float);n=int(np.prod(shape))
    if not len(a):return np.zeros(n)
    # Canonical constraint order and strongest duplicate make source face/UV
    # alias ordering irrelevant to the capped iterative solve.
    unique,inverse=np.unique(a,axis=0,return_inverse=True)
    strongest=np.full(len(unique),-np.inf);np.maximum.at(strongest,inverse,b)
    a=unique;b=strongest
    total=a.sum(axis=1)
    if np.any(total<=0):raise ValueError('Invalid offset support coefficients')
    upper=float(np.max(np.maximum(b,0)/total))*1.05+1e-6
    if np.all(b<=0):return np.zeros(n)
    grid=np.arange(n).reshape(shape);energy=np.eye(n)
    for axis in range(len(shape)):
        left=np.take(grid,np.arange(shape[axis]-1),axis=axis).ravel()
        right=np.take(grid,np.arange(1,shape[axis]),axis=axis).ravel()
        energy[left,left]+=regularization;energy[right,right]+=regularization
        energy[left,right]-=regularization;energy[right,left]-=regularization
    a=np.vstack((a,np.eye(n),-np.eye(n)));b=np.r_[b,np.zeros(n),np.full(n,-upper)]
    directions=np.linalg.solve(energy,a.T).T
    diagonal=np.einsum('ij,ij->i',a,directions)
    alpha=np.zeros(len(a));field=np.zeros(n)
    for _ in range(600):
        for i in range(len(a)):
            delta=max(-alpha[i],(b[i]-a[i]@field)/diagonal[i])
            if abs(delta)>1e-12:alpha[i]+=delta;field+=directions[i]*delta
        violation=b-a@field
        if np.max(np.where(alpha>1e-9,np.abs(violation),np.maximum(violation,0)))<1e-5:break
    field=np.clip(field,0,upper);witness=np.full(n,upper)
    value=a@field;margin=a@witness-value;missing=b-value
    mix=float(np.max(np.divide(missing,margin,out=np.zeros_like(missing),where=(missing>0)&(margin>0))))
    field=(1-mix)*field+mix*witness
    if mix>1+1e-9 or not np.isfinite(field).all() or np.max(b-a@field)>1e-5:
        raise ValueError('Bounded field clearance certificate failed')
    return field


def expand_projected_sections(positions, triangles, supports, clearance, spacing=2., side_threshold=.05, *, offset_width=None, outward_cosine=None, lateral_spacing=None, field_regularization=None):
    """Expand a common smooth X field instead of flattening folds.

    Y/Z and source cut boundaries remain ordered. Every vertex at the same
    height/sign shares the same scale, including UV aliases. Solving measured
    face support constraints on this field preserves the depth of neighboring
    source folds which independent nearest-surface projection would collapse.
    With offset_width, use a smooth signed translation that vanishes at X=0
    instead of multiplying fold depth. outward_cosine restricts support tests
    to outward facing surfaces: grazing and turned-back folds are not front
    envelope constraints and require separate full-surface clearance review.
    field_regularization replaces raise-only filtering with a bounded smooth
    offset solve; it needs offset_width and preserves the same measured face
    inequalities. The global field cap is 1.05 times a uniform feasible witness.
    """
    out=np.array(positions,dtype=float,copy=True);supports=np.asarray(supports,dtype=float)
    if clearance<=0 or spacing<=0 or not np.isfinite(out).all():raise ValueError("Invalid section fit")
    if offset_width is not None and (not np.isfinite(offset_width) or offset_width<=0):raise ValueError("Invalid offset width")
    if outward_cosine is not None and (not np.isfinite(outward_cosine) or not 0<=outward_cosine<=1):raise ValueError("Invalid outward support cosine")
    if field_regularization is not None and (offset_width is None or not np.isfinite(field_regularization) or field_regularization<=0):raise ValueError('Invalid offset field regularization')
    knots=np.arange(out[:,2].min()-spacing,out[:,2].max()+2*spacing,spacing)
    fraction=(out[:,2]-knots[0])/spacing;left=np.floor(fraction).astype(int);blend=fraction-left
    basis=np.zeros((len(out),len(knots)))
    basis[np.arange(len(out)),left]=1-blend;basis[np.arange(len(out)),left+1]=blend
    field_shape=(len(knots),)
    if lateral_spacing is not None:
        if not np.isfinite(lateral_spacing) or lateral_spacing<=0:raise ValueError('Invalid lateral field spacing')
        lateral=np.arange(out[:,1].min()-lateral_spacing,out[:,1].max()+2*lateral_spacing,lateral_spacing)
        fraction=(out[:,1]-lateral[0])/lateral_spacing;left=np.floor(fraction).astype(int);blend=fraction-left
        by=np.zeros((len(out),len(lateral)));by[np.arange(len(out)),left]=1-blend;by[np.arange(len(out)),left+1]=blend
        basis=(basis[:,:,None]*by[:,None,:]).reshape(len(out),-1);field_shape=(len(knots),len(lateral))
    scales=np.ones((2,basis.shape[1])) if offset_width is None else np.zeros((2,basis.shape[1]))
    direction=out[:,0] if offset_width is None else np.tanh(out[:,0]/offset_width)
    constraints=[]
    faces=np.asarray(triangles,dtype=np.int64)
    ns=np.cross(out[faces[:,1]]-out[faces[:,0]],out[faces[:,2]]-out[faces[:,0]])
    radial=out[faces].mean(axis=1);radial[:,2]=0
    winding=1 if np.median(np.sum(ns*radial,axis=1))>=0 else -1
    for face in np.asarray(triangles,dtype=np.int64):
        a,b,c=out[face];sign=1 if np.all(out[face,0]>side_threshold) else (-1 if np.all(out[face,0]<-side_threshold) else 0)
        if not sign:continue
        det=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1])
        if abs(det)<1e-9:continue
        normal=np.cross(b-a,c-a)
        if abs(normal[0])<.25*np.linalg.norm(normal):continue
        if outward_cosine is not None and winding*sign*normal[0]<outward_cosine*np.linalg.norm(normal):continue
        q=supports[(supports[:,0]*sign>0)&np.all(supports[:,1:]>=out[face,1:].min(0)-1e-6,axis=1)&np.all(supports[:,1:]<=out[face,1:].max(0)+1e-6,axis=1)]
        u=((q[:,1]-a[1])*(c[2]-a[2])-(q[:,2]-a[2])*(c[1]-a[1]))/det
        v=((b[1]-a[1])*(q[:,2]-a[2])-(b[2]-a[2])*(q[:,1]-a[1]))/det
        weights=np.column_stack((1-u-v,u,v));valid=np.all(weights>=-1e-6,axis=1)
        for w,x in zip(weights[valid],q[valid,0]):
            coefficient=(w*sign*direction[face])@basis[face]
            target=sign*x+clearance
            if offset_width is not None:target-=sign*np.dot(w,out[face,0])
            constraints.append((int(sign<0),coefficient,float(target)))
    if field_regularization is not None:
        for side in range(2):
            selected=[(c,target) for s,c,target in constraints if s==side]
            scales[side]=_bounded_smooth_field([c for c,target in selected],[target for c,target in selected],field_shape,field_regularization)
    for _ in range(0 if field_regularization is not None else 8):
        # Only raise valleys; this preserves every previously satisfied support.
        for side in range(2):
            grid=scales[side].reshape(field_shape)
            if lateral_spacing is None:smooth=np.convolve(np.pad(grid,(2,2),mode='edge'),[1/16,4/16,6/16,4/16,1/16],mode='valid')
            else:
                smooth=np.apply_along_axis(lambda v:np.convolve(np.pad(v,(2,2),mode='edge'),[1/16,4/16,6/16,4/16,1/16],mode='valid'),0,grid)
                smooth=np.apply_along_axis(lambda v:np.convolve(np.pad(v,(1,1),mode='edge'),[.25,.5,.25],mode='valid'),1,smooth).ravel()
            scales[side]=np.maximum(scales[side],smooth)
        for side,c,target in constraints:
            missing=target-np.dot(c,scales[side])
            if missing>1e-8:scales[side]+=c*missing/np.dot(c,c)
    for side,c,target in constraints:
        if target-np.dot(c,scales[side])>1e-5:raise ValueError("Section clearance did not converge")
    factors=np.sum(basis*scales[(out[:,0]<0).astype(int)],axis=1)
    if offset_width is None:out[:,0]*=factors
    else:out[:,0]+=direction*factors
    return out


def curved_boundary_midpoints(source, triangles, refined, lineage, enabled, maximum_offset):
    """Round newly refined cut edges while retaining every original corner.

    Adapter-supplied masks select measured cuts. A bounded cubic midpoint uses
    the neighbouring original edge tangents; texture/skin lineage stays on the
    original edge. UV seams weld only for boundary connectivity.
    """
    p=np.asarray(source,dtype=float);q=np.array(refined,dtype=float,copy=True)
    enabled=np.asarray(enabled,dtype=bool)
    if enabled.shape!=(len(p),) or maximum_offset<=0 or not np.isfinite(maximum_offset):raise ValueError('Invalid cut mask or rounding bound')
    keys={};ids=[];points=[]
    for v in p:
        key=tuple(np.round(v,5))
        if key not in keys:keys[key]=len(points);points.append(v)
        ids.append(keys[key])
    ids=np.asarray(ids);points=np.asarray(points);edges={}
    for f in np.asarray(triangles,dtype=int):
        for a,b in zip(f,np.roll(f,-1)):
            key=tuple(sorted((int(ids[a]),int(ids[b]))));edges[key]=edges.get(key,0)+1
    neighbors={}
    for (a,b),count in edges.items():
        if count==1:neighbors.setdefault(a,set()).add(b);neighbors.setdefault(b,set()).add(a)
    for i,(a,b) in enumerate(np.asarray(lineage,dtype=int)):
        x,y=int(ids[a]),int(ids[b]);edge=tuple(sorted((x,y)))
        if a==b or not enabled[a] or not enabled[b] or edges.get(edge)!=1 or len(neighbors.get(x,()))!=2 or len(neighbors.get(y,()))!=2:continue
        before=next(v for v in neighbors[x] if v!=y);after=next(v for v in neighbors[y] if v!=x)
        midpoint=(-points[before]+9*points[x]+9*points[y]-points[after])/16
        delta=midpoint-(p[a]+p[b])*.5;length=np.linalg.norm(delta)
        q[i]+=delta*min(1.,maximum_offset/max(length,1e-12))
    return q


def refit_radially(positions, body, triangles, clearance, center=(0., 0.), fallback_distance=0.):
    positions=np.asarray(positions, dtype=float)
    body=np.asarray(body, dtype=float)
    triangles=np.asarray(triangles, dtype=np.int64)
    if positions.ndim!=2 or positions.shape[1]!=3 or body.ndim!=2 or body.shape[1]!=3:
        raise ValueError("Expected finite Nx3 reference positions")
    if not np.isfinite(positions).all() or not np.isfinite(body).all() or not np.isfinite(clearance) or clearance<=0:
        raise ValueError("Finite surfaces and positive clearance required")
    if triangles.ndim!=2 or triangles.shape[1]!=3 or np.any(triangles<0) or np.any(triangles>=len(body)):
        raise ValueError("Invalid torso topology")
    a=body[triangles[:,0]]; e1=body[triangles[:,1]]-a; e2=body[triangles[:,2]]-a
    result=positions.copy(); contacts=[]
    for i,p in enumerate(positions):
        origin=np.array([*center,p[2]]); d=p-origin; radius=np.linalg.norm(d)
        if radius<1e-9: raise ValueError("Garment vertex lies on radial axis")
        d/=radius; h=np.cross(np.broadcast_to(d,e2.shape),e2); det=np.einsum('ij,ij->i',e1,h)
        valid=np.abs(det)>1e-10; inv=np.divide(1.,det,out=np.zeros_like(det),where=valid)
        s=origin-a; u=np.einsum('ij,ij->i',s,h)*inv; q=np.cross(s,e1)
        v=np.einsum('j,ij->i',d,q)*inv; t=np.einsum('ij,ij->i',e2,q)*inv
        hits=np.flatnonzero(valid&(u>=-1e-7)&(v>=-1e-7)&(u+v<=1+1e-7)&(t>0))
        if not len(hits):
            # Open anatomical patches at armholes need a bounded closest-point
            # binding. Test the plane projection and all three finite edges;
            # this remains a measured triangle, never an invented body shell.
            candidates=[]
            for start,edge,wa,wb in ((a,e1,[1,0,0],[-1,1,0]),(a,e2,[1,0,0],[-1,0,1]),(a+e1,e2-e1,[0,1,0],[0,-1,1])):
                fraction=np.clip(np.einsum('ij,ij->i',p-start,edge)/np.maximum(np.einsum('ij,ij->i',edge,edge),1e-15),0,1)
                candidates.append((start+fraction[:,None]*edge,np.array(wa)+fraction[:,None]*np.array(wb)))
            n=np.cross(e1,e2); ns=np.einsum('ij,ij->i',n,n)
            plane=p-n*(np.einsum('ij,ij->i',p-a,n)/np.maximum(ns,1e-15))[:,None]
            aa=np.einsum('ij,ij->i',e1,e1);bb=np.einsum('ij,ij->i',e2,e2);ab=np.einsum('ij,ij->i',e1,e2)
            ap=np.einsum('ij,ij->i',plane-a,e1);bp=np.einsum('ij,ij->i',plane-a,e2);den=aa*bb-ab*ab
            uu=np.divide(bb*ap-ab*bp,den,out=np.zeros_like(den),where=den>1e-15);vv=np.divide(aa*bp-ab*ap,den,out=np.zeros_like(den),where=den>1e-15)
            weights=np.column_stack((1-uu-vv,uu,vv));plane[np.any(weights<0,axis=1)]=np.inf
            candidates.append((plane,weights))
            distances=np.array([np.linalg.norm(q-p,axis=1) for q,w in candidates]);ci,j=np.unravel_index(np.argmin(distances),distances.shape)
            if fallback_distance<=0 or distances[ci,j]>fallback_distance:raise ValueError(f"No measured torso intersection for garment vertex {i}")
            q,w=candidates[ci][0][j],candidates[ci][1][j];normal=n[j]/max(np.linalg.norm(n[j]),1e-15)
            if np.dot(normal,q-origin)<0:normal=-normal
            result[i]=p+normal*max(0,clearance-np.dot(p-q,normal))
            contacts.append((int(j),w.tolist(),float(np.linalg.norm(result[i]-q))))
            continue
        # Choose the nearest shell on the supplied torso, rather than a remote
        # limb lying on the same ray. Adapters provide the observed torso patch.
        j=hits[np.argmin(np.abs(t[hits]-radius))]
        fitted=max(radius,t[j]+clearance)
        result[i]=origin+d*fitted
        contacts.append((int(j),[float(1-u[j]-v[j]),float(u[j]),float(v[j])],float(fitted-t[j])))
    return result,contacts


def refit_between_bodies(positions, source_body, source_triangles, target_body,
                        target_triangles, clearance, *, fallback_distance=0.,
                        garment_triangles=None, smoothing_passes=0):
    """Transport measured garment ease with the body's radial displacement.

    Bind both reference bodies in the same adapter-supplied frame. Retain the
    garment-to-source offset, including fold depth and cut locations, instead
    of projecting all folds onto the target shell. Clearance is a lower bound.
    Returned bindings refer exclusively to the target for runtime transport.
    """
    p=np.asarray(positions,dtype=float)
    _,old=refit_radially(p,source_body,source_triangles,clearance,
                         fallback_distance=fallback_distance)
    _,new=refit_radially(p,target_body,target_triangles,clearance,
                         fallback_distance=fallback_distance)
    source_body=np.asarray(source_body,dtype=float);source_triangles=np.asarray(source_triangles)
    target_body=np.asarray(target_body,dtype=float);target_triangles=np.asarray(target_triangles)
    delta=np.zeros(len(p))
    for i,((sf,sw,_),(tf,tw,_)) in enumerate(zip(old,new)):
        a=np.asarray(sw)@source_body[source_triangles[sf]]
        b=np.asarray(tw)@target_body[target_triangles[tf]]
        # Bounded armhole donors may lie off the query ray. Their height/angle
        # must not pull a source cut across its neighbours and invert triangles.
        delta[i]=np.linalg.norm(b[:2])-np.linalg.norm(a[:2])
    if smoothing_passes:
        if garment_triangles is None or smoothing_passes<0:raise ValueError('Garment topology required for displacement smoothing')
        faces=np.asarray(garment_triangles,dtype=int)
        if faces.ndim!=2 or faces.shape[1]!=3 or np.any(faces<0) or np.any(faces>=len(p)):raise ValueError('Invalid garment topology')
        # Relax the expansion FIELD, never the original folds or cut chart.
        # UV aliases share a graph node and therefore move together.
        keys={};ids=[]
        for point in p:
            key=tuple(np.round(point,5));ids.append(keys.setdefault(key,len(keys)))
        ids=np.asarray(ids);neighbors=[set() for _ in keys]
        for f in faces:
            for i in f:
                neighbors[ids[i]].update(ids[f]);neighbors[ids[i]].discard(ids[i])
        field=np.array([delta[ids==i].mean() for i in range(len(keys))])
        for _ in range(smoothing_passes):
            field=np.array([.5*field[i]+.5*np.mean(field[list(n)]) if n else field[i] for i,n in enumerate(neighbors)])
        delta=field[ids]
    result=p.copy();radius=np.linalg.norm(p[:,:2],axis=1)
    result[:,:2]*=(1+delta/radius)[:,None]
    result,bindings=refit_radially(result,target_body,target_triangles,clearance,
                                  fallback_distance=fallback_distance)
    return result,bindings


def clear_projected_faces(positions, triangles, supports, clearance, passes=8, aliases=None):
    """Cover dense measured torso supports under each coarse front/back face.

    A vertex-only fit can bridge straight through a convex chest between its
    corners. Positive half-space corrections raise the WHOLE interpolated face.
    Only X is changed, so projected neckline/armhole boundaries and triangle
    winding stay fixed. This is an authoring fit, not a dynamic cloth solver.
    """
    out=np.array(positions,dtype=float,copy=True);supports=np.asarray(supports,dtype=float)
    constraints=[]
    groups=[np.array([i]) for i in range(len(out))]
    if aliases is not None:
        aliases=np.asarray(aliases)
        if len(aliases)!=len(out):raise ValueError("Alias count differs from garment vertices")
        for key in np.unique(aliases):
            ids=np.flatnonzero(aliases==key)
            if not np.allclose(out[ids],out[ids[0]],atol=1e-5):raise ValueError("Position aliases already disagree")
            out[ids]=out[ids].mean(axis=0)
            for i in ids:groups[i]=ids
    for face in np.asarray(triangles,dtype=np.int64):
        p=out[face];sign=1 if np.all(p[:,0]>.05) else (-1 if np.all(p[:,0]<-.05) else 0)
        if not sign:continue
        a,b,c=p;det=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1])
        if abs(det)<1e-9:continue
        q=supports[(supports[:,0]*sign>0)&np.all(supports[:,1:]>=p[:,1:].min(0)-1e-6,axis=1)&np.all(supports[:,1:]<=p[:,1:].max(0)+1e-6,axis=1)]
        u=((q[:,1]-a[1])*(c[2]-a[2])-(q[:,2]-a[2])*(c[1]-a[1]))/det
        v=((b[1]-a[1])*(q[:,2]-a[2])-(b[2]-a[2])*(q[:,1]-a[1]))/det
        weights=np.column_stack((1-u-v,u,v));valid=np.all(weights>=-1e-6,axis=1)
        for w,x in zip(weights[valid],q[valid,0]):constraints.append((face,w,float(sign),float(sign*x+clearance)))
    for _ in range(passes):
        for face,w,sign,required in constraints:
            missing=required-sign*np.dot(w,out[face,0])
            if missing>1e-7:
                for vertex,weight in zip(face,w):out[groups[vertex],0]+=sign*weight*missing/np.dot(w,w)
    for face,w,sign,required in constraints:
        if required-sign*np.dot(w,out[face,0])>1e-5:raise ValueError("Projected garment clearance did not converge")
    return out
