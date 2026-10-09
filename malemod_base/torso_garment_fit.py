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


def expand_projected_sections(positions, triangles, supports, clearance, spacing=2., side_threshold=.05):
    """Expand a common positive X scale field instead of flattening folds.

    Y/Z and source cut boundaries remain ordered. Every vertex at the same
    height/sign shares the same scale, including UV aliases. Solving measured
    face support constraints on this field preserves the depth of neighboring
    source folds which independent nearest-surface projection would collapse.
    """
    out=np.array(positions,dtype=float,copy=True);supports=np.asarray(supports,dtype=float)
    if clearance<=0 or spacing<=0 or not np.isfinite(out).all():raise ValueError("Invalid section fit")
    knots=np.arange(out[:,2].min()-spacing,out[:,2].max()+2*spacing,spacing)
    fraction=(out[:,2]-knots[0])/spacing;left=np.floor(fraction).astype(int);blend=fraction-left
    basis=np.zeros((len(out),len(knots)))
    basis[np.arange(len(out)),left]=1-blend;basis[np.arange(len(out)),left+1]=blend
    scales=np.ones((2,len(knots)));constraints=[]
    for face in np.asarray(triangles,dtype=np.int64):
        a,b,c=out[face];sign=1 if np.all(out[face,0]>side_threshold) else (-1 if np.all(out[face,0]<-side_threshold) else 0)
        if not sign:continue
        det=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1])
        if abs(det)<1e-9:continue
        normal=np.cross(b-a,c-a)
        if abs(normal[0])<.25*np.linalg.norm(normal):continue
        q=supports[(supports[:,0]*sign>0)&np.all(supports[:,1:]>=out[face,1:].min(0)-1e-6,axis=1)&np.all(supports[:,1:]<=out[face,1:].max(0)+1e-6,axis=1)]
        u=((q[:,1]-a[1])*(c[2]-a[2])-(q[:,2]-a[2])*(c[1]-a[1]))/det
        v=((b[1]-a[1])*(q[:,2]-a[2])-(b[2]-a[2])*(q[:,1]-a[1]))/det
        weights=np.column_stack((1-u-v,u,v));valid=np.all(weights>=-1e-6,axis=1)
        for w,x in zip(weights[valid],q[valid,0]):
            coefficient=(w*sign*out[face,0])@basis[face]
            constraints.append((int(sign<0),coefficient,float(sign*x+clearance)))
    for _ in range(8):
        # Only raise valleys; this preserves every previously satisfied support.
        for side in range(2):
            smooth=np.convolve(np.pad(scales[side],(2,2),mode='edge'),[1/16,4/16,6/16,4/16,1/16],mode='valid')
            scales[side]=np.maximum(scales[side],smooth)
        for side,c,target in constraints:
            missing=target-np.dot(c,scales[side])
            if missing>1e-8:scales[side]+=c*missing/np.dot(c,c)
    for side,c,target in constraints:
        if target-np.dot(c,scales[side])>1e-5:raise ValueError("Section clearance did not converge")
    factors=np.sum(basis*scales[(out[:,0]<0).astype(int)],axis=1)
    out[:,0]*=factors
    return out


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
