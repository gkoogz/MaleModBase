"""SDK-free refit of a measured tubular garment over a measured torso.

Positions are in the supplied common reference frame, never assumed SI units.
The adapter retains topology, UV aliases, source IDs and skinning. Radial
projection preserves height and the source neckline/armhole vertex ordering.
Only outward movement is allowed; existing ease and folds remain intact.
"""
import numpy as np


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


def clear_projected_faces(positions, triangles, supports, clearance, passes=8):
    """Cover dense measured torso supports under each coarse front/back face.

    A vertex-only fit can bridge straight through a convex chest between its
    corners. Positive half-space corrections raise the WHOLE interpolated face.
    Only X is changed, so projected neckline/armhole boundaries and triangle
    winding stay fixed. This is an authoring fit, not a dynamic cloth solver.
    """
    out=np.array(positions,dtype=float,copy=True);supports=np.asarray(supports,dtype=float)
    constraints=[]
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
            if missing>1e-7:out[face,0]+=sign*w*missing/np.dot(w,w)
    for face,w,sign,required in constraints:
        if required-sign*np.dot(w,out[face,0])>1e-5:raise ValueError("Projected garment clearance did not converge")
    return out
