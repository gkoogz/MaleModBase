"""Triangle-preserving fly panels with explicit source-face barycentric lineage.

All dimensions and axes are supplied in the adapter's measured rest frame.
The cut subdivides triangles rather than deleting whole faces at the zipper.
Fold hinges remain attached. No game, graphics, skeleton or units assumptions.
"""
import numpy as np


def fly_motion_bindings(rest, panels, *, lower, upper, half_width,
                        belt_lower, belt_upper):
    """Continuous pinned-seam weights for a fly and its original upper band.

    Inputs are reconstructed pre-fold source positions and authored panel sides.
    Belt heights are measured by the adapter, never inferred from a skeleton.
    The shared outer hinge has zero response, including duplicate cut vertices.
    """
    p=np.asarray(rest,dtype=float);side=np.asarray(panels,dtype=int)
    if p.ndim!=2 or p.shape[1]!=3 or len(side)!=len(p) or not np.isfinite(p).all() or not np.isin(side,[-1,0,1]).all() or upper<=lower or half_width<=0 or belt_upper<=belt_lower:
        raise ValueError('Invalid motion binding frame')
    width=half_width*np.maximum(0,(p[:,2]-lower)/(upper-lower))
    hinge=p.copy();hinge[:,1]=side*width
    free=np.clip(1-np.abs(p[:,1])/np.maximum(width,1e-12),0,1)
    free=free*free*(3-2*free);free[side==0]=0
    band=np.clip((p[:,2]-belt_lower)/(belt_upper-belt_lower),0,1)
    band=band*band*(3-2*band)
    return dict(hinges=hinge,fly=free,belt=free*band,sides=side)


def fold_rigid_attachment(positions, *, front_axis, side_axis, height_axis,
                          lower, upper, half_width, angle, side=1):
    """Carry an accessory with a flap without shearing its original shape.

    The accessory centroid follows the panel envelope at its measured height.
    All vertices receive the same proper rotation about that centroid's hinge.
    """
    p=np.asarray(positions,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('Expected a finite nonempty rigid accessory')
    if len({front_axis,side_axis,height_axis})!=3 or set((front_axis,side_axis,height_axis))!={0,1,2} or upper<=lower or half_width<=0 or side not in (-1,1) or not np.isfinite(angle) or not 0<angle<np.pi:
        raise ValueError('Invalid attachment hinge')
    pivot=p.mean(axis=0)
    pivot[side_axis]=side*half_width*(pivot[height_axis]-lower)/(upper-lower)
    q=p-pivot;out=q.copy();c=np.cos(angle);s=side*np.sin(angle)
    out[:,front_axis]=q[:,front_axis]*c-q[:,side_axis]*s
    out[:,side_axis]=q[:,front_axis]*s+q[:,side_axis]*c
    return out+pivot


def folded_fly(positions, triangles, *, front_axis, side_axis, height_axis,
               front_plane, lower, upper, half_width, angle):
    p=np.asarray(positions,dtype=float);tri=np.asarray(triangles,dtype=int)
    if len({front_axis,side_axis,height_axis})!=3 or upper<=lower or half_width<=0 or not 0<angle<np.pi:
        raise ValueError('Invalid fly frame or hinge envelope')
    out=[];faces=[];donors=[];weights=[];panels=[]
    def width(q):return half_width*(q[height_axis]-lower)/(upper-lower)
    def split(poly, distance):
        inside=[];outside=[]
        for a,b in zip(poly,poly[1:]+poly[:1]):
            da,db=distance(a[0]),distance(b[0]);ain=da>=-1e-10;bin=db>=-1e-10
            (inside if ain else outside).append(a)
            if ain!=bin:
                t=da/(da-db);v=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
                inside.append(v);outside.append(v)
        return inside,outside
    def emit(poly, source, panel):
        if len(poly)<3:return
        begin=len(out)
        for q,bary in poly:
            q=q.copy()
            if panel:
                sign=1 if panel==1 else -1;hinge=sign*width(q);offset=q[side_axis]-hinge
                q[front_axis]+=-sign*offset*np.sin(angle)
                q[side_axis]=hinge+offset*np.cos(angle)
            out.append(q);donors.append(source);weights.append(bary);panels.append(panel)
        for i in range(1,len(poly)-1):
            f=[begin,begin+i,begin+i+1]
            if np.linalg.norm(np.cross(out[f[1]]-out[f[0]],out[f[2]]-out[f[0]]))>1e-9:faces.append(f)
    for t in tri:
        poly=[(p[t[i]],np.eye(3)[i]) for i in range(3)]
        for plane in (lambda q:q[front_axis]-front_plane,
                      lambda q:q[height_axis]-lower,
                      lambda q:width(q)-q[side_axis],lambda q:width(q)+q[side_axis]):
            poly,outside=split(poly,plane);emit(outside,t,0)
            if not poly:break
        if poly:
            right,left=split(poly,lambda q:q[side_axis]);emit(right,t,1);emit(left,t,-1)
    return dict(positions=np.asarray(out),triangles=np.asarray(faces,dtype=int),
                donors=np.asarray(donors,dtype=int),weights=np.asarray(weights),panels=np.asarray(panels,dtype=int))
