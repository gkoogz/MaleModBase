"""Triangle-preserving fly panels with explicit source-face barycentric lineage.

All dimensions and axes are supplied in the adapter's measured rest frame.
The cut subdivides triangles rather than deleting whole faces at the zipper.
Fold hinges remain attached. No game, graphics, skeleton or units assumptions.
"""
import numpy as np


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
