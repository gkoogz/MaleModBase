"""Source-derived continuous projected-area bound for a coupled correction."""
import numpy as np


def correction_limit(points, displacement, faces, floor, minimum_area_squared):
    p=np.asarray(points,dtype=float);d=np.asarray(displacement,dtype=float);f=np.asarray(faces)
    if (p.ndim!=2 or p.shape[1]!=3 or d.shape!=p.shape or
            not np.isfinite(p).all() or not np.isfinite(d).all() or
            f.ndim!=2 or f.shape[1]!=3 or not len(f) or not np.issubdtype(f.dtype,np.integer) or
            f.min()<0 or f.max()>=len(p) or not np.isfinite([floor,minimum_area_squared]).all() or
            not 0<floor<1 or minimum_area_squared<0):
        raise ValueError('Invalid correction domain/area bounds')
    tri=p[f];delta=d[f]
    e1,e2=tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]
    u,v=delta[:,1]-delta[:,0],delta[:,2]-delta[:,0]
    n=np.cross(e1,e2);area=np.sum(n*n,axis=1)
    active=area>minimum_area_squared
    a=np.sum(n[active]*np.cross(u[active],v[active]),axis=1)/area[active]
    b=np.sum(n[active]*(np.cross(u[active],e2[active])+np.cross(e1[active],v[active])),axis=1)/area[active]
    if not np.isfinite(a).all() or not np.isfinite(b).all():return 0.
    adverse=np.maximum(0,-a)+np.maximum(0,-b)
    maximum=float(np.max(adverse,initial=0))
    return min(1.,(1-floor)/maximum) if maximum else 1.
