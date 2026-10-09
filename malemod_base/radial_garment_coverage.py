"""Measured radial garment coverage; retains cuts and caller topology."""
import numpy as np

def radial_coverage(surface, triangles, queries, *, radial_axes, height_axis, tolerance=1e-7):
    p=np.asarray(surface,float);t=np.asarray(triangles,int);q=np.asarray(queries,float)
    if len(set((*radial_axes,height_axis)))!=3:raise ValueError('Distinct frame axes required')
    x,y=radial_axes;angles=np.arctan2(p[:,y],p[:,x]);radii=np.hypot(p[:,x],p[:,y])
    theta=np.arctan2(q[:,y],q[:,x]);radius=np.hypot(q[:,x],q[:,y]);covered=np.zeros(len(q),bool)
    for face in t:
        a=angles[face];a=a[0]+(a-a[0]+np.pi)%(2*np.pi)-np.pi;z=p[face,height_axis]
        h=(theta-a[0]+np.pi)%(2*np.pi)-np.pi+a[0]
        candidate=(~covered)&(q[:,height_axis]>=z.min()-tolerance)&(q[:,height_axis]<=z.max()+tolerance)&(h>=a.min()-tolerance)&(h<=a.max()+tolerance)
        ids=np.flatnonzero(candidate)
        if not len(ids):continue
        m=np.array([[a[1]-a[0],a[2]-a[0]],[z[1]-z[0],z[2]-z[0]]])
        if abs(np.linalg.det(m))<1e-12:continue
        w=np.linalg.solve(m,np.array([h[ids]-a[0],q[ids,height_axis]-z[0]]));b=np.vstack((1-w.sum(axis=0),w))
        inside=np.all(b>=-tolerance,axis=0)&(radius[ids]<=radii[face]@b+tolerance)
        covered[ids[inside]]=True
    return covered
