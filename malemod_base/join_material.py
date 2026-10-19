"""Continuous material fields across a joined surface, independent of UV islands.

The caller supplies measured skin detail and a geometric support region. No
geometry, UVs or mechanical ownership are changed. Bake the same field on both
sides of a cut instead of reconciling just the two boundary texel rows.
"""
import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import dijkstra
from scipy.ndimage import map_coordinates, gaussian_filter


def smooth(x):
    x=np.clip(x,0,1)
    return x*x*x*(10+x*(-15+6*x))


def support_weights(points, faces, core, width, protected):
    p=np.asarray(points);f=np.asarray(faces);core=np.asarray(core,dtype=int)
    if width<=0 or not len(core):raise ValueError('Missing material support')
    edges=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
    distance=np.linalg.norm(p[edges[:,0]]-p[edges[:,1]],axis=1)
    graph=sparse.coo_matrix((np.r_[distance,distance],(np.r_[edges[:,0],edges[:,1]],np.r_[edges[:,1],edges[:,0]])),shape=(len(p),len(p))).tocsr()
    d=dijkstra(graph,directed=False,indices=core,min_only=True,limit=width)
    result=1-smooth(d/width)
    protected=np.asarray(protected,dtype=int)
    if np.intersect1d(core,protected).size:raise ValueError('Core overlaps protected surface')
    if len(protected):
        dp=dijkstra(graph,directed=False,indices=protected,min_only=True,limit=width)
        # Where the shaft band meets the pelvic ramp, taper on BOTH distance
        # fields. A hard protected mask would introduce a second material cut.
        near=np.isfinite(dp)&np.isfinite(d)
        result[near]*=smooth(dp[near]/np.maximum(dp[near]+d[near],1e-12))
    return result


def common_detail(tile, points, normals, repeats_per_unit):
    """Measured zero-mean detail sampled in rest space, not component UV space."""
    tile=np.asarray(tile,dtype=float)
    tile=tile-gaussian_filter(tile,sigma=(12,12,0),mode='reflect')
    tile=np.concatenate([tile,tile[:,::-1]],axis=1)
    tile=np.concatenate([tile,tile[::-1]],axis=0)
    p=np.asarray(points)*repeats_per_unit
    w=np.abs(normals)**4;w/=np.maximum(w.sum(-1,keepdims=True),1e-12)
    out=np.zeros((*p.shape[:-1],tile.shape[-1]))
    for axis,(a,b) in enumerate([(1,2),(2,0),(0,1)]):
        coords=np.stack([(p[...,b]%1)*(len(tile)-1),(p[...,a]%1)*(tile.shape[1]-1)])
        values=np.stack([map_coordinates(tile[:,:,k],coords,order=1,mode='wrap') for k in range(tile.shape[-1])],-1)
        out+=values*w[...,axis,None]
    return out


def pixel_triangles(uv, width, height):
    """Yield covered texel centers and barycentrics for the actual UV triangle."""
    q=np.asarray(uv)*[width,height]-.5
    lo=np.maximum(np.floor(q.min(0)).astype(int),0)
    hi=np.minimum(np.ceil(q.max(0)).astype(int),[width-1,height-1])
    if np.any(hi<lo):return
    a,b=q[1]-q[0],q[2]-q[0];det=a[0]*b[1]-a[1]*b[0]
    if abs(det)<1e-9:return
    x,y=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1));d=np.stack([x-q[0,0],y-q[0,1]],-1)
    v=(d[...,0]*b[1]-d[...,1]*b[0])/det;w=(a[0]*d[...,1]-a[1]*d[...,0])/det;u=1-v-w
    mask=(u>=-1e-9)&(v>=-1e-9)&(w>=-1e-9)
    if mask.any():yield y[mask],x[mask],np.stack([u[mask],v[mask],w[mask]],-1)
