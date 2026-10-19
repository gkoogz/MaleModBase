"""A dedicated material chart for generated skin; retained charts stay intact.

The strip uses measured arclength. Boundary color/normal samples come from the
two retained surfaces. The interior reuses a measured skin-detail tile rather
than stretching boundary texels over the new surface. Engine texture formats,
licensed sources and cooker integration belong to the adapter.
"""
import numpy as np
from scipy.ndimage import map_coordinates,gaussian_filter


def smooth(x):
    x=np.clip(x,0,1)
    return x*x*x*(10+x*(-15+6*x))


def chart(points,rings,faces,rectangle):
    p=np.asarray(points);r=np.asarray(rings);f=np.asarray(faces)
    edge=np.linalg.norm(np.roll(p[r[0]],-1,axis=0)-p[r[0]],axis=1)
    if np.any(edge<1e-12):raise ValueError('Repeated chart knot')
    s=np.r_[0,np.cumsum(edge[:-1])]/edge.sum()
    d=np.linalg.norm(np.diff(p[r],axis=0),axis=2);length=d.sum(0)
    if np.any(length<1e-12):raise ValueError('Collapsed material strip')
    t=np.vstack([np.zeros(len(s)),np.cumsum(d,axis=0)])/length
    mask=np.zeros(len(p),bool);mask[r]=True
    row=np.full(len(p),-1,dtype=int);row[r]=np.arange(len(r))[:,None]
    # A retained surface can contain a triangle whose three vertices are on
    # the attachment boundary. It is not a strip triangle and must keep its
    # original UVs; projecting it onto one chart row collapses its area.
    selected=mask[f].all(1)&(np.ptp(row[f],axis=1)>0)
    uv=np.zeros((len(p),2));uv[r,0]=s[None];uv[r,1]=t
    corners=uv[f[selected]].copy()
    wrap=np.ptp(corners[:,:,0],axis=1)>.5
    corners[wrap,:,0]+=corners[wrap,:,0]<.5
    lo=np.asarray(rectangle[:2]);hi=np.asarray(rectangle[2:])
    return selected,lo+corners*(hi-lo),s,t,length,edge.sum()


def sample(image,uv):
    data=np.asarray(image,dtype=float);v=np.asarray(uv)
    coords=np.stack([v[...,1]*(len(data)-1),v[...,0]*(data.shape[1]-1)])
    return np.stack([map_coordinates(data[:,:,i],coords,order=1,mode='nearest') for i in range(data.shape[2])],axis=-1)


def periodic_samples(values,s,queries):
    v=np.asarray(values);q=np.asarray(queries)%1
    return np.stack([np.interp(q,np.r_[s,1],np.r_[v[:,i],v[0,i]]) for i in range(v.shape[1])],axis=-1)


def surface_density(points,faces,corner_uv):
    """Area-weighted median UV scale, excluding collapsed source triangles.

    The caller selects one retained material island. Summing a cut boundary's
    UV segments instead can overcount zigzags and make generated detail tiny.
    """
    xyz=np.asarray(points)[faces];uv=np.asarray(corner_uv)
    area=np.linalg.norm(np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]),axis=1)
    du,dv=uv[:,1]-uv[:,0],uv[:,2]-uv[:,0]
    uv_area=np.abs(du[:,0]*dv[:,1]-du[:,1]*dv[:,0])
    valid=(area>1e-10)&(uv_area>1e-14)
    if not valid.any():raise ValueError('No measurable retained material surface')
    density=np.sqrt(uv_area[valid]/area[valid]);order=np.argsort(density)
    return float(density[order[np.searchsorted(np.cumsum(area[valid][order]),area[valid].sum()/2)]])


def bake_patch(lower,upper,s,tile,width,height,repeats,normal=False):
    """Boundary-exact patch with gradual pigment and consistent fine detail.

    Input normals must already use this chart's tangent frame. Caller supplies
    measured source skin detail, never an invented body texture. The returned
    first/last rows exactly sample the retained material boundaries.
    """
    u=np.linspace(0,1,width);t=np.linspace(0,1,height)[:,None,None]
    a=periodic_samples(lower,s,u);b=periodic_samples(upper,s,u)
    tile=np.asarray(tile,dtype=float)
    # Mirror the supplied tile to make a repeat with matching edge values.
    tile=np.concatenate([tile,tile[:,::-1]],axis=1)
    tile=np.concatenate([tile,tile[::-1]],axis=0)
    # The circumference is closed. An integer repeat count makes the detail
    # meet on both sides of its seam, rather than fixing only the final column.
    x,y=np.meshgrid((u*max(1,round(repeats[0])))%1,(np.linspace(0,1,height)*repeats[1])%1)
    detail=sample(tile,np.stack([x,y],axis=-1))
    if normal:
        mid=detail.copy();mid/=np.maximum(np.linalg.norm(mid,axis=2,keepdims=True),1e-12)
    else:
        # Retain the source skin's visible creases as well as its pore detail.
        # A tiny high-pass radius erased those folds on the generated strip.
        low=gaussian_filter(tile,sigma=(20,20,0),mode='wrap')
        detail=sample(tile-low,np.stack([x,y],axis=-1))
        aa=gaussian_filter(a,sigma=(max(1,width*.04),0),mode='wrap')
        bb=gaussian_filter(b,sigma=(max(1,width*.04),0),mode='wrap')
        blend=smooth((t-.25)/.75)
        mid=aa[None]*(1-blend)+bb[None]*blend+detail
    # Continue the retained sac's detail into the lower neck. A 2.5%-wide
    # switch made the two normal/detail fields look like a thick cuff even
    # with coincident vertices and matching endpoint tangents. Fade over the
    # lower fifth of measured meridian length; keep the mounted rim exact.
    start=smooth(t/.20);end=smooth((1-t)/.025)
    result=mid*start+a[None]*(1-start)
    result=result*end+b[None]*(1-end)
    if normal:result/=np.maximum(np.linalg.norm(result,axis=2,keepdims=True),1e-12)
    result[0]=a;result[-1]=b;result[:,-1]=result[:,0]
    return result
