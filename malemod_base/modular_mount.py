"""Measure a front chart on the original body for immutable-topology recruitment."""
import numpy as np
from scipy.ndimage import distance_transform_edt,map_coordinates

def measure_chart(points,faces,origin,basis,side=161,step=.5):
    q=(np.asarray(points)-origin)@basis.T
    h=np.full((side,side),-np.inf);middle=(side-1)*.5
    for face in np.asarray(faces):
        tri=q[face];uv=tri[:,1:]/step+middle
        lo=np.maximum(np.floor(uv.min(0)).astype(int),0);hi=np.minimum(np.ceil(uv.max(0)).astype(int),side-1)
        if np.any(hi<lo):continue
        a,b,c=uv;ab=b-a;ac=c-a;det=ab[0]*ac[1]-ab[1]*ac[0]
        if abs(det)<1e-12:continue
        y,z=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1));v=np.stack([y-a[0],z-a[1]],-1)
        u=(v[...,0]*ac[1]-v[...,1]*ac[0])/det;w=(ab[0]*v[...,1]-ab[1]*v[...,0])/det
        mask=(u>=-1e-9)&(w>=-1e-9)&(u+w<=1+1e-9)
        value=tri[0,0]+u*(tri[1,0]-tri[0,0])+w*(tri[2,0]-tri[0,0])
        target=h[lo[1]:hi[1]+1,lo[0]:hi[0]+1];target[mask]=np.maximum(target[mask],value[mask])
    missing=~np.isfinite(h)
    if missing.all():raise ValueError('Original body has no front projection')
    # Outside the stock perineal silhouette, continue its nearest measured
    # height. These cells are recorded separately; they are not measured hits.
    nearest=distance_transform_edt(missing,return_distances=False,return_indices=True)
    h[missing]=h[tuple(nearest[:,missing])]
    return h,missing

def prepare_mount(rest,body_mask,shaft_mask,body_points,body_faces,origin,basis,ramp,length):
    h,missing=measure_chart(body_points,body_faces,origin,basis)
    q=(rest-origin)@basis.T;middle=(len(h)-1)*.5;step=.5
    height=map_coordinates(h,np.array([q[:,2]/step+middle,q[:,1]/step+middle]),order=1,mode='nearest')
    gap=np.maximum(height-q[:,0],0)
    weight=np.clip(1-gap/.75,0,1);weight=np.clip(weight**3*(10-15*weight+6*weight**2),0,1);weight*=body_mask
    outer=ramp['rings'][0];inner=ramp['rings'][-1]
    # Both exact boundary rows must use the same chart mapping, including the
    # underside where nearest-height extension is needed at very large sizes.
    weight[outer]=1
    theta=np.arctan2(q[outer,2],q[outer,1]);order=np.argsort(theta)
    return dict(origin=origin,basis=basis,step=step,side=len(h),height=h,bodyWeight=weight,shaftWeight=shaft_mask.astype(float),
                outer=outer[order],inner=inner[order],theta=theta[order],station=float(ramp['inner_station']),length=length,
                extendedChartCells=int(missing.sum()))
