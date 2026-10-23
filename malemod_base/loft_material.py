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


def bake_patch(lower,upper,s,tile,width,height,repeats,normal=False,continuations=None):
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
        # A source tile's mean slope belongs to its original surface, not to
        # the newly generated chart. Reuse its relief without imposing that
        # directional bias on the whole neck (a separate shaded patch).
        slopes=tile[:,:,:2]/np.maximum(tile[:,:,2,None],.1)
        slopes-=slopes.mean(axis=(0,1),keepdims=True)
        detail=sample(slopes,np.stack([x,y],axis=-1))
        mid=np.concatenate([detail,np.ones((*detail.shape[:2],1))],axis=2)
        mid/=np.maximum(np.linalg.norm(mid,axis=2,keepdims=True),1e-12)
    else:
        # Retain the source skin's visible creases as well as its pore detail.
        # A tiny high-pass radius erased those folds on the generated strip.
        low=gaussian_filter(tile,sigma=(20,20,0),mode='wrap')
        detail=sample(tile-low,np.stack([x,y],axis=-1))
        aa=gaussian_filter(a,sigma=(max(1,width*.04),0),mode='wrap')
        bb=gaussian_filter(b,sigma=(max(1,width*.04),0),mode='wrap')
        blend=smooth(t)
        mid=aa[None]*(1-blend)+bb[None]*blend+detail
    # Continue the retained sac's detail into the lower neck. A 2.5%-wide
    # switch made the two normal/detail fields look like a thick cuff even
    # with coincident vertices and matching endpoint tangents. Fade over the
    # outer fifths of measured meridian length; keep both rims exact.
    start=smooth(t/.20);end=smooth((1-t)/.20)
    # Transport broad pigment/normal direction farther than individual pores.
    # Extruding the complete boundary signal for 20% draws its fine texels into
    # vertical streaks. Fine detail hands over locally; the broad field remains
    # continuous through the longer transition.
    lower_field=gaussian_filter(a,sigma=(max(1,width*.006),0),mode='wrap')
    if normal:lower_field/=np.maximum(np.linalg.norm(lower_field,axis=1,keepdims=True),1e-12)
    result=mid*start+lower_field[None]*(1-start)
    result+=(a-lower_field)[None]*(1-smooth(t/.03))
    upper_field=gaussian_filter(b,sigma=(max(1,width*.006),0),mode='wrap')
    if normal:upper_field/=np.maximum(np.linalg.norm(upper_field,axis=1,keepdims=True),1e-12)
    result=result*end+upper_field[None]*(1-end)
    result+=(b-upper_field)[None]*(1-smooth((1-t)/.03))
    if continuations is not None:
        # A one-dimensional rim cannot describe folds crossing that rim.
        # Use the retained surface's two-dimensional material continuation
        # when the adapter can supply its original chart differential.
        if len(continuations)!=2:raise ValueError('Expected both material continuations')
        result=mid.copy()
        for field,weight in zip(continuations,(1-start,1-end)):
            field=np.asarray(field)
            if field.shape!=result.shape or not np.isfinite(field).all():
                raise ValueError('Invalid material continuation')
            result=result*(1-weight)+field*weight
    if normal:result/=np.maximum(np.linalg.norm(result,axis=2,keepdims=True),1e-12)
    result[0]=a;result[-1]=b;result[:,-1]=result[:,0]
    return result


def surface_uv_gradients(points,faces,uv):
    """Area-weighted chart derivatives, preserving split UV vertex aliases."""
    p=np.asarray(points);f=np.asarray(faces);v=np.asarray(uv)
    e1=p[f[:,1]]-p[f[:,0]];e2=p[f[:,2]]-p[f[:,0]]
    n=np.cross(e1,e2);area=np.linalg.norm(n,axis=1)
    valid=area>1e-12
    f=f[valid];e1=e1[valid];e2=e2[valid];n=n[valid];area=area[valid]
    a=v[f[:,1]]-v[f[:,0]];b=v[f[:,2]]-v[f[:,0]]
    gradients=(a[:,:,None]*np.cross(e2,n)[:,None,:]+b[:,:,None]*np.cross(n,e1)[:,None,:])/area[:,None,None]**2
    result=np.zeros((len(p),2,3));weight=np.zeros(len(p))
    for k in range(3):
        np.add.at(result,f[:,k],gradients*area[:,None,None])
        np.add.at(weight,f[:,k],area)
    return result/np.maximum(weight[:,None,None],1e-20)


def retained_skin_tile(points,faces,uv,patch,seeds,width,height,normal_image=None):
    """Measure the retained component reached from the source-side rim.

    UV aliases are welded only for component discovery; their original corner
    coordinates remain authoritative for the skin mask and texel density.
    """
    from PIL import Image,ImageDraw
    from scipy import sparse
    from scipy.sparse.csgraph import connected_components
    from scipy.ndimage import distance_transform_edt,uniform_filter,minimum_filter
    _,weld=np.unique(np.round(points,5),axis=0,return_inverse=True)
    triangles=faces[~patch];edges=weld[triangles]
    a=edges.ravel();b=np.roll(edges,-1,axis=1).ravel()
    graph=sparse.coo_matrix((np.ones(len(a)),(a,b)),shape=(weld.max()+1,)*2).tocsr()
    _,components=connected_components(graph,directed=False)
    selected=np.isin(components[weld[triangles]],np.unique(components[weld[seeds]])).all(1)
    coords=uv[triangles];selected&=np.ptp(coords[:,:,0],axis=1)<.1
    selected&=(coords[:,:,0]>.5).all(1)
    triangles=triangles[selected];coords=coords[selected]
    if len(triangles)<100:raise ValueError('Retained source skin component is incomplete')
    mask=Image.new('L',(1024,512));draw=ImageDraw.Draw(mask)
    for triangle in coords:draw.polygon([tuple(q*[1023,511]) for q in triangle],fill=255)
    distance=distance_transform_edt(np.asarray(mask)>0)
    y,x=np.unravel_index(distance.argmax(),distance.shape)
    radius=min(12.,distance[y,x]*.6)
    if radius<2:raise ValueError('No safe retained skin tile interior')
    center=np.array([x/1023,y/511]);pixel=np.rint(center*[width-1,height-1]).astype(int)
    half=np.maximum(4,np.floor(radius*np.array([width/1024,height/512])).astype(int))
    if normal_image is not None:
        # The widest UV interior may contain a flat band in a repeated source
        # normal map. Repeating that band creates rectangular smooth panels.
        # Choose a compact retained tile whose entire neighborhood contains
        # measured relief, rather than choosing a strong single crease.
        small=np.asarray(Image.fromarray(np.asarray(normal_image,dtype='u1')[:,:,:3]).resize(mask.size,Image.Resampling.BOX),dtype=float)
        energy=np.sum((small[:,:,:2]-127.5)**2,axis=2)
        score=minimum_filter(uniform_filter(energy,size=3),size=5)
        score=np.where(distance>=4,score,-1)
        y,x=np.unravel_index(np.argmax(score),score.shape)
        if score[y,x]<=0:raise ValueError('No consistently detailed retained normal tile')
        pixel=np.rint(np.array([x/1023,y/511])*[width-1,height-1]).astype(int)
        half=np.maximum(4,np.rint(np.array([width/1024,height/512])*2).astype(int))
    area=np.linalg.norm(np.cross(points[triangles[:,1]]-points[triangles[:,0]],points[triangles[:,2]]-points[triangles[:,0]]),axis=1)
    tex=coords*np.array([width-1,height-1]);du=tex[:,1]-tex[:,0];dv=tex[:,2]-tex[:,0]
    texarea=np.abs(du[:,0]*dv[:,1]-du[:,1]*dv[:,0]);valid=(area>1e-10)&(texarea>1e-10)
    density=np.sqrt(texarea[valid]/area[valid]);order=np.argsort(density)
    density=float(density[order[np.searchsorted(np.cumsum(area[valid][order]),area[valid].sum()*.5)]])
    return pixel,half,density,int(len(triangles))
