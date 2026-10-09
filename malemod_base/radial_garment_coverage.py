"""Measured radial garment coverage; retains cuts and caller topology."""
import numpy as np

def radial_coverage(surface, triangles, queries, *, radial_axes, height_axis, tolerance=1e-7, cut_inset=0., check_depth=True):
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
        inside=np.all(b>=-tolerance,axis=0)
        if check_depth:inside &= radius[ids]<=radii[face]@b+tolerance
        covered[ids[inside]]=True
    if not np.isfinite(cut_inset) or cut_inset<0:raise ValueError('Invalid cut inset')
    if cut_inset:
        # A cut is an open positional edge, not a render UV seam. Keep body
        # support near that edge: reference coverage cannot certify a posed cut.
        _,aliases=np.unique(np.round(p,6),axis=0,return_inverse=True);edges={}
        for face in t:
            for i,j in ((0,1),(1,2),(2,0)):
                a,b=map(int,face[[i,j]]);key=tuple(sorted((int(aliases[a]),int(aliases[b]))))
                count,donors=edges.get(key,(0,(a,b)));edges[key]=(count+1,donors)
        for count,(a,b) in edges.values():
            if count!=1:continue
            scale=(radii[a]+radii[b])*.5
            delta=(angles[b]-angles[a]+np.pi)%(2*np.pi)-np.pi
            edge=np.array([delta*scale,p[b,height_axis]-p[a,height_axis]])
            ids=np.flatnonzero(covered)
            chart=np.column_stack((((theta[ids]-angles[a]+np.pi)%(2*np.pi)-np.pi)*scale,q[ids,height_axis]-p[a,height_axis]))
            parameter=np.clip(chart@edge/max(float(edge@edge),1e-15),0,1)
            gap=np.linalg.norm(chart-parameter[:,None]*edge,axis=1)
            covered[ids[gap<cut_inset]]=False
    return covered


def radial_triangle_coverage(surface, triangles, body, body_triangles, *, radial_axes, height_axis, cut_inset, check_depth=True):
    """Conservative whole-face mask with cut support and interior witnesses.

    Removing a body face because its corners are covered can leave a hole at a
    concave neckline. Check edge/interior witnesses as well; preserve the full
    original body face if any witness enters an opening or the protected cut.
    The inset is an adapter-measured reference-frame distance, not a posed
    clearance claim. This changes only visibility, never the body surface.
    check_depth=False authorizes masking the body beneath the garment's chart
    footprint even where stock cloth folds lie slightly inside the skin. This
    is a clothing visibility mask, not collision or containment evidence.
    """
    p=np.asarray(body,float);t=np.asarray(body_triangles,int)
    args=dict(radial_axes=radial_axes,height_axis=height_axis,cut_inset=cut_inset,check_depth=check_depth)
    vertices=radial_coverage(surface,triangles,p,**args)
    mask=np.all(vertices[t],axis=1);ids=np.flatnonzero(mask)
    witnesses=np.array([[.5,.5,0],[.5,0,.5],[0,.5,.5],[1/3,1/3,1/3],[.5,.25,.25],[.25,.5,.25],[.25,.25,.5]])
    queries=np.einsum('sk,fkd->fsd',witnesses,p[t[ids]])
    covered=radial_coverage(surface,triangles,queries.reshape(-1,3),**args)
    mask[ids]=np.all(covered.reshape(-1,len(witnesses)),axis=1)
    return mask
