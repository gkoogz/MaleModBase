"""Source-preserving shading for welded authoring surfaces.

Normals belong to geometric vertices; UVs and their tangent handedness belong
to corners. An adapter must preserve this distinction when splitting its draw
buffers. This does not transfer textures or certify native normal-map shading.
"""
from dataclasses import replace
import numpy as np
from .modular_anatomy import vertex_normals


def unit(values):
    values=np.asarray(values,dtype=float)
    length=np.linalg.norm(values,axis=-1,keepdims=True)
    if not np.isfinite(values).all() or np.any(length<1e-12):
        raise ValueError('Undefined shading direction')
    return values/length


def with_normal_field(mesh):
    """Initialize once, before clipping; never recompute one-sided cut normals."""
    if mesh.normal_field is not None:return mesh
    return replace(mesh,normal_field=vertex_normals(mesh))


def loft_normal_field(points,rings,start,end,harmonics=12):
    """Differential of the loft, with inherited endpoint shading directions.

    Fit around the original boundary with arclength weights, not vertex count.
    Derivatives follow the actual cosine-spaced rows. Compact C2 corrections
    reconcile the geometric differential with each retained surface's shading
    field. Only the newly generated strip is affected; no position is changed.
    """
    q=np.asarray(points)[np.asarray(rings)]
    length=np.linalg.norm(np.roll(q[0],-1,axis=0)-q[0],axis=1)
    if np.any(length<1e-12):raise ValueError('Repeated loft boundary knot')
    theta=2*np.pi*np.r_[0,np.cumsum(length[:-1])]/length.sum()
    weight=(length+np.roll(length,1))*.5
    harmonics=min(harmonics,(len(theta)-1)//2)
    basis=np.column_stack([np.ones(len(theta))]+[f(k*theta) for k in range(1,harmonics+1) for f in (np.cos,np.sin)])
    derivative=np.column_stack([np.zeros(len(theta))]+[v for k in range(1,harmonics+1) for v in (-k*np.sin(k*theta),k*np.cos(k*theta))])
    coef=np.linalg.lstsq(basis*np.sqrt(weight[:,None]),q.transpose(1,0,2).reshape(len(theta),-1)*np.sqrt(weight[:,None]),rcond=None)[0]
    fitted=(basis@coef).reshape(len(theta),len(rings),3).transpose(1,0,2)
    angular=(derivative@coef).reshape(len(theta),len(rings),3).transpose(1,0,2)
    t=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
    field=unit(np.cross(angular,np.gradient(fitted,t,axis=0,edge_order=2)))
    # Ring winding can be either direction; orient the whole field coherently.
    if np.sum(field[0]*unit(start))<0:field=-field
    def fade(x):
        x=np.clip(x,0,1)
        return 1-x*x*x*(10+x*(-15+6*x))
    field+=fade(t/.2)[:,None,None]*(unit(start)-field[0])[None]
    field+=fade((1-t)/.2)[:,None,None]*(unit(end)-field[-1])[None]
    field=unit(field)
    field[0]=unit(start);field[-1]=unit(end)
    return field


def corner_frames(mesh):
    """Expand a welded surface into explicit draw corners without normal seams.

    Tangents follow each face's UV chart, including mirrored handedness. A
    singular UV face is identified explicitly and uses a geometric fallback;
    that fallback is not approval for a native textured material.
    """
    mesh=with_normal_field(mesh);mesh.validate()
    ids=mesh.faces
    normal=unit(mesh.normal_field[ids])
    p=mesh.points[ids];uv=mesh.corner_uv
    e1=p[:,1]-p[:,0];e2=p[:,2]-p[:,0]
    d1=uv[:,1]-uv[:,0];d2=uv[:,2]-uv[:,0]
    determinant=d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0]
    singular=abs(determinant)<1e-12
    safe=np.where(singular,1,determinant)
    tangent=(e1*d2[:,1,None]-e2*d1[:,1,None])/safe[:,None]
    bitangent=(e2*d1[:,0,None]-e1*d2[:,0,None])/safe[:,None]
    tangent[singular]=e1[singular];bitangent[singular]=e2[singular]
    # Accumulate only within the exact UV alias and handedness. Normals remain
    # shared across those groups; mirrored islands must never cancel tangents.
    geometric=np.cross(e1,e2)
    handedness=np.where(np.sum(np.cross(geometric,tangent)*bitangent,axis=1)<0,-1,1)
    keys=np.c_[ids.ravel(),uv.reshape(-1,2),np.repeat(handedness,3)]
    groups,inverse=np.unique(keys,axis=0,return_inverse=True)
    angle=np.empty(ids.shape)
    for j in range(3):
        a=unit(p[:,(j+1)%3]-p[:,j]);b=unit(p[:,(j+2)%3]-p[:,j])
        angle[:,j]=np.arccos(np.clip(np.sum(a*b,axis=1),-1,1))
    sums=np.zeros((len(groups),3));bsums=np.zeros_like(sums)
    np.add.at(sums,inverse,(angle[:,:,None]*tangent[:,None,:]).reshape(-1,3))
    np.add.at(bsums,inverse,(angle[:,:,None]*bitangent[:,None,:]).reshape(-1,3))
    tangent=sums[inverse].reshape(normal.shape);bitangent=bsums[inverse].reshape(normal.shape)
    tangent-=np.sum(tangent*normal,axis=2)[:,:,None]*normal
    collapsed=np.linalg.norm(tangent,axis=2)<1e-12
    if collapsed.any():
        n=normal[collapsed];basis=np.eye(3)[np.argmin(abs(n),axis=1)]
        tangent[collapsed]=np.cross(basis,n)
    tangent=unit(tangent)
    sign=np.where(np.sum(np.cross(normal,tangent)*bitangent,axis=2)<0,-1.,1.)
    return dict(geometric_vertices=ids,normal=normal,tangent=tangent,sign=sign,
                uv=uv.copy(),singular_uv_faces=np.flatnonzero(singular))
