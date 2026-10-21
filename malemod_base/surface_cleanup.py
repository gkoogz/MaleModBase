"""Remove sub-tolerance cut slivers without moving surviving surface samples.

The topology map is deliberately separate from the deformation graph. Removed
display samples may remain in that graph, so endpoint/loft dependencies do not
change. Each surviving face keeps its original UV chart and source lineage.
"""
from dataclasses import replace
import numpy as np
from .modular_anatomy import directed_boundary


def collapse_cut_slivers(mesh, protected=None, relative_area=1e-12,
                         relative_distance=1e-5):
    if not np.isfinite([relative_area,relative_distance]).all() or min(relative_area,relative_distance)<=0:
        raise ValueError('Cleanup tolerances must be finite and positive')
    mesh.validate()
    points=mesh.points
    diagonal=float(np.linalg.norm(np.ptp(points,axis=0)))
    threshold=diagonal*diagonal*relative_area
    distance_limit=diagonal*relative_distance
    fixed=~mesh.generated.copy()
    if protected is not None:
        protected=np.asarray(protected,dtype=np.int64)
        if protected.ndim!=1 or np.any(protected<0) or np.any(protected>=len(points)):
            raise ValueError('Protected cleanup sample leaves the surface')
        fixed[protected]=True
    for loop in directed_boundary(mesh):fixed[loop]=True
    faces=mesh.faces.copy();uv=mesh.corner_uv.copy()
    origins=np.arange(len(faces));mapping=np.arange(len(points))
    operations=[]

    def normals(f):
        q=points[f]
        return np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0])

    for iteration in range(512):
        area=np.linalg.norm(normals(faces),axis=1)
        bad=np.flatnonzero(area<threshold)
        if not len(bad):break
        accepted=False
        for fi in bad[np.argsort(area[bad])]:
            face=faces[fi]
            edges=[(int(face[j]),int(face[(j+1)%3])) for j in range(3)]
            edges.sort(key=lambda e:np.linalg.norm(points[e[0]]-points[e[1]]))
            for a,b in edges:
                distance=float(np.linalg.norm(points[a]-points[b]))
                if distance>distance_limit:continue
                for remove,keep in [(a,b),(b,a)]:
                    if fixed[remove]:continue
                    incident=np.flatnonzero(np.any(faces==remove,axis=1))
                    both=incident[np.any(faces[incident]==keep,axis=1)]
                    # The link condition forbids pinched vertices/new holes.
                    if len(both)!=2:continue
                    neighbors_a=set(faces[incident].ravel())-{remove,keep}
                    neighbors_b=set(faces[np.any(faces==keep,axis=1)].ravel())-{remove,keep}
                    opposite=set(faces[both].ravel())-{remove,keep}
                    if neighbors_a & neighbors_b != opposite:continue
                    retained=incident[~np.isin(incident,both)]
                    before=faces[retained];after=before.copy();after[after==remove]=keep
                    n=normals(before);new=normals(after)
                    old_area=np.linalg.norm(n,axis=1);new_area=np.linalg.norm(new,axis=1)
                    if np.any(new_area<np.minimum(old_area,threshold)*.999):continue
                    cosine=np.einsum('ij,ij->i',n,new)/np.maximum(old_area*new_area,1e-30)
                    if np.any(cosine<.995):continue
                    # Evaluate the replacement corner in each face's own
                    # chart. UV aliases across a material seam remain distinct.
                    revised=uv[retained].copy()
                    for index,(old,newface) in enumerate(zip(before,after)):
                        corner=int(np.flatnonzero(old==remove)[0])
                        q=points[old];basis=np.stack([q[1]-q[0],q[2]-q[0]],axis=1)
                        w=np.linalg.lstsq(basis,points[keep]-q[0],rcond=None)[0]
                        revised[index,corner]=np.array([1-w.sum(),*w])@uv[retained[index]]
                    faces[retained]=after;uv[retained]=revised
                    mask=np.ones(len(faces),bool);mask[both]=False
                    faces=faces[mask];uv=uv[mask];origins=origins[mask]
                    mapping[mapping==remove]=keep
                    operations.append((remove,keep,distance))
                    accepted=True;break
                if accepted:break
            if accepted:break
        if not accepted:
            raise ValueError(f'Cannot safely remove {len(bad)} cut slivers')
    else:raise ValueError('Cut sliver cleanup exceeded its bounded work limit')
    result=replace(mesh,faces=faces,corner_uv=uv,source_faces=mesh.source_faces[origins])
    result.validate()
    return result,dict(vertex_map=mapping,face_origins=origins,
        collapsed_edges=np.asarray(operations,dtype=float).reshape(-1,3),
        double_area_threshold=np.asarray(threshold),
        maximum_removed_edge=np.asarray(max((v[2] for v in operations),default=0.)))
