"""Fit a source module by cutting its exact outline into a measured target.

The module's positions remain unchanged. Only target seam depth changes, within
an explicit caller budget. Original target vertices outside the cut are exact;
new vertices retain sparse barycentric source/target lineage and UV aliases.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
from .graft import topology_ids,boundary_loops,edges


@dataclass
class ReferenceGraft:
    points: np.ndarray
    faces: np.ndarray
    body_lineage: object
    module_lineage: object
    body_faces: np.ndarray
    module_faces: np.ndarray
    body_face_lineage: np.ndarray
    body_count: int
    seam_body: np.ndarray
    seam_module: np.ndarray
    maximum_body_move: float


def cut_reference(body,body_faces,module,module_faces,projection,forward,
                  maximum_body_move,tolerance=1e-5):
    import shapely
    from shapely.geometry import Polygon,Point
    body=np.asarray(body,float);module=np.asarray(module,float)
    bf=np.asarray(body_faces,int);mf=np.asarray(module_faces,int)
    projection=np.asarray(projection,float);forward=np.asarray(forward,float)
    if projection.shape!=(2,3) or forward.shape!=(3,) or not np.allclose(projection@projection.T,np.eye(2)) or not np.allclose(projection@forward,0):
        raise ValueError('Expected calibrated orthogonal projection/forward')
    if not np.isclose(forward@forward,1) or maximum_body_move<=0:raise ValueError('Invalid fit budget')
    mk,mi=topology_ids(module,tolerance);loops=boundary_loops(mi[mf])
    if len(loops)!=1:raise ValueError('Reference must have exactly one root boundary')
    loop=loops[0];boundary=module[mk[loop]];uv=boundary@projection.T
    opening=Polygon(uv)
    if not opening.is_valid or opening.area<=0:raise ValueError('Invalid projected reference boundary')
    # Extend the small depth correction into a compact target annulus. Moving
    # seam points alone creates nearly vertical slivers where a cut approaches
    # an existing target edge, despite a small absolute displacement.
    target_projection=body@projection.T
    root_edges=np.roll(uv,-1,axis=0)-uv
    query=target_projection[:,None,:]-uv[None,:,:]
    rt=np.clip(np.sum(query*root_edges[None,:,:],axis=2)/np.sum(root_edges*root_edges,axis=1)[None,:],0,1)
    closest=uv[None,:,:]+rt[:,:,None]*root_edges[None,:,:]
    distances=np.linalg.norm(closest-target_projection[:,None,:],axis=2)
    nearest=distances.argmin(1);distance=distances[np.arange(len(body)),nearest]
    support=2.0
    candidates=np.flatnonzero((distance<support)&(body@forward>boundary.mean(0)@forward-3))
    correction=np.zeros(len(body))
    projected=body[bf]@projection.T
    origin=projected[:,0];e=projected[:,1]-origin;f=projected[:,2]-origin
    det=e[:,0]*f[:,1]-e[:,1]*f[:,0];valid=np.abs(det)>1e-10
    if len(candidates):
        xy=closest[candidates,nearest[candidates]];delta=xy[:,None,:]-origin[None,:,:]
        u=(delta[:,:,0]*f[None,:,1]-delta[:,:,1]*f[None,:,0])/np.where(valid,det,1)[None,:]
        v=(e[None,:,0]*delta[:,:,1]-e[None,:,1]*delta[:,:,0])/np.where(valid,det,1)[None,:]
        inside=valid[None,:]&(u>=-1e-7)&(v>=-1e-7)&(u+v<=1+1e-7)
        depths=body[bf]@forward
        depth=depths[None,:,0]+u*(depths[None,:,1]-depths[None,:,0])+v*(depths[None,:,2]-depths[None,:,0])
        native=np.max(np.where(inside,depth,-1e9),axis=1)
        j=nearest[candidates];t=rt[candidates,j]
        desired=((1-t)[:,None]*boundary[j]+t[:,None]*boundary[(j+1)%len(boundary)])@forward
        x=np.clip(distance[candidates]/support,0,1);weight=1-x*x*x*(x*(x*6-15)+10)
        correction[candidates]=(desired-native)*weight
        if np.max(np.abs(correction))>maximum_body_move+1e-7:raise ValueError('Reference annulus exceeds target fitting budget')
    adjusted=body+correction[:,None]*forward
    points=list(adjusted.copy());lineage=[{i:1.}for i in range(len(body))]
    faces=[];face_lineage=[];seam=[];seam_bindings=[];edge_points={}
    maximum=float(np.max(np.abs(correction)))
    # Native UV aliases stay distinct. Original rows are never merged.
    for fi,face in enumerate(bf):
        triangle=body[face];q=target_projection[face];shape=Polygon(q)
        if shape.area<tolerance*tolerance or not shape.intersects(opening):
            faces.append(tuple(face));face_lineage.append(fi);continue
        # The projected outline may overlap rear skin. Keep the rear resource
        # intact by requiring the measured face to be in front of the module.
        if np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])@forward<=1e-10:
            faces.append(tuple(face));face_lineage.append(fi);continue
        clipped=shape.difference(opening)
        if clipped.equals(shape):faces.append(tuple(face));face_lineage.append(fi);continue
        if clipped.is_empty:continue
        generated={}
        matrix=np.vstack([q.T,np.ones(3)])
        if abs(np.linalg.det(matrix))<1e-12:raise ValueError('Singular target projection')
        inverse=np.linalg.inv(matrix)
        def vertex(xy):
            nonlocal maximum
            xy=np.asarray(xy,float);key=tuple(np.round(xy,10))
            if key in generated:return generated[key]
            weights=inverse@np.r_[xy,1.]
            nearest=int(np.argmax(weights))
            # Preserve original native vertex/attributes exactly unless it is
            # on the sewn boundary itself.
            source_edge=uv[np.roll(np.arange(len(uv)),-1)]-uv
            t=np.clip(np.sum((xy-uv)*source_edge,axis=1)/np.sum(source_edge*source_edge,axis=1),0,1)
            distance=np.linalg.norm(uv+source_edge*t[:,None]-xy,axis=1)
            edge=int(np.argmin(distance));on_seam=distance[edge]<tolerance
            if weights[nearest]>1-1e-8 and not on_seam:
                generated[key]=int(face[nearest]);return int(face[nearest])
            position=weights@adjusted[face]
            if on_seam:
                nxt=(edge+1)%len(loop);amount=float(t[edge]);desired=boundary[edge]*(1-amount)+boundary[nxt]*amount
                movement=abs(float((desired-weights@triangle)@forward));maximum=max(maximum,movement)
                if movement>maximum_body_move+1e-7:raise ValueError(f'Reference root exceeds target fitting budget: {movement:.9g}')
                position=desired
            index=len(points);points.append(position);lineage.append({int(i):float(w)for i,w in zip(face,weights)if abs(w)>1e-12});generated[key]=index
            if on_seam:
                a,b=int(loop[edge]),int(loop[(edge+1)%len(loop)])
                seam.append(index);seam_bindings.append((a,b,float(t[edge])))
                edge_points.setdefault(tuple(sorted((a,b))),[]).append((float(t[edge])if a<b else 1-float(t[edge]),index))
            return index
        polygons=list(clipped.geoms)if hasattr(clipped,'geoms')else[clipped]
        for polygon in polygons:
            if polygon.area<tolerance*tolerance:continue
            for tri in shapely.constrained_delaunay_triangles(polygon).geoms:
                xy=np.asarray(tri.exterior.coords)[:3];ids=[vertex(p)for p in xy]
                # Preserve the observed source triangle orientation.
                signed=(xy[1,0]-xy[0,0])*(xy[2,1]-xy[0,1])-(xy[1,1]-xy[0,1])*(xy[2,0]-xy[0,0])
                native_signed=np.linalg.det(np.stack([q[1]-q[0],q[2]-q[0]]))
                if signed*native_signed<0:ids[1],ids[2]=ids[2],ids[1]
                faces.append(tuple(ids));face_lineage.append(fi)
    # Split source boundary faces only, on exact original source edges. Added
    # positions/attributes are barycentric; no source sculpt is deformed.
    module_points=list(module.copy());module_rows=[{i:1.}for i in range(len(module))]
    source_inserts={};seam_module=[];cache={}
    for a,b,t in seam_bindings:
        if t<1e-8:index=int(mk[a])
        elif t>1-1e-8:index=int(mk[b])
        else:
            key=(min(a,b),max(a,b),round(t if a<b else 1-t,10))
            if key not in cache:
                index=len(module_points);cache[key]=index
                module_points.append(module[mk[a]]*(1-t)+module[mk[b]]*t)
                module_rows.append({int(mk[a]):1-t,int(mk[b]):t})
                source_inserts.setdefault(key[:2],[]).append((key[2],index))
            index=cache[key]
        seam_module.append(index)
    module_out=[]
    for face in mf:
        polygon=[]
        for a,b in zip(face,np.roll(face,-1)):
            polygon.append(int(a));ga,gb=int(mi[a]),int(mi[b]);key=tuple(sorted((ga,gb)))
            candidates=source_inserts.get(key,[])
            polygon.extend(i for _,i in sorted(((t if ga<gb else 1-t,i)for t,i in candidates)))
        if len(polygon)==3:module_out.append(tuple(polygon));continue
        center=len(module_points);module_points.append(module[face].mean(0));module_rows.append({int(i):1/3 for i in face})
        module_out.extend((center,a,b)for a,b in zip(polygon,np.roll(polygon,-1)))
    def matrix(rows,n):
        rr=[];cc=[];vv=[]
        for i,row in enumerate(rows):
            for j,w in row.items():rr.append(i);cc.append(j);vv.append(w)
        return sparse.csr_matrix((vv,(rr,cc)),shape=(len(rows),n))
    count=len(points);all_points=np.vstack([points,module_points]);all_faces=np.vstack([faces,np.asarray(module_out)+count])
    # Every root edge must be shared by exactly two oppositely wound triangles.
    k,alias=topology_ids(all_points,tolerance);directed=edges(alias[all_faces]);u,counts=np.unique(np.sort(directed,axis=1),axis=0,return_counts=True)
    if np.any(counts>2):raise ValueError('Reference fit creates nonmanifold edges')
    sb=np.asarray(seam,int);sm=np.asarray(seam_module,int)+count
    if not np.array_equal(alias[sb],alias[sm]):raise ValueError('Reference seam is not exact')
    original_boundary={int(x)for loop in boundary_loops(alias[:count][np.asarray(faces)])for x in loop}
    # The combined surface may retain native waist/ankle boundaries, but every
    # sewn module boundary vertex must be absent from the combined open loops.
    combined_boundary={int(x)for loop in boundary_loops(alias[all_faces])for x in loop}
    if set(alias[sb])&combined_boundary:raise ValueError('Reference seam remains open')
    seam_groups=set(alias[sb])
    for a,b in u:
        if a not in seam_groups or b not in seam_groups:continue
        matches=directed[np.all(np.sort(directed,axis=1)==[a,b],axis=1)]
        if len(matches)!=2 or not np.array_equal(matches[0],matches[1][::-1]):
            raise ValueError('Reference seam winding is inconsistent with target')
    return ReferenceGraft(all_points,all_faces,matrix(lineage,len(body)),matrix(module_rows,len(module)),
        np.asarray(faces),np.asarray(module_out),np.asarray(face_lineage),count,sb,sm,maximum)
