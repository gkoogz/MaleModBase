"""Authoring-only visible union; complete mechanical objects remain separate.

Requires Shapely 2.1 constrained triangulation. All surface positions are lifted
from original triangles through exact barycentric donors; no voxel remeshing.
"""
import numpy as np
from scipy import sparse
from .modular_anatomy import Mesh, SurfaceAttachment, directed_boundary, trumpet_neck, shaft_surface_normals
from .graft import refine_boundary


def chart_cut(mesh, chart, footprint, eligible):
    """Cut one chart-space footprint, retaining outside faces and corner UVs."""
    from shapely import Polygon, constrained_delaunay_triangles
    region=Polygon(footprint)
    if not region.is_valid or region.area<=1e-10:raise ValueError('Invalid attachment footprint')
    points=list(mesh.points);donors=[mesh.donors.getrow(i) for i in range(len(points))]
    generated=list(mesh.generated);faces=[];uv=[];lineage=[];chart_points=list(chart);cut_donors={}
    normal_field=None if mesh.normal_field is None else list(mesh.normal_field)
    def key(p):return tuple(np.rint(np.asarray(p)*1e9).astype(np.int64))
    registry={key(p):i for i,p in enumerate(points)}
    for fi,face in enumerate(mesh.faces):
        xy=chart[face]
        poly=Polygon(xy) if eligible[fi] else None
        if poly is None or not poly.intersects(region):
            faces.append(face);uv.append(mesh.corner_uv[fi]);lineage.append(mesh.source_faces[fi]);continue
        if not poly.is_valid or poly.area<=1e-12:raise ValueError('Degenerate surface chart')
        remainder=poly.difference(region)
        if remainder.is_empty:continue
        inverse=np.linalg.inv(np.vstack([xy.T,np.ones(3)]))
        reference=np.cross(mesh.points[face[1]]-mesh.points[face[0]],mesh.points[face[2]]-mesh.points[face[0]])
        for triangle in constrained_delaunay_triangles(remainder).geoms:
            coords=np.asarray(triangle.exterior.coords)[:3]
            weights=np.c_[coords,np.ones(3)]@inverse.T
            weights[abs(weights)<1e-10]=0
            if weights.min()<-1e-8:raise ValueError('Triangulation left source triangle')
            weights=np.maximum(weights,0);weights/=weights.sum(1)[:,None]
            q=weights@mesh.points[face];ids=[]
            for p,w,xy_new in zip(q,weights,coords):
                k=key(p);index=registry.get(k)
                if index is None:
                    index=len(points);registry[k]=index;points.append(p)
                    donors.append(sparse.csr_matrix(w.reshape(1,-1))@mesh.donors[face]);generated.append(True)
                    chart_points.append(xy_new)
                    if normal_field is not None:normal_field.append(w@mesh.normal_field[face])
                cut_donors[index]=(fi,w.copy())
                ids.append(index)
            # Exact collinear chart knots can leave roundoff-width slivers in
            # polygon clipping. They collapse to one edge in the shared vertex
            # registry and must not become a face with repeated vertex IDs.
            if len(set(ids))<3:continue
            if np.dot(np.cross(q[1]-q[0],q[2]-q[0]),reference)<0:
                ids=[ids[0],ids[2],ids[1]];weights=weights[[0,2,1]]
            faces.append(ids);uv.append(weights@mesh.corner_uv[fi]);lineage.append(mesh.source_faces[fi])
    result=Mesh(np.asarray(points),np.asarray(faces),np.asarray(uv),sparse.vstack(donors,format='csr'),np.asarray(lineage),np.asarray(generated),
                None if normal_field is None else np.asarray(normal_field))
    result.validate()
    return result,np.asarray(chart_points),cut_donors


def _attachment_cut(shaft,binding,shaft_rings,root,center):
    axis=np.asarray(center)-root;axis/=np.linalg.norm(axis)
    source_rim=binding.targets(shaft);stations=(source_rim-root)@axis
    down=(source_rim-root-np.outer(stations,axis)).mean(0);down/=np.linalg.norm(down)
    side=np.cross(axis,down)
    q=shaft.points-root;chart=np.c_[q@axis,np.arctan2(q@side,q@down)]
    footprint=np.einsum('ij,ijk->ik',binding.barycentric,chart[shaft.faces[binding.target_faces]])
    # Only longitudinal proximal faces live in this chart; caps, original tip
    # and the dorsal wraparound seam cannot be candidates for the cut.
    ringmask=np.zeros(len(shaft.points),bool);ringmask[np.asarray(shaft_rings).ravel()]=True
    eligible=ringmask[shaft.faces].all(1)&(np.ptp(chart[shaft.faces][...,1],axis=1)<np.pi)
    opened,cut_chart,cut_donors=chart_cut(shaft,chart,footprint,eligible)
    loops=directed_boundary(opened)
    if len(loops)!=1:raise ValueError('Display cut must create one shaft boundary')
    hole=loops[0]
    # Keep the actual triangle/edge intersection donors from the cutter. A
    # nearest-face search would lose certainty on coarse or narrow triangles.
    hole_xy=cut_chart[hole];hole_donors=[cut_donors[int(i)] for i in hole]
    edge=np.roll(footprint,-1,axis=0)-footprint;length2=(edge*edge).sum(1)
    records=[]
    for i,xy in enumerate(hole_xy):
        t=np.clip(np.einsum('ij,ij->i',xy-footprint,edge)/length2,0,1)
        error=np.linalg.norm(xy-(footprint+t[:,None]*edge),axis=1);j=int(error.argmin())
        if error[j]>1e-7:raise ValueError('Shaft cut boundary leaves attachment curve')
        if t[j]>1-1e-8:j=(j+1)%len(edge);u=0.
        elif t[j]<1e-8:u=0.
        else:u=float(t[j])
        records.append((j+u,int(hole[i]),j,u,*hole_donors[i]))
    records.sort(key=lambda r:r[0])
    return opened,records


def _refine_rim(scrotum,loop,records):
    samples=np.array([r[0] for r in records])
    if len(np.unique(np.round(samples,8)))!=len(samples):raise ValueError('Repeated attachment knot')
    a=np.array([loop[r[2]] for r in records]);b=np.array([loop[(r[2]+1)%len(loop)] for r in records]);u=np.array([r[3] for r in records])
    count=len(scrotum.points);ids=np.arange(count)
    matrix,f,seam,_,face_ids=refine_boundary(scrotum.points,scrotum.faces,ids,ids,loop,np.arange(len(loop)),samples,(a,b,u))
    # Attribute transfer is per original face: UV aliases never get averaged.
    uv=np.empty((len(f),3,2))
    for i,(face,fi) in enumerate(zip(f,face_ids)):
        uv[i]=matrix[face][:,scrotum.faces[fi]].toarray()@scrotum.corner_uv[fi]
    refined=Mesh(matrix@scrotum.points,f,uv,matrix@scrotum.donors,scrotum.source_faces[face_ids],
                 np.r_[scrotum.generated,np.ones(matrix.shape[0]-count,bool)],
                 None if scrotum.normal_field is None else matrix@scrotum.normal_field)
    return refined,seam,matrix


def _join(opened,scrotum,seam,shaft_seam,closed=True):
    if opened.donors.shape[1]!=scrotum.donors.shape[1]:
        raise ValueError('Both objects must name the same explicit source donor bank')
    if np.max(np.linalg.norm(scrotum.points[seam]-opened.points[shaft_seam],axis=1))>1e-7:
        raise ValueError('Attempted weld of different boundary surfaces')
    from .modular_shading import unit
    field=None
    if opened.normal_field is not None and scrotum.normal_field is not None:
        if np.max(np.linalg.norm(unit(opened.normal_field[shaft_seam])-unit(scrotum.normal_field[seam]),axis=1))>1e-7:
            raise ValueError('Welded boundary has different shading normals')
        field=np.vstack([opened.normal_field,scrotum.normal_field])
    mapping=np.arange(len(scrotum.points))+len(opened.points);mapping[seam]=shaft_seam
    union=Mesh(np.vstack([opened.points,scrotum.points]),np.vstack([opened.faces,mapping[scrotum.faces]]),np.concatenate([opened.corner_uv,scrotum.corner_uv]),
               sparse.vstack([opened.donors,scrotum.donors],format='csr'),
               np.r_[opened.source_faces,scrotum.source_faces],np.r_[opened.generated,scrotum.generated],field)
    # Drop covered, unreferenced shaft vertices and duplicate seam rows.
    used,inverse=np.unique(union.faces,return_inverse=True)
    union=Mesh(union.points[used],inverse.reshape(-1,3),union.corner_uv,union.donors[used],union.source_faces,union.generated[used],
               None if field is None else field[used])
    union.validate(closed=closed)
    return union


def charted_trumpet_union(target,detached,loop,chart,footprint,eligible,rows=24,source_shift=None):
    """Exact graft across a measured compound support, preserving old holes.

    Unlike the cylindrical shaft-only cutter, this supports a chart spanning
    a root transition and retained body. Both inputs use the same donor bank.
    """
    original_boundaries=directed_boundary(target)
    protected={int(i) for boundary in original_boundaries for i in boundary}
    opened,cut_chart,cut_donors=chart_cut(target,chart,footprint,eligible)
    boundaries=directed_boundary(opened)
    candidates=[boundary for boundary in boundaries if not any(int(i) in protected for i in boundary)]
    if len(candidates)!=1 or len(boundaries)!=len(original_boundaries)+1:
        raise ValueError('Expanded contact must preserve existing body boundaries')
    edge=np.roll(footprint,-1,axis=0)-footprint;length2=(edge*edge).sum(1)
    records=[]
    for vertex in candidates[0]:
        xy=cut_chart[vertex]
        t=np.clip(np.einsum('ij,ij->i',xy-footprint,edge)/length2,0,1)
        error=np.linalg.norm(xy-footprint-t[:,None]*edge,axis=1);j=int(error.argmin())
        if error[j]>1e-7:raise ValueError('Contact cut left its measured curve')
        if t[j]>1-1e-8:j=(j+1)%len(edge);u=0.
        elif t[j]<1e-8:u=0.
        else:u=float(t[j])
        face,weights=cut_donors[int(vertex)]
        records.append((j+u,int(vertex),j,u,face,weights))
    records.sort(key=lambda row:row[0])
    refined,seam,matrix=_refine_rim(detached,np.asarray(loop),records)
    binding=SurfaceAttachment(seam,np.array([r[4] for r in records]),np.array([r[5] for r in records]))
    baseline=refined.points.copy();translation=np.zeros(3);applied_shift=0.
    def loft(shift=0.):
        refined.points=baseline-shift*direction if source_shift is not None else baseline
        return trumpet_neck(refined,binding,target,rows=rows,
            source_normals=refined.normal_field,shaft_normals=target.normal_field,surface_conormal=True)
    if source_shift is None:
        neck,rim,info=loft()
    else:
        direction,requested_shift=source_shift;direction=np.asarray(direction,float)
        if direction.shape!=(3,) or not np.isfinite(direction).all() or abs(np.linalg.norm(direction)-1)>1e-8 or not np.isfinite(requested_shift) or requested_shift<0:
            raise ValueError('Invalid compound source translation')
        # Follow the newly measured support centroid as far as the retained
        # skin can reach without folding. This changes no contact-area samples.
        def try_loft(shift):
            try:return loft(shift)
            except ValueError as error:
                if str(error)!='No orientation-preserving tangent length for neck footprint':raise
                return None
        best=try_loft(requested_shift)
        if best is not None:applied_shift=float(requested_shift)
        else:
            best=loft(0.);low=0.;high=float(requested_shift)
            for _ in range(9):
                mid=(low+high)*.5;candidate=try_loft(mid)
                if candidate is None:high=mid
                else:low=mid;best=candidate
            applied_shift=low
        neck,rim,info=best;translation=-applied_shift*direction
    info['source_translation']=translation
    info['geometry_binding_revision']=np.asarray(1,dtype=np.uint32)
    # These are donors on the compound support, NOT shaft-frame assignments.
    # Evaluate the recruited body/root surface before evaluating this rim.
    # Keeping them explicit prevents downstream authoring from accidentally
    # making the rear rim rigid or assigning pelvic points to shaft frames.
    info['support_triangles']=target.faces[rim.target_faces].copy()
    info['support_barycentric']=rim.barycentric.copy()
    info['support_face_ids']=rim.target_faces.copy()
    info['source_refinement_indptr']=matrix.indptr.copy()
    info['source_refinement_indices']=matrix.indices.copy()
    info['source_refinement_data']=matrix.data.copy()
    info['source_refinement_shape']=np.asarray(matrix.shape)
    incident=[set() for _ in detached.points]
    for fi,face in enumerate(detached.faces):
        for v in face:incident[int(v)].add(fi)
    source_triangles=[];source_barycentric=[]
    for vertex in info['rings'][0]:
        donor=matrix.getrow(int(vertex));ids=donor.indices[donor.data>1e-12]
        candidates=set.intersection(*(incident[int(v)] for v in ids))
        if not candidates:raise ValueError('Refined source rim has no retained triangle owner')
        face=detached.faces[min(candidates)]
        weights=donor[:,face].toarray().ravel()
        if abs(weights.sum()-1)>1e-10:raise ValueError('Source rim donor lost weight')
        source_triangles.append(face);source_barycentric.append(weights)
    info['source_triangles']=np.asarray(source_triangles)
    info['source_barycentric']=np.asarray(source_barycentric)
    source_order={int(v):i for i,v in enumerate(seam)}
    ordered=[records[source_order[int(v)]] for v in info['rings'][0]]
    target_seam=np.array([record[1] for record in ordered])
    info['source_curve_parameter']=np.asarray([record[0] for record in ordered])
    union=_join(opened,neck,rim.vertices,target_seam,closed=not original_boundaries)
    final_boundaries=directed_boundary(union)
    if len(final_boundaries)!=len(original_boundaries):raise ValueError('Expanded graft changed body boundaries')
    report=union.validate(closed=not original_boundaries)
    report.update(originalBoundaryLoops=len(original_boundaries),finalBoundaryLoops=len(final_boundaries),
        neckReversedFaces=int(info['reversed_faces']),neckMinimumOutwardCosine=float(info['outward_minimum_cosine']),
        neckMinimumAdjacentCosine=float(info['minimum_adjacent_cosine']),
        retainedTargetDisplacement=0.,sharedSeamVertices=len(target_seam),appliedSourceShift=applied_shift)
    return union,neck,rim,info,report


def visible_union(shaft,scrotum,binding,shaft_rings,root,center):
    """Weld an already lofted rim, rejecting a fold caused by refinement."""
    opened,records=_attachment_cut(shaft,binding,shaft_rings,root,center)
    refined,seam,matrix=_refine_rim(scrotum,binding.vertices,records)
    rest=refined.points.copy();shaft_seam=np.array([r[1] for r in records]);refined.points[seam]=opened.points[shaft_seam]
    f=refined.faces;p=refined.points
    before=np.cross(rest[f[:,1]]-rest[f[:,0]],rest[f[:,2]]-rest[f[:,0]])
    after=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
    reversed_faces=int(np.sum(np.einsum('ij,ij->i',before,after)<=0))
    if reversed_faces:raise ValueError('Exact rim refinement reverses neck triangles; rebuild the neck with the refined boundary')
    union=_join(opened,refined,seam,shaft_seam);report=union.validate(True)
    report.update(sharedSeamVertices=len(seam),shaftInputClosed=shaft.validate(True)['boundaryEdges']==0,
                  refinementReversedFaces=0,maximumRimRefinement=float(np.linalg.norm(p[seam]-rest[seam],axis=1).max()))
    return union,report


def trumpet_union(shaft,detached,binding,shaft_rings,root,center,source_normals,rows=24):
    """Cut/refine first, then loft to the exact shaft boundary without snapping.

    This order prevents fine end strips from being inverted when a curved
    shaft edge introduces additional knots after the loft has been constructed.
    """
    from dataclasses import replace
    normal_field=shaft_surface_normals(shaft,shaft_rings,root,center)
    shaft=replace(shaft,normal_field=normal_field)
    detached=replace(detached,normal_field=np.asarray(source_normals).copy())
    opened,records=_attachment_cut(shaft,binding,shaft_rings,root,center)
    refined,seam,matrix=_refine_rim(detached,binding.vertices,records)
    target=SurfaceAttachment(seam,np.array([r[4] for r in records]),np.array([r[5] for r in records]))
    scrotum,rim,neck=trumpet_neck(refined,target,shaft,rows=rows,source_normals=matrix@source_normals,shaft_normals=normal_field)
    # trumpet_neck orders the input boundary by face winding; recover its
    # original canonical knot order explicitly for the welded display surface.
    source_order={int(v):i for i,v in enumerate(seam)}
    order=[source_order[int(v)] for v in neck['rings'][0]]
    shaft_seam=np.array([records[i][1] for i in order])
    union=_join(opened,scrotum,rim.vertices,shaft_seam);report=union.validate(True)
    report.update(sharedSeamVertices=len(seam),shaftInputClosed=shaft.validate(True)['boundaryEdges']==0,
                  refinementReversedFaces=0,neckReversedFaces=int(neck['reversed_faces']),
                  neckMinimumOutwardCosine=float(neck['outward_minimum_cosine']),
                  maximumRimRefinement=0.,method='cut before loft; differential target normals')
    return union,scrotum,rim,neck,report
