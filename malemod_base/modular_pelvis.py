"""Exact original-body cut and compact, welded authoring ramp.

The body and anatomical module stay independent source objects. Only the
covered body patch and hidden module root are removed in the display union.
This does not implement native body-resource or material/physics adoption.
"""
import numpy as np
from scipy import sparse
from .graft import ordered_loop,sample_loop,refine_boundary
from .modular_anatomy import Mesh,clip,directed_boundary,vertex_normals,original_pelvis_ramp,ray_hits
from .modular_union import chart_cut
from .modular_shading import with_normal_field,unit,loft_normal_field


def ramp_orientation(points,rings,angles):
    """Compare strip winding with the smooth parametric ramp differential.

    Interpolating the body and shaft endpoint normals is not valid here: a
    raised upper arc turns through that normal range near the start, not at a
    constant rate. It can legitimately face backward while facing outward.
    A density-weighted Fourier differential follows that curvature, but rejects
    short reversed strips caused by irregular knots. This is a local test, not
    a global intersection proof.
    """
    q=np.asarray(points)[np.asarray(rings)];theta=np.asarray(angles)
    order=np.argsort(theta);a=theta[order];weight=np.empty(len(a))
    weight[order]=(np.roll(a,-1)-np.roll(a,1))%(2*np.pi)
    h=min(12,(len(theta)-1)//2)
    basis=np.column_stack([np.ones(len(a))]+[f(k*theta) for k in range(1,h+1) for f in (np.cos,np.sin)])
    derivative=np.column_stack([np.zeros(len(a))]+[v for k in range(1,h+1) for v in (-k*np.sin(k*theta),k*np.cos(k*theta))])
    coef=np.linalg.lstsq(basis*np.sqrt(weight[:,None]),q.transpose(1,0,2).reshape(len(a),-1)*np.sqrt(weight[:,None]),rcond=None)[0]
    fitted=(basis@coef).reshape(len(a),len(rings),3).transpose(1,0,2)
    angular=(derivative@coef).reshape(len(a),len(rings),3).transpose(1,0,2)
    t=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
    longitudinal=np.gradient(fitted,t,axis=0,edge_order=2)
    reference=np.cross(angular,longitudinal)
    reference/=np.maximum(np.linalg.norm(reference,axis=2)[:,:,None],1e-15)
    a=q[:-1];b=np.roll(a,-1,axis=1);c=q[1:];d=np.roll(c,-1,axis=1)
    na=reference[:-1];nb=np.roll(na,-1,axis=1);nc=reference[1:];nd=np.roll(nc,-1,axis=1)
    normals=np.stack([np.cross(a-b,c-b),np.cross(c-b,d-b)],axis=2)
    refs=np.stack([na+nb+nc,nb+nc+nd],axis=2)
    cosine=np.sum(normals*refs,axis=3)/np.maximum(np.linalg.norm(normals,axis=3)*np.linalg.norm(refs,axis=3),1e-30)
    return dict(reversedFaces=int(np.sum(cosine<=0)),minimumOutwardCosine=float(cosine.min()),reversedCells=np.argwhere(cosine<=0))


def _refine(mesh,loop,theta,knots,projection,root):
    a,b,t=sample_loop(loop,theta,knots)
    xy=(mesh.points-root)@projection.T
    ray=np.c_[np.cos(knots),np.sin(knots)];edge=xy[b]-xy[a]
    cross=lambda a,b:a[:,0]*b[:,1]-a[:,1]*b[:,0]
    t=np.clip(-cross(xy[a],ray)/cross(edge,ray),0,1)
    distance=abs(np.angle(np.exp(1j*(knots[:,None]-theta[None,:]))))
    nearest=distance.argmin(1);exact=distance[np.arange(len(knots)),nearest]<1e-9
    a[exact]=loop[nearest[exact]];b[exact]=loop[(nearest[exact]+1)%len(loop)];t[exact]=0
    ids=np.arange(len(mesh.points))
    matrix,f,seam,_,parents=refine_boundary(mesh.points,mesh.faces,ids,ids,loop,theta,knots,(a,b,t))
    uv=np.empty((len(f),3,2))
    for i,(face,parent) in enumerate(zip(f,parents)):
        uv[i]=matrix[face][:,mesh.faces[parent]].toarray()@mesh.corner_uv[parent]
    refined=Mesh(matrix@mesh.points,f,uv,matrix@mesh.donors,mesh.source_faces[parents],
                 np.r_[mesh.generated,np.ones(matrix.shape[0]-len(mesh.points),bool)],
                 None if mesh.normal_field is None else matrix@mesh.normal_field)
    refined.validate();return refined,seam


def pelvic_union(body,module,root,axis,radial,margin=.12,rows=20,mount_axis=None,mount_radial=None,underside_margin=.08):
    """Preserve the virgin body outside a measured shaft-sized opening.

    Both inputs must use outward winding and an explicit common donor bank.
    The body may retain pre-existing waist/ankle boundaries. New boundaries
    are shared mesh vertices, not coincident overlapping shells.
    """
    if body.donors.shape[1]!=module.donors.shape[1]:raise ValueError('Expected an explicit common source bank')
    if not np.isfinite([margin,underside_margin]).all() or margin<=0 or underside_margin<0:
        raise ValueError('Ramp margins must be finite and nonnegative, with a positive radial margin')
    body=with_normal_field(body);module=with_normal_field(module)
    root=np.asarray(root,dtype=float);axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    mount_axis=axis.copy() if mount_axis is None else np.asarray(mount_axis,dtype=float).copy()
    mount_axis/=np.linalg.norm(mount_axis)
    mount_radial=np.asarray(radial if mount_radial is None else mount_radial,dtype=float)
    u=np.array([0.,1,0]);u-=np.dot(u,axis)*axis
    if np.linalg.norm(u)<1e-5:u=np.cross(axis,[1.,0,0])
    u/=np.linalg.norm(u);v=np.cross(axis,u);projection=np.array([u,v])
    mount_u=np.array([0.,1,0]);mount_u-=np.dot(mount_u,mount_axis)*mount_axis
    if np.linalg.norm(mount_u)<1e-5:mount_u=np.cross(mount_axis,[1.,0,0])
    mount_u/=np.linalg.norm(mount_u);mount_projection=np.array([mount_u,np.cross(mount_axis,mount_u)])
    # Source-frame ventral direction follows the measured mount chart, not the
    # rotated shaft. Extra support fades to zero before reaching either side;
    # it must not widen the lateral collar or move the original pelvis.
    ventral=np.maximum(-unit(mount_radial)@mount_projection[1],0)**4
    radial_margin=margin+underside_margin*ventral
    support_radial=mount_radial*((1+radial_margin)/(1+margin))[:,None]
    proposal=original_pelvis_ramp(body.points,body.faces,root,support_radial,mount_axis,margin,rows)
    outer=proposal['points'][:len(mount_radial)]
    facing=float(axis@mount_axis)
    if facing<.12:raise ValueError('Shaft is outside the forward mount envelope')
    footprint=mount_radial@mount_projection.T*(1+radial_margin)[:,None]
    footprint=footprint[np.argsort(np.arctan2(footprint[:,1],footprint[:,0]))]
    if facing<.999:
        # Recruit the compressed arc from the actual oblique cylinder/body
        # intersection. The opposite underside can miss the body entirely;
        # its neutral support remains instead of creating a hole or chasing
        # the ray through the space between the legs.
        reach=np.linalg.norm(np.ptp(body.points,axis=0))+np.max(np.linalg.norm(radial,axis=1))
        hit,ids,_=ray_hits(body.points,body.faces,root+(1+radial_margin)[:,None]*np.asarray(radial)+reach*axis,-axis,allow_miss=True)
        compressed=mount_radial@(axis-facing*mount_axis)>0
        selected=(ids>=0)&compressed
        if selected.any():
            from scipy.spatial import ConvexHull
            cloud=np.vstack([footprint,(hit[selected]-root)@mount_projection.T])
            footprint=cloud[ConvexHull(cloud).vertices]
            footprint=footprint[np.argsort(np.arctan2(footprint[:,1],footprint[:,0]))]
            origins=root+footprint@mount_projection+reach*mount_axis
            outer,_,_=ray_hits(body.points,body.faces,origins,-mount_axis)
    # The body opening lives in the neutral body frame. Tilting its ray field
    # with the shaft sends the underside through empty space below the pelvis.
    # The shaft section may rotate independently, but must clear the body's
    # foremost support plane before the exposed ramp meets it.
    clearance=.35*np.median(np.linalg.norm(mount_radial,axis=1))
    station=(np.max((outer-root)@mount_axis)+clearance-np.min(np.asarray(radial)@mount_axis))/facing
    # Only excise the connected hidden root. A whole-object plane cut would
    # amputate posterior portions of the hanging sack below the same plane.
    offset=float(root@axis+station);negative=module.points@axis<offset
    from .graft import edges
    from scipy.sparse.csgraph import connected_components
    edge=edges(module.faces);edge=edge[negative[edge].all(1)]
    graph=sparse.coo_matrix((np.ones(len(edge)),(edge[:,0],edge[:,1])),shape=(len(module.points),)*2).tocsr()
    _,labels=connected_components(graph,directed=False)
    seed=np.argmin(np.where(negative,np.linalg.norm(module.points-root,axis=1),np.inf))
    hidden=negative&(labels==labels[seed]);selected=hidden[module.faces].any(1)
    if np.any(hidden&~module.generated):
        raise ValueError('Root cut would remove retained anatomy; reserve a clear shaft interval before mounting')
    cut_module=clip(module,axis,offset,selected)
    loops=directed_boundary(cut_module)
    if len(loops)!=1:raise ValueError('Pelvic ramp must end on one complete shaft section')
    ml,mt=ordered_loop(cut_module.points,loops[0],projection,root)
    chart=(body.points-root)@mount_projection.T
    f=body.faces;normal=np.cross(body.points[f[:,1]]-body.points[f[:,0]],body.points[f[:,2]]-body.points[f[:,0]])
    eligible=normal@mount_axis>1e-10
    original_loops=directed_boundary(body)
    protected=set(int(i) for loop in original_loops for i in loop)
    opened,_,_=chart_cut(body,chart,footprint,eligible)
    loops=directed_boundary(opened)
    candidates=[loop for loop in loops if not set(map(int,loop))&protected]
    if len(candidates)!=1 or len(loops)!=len(original_loops)+1:
        raise ValueError('Pelvic opening must preserve all original body boundaries')
    bl,bt=ordered_loop(opened.points,candidates[0],mount_projection,root)
    # Share roundoff-equivalent angles across the two boundaries. ordered_loop
    # already rejects this spacing within either boundary. _refine explicitly
    # retains each side's own original vertex rather than moving it to a mean.
    candidates=np.sort(np.r_[bt,mt]);knots=candidates[np.r_[True,np.diff(candidates)>1e-9]]
    if len(knots)>1 and knots[0]+2*np.pi-knots[-1]<1e-9:knots=knots[:-1]
    opened,bs=_refine(opened,bl,bt,knots,mount_projection,root)
    cut_module,ms=_refine(cut_module,ml,mt,knots,projection,root)
    # Match the outward body hole's directed boundary.
    if (int(bs[0]),int(bs[1])) not in {tuple(e) for e in np.vstack([opened.faces[:,[0,1]],opened.faces[:,[1,2]],opened.faces[:,[2,0]]])}:
        bs=bs[::-1];ms=ms[::-1];knots=knots[::-1]
    start=opened.points[bs];end=cut_module.points[ms]
    # Cuts carry the original shading field directly. Never estimate it from
    # the remaining one-sided triangles or invert anatomical source donors.
    first_normal=unit(opened.normal_field[bs])
    last_normal=unit(cut_module.normal_field[ms])
    distance=np.linalg.norm(end-start,axis=1)
    # Leave the retained body INTO its opening, even when a raised end ring is
    # above it. Projecting end-start instead can point out of the opening and
    # fold the first strips back over the original body.
    first=-(start-root)+np.outer((start-root)@mount_axis,mount_axis)
    first-=np.einsum('ij,ij->i',first,first_normal)[:,None]*first_normal
    first/=np.maximum(np.linalg.norm(first,axis=1)[:,None],1e-15)
    seam_ventral=np.maximum(-unit((start-root)-np.outer((start-root)@mount_axis,mount_axis))@mount_projection[1],0)**4
    first*=np.minimum(distance,2*(margin+underside_margin*seam_ventral)*np.median(np.linalg.norm(mount_radial,axis=1)))[:,None]
    last=distance[:,None]*axis
    points=[opened.points,cut_module.points];faces=[opened.faces,cut_module.faces+len(opened.points)]
    uvs=[opened.corner_uv,cut_module.corner_uv];donors=[opened.donors,cut_module.donors]
    generated=[opened.generated,cut_module.generated];sources=[opened.source_faces,np.where(cut_module.source_faces<0,-1,cut_module.source_faces+len(body.faces))]
    offset=len(opened.points)+len(cut_module.points);rings=[bs];count=len(bs)
    for row in range(1,rows+1):
        t=.5*(1-np.cos(np.pi*row/rows))
        if row==rows:ring=ms+len(opened.points)
        else:
            p=(2*t**3-3*t*t+1)*start+(t**3-2*t*t+t)*first+(-2*t**3+3*t*t)*end+(t**3-t*t)*last
            ring=np.arange(offset,offset+count);offset+=count;points.append(p)
            donors.append((1-t)*opened.donors[bs]+t*cut_module.donors[ms]);generated.append(np.ones(count,bool))
        previous=rings[-1];fs=[];uv=[]
        for j in range(count):
            k=(j+1)%count;a,b,c,d=int(previous[j]),int(previous[k]),int(ring[j]),int(ring[k])
            fs.extend([[b,a,c],[b,c,d]])
            u0=j/count;u1=(j+1)/count;v0=.5*(1-np.cos(np.pi*(row-1)/rows))
            uv.extend([[[u1,v0],[u0,v0],[u0,t]],[[u1,v0],[u0,t],[u1,t]]])
        rings.append(ring);faces.append(np.asarray(fs));uvs.append(np.asarray(uv));sources.append(np.full(len(fs),-1))
    p=np.vstack(points);f=np.vstack(faces)
    audit=ramp_orientation(p,rings,knots)
    if audit['reversedFaces']:raise ValueError(f"Pelvic ramp reverses {audit['reversedFaces']} triangles")
    result=Mesh(p,f,np.concatenate(uvs),sparse.vstack(donors,format='csr'),np.concatenate(sources),np.concatenate(generated))
    result.normal_field=np.vstack([opened.normal_field,cut_module.normal_field,np.zeros((len(p)-len(opened.points)-len(cut_module.points),3))])
    result.normal_field[np.asarray(rings)]=loft_normal_field(p,rings,first_normal,last_normal)
    result.normal_field[:len(opened.points)]=opened.normal_field
    result.normal_field[len(opened.points):len(opened.points)+len(cut_module.points)]=cut_module.normal_field
    result.validate()
    # Remove vertices hidden by the two cut patches; preserve all source maps.
    used,inverse=np.unique(result.faces,return_inverse=True);old_to_new=np.full(len(result.points),-1);old_to_new[used]=np.arange(len(used))
    result=Mesh(result.points[used],inverse.reshape(-1,3),result.corner_uv,result.donors[used],result.source_faces,result.generated[used],result.normal_field[used])
    report=result.validate();final_loops=directed_boundary(result)
    if len(final_loops)!=len(original_loops):raise ValueError('Pelvic union leaves an unexpected open seam')
    expected={tuple(sorted((int(old_to_new[a]),int(old_to_new[b])))) for loop in original_loops for a,b in zip(loop,np.roll(loop,-1))}
    actual={tuple(sorted((int(a),int(b)))) for loop in final_loops for a,b in zip(loop,np.roll(loop,-1))}
    if expected!=actual or any(a<0 for edge in expected for a in edge):
        raise ValueError('Pelvic union changed an original body-part boundary')
    retained=np.flatnonzero(old_to_new[:len(body.points)]>=0)
    displacement=float(np.linalg.norm(result.points[old_to_new[retained]]-body.points[retained],axis=1).max())
    report.update(rampReversedFaces=audit['reversedFaces'],rampMinimumOutwardCosine=audit['minimumOutwardCosine'],
                  rampOrientationReference='periodic surface differential; local test only',
                  rampMargin=margin,undersideExtraMargin=underside_margin,rampInnerStation=float(station),sharedOuterKnots=count,sharedInnerKnots=count,
                  outerFootprintWidth=float(np.ptp(footprint[:,0])),outerFootprintHeight=float(np.ptp(footprint[:,1])),
                  originalBodyBoundaryLoops=len(original_loops),finalBoundaryLoops=len(final_loops),
                  originalBoundaryEdgesPreserved=True,bodyDisplacement=displacement)
    return result,dict(rings=old_to_new[np.asarray(rings)],angles=knots,root=root,axis=axis,mount_axis=mount_axis,inner_station=station,
                       outer_normal=first_normal,inner_normal=last_normal,footprint=footprint),report
