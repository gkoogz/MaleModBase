"""Independent anatomical surfaces with explicit source and attachment lineage.

Authoring utilities only. These do not replace a native runtime contract.
Coordinates and cut planes are measured inputs, never inferred engine units.
UVs are corner attributes so topological joins do not erase texture seams.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
from .graft import boundary_loops, edges, validate_mesh


@dataclass
class Mesh:
    points: np.ndarray
    faces: np.ndarray
    corner_uv: np.ndarray
    donors: sparse.csr_matrix
    source_faces: np.ndarray
    generated: np.ndarray
    # An interpolated direction field, deliberately independent of UV aliases.
    # Keep it unnormalized through cuts so subdivision does not change shading.
    normal_field: np.ndarray | None = None

    def reversed(self):
        return Mesh(self.points.copy(),self.faces[:,[0,2,1]].copy(),self.corner_uv[:,[0,2,1]].copy(),
                    self.donors.copy(),self.source_faces.copy(),self.generated.copy(),
                    None if self.normal_field is None else -self.normal_field.copy())

    @classmethod
    def source(cls, points, faces, uv):
        p=np.asarray(points,dtype=float).copy();f=np.asarray(faces,dtype=np.int64).copy()
        validate_mesh(p,f)
        return cls(p,f,np.asarray(uv,dtype=float)[f].copy(),sparse.eye(len(p),format='csr'),
                   np.arange(len(f),dtype=np.int64),np.zeros(len(p),dtype=bool))

    def validate(self, closed=False):
        validate_mesh(self.points,self.faces)
        if self.corner_uv.shape!=(len(self.faces),3,2) or not np.isfinite(self.corner_uv).all():
            raise ValueError('Invalid corner UV lineage')
        if self.donors.shape[0]!=len(self.points) or not np.allclose(np.asarray(self.donors.sum(1)).ravel(),1,atol=1e-10):
            raise ValueError('Invalid normalized source donors')
        if self.source_faces.shape!=(len(self.faces),) or self.generated.shape!=(len(self.points),):
            raise ValueError('Invalid source metadata')
        if self.normal_field is not None:
            if self.normal_field.shape!=self.points.shape or not np.isfinite(self.normal_field).all():
                raise ValueError('Invalid shading normal field')
            if np.any(np.linalg.norm(self.normal_field[np.unique(self.faces)],axis=1)<1e-12):
                raise ValueError('Undefined shading normal on visible surface')
        e=edges(self.faces);u,counts=np.unique(np.sort(e,axis=1),axis=0,return_counts=True)
        if (counts>2).any() or (closed and (counts!=2).any()):
            raise ValueError('Nonmanifold or open surface')
        # Adjacent faces must traverse a shared edge in opposite directions.
        directed=e[:,0]<e[:,1];_,inverse=np.unique(np.sort(e,axis=1),axis=0,return_inverse=True)
        winding=np.bincount(inverse,weights=np.where(directed,1,-1))
        if (winding[counts==2]!=0).any():raise ValueError('Inconsistent face winding')
        area=np.linalg.norm(np.cross(self.points[self.faces[:,1]]-self.points[self.faces[:,0]],
                                     self.points[self.faces[:,2]]-self.points[self.faces[:,0]]),axis=1)
        if (area<=1e-14).any():raise ValueError('Degenerate triangle')
        return dict(vertices=len(self.points),triangles=len(self.faces),boundaryEdges=int(sum(counts==1)),
                    generatedVertices=int(self.generated.sum()),minimumDoubleArea=float(area.min()))


def clip(mesh, normal, offset, faces_to_clip=None):
    """Keep dot(p,normal)>=offset; one edge donor per geometric intersection.

    Existing retained faces and corner attributes survive exactly. New cut rows
    retain sparse interpolation donors and each face's independent UV aliases.
    """
    normal=np.asarray(normal,dtype=float)
    if normal.shape!=(3,) or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-12 or not np.isfinite(offset):
        raise ValueError('Invalid cut plane')
    value=mesh.points@normal-offset
    if faces_to_clip is None:faces_to_clip=np.ones(len(mesh.faces),bool)
    faces_to_clip=np.asarray(faces_to_clip,dtype=bool)
    if faces_to_clip.shape!=(len(mesh.faces),):raise ValueError('Cut face mask must match topology')
    points=list(mesh.points);donors=[mesh.donors.getrow(i) for i in range(len(points))]
    generated=list(mesh.generated);cut_edges={};faces=[];uvs=[];sources=[]
    normal_field=None if mesh.normal_field is None else list(mesh.normal_field)
    for face,uv,source,selected in zip(mesh.faces,mesh.corner_uv,mesh.source_faces,faces_to_clip):
        if not selected:
            faces.append(face);uvs.append(uv);sources.append(source);continue
        polygon=[]
        for j in range(3):
            a=int(face[j]);b=int(face[(j+1)%3]);ua=uv[j];ub=uv[(j+1)%3]
            inside=value[a]>=0
            if inside:polygon.append((a,ua))
            if inside!=(value[b]>=0):
                t=value[a]/(value[a]-value[b]);key=tuple(sorted((a,b)))
                if t<1e-12:index=a
                elif t>1-1e-12:index=b
                elif key in cut_edges:index=cut_edges[key]
                else:
                    index=len(points);cut_edges[key]=index
                    points.append((1-t)*mesh.points[a]+t*mesh.points[b])
                    donors.append((1-t)*mesh.donors.getrow(a)+t*mesh.donors.getrow(b));generated.append(True)
                    if normal_field is not None:normal_field.append((1-t)*mesh.normal_field[a]+t*mesh.normal_field[b])
                polygon.append((index,(1-t)*ua+t*ub))
        clean=[]
        for item in polygon:
            if not clean or item[0]!=clean[-1][0]:clean.append(item)
        if len(clean)>1 and clean[0][0]==clean[-1][0]:clean.pop()
        for j in range(1,len(clean)-1):
            tri=[clean[0],clean[j],clean[j+1]]
            faces.append([x[0] for x in tri]);uvs.append([x[1] for x in tri]);sources.append(source)
    if not faces:raise ValueError('Cut removed the entire surface')
    faces=np.asarray(faces,dtype=np.int64);used,inverse=np.unique(faces,return_inverse=True)
    result=Mesh(np.asarray(points)[used],inverse.reshape(-1,3),np.asarray(uvs),
                sparse.vstack([donors[i] for i in used],format='csr'),np.asarray(sources),np.asarray(generated)[used],
                None if normal_field is None else np.asarray(normal_field)[used])
    result.validate();return result


def directed_boundary(mesh):
    loops=boundary_loops(mesh.faces)
    all_edges={tuple(map(int,e)) for e in edges(mesh.faces)}
    return [loop if (int(loop[0]),int(loop[1])) in all_edges else loop[::-1] for loop in loops]


def cap(mesh, loop):
    """Close a star-shaped planar authoring boundary; cap is internal at mount."""
    loop=np.asarray(loop,dtype=int);p=mesh.points[loop];center=p.mean(0)
    normals=np.cross(p-center,np.roll(p,-1,axis=0)-center);axis=normals.sum(0)
    if np.linalg.norm(axis)<1e-12 or (normals@axis<=0).any():
        raise ValueError('Cap boundary must be star-shaped; explicit patch required')
    index=len(mesh.points);faces=np.asarray([[int(b),int(a),index] for a,b in zip(loop,np.roll(loop,-1))])
    # Hidden cap UV chart, distinct from preserved visible corner UVs.
    local=p-center;u=local[0]/np.linalg.norm(local[0]);v=np.cross(axis/np.linalg.norm(axis),u)
    chart=np.column_stack([local@u,local@v]);scale=max(np.ptp(chart,axis=0).max(),1e-9);chart=chart/scale+.5
    cuv=np.array([[chart[(i+1)%len(loop)],chart[i],[.5,.5]] for i in range(len(loop))])
    return Mesh(np.vstack([mesh.points,center]),np.vstack([mesh.faces,faces]),np.concatenate([mesh.corner_uv,cuv]),
                sparse.vstack([mesh.donors,sparse.csr_matrix(mesh.donors[loop].mean(axis=0))],format='csr'),
                np.r_[mesh.source_faces,np.full(len(loop),-1)],np.r_[mesh.generated,True])


def refine_section(mesh,loop,center,u,v,samples=128):
    """Add evenly spaced angular knots without moving the retained surface."""
    from .graft import ordered_loop,sample_loop,refine_boundary
    projection=np.array([u,v]);ordered,theta=ordered_loop(mesh.points,loop,projection,center)
    regular=np.linspace(0,2*np.pi,samples,endpoint=False)
    regular=regular[np.min(abs(np.angle(np.exp(1j*(regular[:,None]-theta[None,:])))),axis=1)>1e-7]
    knots=np.sort(np.r_[theta,regular]);a,b,t=sample_loop(ordered,theta,knots)
    xy=(mesh.points-center)@projection.T;ray=np.c_[np.cos(knots),np.sin(knots)];edge=xy[b]-xy[a]
    cross=lambda a,b:a[:,0]*b[:,1]-a[:,1]*b[:,0]
    t=np.clip(-cross(xy[a],ray)/cross(edge,ray),0,1)
    ids=np.arange(len(mesh.points))
    matrix,faces,_,_,parents=refine_boundary(mesh.points,mesh.faces,ids,ids,ordered,theta,knots,(a,b,t))
    uv=np.empty((len(faces),3,2))
    for i,(face,parent) in enumerate(zip(faces,parents)):
        uv[i]=matrix[face][:,mesh.faces[parent]].toarray()@mesh.corner_uv[parent]
    result=Mesh(matrix@mesh.points,faces,uv,matrix@mesh.donors,mesh.source_faces[parents],
                 np.r_[mesh.generated,np.ones(matrix.shape[0]-len(mesh.points),bool)],
                 None if mesh.normal_field is None else matrix@mesh.normal_field)
    result.validate();return result


def fair_section(radial,axis,harmonics=8):
    """Fit a smooth section with angular weighting and exact transverse bounds.

    Only the newly reconstructed proximal surface uses this profile. The
    measured distal section and all original distal vertices remain exact.
    """
    u=np.array([0.,1,0]);u-=np.dot(u,axis)*axis
    if np.linalg.norm(u)<1e-5:u=np.cross(axis,[1.,0,0])
    u/=np.linalg.norm(u);v=np.cross(axis,u)
    theta=np.mod(np.arctan2(radial@v,radial@u),2*np.pi);order=np.argsort(theta);angle=theta[order]
    radius=np.linalg.norm(radial[order],axis=1)
    weights=(np.roll(angle,-1)-np.roll(angle,1))%(2*np.pi)
    def basis(t):return np.column_stack([np.ones(len(t))]+[f(k*t) for k in range(1,harmonics+1) for f in (np.cos,np.sin)])
    coef=np.linalg.lstsq(basis(angle)*np.sqrt(weights[:,None]),radius*np.sqrt(weights),rcond=None)[0]
    fitted=basis(theta)@coef
    if np.min(fitted)<=0 or np.max(abs((basis(angle)@coef)-radius))>.04*np.median(radius):
        raise ValueError('Measured section needs more detail than the bounded smooth reconstruction')
    xy=np.c_[fitted*np.cos(theta),fitted*np.sin(theta)];original=np.c_[radial@u,radial@v]
    # Preserve measured width, height and the asymmetric ventral extent.
    xy=(xy-xy.min(0))/np.ptp(xy,axis=0)*np.ptp(original,axis=0)+original.min(0)
    return xy[:,0,None]*u+xy[:,1,None]*v


def complete_shaft(distal, root, rows=16, refine=False):
    """Extend a measured complete shaft section to a root, retaining distal mesh.

    Each longitudinal row retains its section donor and independent material
    coordinate. The root cap makes this object closed without a scrotal mesh.
    No scrotal surface participates in the completion.
    """
    loops=directed_boundary(distal)
    if len(loops)!=1:raise ValueError('Shaft cut must expose exactly one boundary')
    if rows<2:raise ValueError('Need at least two longitudinal rows')
    loop=loops[0];section=distal.points[loop];center=section.mean(0);root=np.asarray(root,dtype=float)
    # Boundary tessellation density must not offset the centerline. Use the
    # polygon area centroid in its plane, not the average of edge-cut vertices.
    _,_,basis=np.linalg.svd(section-center,full_matrices=False);plane=(section-center)@basis[:2].T
    cross=plane[:,0]*np.roll(plane[:,1],-1)-np.roll(plane[:,0],-1)*plane[:,1]
    if abs(cross.sum())<1e-12:raise ValueError('Degenerate shaft section')
    center+=(((plane+np.roll(plane,-1,axis=0))*cross[:,None]).sum(0)/(3*cross.sum()))@basis[:2]
    if root.shape!=(3,) or not np.isfinite(root).all() or np.linalg.norm(center-root)<1e-6:raise ValueError('Invalid shaft root')
    axis=center-root;axis/=np.linalg.norm(axis)
    if refine:
        u=np.array([0.,1,0]);u-=np.dot(u,axis)*axis
        if np.linalg.norm(u)<1e-5:u=np.cross(axis,[1.,0,0])
        u/=np.linalg.norm(u);v=np.cross(axis,u)
        distal=refine_section(distal,loop,center,u,v)
        loop=directed_boundary(distal)[0];section=distal.points[loop]
    # Transport the measured cross-section without its scrotal bridge.
    radial=section-center;radial-=np.outer(radial@axis,axis)
    smooth_radial=fair_section(radial,axis) if refine else radial
    points=[distal.points];faces=[distal.faces];uvs=[distal.corner_uv];donors=[distal.donors]
    source_faces=[distal.source_faces];generated=[distal.generated]
    previous=loop;offset=len(distal.points);rings=[loop]
    for row in range(1,rows+1):
        t=row/rows;ring=np.arange(offset,offset+len(loop));offset+=len(loop)
        # Zero axial skew at the root, exact original section at the distal join.
        blend=min(t/.35,1.);blend=blend*blend*(3-2*blend)
        q=(1-t)*section+t*(root+radial)+blend*(smooth_radial-radial)
        points.append(q);donors.append(distal.donors[loop]);generated.append(np.ones(len(loop),dtype=bool))
        fs=[];uv=[]
        for j in range(len(loop)):
            k=(j+1)%len(loop);a,b,c,d=int(previous[j]),int(previous[k]),int(ring[j]),int(ring[k])
            fs.extend([[b,a,c],[b,c,d]])
            x0=j/len(loop);x1=(j+1)/len(loop);y0=1-(row-1)/rows;y1=1-t
            uv.extend([[[x1,y0],[x0,y0],[x0,y1]],[[x1,y0],[x0,y1],[x1,y1]]])
        faces.append(np.array(fs));uvs.append(np.array(uv));source_faces.append(np.full(len(fs),-1));previous=ring;rings.append(ring)
    result=Mesh(np.vstack(points),np.vstack(faces),np.concatenate(uvs),sparse.vstack(donors,format='csr'),np.concatenate(source_faces),np.concatenate(generated))
    boundary=directed_boundary(result)
    if len(boundary)!=1:raise ValueError('Shaft completion introduced extra boundaries')
    result=cap(result,boundary[0]);result.validate(closed=True)
    return result,np.asarray(rings),center,smooth_radial


def ray_hits(points, faces, origins, direction, allow_miss=False):
    """First positive intersections with measured triangles and exact donors."""
    direction=np.asarray(direction,dtype=float);direction=direction/np.linalg.norm(direction)
    tri=points[faces];a=tri[:,0];e1=tri[:,1]-a;e2=tri[:,2]-a
    h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h)
    inv=np.divide(1,det,out=np.zeros_like(det),where=abs(det)>1e-12)
    ids=[];weights=[]
    for origin in origins:
        s=origin-a;u=np.einsum('ij,ij->i',s,h)*inv;q=np.cross(s,e1);v=(q@direction)*inv;t=np.einsum('ij,ij->i',e2,q)*inv
        hits=np.flatnonzero((abs(det)>1e-12)&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(t>1e-8))
        if not len(hits):
            if not allow_miss:raise ValueError('Attachment ray misses original pelvis')
            ids.append(-1);weights.append([np.nan,np.nan,np.nan]);continue
        i=int(hits[np.argmin(t[hits])]);w=np.maximum([1-u[i]-v[i],u[i],v[i]],0);w/=sum(w);ids.append(i);weights.append(w)
    ids=np.array(ids);weights=np.array(weights)
    return np.einsum('ij,ijk->ik',weights,tri[ids]),ids,weights


def original_pelvis_ramp(body_points,body_faces,root,section_radial,axis,margin=.12,rows=12):
    """Explicit small surface bridge; no displacement field on the pelvis.

    Outer boundary is located by exact ray/triangle intersections on the virgin
    body. The interior body and shaft faces are retained in authoring objects;
    the adapter must cut their covered patch for a welded visible union.
    """
    if not .02<=margin<=.25 or rows<2:raise ValueError('Invalid compact ramp dimensions')
    root=np.asarray(root);axis=np.asarray(axis);axis=axis/np.linalg.norm(axis)
    radial=np.asarray(section_radial);radial=radial-np.outer(radial@axis,axis)
    radius=np.median(np.linalg.norm(radial,axis=1))
    reach=np.linalg.norm(np.ptp(body_points,axis=0))+radius
    outer,face_ids,bary=ray_hits(body_points,body_faces,root+(1+margin)*radial+reach*axis,-axis)
    # The inner ring lies on the complete shaft, just outside the foremost
    # original pelvic contact. This does not push the abdomen or thighs forward.
    station=np.max((outer-root)@axis)+.35*radius
    inner=root+station*axis+radial
    face=body_points[body_faces[face_ids]];normal=np.cross(face[:,1]-face[:,0],face[:,2]-face[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
    inward=-radial;inward-=np.einsum('ij,ij->i',inward,normal)[:,None]*normal
    inward/=np.maximum(np.linalg.norm(inward,axis=1)[:,None],1e-12)
    distance=np.linalg.norm(inner-outer,axis=1)
    points=[]
    for row in range(rows+1):
        t=row/rows;h00=2*t**3-3*t*t+1;h10=t**3-2*t*t+t;h01=-2*t**3+3*t*t;h11=t**3-t*t
        points.append(h00*outer+h10*inward*distance[:,None]+h01*inner+h11*axis*distance[:,None])
    points=np.vstack(points);count=len(radial);faces=[]
    for row in range(rows):
        for j in range(count):
            k=(j+1)%count;a=row*count+j;b=row*count+k;c=a+count;d=b+count
            faces.extend([[a,b,c],[b,d,c]])
    return dict(points=points,faces=np.asarray(faces),outer_body_faces=face_ids,outer_barycentric=bary,
                inner_station=station,root=root,radial=radial,body_modified=False)


@dataclass
class SurfaceAttachment:
    vertices: np.ndarray
    target_faces: np.ndarray
    barycentric: np.ndarray

    def targets(self, target):
        if self.barycentric.shape!=(len(self.vertices),3) or not np.allclose(self.barycentric.sum(1),1):
            raise ValueError('Invalid attachment barycentric coordinates')
        return np.einsum('ij,ijk->ik',self.barycentric,target.points[target.faces[self.target_faces]])


def radial_attachment(source, boundary, target, root, distal_center, flare=1., side_wrap_degrees=None):
    """Bind the detached rim to actual shaft triangles along radial rays.

    The binding is independent of the pelvis and is reevaluated after shaft
    deformation. It never feeds scrotal displacement back into shaft geometry.
    """
    if not np.isfinite(flare) or not 1<=flare<=2:raise ValueError('Invalid neck flare')
    root=np.asarray(root,dtype=float);axis=np.asarray(distal_center)-root;length=np.linalg.norm(axis);axis/=length
    rim=source.points[boundary];stations=(rim-root)@axis
    radial=rim-root-np.outer(stations,axis)
    down=radial.mean(0);down/=np.linalg.norm(down);side=np.cross(axis,down)
    angles=np.arctan2(radial@side,radial@down)
    if np.ptp(angles)>=np.pi:raise ValueError('Attachment footprint wraps around shaft')
    angles=angles*flare
    if side_wrap_degrees is not None:
        if not np.isfinite(side_wrap_degrees) or not 30<=side_wrap_degrees<=135:
            raise ValueError('Side wrap must stay between 30 and 135 degrees from underside')
        # Independent circumferential reach: the sack can reach up either side
        # without making its longitudinal footprint or the shaft larger.
        for positive in [False,True]:
            selected=angles>0 if positive else angles<0
            reach=np.max(abs(angles[selected]),initial=0)
            if reach<1e-8:raise ValueError('Neck has no bilateral footprint')
            angles[selected]*=np.deg2rad(side_wrap_degrees)/reach
    middle=(stations.min()+stations.max())*.5
    triangles=target.points[target.faces];a=triangles[:,0];e1=triangles[:,1]-a;e2=triangles[:,2]-a
    face_ids=[];bary=[]
    for station,angle in zip(middle+flare*(stations-middle),angles):
        if station<=0 or station>=length:raise ValueError('Scrotal rim leaves complete proximal shaft')
        if abs(angle)>=np.pi*.9:raise ValueError('Flared neck reaches dorsal centerline')
        origin=root+axis*station;direction=np.cos(angle)*down+np.sin(angle)*side
        h=np.cross(np.broadcast_to(direction,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
        inv=np.divide(1,det,out=np.zeros_like(det),where=abs(det)>1e-12);s=origin-a
        u=np.einsum('ij,ij->i',s,h)*inv;q=np.cross(s,e1);v=(q@direction)*inv;t=np.einsum('ij,ij->i',e2,q)*inv
        valid=(abs(det)>1e-12)&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(t>1e-8)
        hits=np.flatnonzero(valid)
        if not len(hits):raise ValueError('Scrotal rim ray missed shaft')
        face=int(hits[np.argmin(t[hits])]);weights=np.maximum([1-u[face]-v[face],u[face],v[face]],0);weights/=sum(weights)
        face_ids.append(face);bary.append(weights)
    return SurfaceAttachment(np.asarray(boundary),np.asarray(face_ids),np.asarray(bary))


def vertex_normals(mesh):
    """Corner-angle normals, invariant under coplanar triangle subdivision.

    Area weighting lets an arbitrarily small edge cut change the normal used
    by an attachment derivative. Tiny refined columns can then point opposite
    neighboring columns and fold the loft despite exact welded positions.
    """
    f=mesh.faces;p=mesh.points;n=np.zeros_like(p)
    face=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
    face/=np.maximum(np.linalg.norm(face,axis=1)[:,None],1e-15)
    for j in range(3):
        a=p[f[:,(j+1)%3]]-p[f[:,j]];b=p[f[:,(j+2)%3]]-p[f[:,j]]
        angle=np.arctan2(np.linalg.norm(np.cross(a,b),axis=1),np.einsum('ij,ij->i',a,b))
        np.add.at(n,f[:,j],face*angle[:,None])
    return n/np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-15)


def shaft_surface_normals(shaft,rings,root,center,harmonics=8):
    """Differential normals of the measured longitudinal surface.

    A cut can put two knots almost together with large intervals on their
    other sides. Even angle-weighted polygon normals change by half a facet
    angle over that tiny interval. Using them as Hermite control directions
    amplifies a tessellation artifact into a geometric fold. Fit positions in
    the section parameter, then differentiate that continuous field instead.
    Exact shaft positions, widths and source geometry are never changed here.
    """
    axis=np.asarray(center)-root;axis/=np.linalg.norm(axis)
    q=shaft.points[rings];radial=q[-1]-root
    u=np.array([0.,1,0]);u-=np.dot(u,axis)*axis
    if np.linalg.norm(u)<1e-5:u=np.cross(axis,[1.,0,0])
    u/=np.linalg.norm(u);v=np.cross(axis,u)
    theta=np.mod(np.arctan2(radial@v,radial@u),2*np.pi)
    order=np.argsort(theta);a=theta[order]
    weight=np.empty(len(a));weight[order]=(np.roll(a,-1)-np.roll(a,1))%(2*np.pi)
    basis=np.column_stack([np.ones(len(a))]+[f(k*theta) for k in range(1,harmonics+1) for f in (np.cos,np.sin)])
    derivative=np.column_stack([np.zeros(len(a))]+[value for k in range(1,harmonics+1) for value in (-k*np.sin(k*theta),k*np.cos(k*theta))])
    coef=np.linalg.lstsq(basis*np.sqrt(weight[:,None]),q.transpose(1,0,2).reshape(len(a),-1)*np.sqrt(weight[:,None]),rcond=None)[0]
    fitted=(basis@coef).reshape(len(a),len(rings),3).transpose(1,0,2)
    if np.max(np.linalg.norm(fitted-q,axis=2))>.04*np.median(np.linalg.norm(radial,axis=1)):
        raise ValueError('Section detail exceeds differential normal fit budget')
    circumferential=(derivative@coef).reshape(len(a),len(rings),3).transpose(1,0,2)
    longitudinal=np.gradient(fitted,axis=0,edge_order=2)
    normal=np.cross(circumferential,longitudinal)
    normal/=np.maximum(np.linalg.norm(normal,axis=2)[:,:,None],1e-15)
    normal*=np.where(np.sum(normal*radial[None,:,:],axis=2)>=0,1,-1)[:,:,None]
    result=vertex_normals(shaft);result[rings]=normal
    return result


def loft_orientation(points,rings,start_normals,end_normals):
    """Audit every loft strip against its transported outward normal field.

    This detects local reversed strips, including folds between correct end
    rings. It is not a global triangle/triangle self-intersection proof.
    """
    rings=np.asarray(rings);rows=len(rings)-1
    t=.5*(1-np.cos(np.pi*np.arange(rows+1)/rows))
    reference=(1-t[:,None,None])*start_normals+t[:,None,None]*end_normals
    p=np.asarray(points)[rings];a=p[:-1];b=np.roll(a,-1,axis=1);c=p[1:];d=np.roll(c,-1,axis=1)
    na=reference[:-1];nb=np.roll(na,-1,axis=1);nc=reference[1:];nd=np.roll(nc,-1,axis=1)
    normals=np.stack([np.cross(a-b,c-b),np.cross(c-b,d-b)],axis=2)
    refs=np.stack([na+nb+nc,nb+nc+nd],axis=2)
    cosine=np.sum(normals*refs,axis=3)/np.maximum(np.linalg.norm(normals,axis=3)*np.linalg.norm(refs,axis=3),1e-30)
    return dict(reversedFaces=int(np.sum(cosine<=0)),minimumOutwardCosine=float(cosine.min()),triangles=int(cosine.size))


def periodic_boundary_field(points,values,harmonics=12):
    """Fit a density-independent control field along a closed source boundary."""
    length=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1)
    if np.min(length)<=1e-12:raise ValueError('Repeated boundary points')
    theta=2*np.pi*np.r_[0,np.cumsum(length[:-1])]/length.sum()
    weight=(length+np.roll(length,1))*.5
    harmonics=min(harmonics,(len(points)-1)//2)
    basis=np.column_stack([np.ones(len(points))]+[f(k*theta) for k in range(1,harmonics+1) for f in (np.cos,np.sin)])
    coef=np.linalg.lstsq(basis*np.sqrt(weight[:,None]),values*np.sqrt(weight[:,None]),rcond=None)[0]
    return basis@coef


def trumpet_neck(source, attachment, shaft, rows=24, source_normals=None, shaft_normals=None):
    """Extend the detached sack to a wider, tangent-matched shaft footprint.

    Original sack vertices, triangles and corner UVs are untouched. A Hermite
    patch carries the source boundary conormal into the shaft tangent plane.
    The binding owns the *new* rim; its position and shading normals come from
    the same shaft triangles. An adapter must still refine/weld the visible
    union and transfer materials; overlapping objects are not a seam solution.
    """
    if rows<8:raise ValueError('Trumpet requires at least eight longitudinal rows')
    loops=directed_boundary(source)
    if len(loops)!=1:raise ValueError('Trumpet requires a single detached neck')
    loop=loops[0]
    order={int(v):i for i,v in enumerate(attachment.vertices)}
    if set(order)!=set(map(int,loop)):raise ValueError('Binding must cover exact neck boundary')
    permutation=[order[int(v)] for v in loop]
    binding=SurfaceAttachment(loop,attachment.target_faces[permutation],attachment.barycentric[permutation])
    start=source.points[loop];end=binding.targets(shaft)
    normals=vertex_normals(shaft) if shaft_normals is None else np.asarray(shaft_normals)
    if normals.shape!=shaft.points.shape or not np.isfinite(normals).all():
        raise ValueError('Target normal field must match complete shaft')
    end_normal=np.einsum('ij,ijk->ik',binding.barycentric,normals[shaft.faces[binding.target_faces]])
    end_normal/=np.linalg.norm(end_normal,axis=1)[:,None]
    if source_normals is None:source_normals=vertex_normals(source)
    source_normals=np.asarray(source_normals)
    if source_normals.shape!=source.points.shape or not np.isfinite(source_normals).all():
        raise ValueError('Source normal field must match retained sack')
    # The cut is planar. Its surface conormal is the cut-plane normal projected
    # onto the inherited tangent plane. Neighbor-chord estimates depend on knot
    # density and produce ripples after exact shaft-edge refinement adds knots.
    _,_,cut_frame=np.linalg.svd(start-start.mean(0),full_matrices=False)
    cut_normal=cut_frame[-1]
    if np.max(abs((start-start.mean(0))@cut_normal))>1e-7:
        raise ValueError('Trumpet source must use a measured planar cut')
    if np.mean((end-start)@cut_normal)<0:cut_normal=-cut_normal
    initial_normal=source_normals[loop]
    initial_normal=initial_normal/np.maximum(np.linalg.norm(initial_normal,axis=1)[:,None],1e-15)
    first=cut_normal-(initial_normal@cut_normal)[:,None]*initial_normal
    first/=np.maximum(np.linalg.norm(first,axis=1)[:,None],1e-15)
    # Boundary conormal points out of the retained sack. Never flip individual
    # rows to conceal a bad footprint: reject a footprint on the wrong side.
    if np.median(np.einsum('ij,ij->i',first,end-start))<=0:
        raise ValueError('Mounted footprint is behind retained neck boundary')
    last=end-end.mean(0)
    last-=np.einsum('ij,ij->i',last,end_normal)[:,None]*end_normal
    last/=np.maximum(np.linalg.norm(last,axis=1)[:,None],1e-15)
    distance=np.linalg.norm(end-start,axis=1)
    first*=distance[:,None];last*=distance[:,None]
    smooth_first=periodic_boundary_field(start,first)
    if np.min(distance)<1e-6:raise ValueError('Neck patch has no transition width')
    points=[source.points];faces=[source.faces];uvs=[source.corner_uv]
    donors=[source.donors];generated=[source.generated];sources=[source.source_faces]
    rings=[loop];previous=loop;offset=len(source.points);count=len(loop)
    for row in range(1,rows+1):
        # Concentrate samples near both tangent joins; uniform spacing leaves
        # the final polygon strip several degrees away from the shaft tangent.
        t=.5*(1-np.cos(np.pi*row/rows))
        p=(2*t**3-3*t*t+1)*start+(t**3-2*t*t+t)*smooth_first+(-2*t**3+3*t*t)*end+(t**3-t*t)*last
        # Inherit the exact original tangent at the retained edge, but contain
        # its tessellation noise in the first tenth of the NEW patch. The
        # correction and its first two derivatives vanish at that boundary.
        # No retained vertex or source normal is moved or replaced.
        local=t*max(1-t/.1,0)**3
        p+=local*(first-smooth_first)
        if row==rows:p=end.copy()
        ring=np.arange(offset,offset+count);offset+=count;points.append(p);rings.append(ring)
        donors.append(source.donors[loop]);generated.append(np.ones(count,bool))
        fs=[];uv=[]
        for j in range(count):
            k=(j+1)%count;a,b,c,d=int(previous[j]),int(previous[k]),int(ring[j]),int(ring[k])
            fs.extend([[b,a,c],[b,c,d]])
            u0=j/count;u1=(j+1)/count;v0=.5*(1-np.cos(np.pi*(row-1)/rows))
            uv.extend([[[u1,v0],[u0,v0],[u0,t]],[[u1,v0],[u0,t],[u1,t]]])
        faces.append(np.array(fs));uvs.append(np.array(uv));sources.append(np.full(len(fs),-1));previous=ring
    result=Mesh(np.vstack(points),np.vstack(faces),np.concatenate(uvs),sparse.vstack(donors,format='csr'),np.concatenate(sources),np.concatenate(generated))
    result.validate()
    audit=loft_orientation(result.points,rings,initial_normal,end_normal)
    if audit['reversedFaces']:
        raise ValueError(f"Neck loft reverses {audit['reversedFaces']} faces; boundary closure is insufficient")
    from .modular_shading import loft_normal_field
    result.normal_field=np.vstack([source_normals,np.zeros((len(result.points)-len(source.points),3))])
    result.normal_field[np.asarray(rings)]=loft_normal_field(result.points,rings,initial_normal,end_normal)
    # Preserve the source interpolation exactly, not just its direction.
    result.normal_field[:len(source.points)]=source_normals
    result.validate()
    rim=SurfaceAttachment(rings[-1],binding.target_faces,binding.barycentric)
    return result,rim,dict(rings=np.asarray(rings),start_derivative=first,end_derivative=last,
                          shaft_normals=end_normal,source_normals=initial_normal,
                          outward_minimum_cosine=audit['minimumOutwardCosine'],
                          reversed_faces=audit['reversedFaces'],retained_vertices=len(source.points))


def attach_neck(source, attachment, shaft, fixed_below):
    """Harmonic displacement of only the excised neck; lobe surface is exact.

    Fixed-below is a measured authoring plane in the supplied frame. A runtime
    adapter must move lobe-owned vertices from their own mechanical frame and
    solve this same boundary contract, not reclassify them by animated height.
    """
    from scipy.sparse.linalg import spsolve
    boundary=attachment.vertices;fixed=source.points[:,2]<=fixed_below
    if fixed[boundary].any():raise ValueError('Protected lobe overlaps attachment rim')
    locked=fixed.copy();locked[boundary]=True;free=np.flatnonzero(~locked)
    edge=np.unique(np.sort(edges(source.faces),axis=1),axis=0)
    w=1/np.maximum(np.linalg.norm(source.points[edge[:,0]]-source.points[edge[:,1]],axis=1),1e-9)
    adjacency=sparse.coo_matrix((np.r_[w,w],(np.r_[edge[:,0],edge[:,1]],np.r_[edge[:,1],edge[:,0]])),shape=(len(source.points),)*2).tocsr()
    lap=sparse.diags(np.asarray(adjacency.sum(1)).ravel())-adjacency
    delta=np.zeros_like(source.points);delta[boundary]=attachment.targets(shaft)-source.points[boundary]
    delta[free]=spsolve(lap[free][:,free],-lap[free][:,locked]@delta[locked])
    points=source.points+delta
    if not np.isfinite(points).all():raise ValueError('Unanchored scrotal neck')
    # The harmonic field is only a proposal. Preserve local orientation while
    # keeping both the exact mounted rim and all original lobe rows fixed.
    f=source.faces;reference=np.cross(source.points[f[:,1]]-source.points[f[:,0]],source.points[f[:,2]]-source.points[f[:,0]])
    area=np.linalg.norm(reference,axis=1);normal=reference/area[:,None];floor=.02*area
    proposal=points.copy();budget=max(np.linalg.norm(delta,axis=1).max()*.25,1e-8)
    for iteration in range(512):
        current=np.cross(points[f[:,1]]-points[f[:,0]],points[f[:,2]]-points[f[:,0]])
        signed=np.einsum('ij,ij->i',current,normal);bad=np.flatnonzero(signed<floor)
        if not len(bad):break
        for i in bad:
            a,b,c=f[i];g1=np.cross(points[c]-points[a],normal[i]);g2=np.cross(normal[i],points[b]-points[a]);gradient=np.array([-g1-g2,g1,g2])
            gradient[locked[f[i]]]=0;denominator=np.sum(gradient*gradient)
            if denominator<1e-24:raise ValueError('Mounted rim is infeasible without moving protected tissue')
            value=np.dot(np.cross(points[b]-points[a],points[c]-points[a]),normal[i])
            if value<floor[i]:points[f[i]]+=gradient*(1.05*floor[i]-value)/denominator
        if np.linalg.norm(points-proposal,axis=1).max()>budget:raise ValueError('Neck orientation repair exceeds local budget')
    else:raise ValueError('Scrotal neck orientation did not converge')
    result=Mesh(points,source.faces.copy(),source.corner_uv.copy(),source.donors.copy(),source.source_faces.copy(),source.generated.copy())
    result.validate()
    if not np.array_equal(points[fixed],source.points[fixed]):raise ValueError('Lobe changed during attachment')
    return result,fixed


def export_obj(mesh,path):
    mesh.validate()
    with open(path,'w',newline='\n') as out:
        out.write('# Modular source-derived authoring mesh. Not native gameplay evidence.\n')
        for p in mesh.points:out.write('v %.12g %.12g %.12g\n'%tuple(p))
        for uv in mesh.corner_uv.reshape(-1,2):out.write('vt %.12g %.12g\n'%tuple(uv))
        if mesh.normal_field is not None:
            n=mesh.normal_field/np.maximum(np.linalg.norm(mesh.normal_field,axis=1)[:,None],1e-15)
            for row in n:out.write('vn %.12g %.12g %.12g\n'%tuple(row))
        for i,face in enumerate(mesh.faces):
            out.write('f '+' '.join(f'{v+1}/{i*3+j+1}'+(f'/{v+1}' if mesh.normal_field is not None else '') for j,v in enumerate(face))+'\n')
