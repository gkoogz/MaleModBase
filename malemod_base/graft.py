"""Engine-independent rest-pose graft fitting with explicit vertex lineage.

The caller selects a body patch and supplies calibrated, aligned module points.
Both boundary loops must be star-shaped in the caller's projection. Their union
is subdivided on original edges; UV aliases remain separate render vertices.
This fits a rest surface. It does not implement a live deformation/physics bridge.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
from scipy.spatial import cKDTree
from scipy.sparse.csgraph import dijkstra
from scipy.sparse.linalg import spsolve
from scipy.optimize import minimize


def topology_ids(points, tolerance=1e-5):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all() or tolerance <= 0:
        raise ValueError('Expected finite Nx3 points and positive tolerance')
    parent = np.arange(len(p))
    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for a,b in sorted(cKDTree(p).query_pairs(tolerance)):
        a,b=root(a),root(b)
        parent[max(a,b)]=min(a,b)
    representative = np.array([root(i) for i in range(len(p))])
    unique, inverse = np.unique(representative, return_inverse=True)
    return unique,inverse


def edges(faces):
    f=np.asarray(faces,dtype=np.int64)
    return np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]])


def validate_mesh(points, faces):
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Expected finite Nx3 mesh points')
    if faces.ndim!=2 or faces.shape[1]!=3 or not len(faces) or faces.min()<0 or faces.max()>=len(points):
        raise ValueError('Invalid triangle indices')


def boundary_loops(faces):
    e=edges(faces); undirected=np.sort(e,axis=1)
    u, first, count=np.unique(undirected,axis=0,return_index=True,return_counts=True)
    if np.any(count>2): raise ValueError('Nonmanifold surface')
    boundary=e[first[count==1]]
    adjacency={}
    for a,b in boundary:
        adjacency.setdefault(int(a),[]).append(int(b));adjacency.setdefault(int(b),[]).append(int(a))
    if any(len(v)!=2 for v in adjacency.values()): raise ValueError('Boundary is not a set of closed loops')
    remaining=set(adjacency);loops=[]
    while remaining:
        start=min(remaining);loop=[start];prev=start;current=adjacency[start][0]
        while current!=start:
            if current not in remaining: raise ValueError('Boundary loop intersects itself')
            loop.append(current);remaining.remove(current)
            nxt=adjacency[current];prev,current=current,(nxt[0] if nxt[0]!=prev else nxt[1])
        remaining.remove(start);loops.append(np.asarray(loop,dtype=int))
    return loops


def ordered_loop(points, loop, projection, center):
    xy=(points[loop]-center)@projection.T
    theta=np.mod(np.arctan2(xy[:,1],xy[:,0]),2*np.pi)
    order=np.argsort(theta);ordered=loop[order];theta=theta[order]
    actual={tuple(sorted(e)) for e in zip(loop,np.roll(loop,-1))}
    angular={tuple(sorted(e)) for e in zip(ordered,np.roll(ordered,-1))}
    if actual!=angular or np.min(np.diff(np.r_[theta,theta[0]+2*np.pi]))<1e-7:
        raise ValueError('Boundary must be star-shaped around the measured center')
    return ordered,theta


def sample_loop(loop, theta, samples):
    position=np.searchsorted(theta,samples,side='right')-1
    a=position%len(loop);b=(a+1)%len(loop)
    start=theta[a];end=theta[b];end=np.where(end<=start,end+2*np.pi,end)
    s=np.where(samples<start,samples+2*np.pi,samples)
    t=np.clip((s-start)/(end-start),0,1)
    return loop[a],loop[b],t


def refine_boundary(points, faces, ids, representatives, loop, theta, samples):
    """Split boundary edges; return a sparse map from every output to its input."""
    n=len(points); lineage=[{i:1.} for i in range(n)]
    a,b,t=sample_loop(loop,theta,samples)
    records={}; seam=[]; donors=[]
    # Each geometric edge can have only one surviving boundary triangle. Keep
    # that triangle's actual render indices so UV/skin seams retain their side.
    edge_render={tuple(sorted((int(ids[x]),int(ids[y])))):(int(x),int(y))
                 for x,y in edges(faces) if ids[x]!=ids[y]}
    for x,y,u in zip(a,b,t):
        key=tuple(sorted((int(x),int(y))))
        rx,ry=edge_render[key]
        if ids[rx]!=x: rx,ry=ry,rx
        if u<1e-8: index=rx
        elif u>1-1e-8: index=ry
        else:
            index=len(lineage);lineage.append({rx:1-float(u),ry:float(u)})
            records.setdefault(key,[]).append((float(u),index,int(x),int(y)))
        seam.append(index);donors.append((rx,ry,float(u)))
    new_faces=[];face_lineage=[]
    for fi,tri in enumerate(faces):
        polygon=[];split=False
        for x,y in zip(tri,np.roll(tri,-1)):
            polygon.append(int(x));key=tuple(sorted((int(ids[x]),int(ids[y]))))
            inserts=records.get(key,[])
            if inserts:
                split=True
                ordered=sorted(((u if ids[x]==a else 1-u,index) for u,index,a,b in inserts))
                polygon.extend(index for _,index in ordered)
        if split:
            center=len(lineage);lineage.append({int(i):1/3 for i in tri})
            new_faces.extend((center,x,y) for x,y in zip(polygon,np.roll(polygon,-1)))
            face_lineage.extend([fi]*len(polygon))
        else: new_faces.append(tuple(tri));face_lineage.append(fi)
    rows=[];cols=[];values=[]
    for row,entry in enumerate(lineage):
        for col,value in entry.items():rows.append(row);cols.append(col);values.append(value)
    matrix=sparse.csr_matrix((values,(rows,cols)),shape=(len(lineage),n))
    return matrix,np.asarray(new_faces,dtype=int),np.asarray(seam),np.asarray(donors),np.asarray(face_lineage)


def harmonic_fit(points, faces, boundary, targets, support_distance, tolerance):
    """Smooth rest-fit displacement; lock the supplied seam and distant surface."""
    keep,ids=topology_ids(points,tolerance);p=points[keep];f=ids[faces]
    e=np.unique(np.sort(edges(f),axis=1),axis=0);e=e[e[:,0]!=e[:,1]]
    lengths=np.linalg.norm(p[e[:,0]]-p[e[:,1]],axis=1)
    if np.any(lengths<=0):raise ValueError('Degenerate topology edge')
    graph=sparse.coo_matrix((np.r_[lengths,lengths],(np.r_[e[:,0],e[:,1]],np.r_[e[:,1],e[:,0]])),shape=(len(p),len(p))).tocsr()
    boundary_ids=ids[boundary]
    if len(np.unique(boundary_ids))!=len(boundary_ids):raise ValueError('Repeated seam sample')
    distance=dijkstra(graph,directed=False,indices=boundary_ids,min_only=True)
    fixed=(distance>=support_distance);fixed[boundary_ids]=True
    displacement=np.zeros_like(p);displacement[boundary_ids]=targets-p[boundary_ids]
    weights=1/lengths
    adjacency=sparse.coo_matrix((np.r_[weights,weights],(np.r_[e[:,0],e[:,1]],np.r_[e[:,1],e[:,0]])),shape=graph.shape).tocsr()
    laplace=sparse.diags(np.asarray(adjacency.sum(axis=1)).ravel())-adjacency
    free=np.flatnonzero(~fixed);locked=np.flatnonzero(fixed)
    if len(free): displacement[free]=spsolve(laplace[free][:,free],-laplace[free][:,locked]@displacement[locked])
    return points+displacement[ids],distance[ids]


def preserve_orientation(reference, fitted, faces, locked, maximum_move, tolerance=1e-5):
    """Repair local folds while keeping the seam fixed, with a hard move budget.

    Enforces positive signed area relative to each reference face normal. This
    detects local orientation loss; it is not a global self-intersection test.
    Geometry and every UV alias use the same topological solve.
    """
    if not np.isfinite(maximum_move) or maximum_move<=0:raise ValueError('Invalid repair budget')
    keep,ids=topology_ids(reference,tolerance);r=reference[keep];q=fitted[keep].copy();f=ids[faces]
    ref=np.cross(r[f[:,1]]-r[f[:,0]],r[f[:,2]]-r[f[:,0]])
    area=np.linalg.norm(ref,axis=1)
    if np.any(area<1e-14):raise ValueError('Degenerate reference face')
    normals=ref/area[:,None]
    def ratios(p):
        t=p[f];return (np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])*normals).sum(1)/area
    bad=ratios(q)<.05
    if not bad.any():return fitted.copy(),{'repairedFaces':0,'maximumMove':0.,'minimumAreaRatio':float(ratios(q).min())}
    selected=set(f[bad].ravel());e=edges(f)
    for _ in range(2):selected.update(e[np.isin(e,list(selected)).any(1)].ravel())
    free=np.array(sorted(selected-set(ids[locked])),dtype=int)
    if not len(free):raise ValueError('Orientation failure lies entirely on locked boundary')
    affected=np.isin(f,free).any(1);af=f[affected];an=normals[affected];aa=area[affected]
    original=q.copy();fixed=q.copy()
    for penalty in (10.,1000.):
        def objective(flat):
            z=fixed.copy();z[free]=flat.reshape(-1,3);t=z[af];e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0]
            ratio=(np.cross(e1,e2)*an).sum(1)/aa;deficit=np.minimum(ratio-.05,0)
            grad=np.zeros_like(z);coefficient=2*penalty*deficit/aa
            gb=np.cross(e2,an)*coefficient[:,None];gc=np.cross(an,e1)*coefficient[:,None]
            np.add.at(grad,af[:,0],-gb-gc);np.add.at(grad,af[:,1],gb);np.add.at(grad,af[:,2],gc)
            diff=z-original;grad+=2*diff
            return float(np.sum(diff**2)+penalty*np.sum(deficit**2)),grad[free].ravel()
        result=minimize(objective,q[free].ravel(),jac=True,method='L-BFGS-B',
                        options={'maxiter':500,'ftol':1e-13,'gtol':1e-8,'maxls':40,'maxcor':30})
        q[free]=result.x.reshape(-1,3)
        if ratios(q).min()>.01:break
    movement=float(np.linalg.norm(q-original,axis=1).max())
    if not np.isfinite(q).all() or ratios(q).min()<=.01 or movement>maximum_move+1e-8:
        raise ValueError(f'Cannot preserve orientation within repair budget: minimum area ratio {ratios(q).min():.6g}, move {movement:.6g}, optimizer {result.message}')
    return q[ids],{'repairedFaces':int(bad.sum()),'maximumMove':movement,'minimumAreaRatio':float(ratios(q).min())}


@dataclass
class Graft:
    points: np.ndarray
    faces: np.ndarray
    body_lineage: object
    module_lineage: object
    body_face_lineage: np.ndarray
    module_face_lineage: np.ndarray
    body_seam: np.ndarray
    module_seam: np.ndarray
    body_edge_donors: np.ndarray
    module_edge_donors: np.ndarray
    module_distance: np.ndarray
    body_count: int
    removed_faces: np.ndarray


def fit_graft(body_points, body_faces, module_points, module_faces, remove_faces,
              projection, center, support_distance, tolerance=1e-5):
    body_points=np.asarray(body_points,dtype=float);module_points=np.asarray(module_points,dtype=float)
    body_faces=np.asarray(body_faces,dtype=int);module_faces=np.asarray(module_faces,dtype=int)
    validate_mesh(body_points,body_faces);validate_mesh(module_points,module_faces)
    remove_faces=np.asarray(remove_faces,dtype=bool);projection=np.asarray(projection,dtype=float);center=np.asarray(center,dtype=float)
    if remove_faces.shape!=(len(body_faces),) or not remove_faces.any() or remove_faces.all():
        raise ValueError('Select a nonempty proper body patch')
    if projection.shape!=(2,3) or not np.allclose(projection@projection.T,np.eye(2)) or center.shape!=(3,) or not np.isfinite(center).all():
        raise ValueError('Expected an orthonormal projection and measured center')
    if not np.isfinite(support_distance) or support_distance<=0:raise ValueError('Invalid support distance')
    bk,bi=topology_ids(body_points,tolerance);mk,mi=topology_ids(module_points,tolerance)
    kept=body_faces[~remove_faces]
    original_loops=boundary_loops(bi[body_faces])
    original_boundary={int(i) for loop in original_loops for i in loop}
    candidates=[loop for loop in boundary_loops(bi[kept]) if not set(loop)&original_boundary]
    module_loops=boundary_loops(mi[module_faces])
    if len(candidates)!=1 or len(module_loops)!=1:raise ValueError('Expected one new body opening and one module boundary')
    bl,bt=ordered_loop(body_points[bk],candidates[0],projection,center)
    ml,mt=ordered_loop(module_points[mk],module_loops[0],projection,center)
    samples=np.unique(np.r_[bt,mt])
    if np.min(np.diff(samples))<1e-8: raise ValueError('Nearly coincident angular samples require explicit correspondence')
    B,bf,bs,bd,bfl=refine_boundary(body_points,kept,bi,bk,bl,bt,samples)
    M,mf,ms,md,mfl=refine_boundary(module_points,module_faces,mi,mk,ml,mt,samples)
    bp=B@body_points;mp=M@module_points
    mp,distance=harmonic_fit(mp,mf,ms,bp[bs],support_distance,tolerance)
    # Exact equality also for every module render alias of each seam point.
    _,alias=topology_ids(M@module_points,tolerance)
    for i,j in zip(ms,bs):mp[alias==alias[i]]=bp[j]
    result=Graft(np.vstack([bp,mp]),np.vstack([bf,mf+len(bp)]),B,M,
                 np.flatnonzero(~remove_faces)[bfl],mfl,bs,ms+len(bp),bd,md,distance,len(bp),np.flatnonzero(remove_faces))
    # Every new seam segment must have two oppositely directed faces.
    _,ids=topology_ids(result.points,tolerance)
    boundary_loops(ids[result.faces])
    directed=edges(ids[result.faces]);u,counts=np.unique(np.sort(directed,axis=1),axis=0,return_counts=True)
    seam_edges={tuple(sorted(e)) for e in zip(ids[bs],np.roll(ids[bs],-1))}
    for a,b in seam_edges:
        matches=directed[np.all(np.sort(directed,axis=1)==[a,b],axis=1)]
        if len(matches)!=2 or not np.array_equal(matches[0],matches[1][::-1]):
            raise ValueError('Graft seam has a gap or inconsistent winding')
    return result


def extend_seam_field(points, faces, seam, values, far_value, support_distance,
                      tolerance=1e-5):
    """Harmonic attributes with exact seam values and an explicit far value.

    Useful for weights, UVs and material masks. Callers retain source attributes
    separately: this function produces target attributes, not source identity.
    """
    keep,ids=topology_ids(points,tolerance);p=points[keep];f=ids[faces]
    values=np.asarray(values,dtype=float);far=np.asarray(far_value,dtype=float)
    if values.shape!=(len(seam),len(far)) or not np.isfinite(values).all() or not np.isfinite(far).all():
        raise ValueError('Invalid seam field')
    e=np.unique(np.sort(edges(f),axis=1),axis=0);e=e[e[:,0]!=e[:,1]]
    length=np.linalg.norm(p[e[:,0]]-p[e[:,1]],axis=1)
    graph=sparse.coo_matrix((np.r_[length,length],(np.r_[e[:,0],e[:,1]],np.r_[e[:,1],e[:,0]])),shape=(len(p),len(p))).tocsr()
    boundary=ids[seam]
    if len(np.unique(boundary))!=len(boundary):raise ValueError('Repeated field constraint')
    distance=dijkstra(graph,directed=False,indices=boundary,min_only=True)
    fixed=distance>=support_distance;fixed[boundary]=True
    output=np.tile(far,(len(p),1));output[boundary]=values
    weight=1/length
    adjacency=sparse.coo_matrix((np.r_[weight,weight],(np.r_[e[:,0],e[:,1]],np.r_[e[:,1],e[:,0]])),shape=graph.shape).tocsr()
    laplace=sparse.diags(np.asarray(adjacency.sum(axis=1)).ravel())-adjacency
    free=np.flatnonzero(~fixed);locked=np.flatnonzero(fixed)
    if len(free):output[free]=spsolve(laplace[free][:,free],-laplace[free][:,locked]@output[locked])
    return output[ids]


def smooth_normals(points, faces, tolerance=1e-5):
    keep,ids=topology_ids(points,tolerance)
    normals=np.zeros((len(keep),3))
    tri=points[faces];area=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    for i in range(3):np.add.at(normals,ids[faces[:,i]],area)
    length=np.linalg.norm(normals,axis=1);normals/=np.maximum(length[:,None],1e-30)
    return normals[ids]


def limit_influences(weights, maximum, discard_budget):
    """Deterministic target skin limit, preserving the input row sums.

    Quantized source rows need not sum to exactly one. The adapter declares its
    native influence limit and acceptable discarded weight; excess is an error.
    """
    values=np.asarray(weights,dtype=float).copy()
    if values.ndim!=2 or not np.isfinite(values).all() or np.any(values<0) or np.any(values.sum(1)<=0):
        raise ValueError('Invalid skin weights')
    if not isinstance(maximum,int) or maximum<1 or not 0<=discard_budget<1:
        raise ValueError('Invalid influence limit or discard budget')
    discarded=np.zeros(len(values))
    for row,w in enumerate(values):
        total=w.sum();order=np.argsort(-w,kind='stable');discarded[row]=w[order[maximum:]].sum()
        if discarded[row]>discard_budget:
            raise ValueError(f'Skin reduction exceeds budget at row {row}: {discarded[row]}')
        if discarded[row]>0:w[order[maximum:]]=0;w*=total/w.sum()
    return values,discarded
