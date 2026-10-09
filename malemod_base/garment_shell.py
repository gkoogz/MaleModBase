"""Shallow garment volume with source-corner lineage and separate edge normals.

Adapters supply measured thickness and outward normals in their own rest units.
Coincident UV aliases weld for boundary detection, never for texture interpolation.
"""
import numpy as np
import heapq


def boundary_band(positions, triangles, enabled, width):
    """A measured-width strip along selected true cuts, with face barycentrics.

    Distance travels along the alias-welded garment graph. The strip is clipped
    within original faces, so its new corners retain exact texture/skin donors.
    Adapters select their measured hem/cut and supply width in source units.
    """
    p=np.asarray(positions,float);t=np.asarray(triangles,int);enabled=np.asarray(enabled,bool)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or enabled.shape!=(len(p),):
        raise ValueError('Invalid boundary band positions or mask')
    if t.ndim!=2 or t.shape[1]!=3 or not len(t) or t.min()<0 or t.max()>=len(p):
        raise ValueError('Invalid boundary band topology')
    if not np.isfinite(width) or width<=0:raise ValueError('Positive boundary band width required')
    _,alias=np.unique(np.round(p,6),axis=0,return_inverse=True)
    nodes=np.array([p[alias==i].mean(0) for i in range(alias.max()+1)])
    adjacent=[{} for _ in nodes];edges={}
    selected=np.array([np.any(enabled[alias==i]) for i in range(len(nodes))])
    for face in alias[t]:
        for a,b in zip(face,np.roll(face,-1)):
            a,b=int(a),int(b);key=tuple(sorted((a,b)));edges[key]=edges.get(key,0)+1
            distance=float(np.linalg.norm(nodes[a]-nodes[b]));adjacent[a][b]=distance;adjacent[b][a]=distance
    seeds={i for (a,b),count in edges.items() if count==1 and selected[a] and selected[b] for i in (a,b)}
    distance=np.full(len(nodes),np.inf);queue=[]
    for i in seeds:distance[i]=0;heapq.heappush(queue,(0.,i))
    while queue:
        value,i=heapq.heappop(queue)
        if value!=distance[i]:continue
        for j,length in adjacent[i].items():
            candidate=value+length
            if candidate<distance[j]:distance[j]=candidate;heapq.heappush(queue,(candidate,j))
    distance[~np.isfinite(distance)]=width*2
    return clip_scalar_band(p,t,np.ones(len(p)),width-distance[alias])


def clip_scalar_band(positions, triangles, lower_distance, upper_distance):
    """Clip a sheet to two adapter-authored positive half-fields.

    Distances interpolate linearly within each source face. Return exact face
    corners and barycentric weights, including corners created by both cuts.
    """
    p=np.asarray(positions,dtype=float);t=np.asarray(triangles,dtype=int)
    lo=np.asarray(lower_distance,dtype=float);hi=np.asarray(upper_distance,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or lo.shape!=(len(p),) or hi.shape!=lo.shape or not np.isfinite(p).all() or not np.isfinite(lo).all() or not np.isfinite(hi).all():
        raise ValueError('Invalid adapter clipping fields')
    out=[];faces=[];donors=[];weights=[]
    for source in t:
        poly=list(np.eye(3))
        for field in (lo[source],hi[source]):
            clipped=[]
            for a,b in zip(poly,poly[1:]+poly[:1]):
                da,db=float(a@field),float(b@field)
                if da>=-1e-10:clipped.append(a)
                if (da>=-1e-10)!=(db>=-1e-10):clipped.append(a+(b-a)*da/(da-db))
            poly=clipped
            if not poly:break
        if len(poly)<3:continue
        first=len(out)
        for bary in poly:out.append(bary@p[source]);donors.append(source);weights.append(bary)
        for k in range(1,len(poly)-1):
            f=(first,first+k,first+k+1)
            if np.linalg.norm(np.cross(out[f[1]]-out[f[0]],out[f[2]]-out[f[0]]))>1e-9:faces.append(f)
    return dict(positions=np.asarray(out).reshape(-1,3),triangles=np.asarray(faces,dtype=int).reshape(-1,3),
                donors=np.asarray(donors,dtype=int).reshape(-1,3),weights=np.asarray(weights).reshape(-1,3))


def thin_shell(positions, triangles, normals, thickness, *, offset=0., weld_decimals=6):
    p=np.asarray(positions,dtype=float);t=np.asarray(triangles,dtype=int)
    n=np.asarray(normals,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or n.shape!=p.shape or t.ndim!=2 or t.shape[1]!=3 or not len(t):
        raise ValueError('Expected a nonempty triangle sheet and matching normals')
    if not np.isfinite(p).all() or not np.isfinite(n).all() or not np.isfinite(thickness) or thickness<=0 or not np.isfinite(offset) or t.min()<0 or t.max()>=len(p):
        raise ValueError('Invalid shell geometry or thickness')
    length=np.linalg.norm(n,axis=1)
    if np.any(length<1e-10):raise ValueError('Zero shell normal')
    n=n/length[:,None]
    # Weld positions for topology; average the offset direction across UV seams.
    groups={};alias=[]
    for q in p:alias.append(groups.setdefault(tuple(np.round(q,weld_decimals)),len(groups)))
    alias=np.asarray(alias);summed=np.zeros((len(groups),3));np.add.at(summed,alias,n)
    summed/=np.maximum(np.linalg.norm(summed,axis=1)[:,None],1e-12);n=summed[alias]
    outer=p+n*offset;inner=p+n*(offset-thickness);count=len(p)
    q=list(outer)+list(inner);source=list(range(count))*2
    layers=[0]*count+[1]*count;faces=list(t)+list(t[:,::-1]+count)
    edges={}
    for f in t:
        for a,b in zip(f,np.roll(f,-1)):
            key=tuple(sorted((int(alias[a]),int(alias[b]))))
            edges.setdefault(key,[]).append((int(a),int(b)))
    boundary=0
    for occurrences in edges.values():
        if len(occurrences)>2:raise ValueError('Nonmanifold shell sheet')
        if len(occurrences)!=1:continue
        a,b=occurrences[0];i=len(q)
        # Separate wall vertices keep the shallow cut edge crisp. Their UV and
        # skin/motion donors remain the exact corresponding sheet corners.
        q.extend((outer[a],inner[a],inner[b],outer[b]));source.extend((a,a,b,b))
        layers.extend((2+boundary,)*4);faces.extend(((i,i+1,i+2),(i,i+2,i+3)));boundary+=1
    return dict(positions=np.asarray(q),triangles=np.asarray(faces,dtype=int),
                source=np.asarray(source,dtype=int),normal_layers=np.asarray(layers),
                boundary_edges=boundary,thickness=float(thickness))
