"""SDK-free measured Wolverine character route export, not a universal fit law.
The +X anterior/+Y transverse/+Z superior coordinates and target locations below
are calibrated to the recorded Wolverine source mesh in uncalibrated source units.
Native packed formats/draw hooks remain in the adapters.
"""
import heapq
import numpy as np
from malemod_base.garment_routes import classic_route_targets

def measure_wolverine_routes(points,source,faces,waistRecipe):
    points=np.array(points);source=np.array(source);faces=np.array(faces);lookup={int(k):i for i,k in enumerate(source)}
    waist=[];waistRows=[]
    for row in waistRecipe:
        donors=[(int(v),float(w)) for v,w in row.items() if float(w)>0];waistRows.append(donors);waist.append(sum(points[lookup[v]]*w for v,w in donors))
    waist=np.array(waist);adj=[{} for _ in points]
    for tri in faces:
        for a,b in zip(tri,np.roll(tri,-1)):
            a=int(a);b=int(b);d=float(np.linalg.norm(points[a]-points[b]));adj[a][b]=d;adj[b][a]=d
    # Exact duplicate source positions are UV aliases, including the part weld.
    # Retain every native ID in the selected route; no synthetic surface edges.
    groups={}
    for k,p in enumerate(points):groups.setdefault(tuple(p),[]).append(k)
    for group in groups.values():
        for k in group[1:]:adj[group[0]][k]=0.;adj[k][group[0]]=0.
    def nearest(target,side):
        candidates=np.flatnonzero((points[:,1]*side>0)&(points[:,2]>73)&(points[:,2]<98));return int(candidates[np.argmin(np.linalg.norm(points[candidates]-target,axis=1))])
    def path(start,end,side,posterior,limit):
        costs={start:0.};prev={};queue=[(0.,start)]
        while queue:
            cost,k=heapq.heappop(queue)
            if cost!=costs[k]:continue
            if k==end:break
            for n,d in adj[k].items():
                if points[n,1]*side<-.01 or not 73<points[n,2]<98:continue
                penalty=1+max(0.,points[n,0]-limit)*20 if posterior else 1+max(0.,abs(points[n,1])-10)*20
                nc=cost+d*penalty
                if nc<costs.get(n,float('inf')):costs[n]=nc;prev[n]=k;heapq.heappush(queue,(nc,n))
        if end not in costs:raise ValueError('Disconnected measured hip/glute route')
        result=[end]
        while result[-1]!=start:result.append(prev[result[-1]])
        return result[::-1]
    routes=[];proof=[]
    for side in [-1,1]:
        candidates=np.flatnonzero(waist[:,1]*side>.75*np.max(waist[:,1]*side));mid=(waist[candidates,0].min()+waist[candidates,0].max())*.5;at=int(candidates[np.argmin(abs(waist[candidates,0]-mid))])
        targets,steering=classic_route_targets(waist[candidates],waist[at],[-7.8,side*8.5,79],[-4.5,side*3.2,76.7],[1,0,0],[0,0,1])
        at=int(candidates[np.argmin(np.linalg.norm(waist[candidates]-targets[0],axis=1))]);startRow=waistRows[at];start=min((lookup[v] for v,_ in startRow),key=lambda k:np.linalg.norm(points[k]-waist[at]))
        hip=nearest(np.array([-9.7,side*11.8,85]),side);crease=nearest(targets[1],side);medial=nearest(targets[2],side);inner=nearest(np.array([.4,side*.7,76.5]),side)
        waypoints=[start,hip,crease,medial,inner];route=[start]
        for i,(a,b) in enumerate(zip(waypoints,waypoints[1:])):route+=path(a,b,side,i<2,points[a,0] if i==0 else -4)[1:]
        descent=route[:route.index(crease)+1];returned=route[route.index(crease):]
        if np.max(points[descent,0])>waist[at,0]+2 or np.max(abs(points[returned,1]))>10 or np.min(points[route,1]*side)<-.01:raise ValueError('Route left measured posterior/medial corridor')
        rows=[dict(startRow)]+[{int(source[k]):1.} for k in route[1:]];routes.append(rows);proof.append(dict(routeSteering=steering,side=side,waistSample=at,startPosition=waist[at].tolist(),lateralMidpointForward=float(mid),waypointVertexIds=source[waypoints].tolist(),waypointPositions=points[waypoints].tolist(),samples=len(rows),edgeLength=float(sum(np.linalg.norm(points[a]-points[b]) for a,b in zip(route,route[1:]))),descendingMaximumForward=float(points[descent,0].max()),returnMaximumLateral=float(abs(points[returned,1]).max())))
    return routes,proof
