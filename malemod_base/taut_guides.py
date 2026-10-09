"""Taut convex-envelope guide paths in adapter-selected wrapping planes.

Each path is the shorter boundary route around the convex hull of the actual
collision cross-section and its two anchors. This is a planar taut string,
not a global three-dimensional geodesic or a cloth simulation.
"""
import numpy as np
import heapq


def convex_hull(points):
    points = np.unique(np.asarray(points, float), axis=0)
    points = points[np.lexsort((points[:, 1], points[:, 0]))]
    def cross(a, b, c):
        u, v = b-a, c-a
        return u[0]*v[1]-u[1]*v[0]
    def half(sequence):
        result=[]
        for p in sequence:
            while len(result)>1 and cross(result[-2],result[-1],p)<=1e-10:
                result.pop()
            result.append(p)
        return result
    return np.array(half(points)[:-1]+half(points[::-1])[:-1])


def plane_section(triangles, origin, normal):
    triangles=np.asarray(triangles,float)
    d=(triangles-origin)@normal
    points=[triangles[np.abs(d)<1e-10]]
    for i,j in [(0,1),(1,2),(2,0)]:
        mask=d[:,i]*d[:,j]<0
        t=d[mask,i]/(d[mask,i]-d[mask,j])
        points.append(triangles[mask,i]+t[:,None]*(triangles[mask,j]-triangles[mask,i]))
    return np.concatenate(points)


def taut_paths(starts, target, axis, triangles):
    paths, receipts=[],[]
    target,axis=np.asarray(target,float),np.asarray(axis,float)
    for start in np.asarray(starts,float):
        direction=target-start;length=np.linalg.norm(direction);u=direction/length
        n=np.cross(u,axis);n/=np.linalg.norm(n);v=np.cross(n,u)
        section=plane_section(triangles,start,n)
        xy=np.column_stack([(section-start)@u,(section-start)@v])
        anchors=np.array([[0.,0.],[length,0.]])
        hull=convex_hull(np.vstack([xy,anchors]))
        ids=[int(np.argmin(np.linalg.norm(hull-p,axis=1))) for p in anchors]
        if any(np.linalg.norm(hull[i]-p)>1e-7 for i,p in zip(ids,anchors)):
            raise ValueError('Wrapping anchor is inside the sectional convex envelope')
        candidates=[]
        for step in [1,-1]:
            indices=[ids[0]]
            while indices[-1]!=ids[1]:indices.append((indices[-1]+step)%len(hull))
            curve=hull[indices]
            candidates.append((np.linalg.norm(np.diff(curve,axis=0),axis=1).sum(),curve))
        path_length,curve=min(candidates,key=lambda x:x[0])
        delta=np.diff(curve,axis=0)
        turns=delta[:-1,0]*delta[1:,1]-delta[:-1,1]*delta[1:,0]
        if len(turns) and not ((turns>=-1e-8).all() or (turns<=1e-8).all()):
            raise ValueError('Concave turn in taut guide')
        # Every bent segment supports the section: obstacle points stay on
        # one side of the segment's line. Straight unobstructed paths need no bend.
        support_error=0.
        if len(curve)>2:
            for a,b in zip(curve[:-1],curve[1:]):
                side=(b[0]-a[0])*(xy[:,1]-a[1])-(b[1]-a[1])*(xy[:,0]-a[0])
                support_error=max(support_error,max(0.,min(side.max(),-side.min())))
            if support_error>1e-7:raise ValueError('Taut segment enters collision envelope')
        world=start+curve[:,0,None]*u+curve[:,1,None]*v
        world[0]=start;world[-1]=target
        paths.append(world)
        receipts.append(dict(length=float(path_length),straightDistance=float(length),
                             bends=max(0,len(curve)-2),supportError=float(support_error)))
    return paths,receipts


def sample_polyline(points,count):
    points=np.asarray(points,float)
    length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    return np.column_stack([np.interp(np.linspace(0,length[-1],count),length,points[:,i]) for i in range(3)])


def lift_embedded_anchors(points, meshes, direction, clearance=.015, surface_margin=0.):
    """Move only embedded anchors outward from the union of convex proxies."""
    from scipy.spatial import ConvexHull
    planes=[ConvexHull(p).equations for p,_ in meshes]
    result=np.asarray(points,float).copy()
    direction=np.asarray(direction,float);direction/=np.linalg.norm(direction)
    for i in range(len(result)):
        for _ in range(20):
            moved=False
            for plane in planes:
                signed=plane[:,:3]@result[i]+plane[:,3]-surface_margin
                if signed.max()<-1e-9:
                    rate=plane[:,:3]@direction
                    exit_distance=np.min(-signed[rate>1e-9]/rate[rate>1e-9])
                    result[i]+=direction*(exit_distance+clearance);moved=True
            if not moved:break
        else:raise ValueError('Could not clear embedded guide anchor')
    return result


def union_taut_paths(starts,target,axis,meshes,radial_halfplane=None):
    """Shortest visible polygonal route with one turn sign in each ray plane.

    Convex per-object cross-sections preserve gaps between separate colliders.
    Straight spans and tangent bends are found by a visibility graph. If the
    ordinary shortest path changes bend sign, A* finds the shortest available
    path with consistently convex turns in that same plane.
    """
    result,receipts=[],[]
    for start in starts:
        start=np.asarray(start,float);u=target-start;length=np.linalg.norm(u);u/=length
        n=np.cross(u,axis);n/=np.linalg.norm(n);v=np.cross(n,u)
        polygons=[]
        for p,f in meshes:
            section=plane_section(p[f],start,n)
            if len(section)>2:
                xy=np.column_stack([(section-start)@u,(section-start)@v])
                poly=convex_hull(xy)
                if len(poly)>2:polygons.append(poly)
        normals=[np.column_stack([-np.diff(np.vstack([p,p[0]]),axis=0)[:,1],np.diff(np.vstack([p,p[0]]),axis=0)[:,0]]) for p in polygons]
        nodes=np.vstack([[[0.,0.],[length,0.]]]+polygons)
        keep=np.ones(len(nodes),bool)
        for p,norm in zip(polygons,normals):
            signed=nodes@norm.T-(p*norm).sum(1)
            keep[2:]&=~(signed[2:]>1e-8).all(1)
            if (signed[:2]>1e-8).all(1).any():raise ValueError('Anchor inside collision section')
        nodes=nodes[keep]
        # Shared proxy boundaries can emit numerically coincident vertices.
        # A zero-length graph edge must never reset a convex-turn constraint.
        _,unique=np.unique(np.round(nodes,7),axis=0,return_index=True)
        nodes=nodes[np.sort(unique)];count=len(nodes)
        if radial_halfplane is not None:
            # A meridian stays on its own outward half-plane. It may touch
            # the common polar axis only at the terminal pole.
            radial=np.asarray(radial_halfplane,float)
            world=start+nodes[:,0,None]*u+nodes[:,1,None]*v
            outward=(world-target)@radial
            keep=outward>1e-8;keep[:2]=True
            nodes=nodes[keep];count=len(nodes)
        delta=nodes[None,:,:]-nodes[:,None,:]
        visible=~np.eye(count,dtype=bool)
        for p,norm in zip(polygons,normals):
            low=np.zeros((count,count));high=np.ones((count,count));possible=np.ones((count,count),bool)
            for a,inward in zip(p,norm):
                d=(nodes-a)@inward-1e-8
                slope=delta@inward
                constant=np.abs(slope)<1e-12
                possible&=~(constant&(d[:,None]<=0))
                t=-d[:,None]/np.where(constant,1.,slope)
                low=np.maximum(low,np.where(slope>1e-12,t,-np.inf))
                high=np.minimum(high,np.where(slope<-1e-12,t,np.inf))
            visible&=~(possible&(low<high-1e-9))
        weights=np.linalg.norm(delta,axis=2)
        adjacent=[np.flatnonzero(row).tolist() for row in visible]
        def search(sign=None):
            queue=[(length,0.,(-1,0))];cost={(-1,0):0.};parent={}
            while queue:
                _,distance,state=heapq.heappop(queue)
                if distance>cost[state]+1e-10:continue
                previous,current=state
                if current==1:
                    route=[current]
                    while state in parent:state=parent[state];route.append(state[1])
                    return distance,nodes[route[::-1]]
                for nxt in adjacent[current]:
                    if nxt==previous or nxt==0:continue
                    if sign is not None and previous>=0:
                        a=nodes[current]-nodes[previous];b=nodes[nxt]-nodes[current]
                        if sign*(a[0]*b[1]-a[1]*b[0])<-1e-8:continue
                    successor=(current,nxt) if sign is not None else (-1,nxt)
                    candidate=distance+weights[current,nxt]
                    if candidate<cost.get(successor,np.inf)-1e-10:
                        cost[successor]=candidate;parent[successor]=state
                        heapq.heappush(queue,(candidate+np.linalg.norm(nodes[nxt]-nodes[1]),candidate,successor))
            return None
        found=search()
        if found is None:raise ValueError('No collision-free taut route')
        distance,curve=found
        edges=np.diff(curve,axis=0);turns=edges[:-1,0]*edges[1:,1]-edges[:-1,1]*edges[1:,0]
        if not ((turns>=-1e-8).all() or (turns<=1e-8).all()):
            choices=[x for x in [search(1),search(-1)] if x is not None]
            if not choices:raise ValueError('No route with consistently convex turns')
            distance,curve=min(choices,key=lambda x:x[0])
        edges=np.diff(curve,axis=0)
        turns=edges[:-1,0]*edges[1:,1]-edges[:-1,1]*edges[1:,0]
        if not ((turns>=-1e-8).all() or (turns<=1e-8).all()):
            raise ValueError('Final guide contains a concave turn')
        world=start+curve[:,0,None]*u+curve[:,1,None]*v
        world[0]=start;world[-1]=target
        world=world[np.r_[True,np.linalg.norm(np.diff(world,axis=0),axis=1)>1e-8]]
        result.append(world)
        receipts.append(dict(length=float(distance),straightDistance=float(length),bends=max(0,len(world)-2),collisionFree=True,convexTurns=True))
    return result,receipts


def parallel_taut_paths(starts, tip_row, lateral_axis, meshes):
    """Wrap in parallel planes, preserving each anchor's lateral coordinate.

    The terminal row preserves the full starting width. Distinct lanes cannot
    intersect in 3D even if a camera projection makes rear/front spans overlap.
    """
    starts=np.asarray(starts,float);lateral=np.asarray(lateral_axis,float)
    lateral/=np.linalg.norm(lateral);tip_row=np.asarray(tip_row,float)
    targets=tip_row+((starts-tip_row)@lateral)[:,None]*lateral
    paths,receipts=[],[]
    for start,target in zip(starts,targets):
        direction=target-start
        plane_axis=np.cross(lateral,direction)
        plane_axis/=np.linalg.norm(plane_axis)
        found,proof=union_taut_paths([start],target,plane_axis,meshes)
        paths.extend(found);receipts.extend(proof)
    error=max(float(np.abs((p-p[0])@lateral).max()) for p in paths)
    separation=float(np.diff(np.sort(starts@lateral)).min())
    if error>1e-9 or separation<1e-9:
        raise ValueError('Guide lanes coincide or drift laterally')
    return paths,receipts,targets,dict(laneDrift=error,minimumLaneSeparation=separation)


def meridian_taut_paths(starts, pole, polar_axis, meshes):
    """Distinct longitude half-planes, meeting only at a common pole.

    Each anchor fixes its longitude. Lateral position can change along a
    meridian; crossing the polar axis or changing longitude is forbidden.
    """
    starts=np.asarray(starts,float);pole=np.asarray(pole,float)
    axis=np.asarray(polar_axis,float);axis/=np.linalg.norm(axis)
    offset=starts-pole
    radial=offset-(offset@axis)[:,None]*axis
    radial/=np.linalg.norm(radial,axis=1)[:,None]
    a=radial[0];b=np.cross(axis,a)
    angles=np.mod(np.arctan2(radial@b,radial@a),2*np.pi)
    cyclic_steps=np.angle(np.exp(1j*(np.roll(angles,-1)-angles)))
    if not ((cyclic_steps>1e-8).all() or (cyclic_steps<-1e-8).all()):
        raise ValueError('Meridian anchors reverse their cyclic boundary order')
    ordered=np.sort(angles)
    separation=float(np.diff(np.r_[ordered,ordered[0]+2*np.pi]).min())
    if separation<1e-8:raise ValueError('Meridian anchors have coincident longitudes')
    paths,receipts=[],[];plane_error=0.;minimum_outward=np.inf
    for start,direction in zip(starts,radial):
        found,proof=union_taut_paths([start],pole,axis,meshes,radial_halfplane=direction)
        path=found[0];normal=np.cross(axis,direction)
        plane_error=max(plane_error,float(np.abs((path-pole)@normal).max()))
        outward=(path[:-1]-pole)@direction
        minimum_outward=min(minimum_outward,float(outward.min()))
        if outward.min()<=1e-8:raise ValueError('Meridian touches polar axis before pole')
        paths.extend(found);receipts.extend(proof)
    return paths,receipts,dict(polarAxis=axis.tolist(),longitudes=angles.tolist(),
        minimumLongitudeSeparation=separation,planeError=plane_error,
        minimumOutwardRadius=minimum_outward,sharedPole=pole.tolist(),
        intersectionPolicy='Distinct outward longitude half-planes; common pole only')


def ordered_meridian_anchors(starts,pole,polar_axis,meshes,angular_gap=.003):
    """Minimally repair local anchor-order reversals, preserving polar height.

    Isotonic angles retain the boundary's cyclic indexing. Collider clearance
    moves outward in each repaired longitude, so it cannot undo the ordering.
    """
    starts=np.asarray(starts,float);pole=np.asarray(pole,float)
    axis=np.asarray(polar_axis,float);axis/=np.linalg.norm(axis)
    offset=starts-pole;height=offset@axis
    radial=offset-height[:,None]*axis;radius=np.linalg.norm(radial,axis=1)
    a=radial[0]/radius[0];b=np.cross(axis,a)
    angle=np.unwrap(np.arctan2(radial@b,radial@a))
    orientation=1 if angle[-1]>angle[0] else -1
    angle*=orientation;b*=orientation
    values=angle-np.arange(len(angle))*angular_gap
    blocks=[]
    for i,value in enumerate(values):
        blocks.append([i,i+1,float(value),1])
        while len(blocks)>1 and blocks[-2][2]>blocks[-1][2]:
            right=blocks.pop();left=blocks.pop();weight=left[3]+right[3]
            blocks.append([left[0],right[1],(left[2]*left[3]+right[2]*right[3])/weight,weight])
    fixed=np.empty(len(values))
    for first,last,value,_ in blocks:fixed[first:last]=value
    fixed+=np.arange(len(fixed))*angular_gap
    if fixed[-1]-fixed[0]>=2*np.pi-angular_gap:raise ValueError('Boundary cannot fit ordered meridian longitudes')
    directions=np.cos(fixed)[:,None]*a+np.sin(fixed)[:,None]*b
    adjusted=pole+height[:,None]*axis+radius[:,None]*directions
    for i,direction in enumerate(directions):
        adjusted[i]=lift_embedded_anchors([adjusted[i]],meshes,direction)[0]
    return adjusted,dict(maxAnchorAdjustment=float(np.linalg.norm(adjusted-starts,axis=1).max()),
                         minimumAngularGap=angular_gap,order='Cyclic boundary order preserved')


def bend_density(path):
    """Dimensionless wrapping demand; straight free spans have zero demand."""
    edges=np.diff(np.asarray(path,float),axis=0)
    lengths=np.linalg.norm(edges,axis=1)
    # Coincident vertices from adjoining proxy sections are not physical bends.
    keep=lengths>1e-8;edges=edges[keep];lengths=lengths[keep]
    directions=edges/np.maximum(lengths[:,None],1e-15)
    turns=np.arccos(np.clip((directions[:-1]*directions[1:]).sum(1),-1.,1.))
    chord=np.linalg.norm(path[-1]-path[0])
    detour=max(0.,float(lengths.sum()/max(chord,1e-15)-1.))
    return float(turns.sum()+(.5*turns.max() if len(turns) else 0.)+2*detour)


def adaptive_meridian_paths(outline,pole,polar_axis,meshes,count=40,candidates=160):
    """Recompute origin density from current wrapping bends on every call.

    Cyclic smoothing, a nonzero density floor and quantile sampling retain
    full boundary coverage and ordered neighbors. No per-character origin
    locations or static curvature weights are baked into the contract.
    """
    outline=np.asarray(outline,float)
    if np.linalg.norm(outline[0]-outline[-1])>1e-8:
        raise ValueError('Adaptive meridians require a closed attachment outline')
    dense=sample_polyline(outline,candidates+1)[:-1]
    dense,_=ordered_meridian_anchors(dense,pole,polar_axis,meshes,angular_gap=.0005)
    trial,_,_=meridian_taut_paths(dense,pole,polar_axis,meshes)
    demand=np.array([bend_density(p) for p in trial])
    smoothed=sum(weight*np.roll(demand,shift) for shift,weight in [(-2,1.),(-1,2.),(0,3.),(1,2.),(2,1.)])/9.
    weights=.35+3*smoothed
    # Bin-center demand controls density over the closed boundary's arclength.
    cdf=np.r_[0.,np.cumsum((weights+np.roll(weights,-1))/2)]
    fractions=np.interp(np.arange(count)*cdf[-1]/count,cdf,np.linspace(0,1,candidates+1))
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(outline,axis=0),axis=1))]
    origins=np.column_stack([np.interp(fractions*distance[-1],distance,outline[:,i]) for i in range(3)])
    origins,repair=ordered_meridian_anchors(origins,pole,polar_axis,meshes)
    paths,receipts,proof=meridian_taut_paths(origins,pole,polar_axis,meshes)
    density=dict(mode='Recomputed from current wrap geometry on each evaluation',
        candidates=candidates,originFractions=fractions.tolist(),
        candidateBendDemand=demand.tolist(),candidateDensity=weights.tolist(),
        densityRatio=float(weights.max()/weights.min()),densityFloor=.35,
        metric='Total turning + half maximum turn + twice path detour; cyclic smoothing',
        anchorOrderRepair=repair,selectedBendDemand=[bend_density(p) for p in paths])
    return paths,receipts,origins,proof,density
