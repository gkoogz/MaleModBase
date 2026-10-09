"""Portable cloth reference mesh built from ordered wrap guides, not a solver."""
import numpy as np
from scipy.spatial import ConvexHull
from .taut_guides import sample_polyline


def triangle_clearance(points, faces, planes):
    """Sufficient whole-triangle separation: one hull plane clears all corners.

    Unlike point sampling, this certifies the entire affine triangle. A negative
    value may be conservative at a hull corner; it is never a false clear.
    """
    values = points @ planes[:, :3].T + planes[:, 3]
    scores = np.minimum(np.minimum(values[faces[:, 0]], values[faces[:, 1]]), values[faces[:, 2]])
    choices = scores.argmax(axis=1)
    return scores[np.arange(len(faces)), choices], choices


def cloth_surface(paths, outline, origin_fractions, pole, axis, tip_normal,
                  meshes, rows=80, subdivisions=4, clearance=.06, end_density=False,
                  boundary_points=None):
    """Loft cyclic meridians, retain the entire seam, and clear convex proxies.

    Guide arclength permits the dome's small axial backtrack. Added angular
    lanes interpolate radius and height, rather than taking inward 3D chords.
    The cloth pole is offset by thickness; the construction pole stays intact.
    """
    if rows<4 or subdivisions<1 or len(paths)<3:raise ValueError('Invalid meridian mesh resolution')
    outline=np.asarray(outline,float)
    pole=np.asarray(pole,float);axis=np.asarray(axis,float);axis/=np.linalg.norm(axis)
    row_parameters=(.5-.5*np.cos(np.linspace(0,np.pi,rows+1))) if end_density else np.linspace(0,1,rows+1)
    curves=[]
    for path in paths:
        path=np.asarray(path,float)
        distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        curves.append(np.column_stack([np.interp(row_parameters[:-1]*distances[-1],distances,path[:,d]) for d in range(3)]))
    curves=np.array(curves)
    radial=curves[:,0]-pole;radial-= (radial@axis)[:,None]*axis
    a=radial[0]/np.linalg.norm(radial[0]);b=np.cross(axis,a)
    angles=np.unwrap(np.arctan2(radial@b,radial@a))
    orientation=np.sign(angles[-1]-angles[0]);angle_end=angles[0]+orientation*2*np.pi
    n=len(paths);cols=n*subdivisions;grid=[];fractions=[]
    for i in range(n):
        j=(i+1)%n;end=angles[j] if j else angle_end
        h0=(curves[i]-pole)@axis;h1=(curves[j]-pole)@axis
        r0=np.linalg.norm(curves[i]-pole-h0[:,None]*axis,axis=1)
        r1=np.linalg.norm(curves[j]-pole-h1[:,None]*axis,axis=1)
        f0=origin_fractions[i];f1=origin_fractions[j] if j else 1.
        for k in range(subdivisions):
            t=k/subdivisions;theta=angles[i]*(1-t)+end*t
            direction=np.cos(theta)*a+np.sin(theta)*b
            grid.append(pole+((1-t)*h0+t*h1)[:,None]*axis+((1-t)*r0+t*r1)[:,None]*direction)
            fractions.append(f0*(1-t)+f1*t)
    grid=np.array(grid).transpose(1,0,2)
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(outline,axis=0),axis=1))]
    boundary=np.column_stack([np.interp(np.array(fractions)*distance[-1],distance,outline[:,d]) for d in range(3)])
    if boundary_points is not None:
        boundary=np.asarray(boundary_points,float)
        if boundary.shape!=(cols,3) or not np.isfinite(boundary).all():
            raise ValueError('Fitted boundary must match every angular column')
    grid[0]=boundary
    tip_normal=np.asarray(tip_normal,float);tip_normal/=np.linalg.norm(tip_normal)
    points=np.vstack([grid.reshape(-1,3),pole+tip_normal*clearance*3])
    faces=[]
    for row in range(rows-1):
        for col in range(cols):
            x=row*cols+col;y=row*cols+(col+1)%cols
            faces.extend([[x,y,y+cols],[x,y+cols,x+cols]])
    for col in range(cols):faces.append([(rows-1)*cols+col,(rows-1)*cols+(col+1)%cols,len(points)-1])
    faces=np.array(faces,dtype=int)
    planes=[np.unique(np.round(ConvexHull(p).equations,12),axis=0) for p,_ in meshes]
    fixed=np.zeros(len(points),bool);fixed[:cols]=True;fixed[-1]=True
    original=points.copy()
    offset=points-pole;height=offset@axis
    directions=offset-height[:,None]*axis
    radius=np.linalg.norm(directions,axis=1)
    directions/=np.maximum(radius[:,None],1e-15)
    directions[-1]=tip_normal
    # Radial exit from the full union, not the nearest potentially inward face.
    for hull in planes:
        base=pole+height[:,None]*axis
        constant=base@hull[:,:3].T+hull[:,3]
        slope=directions@hull[:,:3].T
        with np.errstate(divide='ignore',invalid='ignore'):
            intersections=-constant/slope
        low=np.where(slope < -1e-10,intersections,-np.inf).max(1)
        high=np.where(slope > 1e-10,intersections,np.inf).min(1)
        parallel_miss=((np.abs(slope)<=1e-10)&(constant>1e-9)).any(1)
        valid=(low<=high)&~parallel_miss&np.isfinite(high)&(high>0)&~fixed
        target=np.maximum(radius,np.where(valid,high+clearance,radius))
        points+=((target-radius)[:,None]*directions);radius=target
    for iteration in range(80):
        increase=np.zeros(len(points));minimum=np.inf
        for hull in planes:
            values=points@hull[:,:3].T+hull[:,3]
            slopes=directions@hull[:,:3].T
            # All movable corners must admit the same outward support plane;
            # pinned seam and pole corners must already clear that plane.
            scores=np.minimum(np.minimum(values[faces[:,0]],values[faces[:,1]]),values[faces[:,2]])
            allowed=np.ones_like(scores,dtype=bool)
            for corner in range(3):
                ids=faces[:,corner]
                allowed &= np.where(fixed[ids,None],values[ids]>=-1e-8,(values[ids]>=clearance)|(slopes[ids]>1e-5))
            cost=np.zeros_like(scores)
            for corner in range(3):
                ids=faces[:,corner]
                need=(values[ids]<clearance)&~fixed[ids,None]
                candidate=np.zeros_like(scores)
                np.divide(clearance-values[ids],slopes[ids],out=candidate,where=need & (slopes[ids]>1e-5))
                cost=np.maximum(cost,candidate)
            cost[~allowed]=np.inf
            choices=cost.argmin(1);best=scores[np.arange(len(faces)),choices]
            best[~np.isfinite(cost[np.arange(len(faces)),choices])]=-np.inf
            if not np.isfinite(best).all():raise ValueError(f'No outward plane can clear pinned cloth seam: face {faces[np.flatnonzero(~np.isfinite(best))[0]].tolist()}')
            minimum=min(minimum,float(best.min()))
            bad=np.flatnonzero(best < -1e-7)
            for corner in range(3):
                ids=faces[bad,corner];movable=~fixed[ids];ids=ids[movable];selected=choices[bad][movable]
                delta=np.zeros(len(ids));need=values[ids,selected]<clearance
                delta[need]=np.maximum(0.,(clearance-values[ids[need],selected[need]])/slopes[ids[need],selected[need]])
                np.maximum.at(increase,ids,delta)
        if minimum>=-1e-7:break
        points+=increase[:,None]*directions
    else:raise ValueError(f'Outward cloth clearance did not converge: {minimum}')
    area=np.linalg.norm(np.cross(points[faces[:,1]]-points[faces[:,0]],points[faces[:,2]]-points[faces[:,0]]),axis=1)/2
    if area.min()<1e-10:raise ValueError('Degenerate cloth triangle')
    edges=np.sort(np.vstack([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    if np.count_nonzero(counts==1)!=cols or counts.max()!=2:raise ValueError('Nonmanifold cloth mesh')
    return points,faces,dict(method='Cyclic meridian loft with whole-triangle hull-plane separation',
        guideCount=n,angularColumns=cols,rows=rows,rowParameters=row_parameters.tolist(),vertices=len(points),triangles=len(faces),
        boundaryVertices=cols,boundaryError=float(np.linalg.norm(points[:cols]-boundary,axis=1).max()),
        minimumTriangleSeparation=minimum if planes else None,clearanceIterations=iteration+1,
        maximumClearanceAdjustment=float(np.linalg.norm(points-original,axis=1).max()),
        minimumTriangleArea=float(area.min()),tipOffset=clearance*3,simulation=False)
