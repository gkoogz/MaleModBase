"""Measured contact-area recruitment on an explicitly supplied surface chart.

Area is integrated on the source triangles, not on their flattened chart or on
the connecting skin. No game units, anatomical proportions or surface topology
are inferred. The caller owns a non-overlapping chart and its eligible faces.
"""
import numpy as np
from .modular_anatomy import rear_extended_stations


def underside_rear_footprint(footprint, extension, transition_end, half_angle):
    """Recruit rear contact into a lower support sector, preserving the front.

    Chart coordinates are axial/radial station and angle from the underside.
    The caller supplies the last ramp station before body contact. Angular
    contraction finishes there, so body recruitment cannot inherit the upper
    shaft's side wrap. This is an injective chart deformation, not smoothing
    of the body or a global reduction of the original attachment width.
    """
    p=np.asarray(footprint,float).copy()
    if p.ndim!=2 or p.shape[1]!=2 or not np.isfinite(p).all():
        raise ValueError('Invalid underside footprint')
    start=float(p[:,0].min())
    if not np.isfinite([transition_end,half_angle]).all() or transition_end>=start or not 0<half_angle<np.pi/2:
        raise ValueError('Invalid underside recruitment sector')
    p[:,0]=rear_extended_stations(p[:,0],extension)
    t=np.clip((start-p[:,0])/(start-transition_end),0,1)
    blend=t*t*t*(10+t*(-15+6*t))
    angular_scale=min(1.,half_angle/max(float(abs(p[:,1]).max()),1e-12))
    p[:,1]*=1-blend*(1-angular_scale)
    return p


def transfer_chart(points, source_points, source_faces, source_chart, tolerance=1e-7, periodic_axes=()):
    """Interpolate a chart from exact source triangles after edge refinement.

    Re-evaluating nonlinear polar coordinates at inserted edge points bends
    chart edges and can overlap adjacent needle triangles. Preserve donors.
    """
    from scipy.spatial import cKDTree
    p=np.asarray(points,float);source=np.asarray(source_points,float)
    faces=np.asarray(source_faces,int);chart=np.asarray(source_chart,float)
    distance,vertex=cKDTree(source).query(p);exact=distance<tolerance
    result=np.zeros((len(p),2));result[exact]=chart[vertex[exact]]
    triangles=source[faces];tree=cKDTree(triangles.mean(1))
    for i in np.flatnonzero(~exact):
        found=False
        for count in [32,256]:
            _,neighbors=tree.query(p[i],k=min(count,len(faces)))
            for fi in np.atleast_1d(neighbors):
                tri=triangles[fi];edge=(tri[1:]-tri[0]).T
                uv=np.linalg.lstsq(edge,p[i]-tri[0],rcond=None)[0]
                weights=np.r_[1-uv.sum(),uv]
                if weights.min()>=-1e-8 and np.linalg.norm(weights@tri-p[i])<tolerance:
                    values=chart[faces[fi]].copy()
                    for axis in periodic_axes:values[:,axis]=np.unwrap(values[:,axis])
                    result[i]=weights@values
                    for axis in periodic_axes:result[i,axis]=np.angle(np.exp(1j*result[i,axis]))
                    found=True;break
            if found:break
        if not found:raise ValueError(f'Refined point {i} at {p[i].tolist()} has no exact chart donor')
    return result


class ContactArea:
    def __init__(self, points, faces, chart, eligible):
        from shapely import Polygon, STRtree, union_all
        points=np.asarray(points,float);faces=np.asarray(faces,int)
        chart=np.asarray(chart,float);eligible=np.asarray(eligible,bool)
        if chart.shape!=(len(points),2) or eligible.shape!=(len(faces),):
            raise ValueError('Contact chart dimensions differ from source mesh')
        self.face_ids=np.flatnonzero(eligible)
        self.points=points;self.faces=faces;self.chart=chart
        xy=chart[faces[self.face_ids]];p=points[faces[self.face_ids]]
        self.polygons=np.array([Polygon(t) for t in xy],dtype=object)
        planar=np.array([t.area for t in self.polygons])
        world=.5*np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)
        if not len(planar) or np.any(planar<=1e-12) or np.any(world<=1e-12):
            raise ValueError('Contact chart contains a degenerate triangle')
        self.density=world/planar
        self.tree=STRtree(self.polygons)
        self.coverage=union_all(self.polygons)
        self.overlap=float(planar.sum()-self.coverage.area)

    def area(self, footprint, require_coverage=True):
        from shapely import Polygon, intersection, area, union_all
        region=Polygon(footprint)
        if not region.is_valid or region.area<=1e-12:
            raise ValueError('Invalid contact footprint')
        missing=region.difference(self.coverage).area
        if require_coverage and missing>max(1e-8,region.area*1e-8):
            raise ValueError(f'Contact footprint leaves measured surface: {missing:g}')
        ids=self.tree.query(region,predicate='intersects')
        parts=intersection(self.polygons[ids],region);areas=area(parts)
        if self.overlap>1e-7:
            overlap=float(areas.sum()-union_all(parts).area)
            if overlap>max(1e-7,region.area*1e-8):
                raise ValueError(f'Contact chart overlaps inside footprint: {overlap:g}')
        return float(np.dot(areas,self.density[ids]))

    def centroid(self, footprint):
        """First moment on the physical surface, independent of knot density."""
        from shapely import Polygon,intersection,area,centroid,get_coordinates
        self.area(footprint)  # enforce coverage and reject multiply counted area
        region=Polygon(footprint);ids=self.tree.query(region,predicate='intersects')
        parts=intersection(self.polygons[ids],region)
        weights=area(parts)*self.density[ids];valid=weights>1e-14
        ids=ids[valid];parts=parts[valid];weights=weights[valid]
        xy=get_coordinates(centroid(parts));faces=self.faces[self.face_ids[ids]]
        chart=self.chart[faces]
        inverse=np.linalg.inv(np.concatenate([chart.transpose(0,2,1),np.ones((len(chart),1,3))],axis=1))
        bary=np.einsum('nij,nj->ni',inverse,np.c_[xy,np.ones(len(xy))])
        points=np.einsum('ni,nij->nj',bary,self.points[faces])
        return np.sum(points*weights[:,None],axis=0)/weights.sum()

    def minimum_projection(self, footprint, origin, direction, selected_faces):
        """Exact half-space check over clipped contact, not just rim knots."""
        from shapely import Polygon, get_coordinates
        selected=np.asarray(selected_faces,bool)
        origin=np.asarray(origin,float);direction=np.asarray(direction,float)
        if selected.shape!=(len(self.faces),) or origin.shape!=(3,) or direction.shape!=(3,) or not np.isfinite([origin,direction]).all() or np.linalg.norm(direction)<=0:
            raise ValueError('Invalid contact half-space')
        self.area(footprint)
        region=Polygon(footprint);minimum=None
        for index in self.tree.query(region,predicate='intersects'):
            fi=self.face_ids[index]
            if not selected[fi]:continue
            part=self.polygons[index].intersection(region)
            if part.area<=1e-14:continue
            coords=get_coordinates(part);face=self.faces[fi]
            weights=np.c_[coords,np.ones(len(coords))]@np.linalg.inv(np.vstack([self.chart[face].T,np.ones(3)])).T
            value=float(np.min((weights@self.points[face]-origin)@direction))
            minimum=value if minimum is None else min(minimum,value)
        return minimum

    def expand_rear(self, footprint, ratio, maximum_extension, tolerance=1e-5, proposal=None):
        """Solve a physical-area target with the original distal half exact."""
        footprint=np.asarray(footprint,float)
        if not np.isfinite([ratio,maximum_extension,tolerance]).all() or ratio<1 or maximum_extension<=0 or tolerance<=0:
            raise ValueError('Invalid contact-area target')
        baseline=self.area(footprint);target=baseline*ratio
        if proposal is None:
            def proposal(extension):
                p=footprint.copy();p[:,0]=rear_extended_stations(p[:,0],extension)
                return p
        low=0.;high=float(maximum_extension)
        if ratio==1:
            return footprint.copy(),dict(baselineArea=baseline,targetArea=baseline,
                measuredArea=baseline,areaRatio=1.,rearExtension=0.,
                fixedFrontSamples=len(footprint),frontMaximumChartError=0.,
                measure='source triangle surface area; not chart or neck area')
        # Capacity is checked rather than silently clamping the requested area.
        capacity=self.area(proposal(high))
        if capacity<target*(1-tolerance):
            raise ValueError(f'Insufficient measured support: {capacity/baseline:g} < {ratio:g}')
        for _ in range(64):
            middle=.5*(low+high);p=proposal(middle);value=self.area(p)
            if abs(value-target)<=target*tolerance:break
            if value<target:low=middle
            else:high=middle
        fixed=footprint[:,0]>=.5*(footprint[:,0].min()+footprint[:,0].max())
        if not np.array_equal(p[fixed],footprint[fixed]):
            raise AssertionError('Recruitment moved the fixed front footprint')
        return p,dict(baselineArea=baseline,targetArea=target,measuredArea=value,
                      areaRatio=value/baseline,rearExtension=middle,
                      fixedFrontSamples=int(fixed.sum()),frontMaximumChartError=0.,
                      measure='source triangle surface area; not chart or neck area')
