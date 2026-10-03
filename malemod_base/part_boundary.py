"""One positional and skin boundary for separately rendered body parts/LODs.

Correspondence is explicitly supplied as measured closed loops. Every side is
refined at the union of its original edge knots; UVs remain lineage attributes.
The canonical boundary is free to move in a combined collar solve. Other part
boundaries remain fixed. No engine resource, joint name or unit is assumed.
"""
from dataclasses import dataclass
import numpy as np
from .graft import (topology_ids, boundary_loops, ordered_loop, refine_boundary,
                    harmonic_fit, preserve_orientation, validate_mesh, sample_loop)


@dataclass
class BoundaryPart:
    points: np.ndarray
    faces: np.ndarray
    lineage: object
    face_lineage: np.ndarray
    seam: np.ndarray
    edge_donors: np.ndarray
    aliases: np.ndarray
    protected: np.ndarray
    orientation: dict

    def field(self, original):
        """Transfer UV/color/skin fields without merging their render aliases."""
        value = np.asarray(original, dtype=float)
        if value.ndim != 2 or value.shape[0] != self.lineage.shape[1] or not np.isfinite(value).all():
            raise ValueError('Attribute does not match the original part')
        return self.lineage @ value

    def bind_field(self, original, canonical):
        value = self.field(original)
        boundary = np.asarray(canonical, dtype=float)
        if boundary.shape != (len(self.seam), value.shape[1]) or not np.isfinite(boundary).all():
            raise ValueError('Canonical field does not match the common boundary')
        # Assign every positional alias of a knot, including both sides of a UV
        # seam. Do not overwrite UV fields with this method.
        for row, knot in enumerate(self.seam):
            value[self.aliases == self.aliases[knot]] = boundary[row]
        return value


@dataclass
class PartBoundary:
    canonical: np.ndarray
    angles: np.ndarray
    parts: list

    def publish(self, canonical, fields):
        """Apply one immutable solved boundary to every output part.

        Interiors come from a combined solver or explicit adapter rest fit.
        A caller cannot publish only one half or mismatched dimensions.
        """
        p = np.asarray(canonical, dtype=float)
        if p.shape != self.canonical.shape or not np.isfinite(p).all() or len(fields) != len(self.parts):
            raise ValueError('Incomplete boundary publication')
        output = []
        for part, field in zip(self.parts, fields):
            q = np.asarray(field, dtype=float).copy()
            if q.shape != part.points.shape or not np.isfinite(q).all():
                raise ValueError('Incomplete part output')
            for row, knot in enumerate(part.seam):
                q[part.aliases == part.aliases[knot]] = p[row]
            output.append(q)
        return output


def weld_parts(meshes, loops, projection, center, support_distance,
               maximum_repair, tolerance=1e-5, reference_part=0, knot_tolerance=0):
    """Refine all part/LOD sides to the reference side's measured edge curve.

    Original knots from every side survive as explicit edge donors. The
    reference part defines canonical geometry and fields, avoiding averaged
    independent surfaces that can drift under posing. A material UV weld is
    deliberately separate: positions/skin may share while UV islands differ.
    """
    if len(meshes) < 2 or len(meshes) != len(loops) or not 0 <= reference_part < len(meshes):
        raise ValueError('Expected matched part meshes and measured loops')
    projection = np.asarray(projection, dtype=float);center = np.asarray(center, dtype=float)
    if projection.shape != (2, 3) or center.shape != (3,) or not np.isfinite(projection).all() or not np.isfinite(center).all():
        raise ValueError('Invalid boundary projection')
    if not np.allclose(projection @ projection.T, np.eye(2), atol=1e-8):
        raise ValueError('Boundary projection must have orthonormal rows')
    if not np.isfinite(support_distance) or support_distance <= 0 or tolerance <= 0:
        raise ValueError('Invalid support distance/tolerance')
    records=[]
    for (points, faces), loop in zip(meshes, loops):
        p=np.asarray(points,dtype=float);f=np.asarray(faces)
        validate_mesh(p,f)
        keep,ids=topology_ids(p,tolerance);loop=np.asarray(loop)
        if loop.ndim!=1 or not np.issubdtype(loop.dtype,np.integer) or len(loop)<3 or loop.min()<0 or loop.max()>=len(keep):
            raise ValueError('Loop must index the measured positional topology')
        candidates=boundary_loops(ids[f])
        if not any(set(l)==set(loop) for l in candidates):
            raise ValueError('Selected loop is not an actual part boundary')
        ordered,theta=ordered_loop(p[keep],loop,projection,center)
        records.append((p,f,keep,ids,ordered,theta,candidates))
    # Distinct measured knots, even close ones, are retained; identical knots
    # share exactly. Near coincident knots are snapped only within angular
    # tolerance of ordered_loop so refinement never creates zero-area slivers.
    minimum_radius=min(np.linalg.norm((r[0][r[2][r[4]]]-center)@projection.T,axis=1).min() for r in records)
    if minimum_radius<=tolerance*10:raise ValueError('Boundary passes through projection center')
    if not np.isfinite(knot_tolerance) or knot_tolerance<0:raise ValueError('Invalid knot tolerance')
    angular_tolerance=max(1e-7,tolerance*4/minimum_radius,knot_tolerance)
    knots=np.sort(np.concatenate([r[5] for r in records]));angles=[knots[0]]
    for a in knots[1:]:
        if a-angles[-1]>=angular_tolerance:angles.append(a)
    if angles[0]+2*np.pi-angles[-1]<angular_tolerance:angles.pop()
    samples=np.asarray(angles)
    refined=[]
    for p,f,keep,ids,loop,theta,candidates in records:
        nearest=np.abs(np.angle(np.exp(1j*(theta[:,None]-samples[None,:])))).argmin(1)
        snapped=samples[nearest]
        # Every original edge endpoint must become a canonical knot, including
        # numerically near-identical exporter coordinates on another side.
        if len(np.unique(nearest))!=len(theta):raise ValueError('Tolerance merges two original boundary vertices')
        order=np.argsort(snapped);loop=loop[order];theta=snapped[order]
        a,b,u=sample_loop(loop,theta,samples)
        # Intersect the actual projected straight edge with the sample ray.
        # Linear interpolation of angles is not linear interpolation of an
        # edge; using it bends corresponding segments onto different curves.
        xy=(p[keep]-center)@projection.T;ray=np.column_stack([np.cos(samples),np.sin(samples)])
        edge=xy[b]-xy[a]
        cross=lambda x,y:x[:,0]*y[:,1]-x[:,1]*y[:,0]
        denominator=cross(edge,ray)
        if np.any(np.abs(denominator)<1e-12):raise ValueError('Boundary edge parallel to its sample ray')
        precise=-cross(xy[a],ray)/denominator
        precise=np.where(u<1e-8,0,np.where(u>1-1e-8,1,precise))
        if np.any((precise < -1e-8)|(precise > 1+1e-8)):raise ValueError('Ray leaves the selected original edge')
        edge_samples=(a,b,np.clip(precise,0,1))
        matrix,faces,seam,donors,face_ids=refine_boundary(p,f,ids,keep,loop,theta,samples,edge_samples)
        refined.append((matrix,faces,seam,donors,face_ids))
    r=reference_part;canonical=np.asarray(refined[r][0] @ records[r][0])[refined[r][2]].copy()
    parts=[]
    for part_id,(rec,ref) in enumerate(zip(records,refined)):
        p,f,keep,ids,loop,theta,candidates=rec
        matrix,faces,seam,donors,face_ids=ref;rest=np.asarray(matrix @ p)
        fitted,distance=harmonic_fit(rest,faces,seam,canonical,support_distance,tolerance)
        # Distant surfaces are exactly fixed by harmonic_fit. Explicitly lock
        # every other native part boundary too, even if nearby.
        other=np.concatenate([l for l in candidates if set(l)!=set(loop)]) if len(candidates)>1 else np.array([],dtype=int)
        original_locked=np.flatnonzero(np.isin(ids,other))
        fitted[original_locked]=rest[original_locked]
        locked=np.unique(np.r_[seam,original_locked,np.flatnonzero(distance>=support_distance)])
        try:fitted,report=preserve_orientation(rest,fitted,faces,locked,maximum_repair,tolerance)
        except ValueError as error:raise ValueError(f'Part {part_id} orientation: {error}') from error
        unique,aliases=topology_ids(fitted,tolerance)
        # Make the canonical knots bit-identical after numerical fitting; aliases
        # selected in original topology survive their UV/color independence.
        for row,knot in enumerate(seam):fitted[aliases==aliases[knot]]=canonical[row]
        protected=np.flatnonzero(np.isin(np.arange(len(fitted)),original_locked))
        parts.append(BoundaryPart(fitted,faces,matrix,face_ids,seam,donors,aliases,protected,report))
    return PartBoundary(canonical,samples,parts)
