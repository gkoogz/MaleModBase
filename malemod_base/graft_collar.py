"""Coupled collar domain for a fitted graft, preserving native part boundaries.

The caller supplies the measured source-space positions/frame and actual guide
targets. This class does not invent a Raphe guide or engine upload capability.
"""
import numpy as np
from .graft import topology_ids
from .collar import CollarPlan, radial_targets
from .surface_limit import correction_limit


class GraftCollar:
    def __init__(self, points, faces, body_seam, module_seam, body_edge_donors,
                 protected_vertices, alias_tolerance):
        self.points = np.asarray(points, dtype=float).copy()
        faces = np.asarray(faces)
        body = np.asarray(body_seam); module = np.asarray(module_seam)
        donors = np.asarray(body_edge_donors, dtype=float)
        protected = np.asarray(protected_vertices)
        if (faces.ndim != 2 or faces.shape[1] != 3 or
                not np.issubdtype(faces.dtype,np.integer) or not len(faces) or
                faces.min()<0 or faces.max()>=len(self.points) or
                body.ndim!=1 or module.shape!=body.shape or donors.shape!=(len(body),3) or
                not np.isfinite(donors).all() or not len(body) or
                any(not np.issubdtype(x.dtype,np.integer) for x in [body,module,protected]) or
                any(x.size and (x.min()<0 or x.max()>=len(self.points)) for x in [body,module,protected])):
            raise ValueError('Invalid graft topology/seam/protected boundary')
        if not np.array_equal(donors[:,:2],np.floor(donors[:,:2])) or np.any((donors[:,2]<0)|(donors[:,2]>1)):
            raise ValueError('Donor IDs/weights must retain the original body edges')
        if np.any(donors[:,:2]<0) or np.any(donors[:,:2]>=len(self.points)):
            raise ValueError('Body edge donor outside the original domain')
        unique, self.aliases = topology_ids(self.points,alias_tolerance)
        self.unique = unique;self.rest = self.points[unique]
        if np.any(self.aliases[body]!=self.aliases[module]):
            raise ValueError('Fitted body and module seams do not coincide')
        raw_faces = self.aliases[faces]
        keep=np.all(np.diff(np.sort(raw_faces,axis=1),axis=1)!=0,axis=1)
        self.faces=raw_faces[keep]
        # Shared original-edge constraints govern both seam halves and their UV
        # aliases. Fine body samples are also slaves, not free approximations.
        self.seams=[];seen={}
        for slave,(a,b,w) in zip(self.aliases[body],donors):
            a,b=self.aliases[int(a)],self.aliases[int(b)]
            expected=self.rest[a]*(1-w)+self.rest[b]*w
            if np.linalg.norm(self.rest[slave]-expected)>alias_tolerance*2:
                raise ValueError('Rest seam no longer matches original edge donors')
            if slave==a and w<=1e-12 or slave==b and w>=1-1e-12:continue
            row=(int(slave),int(a),int(b),float(w))
            if slave in seen and seen[slave]!=row:
                raise ValueError('Inconsistent original-edge donors for a seam alias')
            if slave not in seen:self.seams.append(row);seen[slave]=row
        self.locked=np.unique(self.aliases[protected])
        self.plan=None;self._frame=None

    def prepare(self, frame):
        # One factorization per rest metric/frame, not per rendered vertex/frame.
        if self.plan is None or self._frame!=frame:
            self.plan=CollarPlan(self.rest,self.faces,self.seams,frame,self.locked)
            self._frame=frame
        return self.plan

    def solve(self, targets, frame):
        targets=np.asarray(targets,dtype=float)
        if targets.shape!=self.points.shape or not np.isfinite(targets).all():
            raise ValueError('Expected finite targets on the unchanged render domain')
        plan=self.prepare(frame)
        # Canonical representatives, rather than averaging seam constraints or
        # UV aliases. The hard donor elimination determines every seam output.
        result=plan.solve(targets[self.unique])
        return result[self.aliases]

    def solve_guided(self, frame, guide_centers, guide_radii):
        centers=np.asarray(guide_centers,dtype=float);radii=np.asarray(guide_radii,dtype=float)
        if centers.shape!=self.points.shape or radii.shape!=(len(self.points),):
            raise ValueError('Actual guide samples must match the render domain')
        plan=self.prepare(frame)
        targets=radial_targets(self.rest,frame,plan.mask,centers[self.unique],radii[self.unique])
        return plan.solve(targets)[self.aliases]

    def solve_checked(self, targets, frame, area_floor):
        """Bound the whole correction, maintaining every hard linear constraint.

        The caller chooses the allowed projected-area fraction explicitly. A
        limited correction is NOT evidence that the requested shape was reached.
        Native adapters must expose the accepted fraction and reject an envelope
        whose limited output no longer meets their required shape accuracy.
        """
        output=self.solve(targets,frame)
        proposed=output[self.unique]-self.rest
        # Degenerate topology is excluded by the domain, not silently exempted
        # through a source-unit magic cutoff. Exact zero is the only exemption.
        fraction=correction_limit(self.rest,proposed,self.faces,area_floor,0.)
        accepted=self.rest+proposed*fraction
        return accepted[self.aliases],fraction
