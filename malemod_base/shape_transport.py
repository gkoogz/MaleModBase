"""Source-backed positive section fits for independently driven adapter cages.

Consumes the versioned authored coarse stage, not the final prepared surface.
Native names, frames, units and hierarchy are supplied by the adapter.
"""
import numpy as np
from .authored_shape import AuthoredShape, MORPH_REFERENCE
from .controls import defaults, mapped, limits

AXES = ('overall', 'width', 'length', 'scrotum')


def inverse_mapping(key, value):
    lo, hi = limits(key)
    for _ in range(60):
        mid = (lo + hi) / 2
        if mapped(key, mid) < value: lo = mid
        else: hi = mid
    return (lo + hi) / 2


def mapped_knots(key):
    index = {'overall': 0, 'length': 1, 'width': 2, 'scrotum': 3}[key]
    lo, hi = limits(key)
    return np.array(sorted(set([mapped(key, lo), mapped(key, 50),
        MORPH_REFERENCE[index][1], mapped(key, hi)])))


def coordinates(key):
    knots = mapped_knots(key)
    lo, hi = limits(key)
    return tuple(float(np.interp(mapped(key, ui), knots, np.arange(len(knots))))
                 for ui in range(lo, hi + 1))


class ShapeTransport:
    def __init__(self, bank, origins, frames):
        self.shape = AuthoredShape(bank)
        self.origins = np.asarray(origins, dtype=float)
        self.frames = np.asarray(frames, dtype=float)
        if self.origins.shape != (10, 3) or self.frames.shape != (10, 3, 3):
            raise ValueError('Expected ten calibrated independent cage frames')
        if not np.isfinite(self.origins).all() or not np.isfinite(self.frames).all():
            raise ValueError('Nonfinite cage frames')
        np.testing.assert_allclose(self.frames.transpose(0,2,1) @ self.frames,
                                   np.broadcast_to(np.eye(3), (10,3,3)), atol=1e-6)
        s = self.shape
        self.masks = [(abs(s.flex-t)<.075) & (s.shaft>.5)
                      for t in np.linspace(0,1,8)]
        self.masks += [(s.ball>.8) & (s.base[:,1]*side>.25) for side in [-1,1]]
        if min(m.sum() for m in self.masks) < 8:
            raise ValueError('Insufficient source support for cage fit')
        self.reference = s.evaluate(defaults()).coarse

    def evaluate(self, preferences=None):
        target = self.shape.evaluate(preferences or defaults()).coarse
        result = []
        for origin, frame, mask in zip(self.origins, self.frames, self.masks):
            a = (self.reference[mask]-origin) @ frame
            b = (target[mask]-origin) @ frame
            ac, bc = a.mean(0), b.mean(0)
            aa, bb = a-ac, b-bc
            # Proper Procrustes rotation (no reflections), then positive RMS
            # section dimensions. A fixed-axis regression can invert axes when
            # the authored short/long targets rotate a curved section.
            u,_,vt=np.linalg.svd(aa.T@bb)
            correction=np.eye(3);correction[2,2]=np.linalg.det(vt.T@u.T)
            rotation=vt.T@correction@u.T
            aligned=bb@rotation
            scale=np.sqrt(np.sum(aligned*aligned,axis=0)/np.sum(aa*aa,axis=0))
            translation = bc-rotation@(ac*scale)
            if not np.isfinite(scale).all() or np.min(scale)<=0:
                raise ValueError('Source fit inverted a cage axis')
            result.append(np.r_[translation,scale,rotation.reshape(-1)])
        return np.array(result)

    def lattice(self):
        knots = [mapped_knots(key) for key in AXES]
        result = np.empty(tuple(len(k) for k in knots)+(10,15))
        for index in np.ndindex(result.shape[:4]):
            preferences = {key:inverse_mapping(key,knots[i][index[i]])
                           for i,key in enumerate(AXES)}
            result[index] = self.evaluate(preferences)
        return result
