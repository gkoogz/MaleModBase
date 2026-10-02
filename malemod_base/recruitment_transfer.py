"""Versioned transfer of evaluated coupled body changes to another character.

Inputs share a caller-calibrated source authoring frame. This is an offline
material-field transfer, not a claim of source posed-body or engine parity.
"""
import numpy as np
from scipy.spatial import cKDTree

VERSION = 1

class SourceBodyField:
    def __init__(self, source, query, neighbours=8):
        source=np.asarray(source,dtype=float);query=np.asarray(query,dtype=float)
        if (source.ndim!=2 or source.shape[1]!=3 or query.ndim!=2 or query.shape[1]!=3
                or not np.isfinite(source).all() or not np.isfinite(query).all() or len(source)<neighbours):
            raise ValueError('Expected finite calibrated source/query surfaces')
        self.source=source.copy()
        distance,self.donors=cKDTree(source).query(query,k=neighbours)
        weights=1/np.maximum(distance,.05)**2
        exact=distance[:,0]<1e-8
        weights[exact]=0;weights[exact,0]=1
        self.weights=weights/weights.sum(1)[:,None]
        self.distance=distance[:,0]

    def displacement(self, evaluated):
        evaluated=np.asarray(evaluated,dtype=float)
        if evaluated.shape!=self.source.shape or not np.isfinite(evaluated).all():
            raise ValueError('Evaluated source body topology differs')
        return np.sum((evaluated-self.source)[self.donors]*self.weights[:,:,None],axis=1)

def virtual_bind_translation(bind_point, material_point, current_point, rotation):
    """Native joint translation for a changed material pivot and fixed rig bind.

    The resulting skin map is R*(vertex-material_point)+current_point while
    retaining the engine's original inverse bind. No vertex update is needed.
    """
    bind=np.asarray(bind_point);material=np.asarray(material_point);current=np.asarray(current_point);r=np.asarray(rotation)
    return current-material+(np.eye(3)-r)@(material-bind)
