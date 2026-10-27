"""Project canonical garment coverage through exact source vertex lineage.

Only rendering indices change. The complete anatomy remains available to the
solver, fitting and contacts. Unmapped body and boundary faces fail closed.
"""
import numpy as np


def project_covered_faces(source_positions, source_faces, uncovered_faces,
                          donors, weights, target_faces):
    source_positions=np.asarray(source_positions)
    source_faces=np.asarray(source_faces,dtype=np.int64)
    uncovered_faces=np.asarray(uncovered_faces,dtype=np.int64)
    donors=np.asarray(donors,dtype=np.int64);weights=np.asarray(weights)
    target_faces=np.asarray(target_faces,dtype=np.int64)
    if donors.shape!=weights.shape or donors.ndim!=2 or donors.shape[1]!=3:
        raise ValueError('Expected three exact lineage donors per target vertex')
    if (not np.isfinite(weights).all() or np.any(weights < -1e-5)
            or np.any(np.abs(weights.sum(axis=1)-1)>1e-4)):
        raise ValueError('Invalid source lineage weights')
    if (source_faces.ndim!=2 or source_faces.shape[1]!=3 or uncovered_faces.ndim!=2
            or uncovered_faces.shape[1]!=3 or target_faces.ndim!=2 or target_faces.shape[1]!=3
            or np.any(target_faces<0) or np.any(target_faces>=len(donors))):
        raise ValueError('Invalid coverage topology')
    if any(np.any(f<0) or np.any(f>=len(source_positions)) for f in [source_faces,uncovered_faces]):
        raise ValueError('Canonical coverage leaves source anatomy')
    # Exact coincident UV aliases have one geometric coverage identity.
    _,aliases=np.unique(source_positions,axis=0,return_inverse=True)
    keys=lambda faces:{tuple(sorted(aliases[face])) for face in faces}
    protected=keys(uncovered_faces);covered=keys(source_faces)-protected
    support=[]
    for ids,w in zip(donors,weights):
        used=ids[w>1e-6]
        support.append(None if np.any(used<0) or np.any(used>=len(aliases)) else set(aliases[used]))
    result=np.zeros(len(target_faces),dtype=bool)
    for i,face in enumerate(target_faces):
        rows=[support[v] for v in face]
        if any(row is None or not row for row in rows):continue
        union=set().union(*rows)
        result[i]=len(union)==3 and tuple(sorted(union)) in covered
    return result
