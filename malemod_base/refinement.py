"""Conforming local midpoint refinement with sparse source attribute lineage."""
import numpy as np
from scipy import sparse


def refine(points, faces, selected, locked_edges=(), aliases=None):
    p=np.asarray(points,dtype=float);f=np.asarray(faces,dtype=np.int64);chosen=np.asarray(selected,dtype=bool)
    if p.ndim!=2 or p.shape[1]!=3 or f.ndim!=2 or f.shape[1]!=3 or chosen.shape!=(len(f),) or not np.isfinite(p).all() or f.min()<0 or f.max()>=len(p):
        raise ValueError('Invalid refinement domain')
    locked={tuple(sorted(map(int,e))) for e in locked_edges}
    split={tuple(sorted(map(int,e))) for tri in f[chosen] for e in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]}-locked
    if aliases is not None:
        ids=np.asarray(aliases,dtype=int)
        if ids.shape!=(len(p),):raise ValueError('Invalid positional aliases')
        key=lambda e:tuple(sorted((int(ids[e[0]]),int(ids[e[1]]))))
        wanted={key(e) for e in split}-{key(e) for e in locked}
        split={tuple(sorted(map(int,e))) for tri in f for e in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])] if key(e) in wanted}
    edges=sorted(split);mid={e:len(p)+i for i,e in enumerate(edges)}
    result=[];parents=[]
    for parent,(a,b,c) in enumerate(f):
        ab=mid.get(tuple(sorted((a,b))));bc=mid.get(tuple(sorted((b,c))));ca=mid.get(tuple(sorted((c,a))))
        n=sum(v is not None for v in [ab,bc,ca])
        if n==0:tri=[(a,b,c)]
        elif n==3:tri=[(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)]
        elif n==1:
            if ab is not None:tri=[(a,ab,c),(ab,b,c)]
            elif bc is not None:tri=[(b,bc,a),(bc,c,a)]
            else:tri=[(c,ca,b),(ca,a,b)]
        else:
            if ca is None:tri=[(ab,b,bc),(a,ab,bc),(a,bc,c)]
            elif ab is None:tri=[(bc,c,ca),(b,bc,ca),(b,ca,a)]
            else:tri=[(ca,a,ab),(c,ca,ab),(c,ab,b)]
        result.extend(tri);parents.extend([parent]*len(tri))
    rows=list(range(len(p)));cols=rows.copy();values=[1.]*len(p)
    for (a,b),i in mid.items():rows.extend([i,i]);cols.extend([a,b]);values.extend([.5,.5])
    lineage=sparse.coo_matrix((values,(rows,cols)),shape=(len(p)+len(edges),len(p))).tocsr()
    return lineage@p,np.asarray(result),lineage,np.asarray(parents)
