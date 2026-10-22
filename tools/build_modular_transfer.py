"""Bind retained anatomy to measured source surface donors, without engine data."""
import argparse,hashlib,json,struct,re
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import dijkstra

def axial_shape_weights(base):
    """Keep the complete authored distal component in one rigid material frame.

    The crown ring lies inside that component, after its shoulder. Beginning
    rigid ownership there bends the shoulder and tip in different frames. Fade
    into the adjacent shaft by rest-surface distance, before the authored seam.
    """
    arrays=np.load(base/'assets/wolverine-reference/geometry.npz')
    source=(base/'legacy/wolverine/src/runtime/r14_asset.h').read_text()
    names=['r14NewStart','r14RingCount','r14SegmentCount','r14CrownRing']
    values={name:int(re.search(r'\b'+name+r'\s*=\s*(\d+)',source)[1]) for name in names}
    count=len(arrays['r14_asset__r14Base'])//3
    start,rings,sides,crown=[values[name] for name in names]
    if start+rings*sides>count or crown<4:raise ValueError('Invalid authored crown ownership')
    points=arrays['r14_asset__r14Base'].reshape(-1,3).astype(float)
    faces=arrays['r14_asset__r14Indices'].reshape(-1,3)
    edges=np.unique(np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1),axis=0)
    lengths=np.linalg.norm(points[edges[:,1]]-points[edges[:,0]],axis=1)
    if np.any(lengths<=0):raise ValueError('Degenerate authored material edge')
    graph=sparse.csr_matrix((lengths,(edges[:,0],edges[:,1])),shape=(count,count))
    seam=points[start:start+sides]
    radius=np.sqrt(np.mean(np.sum((seam-seam.mean(axis=0))**2,axis=1)))
    distance=dijkstra(graph,directed=False,indices=np.arange(start,start+sides),min_only=True,limit=radius)
    t=np.clip(1-distance/radius,0,1)
    weights=t*t*t*(10+t*(-15+6*t))
    weights[start:]=1
    fine=arrays['rounded_render_data__rsFineSource'].reshape(-1,3)
    bary=arrays['rounded_render_data__rsFineBary'].reshape(-1,3)
    rounded=np.r_[weights,np.sum(weights[fine]*bary,axis=1)]
    direct=arrays['neck_render_data__nrDirect'];offsets=arrays['neck_render_data__nrRows']
    ids=arrays['neck_render_data__nrSources'];donors=arrays['neck_render_data__nrWeights']
    result=np.empty(len(direct))
    for i,index in enumerate(direct):
        result[i]=rounded[index] if index!=65535 else np.dot(rounded[ids[offsets[i]:offsets[i+1]]],donors[offsets[i]:offsets[i+1]])
    # The retained subdivision stencil includes small negative coefficients.
    # Bound this scalar support field without changing any geometry donors.
    if not np.isfinite(result).all() or result.min()<-.001 or result.max()>1.001:raise ValueError('Crown lineage leaves normalized support')
    return np.clip(result,0,1)


def build(assembly,runtime,output):
    if output.exists():raise ValueError('Use a fresh transfer artifact')
    mesh=np.load(assembly/'body-union.npz');owners=np.load(runtime.parent/'ownership.npz');donors=sparse.load_npz(assembly/'body-union-donors.npz')
    manifest=json.loads((assembly/'manifest.json').read_text());body_count=manifest['sourceBanks'][0]['count']
    body_path=Path(manifest['inputs'][0]['path']);body_faces=len(np.load(body_path)['faces'])
    retained=np.unique(mesh['faces'][mesh['source_faces']>=body_faces]);rows=[]
    axial=axial_shape_weights(Path(__file__).resolve().parents[1])
    if len(axial)!=donors.shape[1]-body_count:raise ValueError('Crown field differs from source topology')
    owner_frames=owners['frames'];owner_weights=owners['weights']
    for i in retained:
        ids=owner_frames[i];weights=owner_weights[i]
        active=ids[weights>1e-10]
        if np.all((active>=1)&(active<=12)):kind=1
        elif np.all(active>=13):kind=2
        else:continue
        row=donors.getrow(i);valid=row.indices>=body_count
        if not valid.all() or abs(row.data.sum()-1)>1e-8:raise ValueError('Retained anatomy includes a body donor')
        rows.append((int(i),kind,row.indices-body_count,row.data,float(row.data@axial[row.indices-body_count]) if kind==1 else 1.))
    with output.open('wb') as out:
        out.write(b'MMXFER02'+hashlib.sha256(runtime.read_bytes()).hexdigest().encode())
        out.write(struct.pack('<3I',donors.shape[1]-body_count,len(mesh['points']),len(rows)))
        for i,kind,ids,w,axial_weight in rows:
            out.write(struct.pack('<3Id',i,kind,len(ids),axial_weight))
            for index,weight in zip(ids,w):out.write(struct.pack('<Id',int(index),float(weight)))
    output.with_suffix(output.suffix+'.sha256').write_text(hashlib.sha256(output.read_bytes()).hexdigest()+'\n')
    print(json.dumps(dict(retainedVertices=len(rows),shaftVertices=sum(r[1]==1 for r in rows),suspendedVertices=sum(r[1]==2 for r in rows))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['assembly','runtime','output']:p.add_argument('--'+key,type=Path,required=True)
    build(**vars(p.parse_args()))
