"""Bind retained anatomy to measured source surface donors, without engine data."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
from scipy import sparse

def build(assembly,runtime,output):
    if output.exists():raise ValueError('Use a fresh transfer artifact')
    mesh=np.load(assembly/'body-union.npz');owners=np.load(runtime.parent/'ownership.npz');donors=sparse.load_npz(assembly/'body-union-donors.npz')
    manifest=json.loads((assembly/'manifest.json').read_text());body_count=manifest['sourceBanks'][0]['count']
    body_path=Path(manifest['inputs'][0]['path']);body_faces=len(np.load(body_path)['faces'])
    retained=np.unique(mesh['faces'][mesh['source_faces']>=body_faces]);rows=[]
    for i in retained:
        ids=owners['frames'][i];weights=owners['weights'][i]
        active=ids[weights>1e-10]
        if np.all((active>=1)&(active<=12)):kind=1
        elif np.all(active>=13):kind=2
        else:continue
        row=donors.getrow(i);valid=row.indices>=body_count
        if not valid.all() or abs(row.data.sum()-1)>1e-8:raise ValueError('Retained anatomy includes a body donor')
        rows.append((int(i),kind,row.indices-body_count,row.data))
    with output.open('wb') as out:
        out.write(b'MMXFER01'+hashlib.sha256(runtime.read_bytes()).hexdigest().encode())
        out.write(struct.pack('<3I',donors.shape[1]-body_count,len(mesh['points']),len(rows)))
        for i,kind,ids,w in rows:
            out.write(struct.pack('<3I',i,kind,len(ids)))
            for index,weight in zip(ids,w):out.write(struct.pack('<Id',int(index),float(weight)))
    output.with_suffix(output.suffix+'.sha256').write_text(hashlib.sha256(output.read_bytes()).hexdigest()+'\n')
    print(json.dumps(dict(retainedVertices=len(rows),shaftVertices=sum(r[1]==1 for r in rows),suspendedVertices=sum(r[1]==2 for r in rows))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['assembly','runtime','output']:p.add_argument('--'+key,type=Path,required=True)
    build(**vars(p.parse_args()))
