"""Export original-surface aperture inputs alongside the modular mechanics."""
import argparse,hashlib,struct,sys
from pathlib import Path
import numpy as np
from scipy import sparse
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_anatomy import vertex_normals

def build(body,assembly,module,runtime,output):
    b=np.load(body);m=np.load(assembly/'body-union.npz');a=np.load(module/'attachment.npz');r=np.load(assembly/'pelvic-ramp.npz')
    d=sparse.load_npz(assembly/'body-union-donors.npz');bank=sparse.load_npz(body.with_name(body.stem+'-donors.npz')).shape[1]
    body_weight=np.asarray(d[:,:bank].sum(1)).ravel();module_faces=np.flatnonzero((body_weight[m['faces']]<1e-10).all(1))
    normal=vertex_normals(SimpleNamespace(points=b['points'],faces=b['faces']))
    if output.exists():raise ValueError('Use a new dynamic aperture asset path')
    with output.open('wb') as f:
        f.write(b'MMMDYN01'+hashlib.sha256(runtime.read_bytes()).hexdigest().encode())
        f.write(struct.pack('<5I',len(b['points']),len(b['faces']),len(a['section_radial']),len(module_faces),len(r['rings'][-1])))
        for values,dtype in [(b['points'],'<f8'),(normal,'<f8'),(b['faces'],'<u4'),(a['section_radial'],'<f8'),(a['root'],'<f8'),(np.eye(3),'<f8'),(module_faces,'<u4'),(r['rings'][-1],'<u4')]:f.write(values.astype(dtype).tobytes())
    print(output)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['body','assembly','module','runtime','output']:p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();build(a.body,a.assembly,a.module,a.runtime,a.output)
