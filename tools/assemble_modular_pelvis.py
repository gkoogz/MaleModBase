"""Assemble measured, co-aligned body/module authoring meshes; never install.

Inputs use the Mesh NPZ plus sparse donor sidecar format exported by
build_modular_anatomy.py. Body calibration and native resource extraction belong
to the adapter. Source donor banks remain distinct in the combined artifact.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy import sparse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_anatomy import Mesh
from malemod_base.modular_pelvis import pelvic_union
from build_modular_anatomy import save


def load(path):
    data=np.load(path,allow_pickle=False)
    donors=path.with_name(path.stem+'-donors.npz')
    mesh=Mesh(data['points'],data['faces'],data['corner_uv'],sparse.load_npz(donors),data['source_faces'],data['generated'],
              data['normal_field'] if 'normal_field' in data else None)
    mesh.validate();return mesh,donors


def assemble(body_path,module_path,attachment_path,output,margin=.12,underside_margin=.08):
    if output.exists():raise ValueError('Use a fresh authoring output directory')
    body,body_donors=load(body_path);module,module_donors=load(module_path)
    contract=np.load(attachment_path,allow_pickle=False)
    bc=body.donors.shape[1];mc=module.donors.shape[1]
    body.donors=sparse.hstack([body.donors,sparse.csr_matrix((len(body.points),mc))],format='csr')
    module.donors=sparse.hstack([sparse.csr_matrix((len(module.points),bc)),module.donors],format='csr')
    root=contract['root'];axis=contract['section_center']-root;radial=contract['section_radial']
    result,ramp,report=pelvic_union(body,module,root,axis,radial,margin=margin,
        mount_axis=contract['mount_axis'] if 'mount_axis' in contract else axis,
        mount_radial=contract['mount_radial'] if 'mount_radial' in contract else radial,underside_margin=underside_margin)
    output.mkdir(parents=True);save(result,output/'body-union');np.savez_compressed(output/'pelvic-ramp.npz',**ramp)
    from malemod_base.modular_shading import corner_frames
    frames=corner_frames(result);np.savez_compressed(output/'corner-frames.npz',**frames)
    report.update(schema='malemod.modular-pelvic-union/1',sourceBanks=[dict(kind='body',first=0,count=bc),dict(kind='module',first=bc,count=mc)],
        inputs=[dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [body_path,body_donors,module_path,module_donors,attachment_path]],
        nativeRuntime=False,installed=False,attachmentGatePassed=False,
        shading=dict(normalOwnership='shared geometric vertex; inherited retained-surface field',
                     singularUVFaces=int(len(frames['singular_uv_faces'])),materialsValidated=False),
        limitations=['Local orientation and boundary checks are not a global self-intersection proof.',
                     'Native material transfer, topology/LOD budgets, skinning, control mapping and physics require adapter adoption.'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['body','module','attachment','output']:parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--margin',type=float,default=.12)
    parser.add_argument('--underside-margin',type=float,default=.08)
    args=parser.parse_args();assemble(args.body,args.module,args.attachment,args.output,args.margin,args.underside_margin)
