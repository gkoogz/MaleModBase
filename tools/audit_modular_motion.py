"""Evaluate the C++ modular plan against measured source packets, offline only."""
import argparse,json,subprocess,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_pelvis import ramp_orientation

def audit(executable,asset,assembly,cases,output):
    output.mkdir(parents=True,exist_ok=False)
    mesh=np.load(assembly/'body-union.npz');ramp=np.load(assembly/'pelvic-ramp.npz')
    report=[]
    for source in cases:
        target=output/(source.stem+'.bin')
        result=subprocess.run([str(executable),str(asset),str(source),str(target)],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(f'{source.name}: {result.stderr.strip()}')
        p=np.fromfile(target,dtype='<f8').reshape(-1,3)
        if len(p)!=len(mesh['points']) or not np.isfinite(p).all():raise ValueError('Invalid evaluator output')
        orientation=ramp_orientation(p,ramp['rings'],ramp['angles'])
        tri=p[mesh['faces']];area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
        report.append(dict(source=str(source),points=str(target),**json.loads(result.stdout),
                           reversedRampFaces=orientation['reversedFaces'],minimumArea=float(area.min()),
                           nativeObserved=False))
    (output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['executable','asset','assembly','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--source',type=Path,action='append',required=True)
    a=p.parse_args();audit(a.executable,a.asset,a.assembly,a.source,a.output)
