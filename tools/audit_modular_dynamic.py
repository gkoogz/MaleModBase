"""Audit exact surface cuts and dynamic ramps against captured solver packets."""
import argparse,json,subprocess,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_pelvis import ramp_orientation

def audit(executable,asset,dynamic,body,sources,output):
    output.mkdir(parents=True,exist_ok=False)
    original=np.load(body);reports=[]
    for source in sources:
        label=source.stem.removesuffix('-source');prefix=output/label
        result=subprocess.run([str(executable),str(asset),str(dynamic),str(source),str(prefix)],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(label+': '+result.stderr)
        row=json.loads(result.stdout);p=np.fromfile(str(prefix)+'-points.bin','<f8').reshape(-1,3);f=np.fromfile(str(prefix)+'-faces.bin','<u4').reshape(-1,3)
        rings=np.fromfile(str(prefix)+'-rings.bin','<u4').reshape(row['rows'],row['knots'])
        q=p[rings[0]]-row['anchor'];theta=np.arctan2(q[:,2],q[:,1])%(2*np.pi)
        orientation=ramp_orientation(p,rings,theta)
        edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1);_,counts=np.unique(edges,axis=0,return_counts=True)
        unchanged=np.array_equal(p[:len(original['points'])],original['points'])
        row.update(label=label,source=str(source),originalVerticesUnchanged=unchanged,boundaryEdges=int(np.sum(counts==1)),nonmanifoldEdges=int(np.sum(counts>2)),reversedRampFaces=orientation['reversedFaces'],minimumRampCosine=orientation['minimumOutwardCosine'],nativeObserved=False)
        reports.append(row)
    (output/'audit.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps(reports,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for k in ['executable','asset','dynamic','body','output']:parser.add_argument('--'+k,type=Path,required=True)
    parser.add_argument('--source',dest='sources',type=Path,action='append',required=True)
    audit(**vars(parser.parse_args()))
