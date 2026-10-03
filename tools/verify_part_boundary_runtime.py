"""Independent SDK-free Dirichlet seam check against the Python collar solve."""
import argparse, json, struct, subprocess, hashlib
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from malemod_base.collar import CollarFrame,CollarPlan


def verify(executable,output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'build') or output.exists():raise ValueError('Use a new owned verification directory')
    output.mkdir(parents=True)
    p=np.array([[7+y,x-2,83+y*.2] for y in range(5) for x in range(5)],float)
    faces=[]
    for y in range(4):
        for x in range(4):
            a=y*5+x;faces.extend([[a,a+1,a+5],[a+1,a+6,a+5]])
    faces=np.asarray(faces,dtype='<u4');protected=np.array([0,4,20,24],dtype='<u4');prescribed=np.arange(10,15,dtype='<u4')
    frame=CollarFrame((5,0,83),(1,0,0),(0,0,1),6,25,1)
    reports=[]
    for phase in [0,.25,1]:
        delta=np.zeros_like(p);delta[prescribed]=[.4*phase,.1*phase,.3*phase]
        plan=CollarPlan(p,faces,[],frame,np.r_[protected,prescribed]);expected=plan.solve_displacement(delta)
        fixture=output/f'phase-{phase}.bin';result=output/f'phase-{phase}.xyz'
        with fixture.open('wb') as f:
            f.write(b'GRAFT002'+struct.pack('<4I',len(p),len(faces),0,len(protected)))
            f.write(p.astype('<f8').tobytes()+faces.tobytes()+protected.tobytes())
            f.write(struct.pack('<I',len(prescribed))+prescribed.tobytes())
            f.write(np.asarray([*frame.root,*frame.axis,*frame.up,frame.radius,frame.length,frame.source_length_scale],dtype='<f8').tobytes())
            f.write(delta.astype('<f8').tobytes())
        process=subprocess.run([str(executable),str(fixture),str(result)],check=True,text=True,capture_output=True)
        actual=np.fromfile(result,dtype='<f8').reshape(-1,3);error=float(np.abs(actual-expected).max())
        if error>1e-10:raise ValueError('Prescribed boundary solve differs from independent Python solve')
        np.testing.assert_array_equal(actual[prescribed],delta[prescribed]);np.testing.assert_array_equal(actual[protected],delta[protected])
        if phase and not np.any(np.abs(actual[np.arange(5,10)])>0):raise ValueError('Movable boundary does not recruit its neighbors')
        reports.append(dict(phase=phase,maximumError=error,prescribedExact=True,protectedExact=True))
    report=dict(offlineOnly=True,observedGameplay=False,cases=reports,executableSHA256=hashlib.sha256(executable.read_bytes()).hexdigest())
    (output/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();verify(a.executable.resolve(),a.output)
