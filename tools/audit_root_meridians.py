"""Measure geometry at an exported root annulus; no material/shading inputs."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def audit(prefix, output):
    meta=json.loads(Path(str(prefix)+'-mesh.json').read_text())
    p=np.fromfile(str(prefix)+'-points.bin','<f8').reshape(-1,3)
    rings=np.fromfile(str(prefix)+'-rings.bin','<u4').reshape(meta['rows'],meta['knots'])
    faces=np.fromfile(str(prefix)+'-faces.bin','<u4').reshape(-1,3)
    tangents=np.fromfile(str(prefix)+'-root-tangents.bin','<f8').reshape(-1,3)
    origin=np.array(meta['anchor']);r=p[rings];angle=np.arctan2(r[0,:,2]-origin[2],r[0,:,1]-origin[1])
    unit=lambda x:x/np.maximum(np.linalg.norm(x,axis=-1,keepdims=True),1e-30)
    steps=np.diff(r,axis=0);turn=np.degrees(np.arccos(np.clip((unit(steps[:-1])*unit(steps[1:])).sum(-1),-1,1)))
    edge={}
    for f in faces[:-2*(len(rings)-1)*len(rings[0])]:
        for j in range(3):edge[tuple(sorted((int(f[j]),int(f[(j+1)%3]))))]=f
    geometric=[];first_strip=[]
    for i,a in enumerate(rings[0]):
        b=rings[0,(i+1)%len(rings[0])];f=edge[tuple(sorted((int(a),int(b))))]
        n=unit(np.cross(p[f[1]]-p[f[0]],p[f[2]]-p[f[0]]))
        geometric.append(float(np.degrees(np.arcsin(np.clip(abs(unit(tangents[i])@n),0,1)))))
        first_strip.append(float(np.degrees(np.arcsin(np.clip(abs(unit(steps[0,i])@n),0,1)))))
    fig,axs=plt.subplots(2,2,figsize=(12,9),layout='constrained');rows=[]
    for ax,deg in zip(axs.flat,[90,0,-90,180]):
        k=int(np.argmin(abs(np.angle(np.exp(1j*(angle-np.deg2rad(deg)))))))
        radial=unit(np.array([0,r[0,k,1]-origin[1],r[0,k,2]-origin[2]]))
        # Forward vs radial coordinate: directly exposes a rolled lip/shoulder.
        ax.plot((r[:,k]-origin)@radial,r[:,k,0],'-o',label='actual meridian')
        a=r[0,k];t=unit(tangents[k]);q=np.stack([a,a+t])
        ax.plot((q-origin)@radial,q[:,0],'--',label='stored outer tangent')
        ax.set(title=f'Arc {deg} degrees; source column {k}',xlabel='radial distance',ylabel='forward position');ax.axis('equal');ax.grid();ax.legend()
        rows.append(dict(arc=deg,column=k,points=r[:,k].tolist(),turn=turn[:,k].tolist(),bodyTangentMismatch=geometric[k]))
    fig.savefig(str(output)+'.png',dpi=150);plt.close(fig)
    stats=lambda v:dict(zip(['median','p90','p99','max'],map(float,np.percentile(v,[50,90,99,100]))))
    result=dict(bodyGeometricTangentMismatchDegrees=stats(geometric),firstStripMismatchDegrees=stats(first_strip),rowTurnDegrees=[stats(v) for v in turn],meridians=rows)
    Path(str(output)+'.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='meridians'},indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('prefix',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args();audit(args.prefix,args.output)
