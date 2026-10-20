"""Compare sampled and parent-preserving recruitment on captured geometry."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.ndimage import map_coordinates
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_mount import measure_chart

def analyze(root):
    m=np.load(root/'trumpet-22-ventral-ramp/body-union.npz');p=m['points'];f=m['faces']
    donors=sparse.load_npz(root/'trumpet-22-ventral-ramp/body-union-donors.npz')[:,:5219]
    stock=np.load(root/'stock-body-lineage.npz');origin=np.array([6.45,0,84.3])
    height,_=measure_chart(stock['points'],stock['faces'],origin,np.eye(3))
    outer=np.load(root/'trumpet-22-ventral-ramp/pelvic-ramp.npz')['rings'][0]
    q=np.fromfile(root/'runtime-03/matrix-04/s0-o100-w100-a15-source.bin','<f8').reshape(-1,3)
    section=p[outer]-origin;theta=np.arctan2(section[:,2],section[:,1]);order=np.argsort(theta);theta=theta[order]
    initial=np.linalg.norm(section[:,1:],axis=1)[order];target=np.linalg.norm((q[outer]-origin)[:,1:],axis=1)[order]
    def H(y,z):return map_coordinates(height,np.array([z/.5+80,y/.5+80]),order=1,mode='nearest')
    def warp(points,depth,support_width):
        z=points-origin;r=np.linalg.norm(z[:,1:],axis=1);angle=np.arctan2(z[:,2],z[:,1])
        r0=np.interp(angle,theta,initial,period=2*np.pi);rt=np.interp(angle,theta,target,period=2*np.pi)
        support=4*np.maximum(r0,rt) if support_width==0 else np.maximum(3*r0,rt+support_width)
        t=np.clip((r-r0)/(support-r0),0,1);d=np.minimum(1,1.5*(support-rt)/(support-r0))*(support-r0)
        change=(2*t**3-3*t*t+1)*rt+(t**3-2*t*t+t)*d+(-2*t**3+3*t*t)*support+(t**3-t*t)*d
        change=np.where(r<r0,r*rt/r0,np.where(r>=support,r,change))
        before=H(z[:,1],z[:,2]);w=np.clip(1-np.maximum(before-z[:,0],0)/depth,0,1);w=w**3*(10-15*w+6*w*w)
        radial=1+(change/np.maximum(r,1e-9)-1)*w;y=z[:,1]*radial;zz=z[:,2]*radial
        return origin+np.c_[z[:,0]+H(y,zz)-before,y,zz]
    def normals(x):return np.cross(x[f[:,1]]-x[f[:,0]],x[f[:,2]]-x[f[:,0]])
    before=normals(p);body=np.asarray(donors.sum(1)).ravel()>1-1e-9;body_faces=(m['source_faces']>=0)&(m['source_faces']<9748)
    report=[]
    for depth in [.75,3,6,12]:
        for support in [0,5,10]:
            current=q.copy();current[body]=p[body]+donors[body]@(warp(stock['points'],depth,support)-stock['points'])
            after=normals(current);bad=body_faces&((before*after).sum(1)<0)
            report.append(dict(depth=depth,supportWidth=support,reversedFaces=int(bad.sum()),reversedParents=np.unique(m['source_faces'][bad]).tolist(),maximumBoundaryDeviation=float(np.linalg.norm(current[outer]-q[outer],axis=1).max())))
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);analyze(parser.parse_args().root)
