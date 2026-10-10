"""Check support samples against cross sections of the emitted cage mesh.

This is a sampled proxy audit, not continuous body collision certification.
The sewn transition (first two display rows) is reported separately by scope.
"""
import argparse,json,struct
from pathlib import Path
import numpy as np
ap=argparse.ArgumentParser(__doc__);ap.add_argument('replay',type=Path);a=ap.parse_args()
data=(a.replay/'candidate-0.bin').read_bytes();offset=0;results=[]
while offset<len(data):
    accepted=struct.unpack_from('I',data,offset)[0];length=struct.unpack_from('I',data,offset+16)[0];offset+=20+length
    p=np.frombuffer(data,dtype='<f4',count=9002*3,offset=offset).reshape(-1,3).astype(float);offset+=9002*12
    if not accepted:continue
    seam=p[:64];center=seam.mean(0);area=np.cross(seam-center,np.roll(seam,-1,axis=0)-center).sum(0);axis=area/np.linalg.norm(area)
    x=seam[32]-seam[0];x-=axis*np.dot(x,axis);x/=np.linalg.norm(x);y=np.cross(axis,x)
    local=(p-center)@np.array([x,y,axis]).T
    rings=local[:2560].reshape(40,64,3);pole=local[2560];heights=rings[:,0,2];worst=0.;outside=0;checked=0;worstRow=None;excludedBehind=0;excludedSewn=0
    for sample in local[4279:9001]:
        h=sample[2]
        if h<0:excludedBehind+=1;continue
        if h<heights[2]:excludedSewn+=1;continue
        if h>=pole[2]:checked+=1;outside+=1;worst=max(worst,float(h-pole[2]));continue
        row=min(39,np.searchsorted(heights,h,side='right')-1)
        low=rings[row];high=rings[row+1] if row<39 else np.tile(pole,(64,1))
        f=(h-low[:,2])/(high[:,2]-low[:,2]);polygon=low[:,:2]+(high[:,:2]-low[:,:2])*f[:,None]
        # Both diagonals of a grid quad lie within the cross-section chords;
        # this test uses the meridian polygon, an explicit approximation.
        q=np.roll(polygon,-1,axis=0);s=sample[:2]
        crossings=((polygon[:,1]>s[1])!=(q[:,1]>s[1])) & (s[0]<(q[:,0]-polygon[:,0])*(s[1]-polygon[:,1])/(q[:,1]-polygon[:,1]+1e-30)+polygon[:,0])
        inside=np.count_nonzero(crossings)%2==1;checked+=1
        if not inside:
            e=q-polygon;t=np.clip(((s-polygon)*e).sum(1)/np.maximum((e*e).sum(1),1e-30),0,1)
            distance=np.linalg.norm(polygon+e*t[:,None]-s,axis=1).min()
            if distance>worst:worst=float(distance);worstRow=int(row)
            outside+=1
    results.append(dict(checked=checked,outside=outside,maxOutsideDistance=worst,worstRow=worstRow,excludedBehindSeam=excludedBehind,excludedSewnTransition=excludedSewn))
proof=dict(scope='sampled proxy meridian polygons excluding sewn first two rows; not a continuous triangle certificate',poses=results,maxOutsideDistance=max(r['maxOutsideDistance'] for r in results),outside=sum(r['outside'] for r in results))
(a.replay/'sample-contact.json').write_text(json.dumps(proof,indent=2));print(json.dumps({k:v for k,v in proof.items() if k!='poses'}))
