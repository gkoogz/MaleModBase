"""Offline behavioral gate for the intentional stable pelvic-frame revision.

The old executable is evidence of prior behavior, never an updated oracle.
Each process contains exactly one source session. No game SDK or installs.
"""
import argparse, hashlib, json, subprocess
from pathlib import Path
import numpy as np

ORDER=[0,1,2,3,5,7,8,9,4,6,10,11,12,13,14,15,16,17]

def main():
    p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,required=True);p.add_argument('--previous-runtime',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--analyze-only',action='store_true');p.add_argument('--reuse-previous',action='store_true')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cases=[('default',[2]+[50]*17,120)]
    for state in range(3):
        for kind in ['requested','combined']:
            for angle in [1,10,50,64,100]:
                c=[state]+[50]*17
                if kind=='requested':c[1:8]=[100,75,100,52,100,95,angle]
                else:c[1:8]=[100,100,100,100,100,100,angle]
                cases.append((f'{kind}-s{state}-a{angle}',c,120))
    report={'observedGameplay':False,'algorithmRevision':2,'cases':[],'runtimeSHA256':hashlib.sha256(args.runtime.read_bytes()).hexdigest(),'previousRuntimeSHA256':hashlib.sha256(args.previous_runtime.read_bytes()).hexdigest()}
    outputs={}
    for label,c,steps in cases:
        prefs=args.output/(label+'.txt');prefs.write_text(' '.join(str(c[i]) for i in ORDER))
        pair=[]
        for name,exe in [('current',args.runtime),('previous',args.previous_runtime)]:
            prefix=args.output/(label+'.'+name)
            if not args.analyze_only and not(args.reuse_previous and name=='previous' and prefix.is_file()):subprocess.run([str(exe.resolve()),str(prefs.resolve()),str(prefix.resolve()),str(steps)],check=True,stdout=subprocess.DEVNULL)
            pos=np.fromfile(prefix,dtype='<f4').reshape(-1,3)
            bodies=[np.fromfile(str(prefix)+f'.body{i}',dtype='<f4').reshape(-1,3) for i in range(2)]
            for surface in [pos]+bodies:assert np.isfinite(surface).all(),(label,name,'nonfinite surface')
            pair.append((pos,bodies,json.loads(Path(str(prefix)+'.mechanics.json').read_text())))
        new,old=pair;metric=new[2]['collarMetric'];assert np.allclose(metric['axis'][0],[np.cos(np.pi/6),0,-.5],atol=2e-7),label
        assert np.allclose(metric['up'][0],[.5,0,np.cos(np.pi/6)],atol=2e-7),label
        assert np.array_equal(np.fromfile(str(args.output/(label+'.current'))+'.indices',dtype='<u2'),np.fromfile(str(args.output/(label+'.previous'))+'.indices',dtype='<u2'))
        assert np.array_equal(np.fromfile(str(args.output/(label+'.current'))+'.uv',dtype='<f4'),np.fromfile(str(args.output/(label+'.previous'))+'.uv',dtype='<f4'))
        for field in ['restLength','proximalRadius','shaftGuide','restGuide','lobeCenters','lobeAnchors','lobeRadii','lobeAxes','rootDirection','bendMultipliers']:
            assert new[2][field]==old[2][field],(label,'mechanical state changed',field)
        item={'label':label,'controls':c,'steps':steps,'stableAxis':metric['axis'][0],'stableUp':metric['up'][0],'liveRootDirection':new[2]['rootDirection'][0],
              'oldToNewAnatomyMaximum':float(np.linalg.norm(new[0]-old[0],axis=1).max()),'oldToNewBodyMaximum':[float(np.linalg.norm(a-b,axis=1).max()) for a,b in zip(new[1],old[1])]}
        report['cases'].append(item);outputs[label]=pair
        print(label,item['oldToNewBodyMaximum'],flush=True)
    # Measure stability over RestAngle commands separately from intended source
    # morphology dependence; never demand that body positions be byte-identical.
    report['angleSweeps']=[]
    for state in range(3):
        for kind in ['requested','combined']:
            low,high=[outputs[f'{kind}-s{state}-a{a}'] for a in [1,100]]
            report['angleSweeps'].append({'state':state,'kind':kind,
                'currentBodyMaximum':[float(np.linalg.norm(a-b,axis=1).max()) for a,b in zip(low[0][1],high[0][1])],
                'previousBodyMaximum':[float(np.linalg.norm(a-b,axis=1).max()) for a,b in zip(low[1][1],high[1][1])],
                'liveRootAngleDegrees':float(np.degrees(np.arccos(np.clip(np.dot(low[0][2]['rootDirection'][0],high[0][2]['rootDirection'][0]),-1,1))))})
    data=np.load(Path(__file__).resolve().parents[1]/'assets/wolverine-reference/geometry.npz')
    keep=data['unified_collar_data__ucKeep'].astype(int)
    faces=data['unified_collar_data__ucFaces'].astype(int).reshape(-1,3)
    seam=data['unified_collar_data__ucSeamVertices'].astype(int).reshape(-1,3);weights=data['unified_collar_data__ucSeamWeights']
    def unique(out):return np.vstack([out[0]]+out[1])[keep].astype(float)
    def areas(pos):
        tri=pos[faces];return np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
    baseline=areas(unique(outputs['default'][0]));eligible=baseline>1e-5
    bodyEligible=eligible&(keep[faces]>=17528).all(axis=1)
    report['topologyGeometry']=[]
    for label,pair in outputs.items():
        pos=unique(pair[0]);area=areas(pos)
        residual=pos[seam[:,0]]-(pos[seam[:,1]]*(1-weights[:,None])+pos[seam[:,2]]*weights[:,None])
        oldpos=unique(pair[1]);oldres=oldpos[seam[:,0]]-(oldpos[seam[:,1]]*(1-weights[:,None])+oldpos[seam[:,2]]*weights[:,None])
        item={'label':label,'minimumAreaRatioToDefault':float(np.min(area[eligible]/baseline[eligible])),
              'bodyMinimumAreaRatioToDefault':float(np.min(area[bodyEligible]/baseline[bodyEligible])),
              'previousMinimumAreaRatioToDefault':float(np.min(areas(oldpos)[eligible]/baseline[eligible])),
              'newCollapsedNondegenerateFaces':int(np.count_nonzero(area[eligible]<1e-8)),
              'originalEdgeSeamMaximum':float(np.linalg.norm(residual,axis=1).max()),
              'previousOriginalEdgeSeamMaximum':float(np.linalg.norm(oldres,axis=1).max())}
        assert item['newCollapsedNondegenerateFaces']==0,(label,'face collapse')
        # Original exporter/postpasses include some preexisting rendered alias
        # residuals. This revision must not worsen them; solver elimination is
        # unchanged. These are recorded explicitly rather than called exact.
        assert item['originalEdgeSeamMaximum']<item['previousOriginalEdgeSeamMaximum']+2e-5,(label,'original edge seam regression')
        report['topologyGeometry'].append(item)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['angleSweeps'],indent=2))
if __name__=='__main__':main()
