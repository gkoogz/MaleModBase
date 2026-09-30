"""Geometry regression over the same 88 static, moving and pulsing captures."""
from pathlib import Path
import json,ctypes
import numpy as np
from scipy.spatial import cKDTree
from compact_runtime import Header,HERE,ROOT,capture

raster=ctypes.CDLL(str(HERE/'raster.dll')).raster
raster.argtypes=[ctypes.c_int]+[ctypes.c_void_p]*5+[ctypes.c_int]*2+[ctypes.c_void_p]*2
# Normal direction is ill-conditioned on nearly collinear return-strip faces.
# Keep their sign changes visible in the report; reject every sign change with
# meaningful area before OR after reduction (.005 square model units is less
# than a pixel in the review figures).
NEAR_DEGENERATE_AREA=.005

def points(raw,n,off):return np.ndarray((n,3),np.float32,buffer=raw,offset=off,strides=(32,4)).astype(float)
def mask(p,f,axes,lo,scale):
    xy=(p@axes-lo)*scale+4
    # Pixel-center barycentric coverage, matching the review renderer. Integer
    # polygon filling artificially widens the more densely triangulated mesh.
    depth=np.full((768,768),-1e5,np.float32);attrs=np.zeros((768,768,3),np.float32)
    arrays=[np.ascontiguousarray(f,np.int32),np.ascontiguousarray(xy[:,0],np.float32),np.ascontiguousarray(xy[:,1],np.float32),np.zeros(len(p),np.float32),np.zeros((len(p),3),np.float32)]
    raster(len(f),*[a.ctypes.data for a in arrays],768,768,depth.ctypes.data,attrs.ctypes.data)
    return depth>-1e4

def distance_to_mesh(p,q,f):
    # Exact distance to a shortlist of 16 triangles near each sample.
    tri=q[f];tree=cKDTree(tri.mean(axis=1));_,ids=tree.query(p,k=min(16,len(f)))
    t=tri[ids];a=t[:,:,0];b=t[:,:,1];c=t[:,:,2];v=p[:,None]-a
    e=b-a;g=c-a;n=np.cross(e,g);nn=np.sum(n*n,axis=2)
    projected=v-n*(np.sum(v*n,axis=2)/np.maximum(nn,1e-30))[:,:,None]
    ee=np.sum(e*e,axis=2);gg=np.sum(g*g,axis=2);eg=np.sum(e*g,axis=2)
    ve=np.sum(projected*e,axis=2);vg=np.sum(projected*g,axis=2);den=ee*gg-eg*eg
    u=(gg*ve-eg*vg)/np.maximum(den,1e-30);w=(ee*vg-eg*ve)/np.maximum(den,1e-30)
    plane=np.sum(v*n,axis=2)**2/np.maximum(nn,1e-30);plane[(u<0)|(w<0)|(u+w>1)]=np.inf
    best=plane
    for x,y in [(a,b),(b,c),(c,a)]:
        edge=y-x;d=p[:,None]-x;alpha=np.clip(np.sum(d*edge,axis=2)/np.maximum(np.sum(edge*edge,axis=2),1e-30),0,1)
        best=np.minimum(best,np.sum((d-edge*alpha[:,:,None])**2,axis=2))
    return np.sqrt(np.min(best,axis=1))

def main():
    report=json.loads((HERE/'compact-report.json').read_text());remap=np.load(HERE/'remaps.npz');candidate=np.load(HERE/'candidate.npz')
    old=Header('neck_render_data.h');oldf=old.a('nrIndices').reshape(-1,3)
    keep=remap['nrKeep'];mapping=np.full(30717,-1,int);mapping[keep]=np.arange(len(keep));newf=mapping[candidate['nr_faces']]
    axes=[np.array([[1,0],[0,1],[0,0]]),np.array([[1,0],[0,0],[0,1]]),np.array([[0,0],[1,0],[0,1]]),np.array([[.707,0],[.707,0],[0,1]]),np.array([[.707,0],[-.707,0],[0,1]])]
    results=[]
    for path in sorted((ROOT/'captures/surface-audit-compact').glob('*.bin')):
        raw=path.read_bytes();offset=report['ucNodes']*28;new=points(raw,report['nrVertices'],offset)
        oldraw=(ROOT/'captures/surface-audit-revised'/path.name).read_bytes();p,oldSupport=capture(ROOT/'captures/surface-audit-revised'/path.name)
        support=points(raw,report['rsVertices'],offset+report['nrVertices']*32)
        supportError=float(np.linalg.norm(support-oldSupport[remap['rsKeep']],axis=1).max())
        assert np.isfinite(new).all()
        assert raw[-168:]==oldraw[-168:],(path.name,'Physics trajectory changed')
        assert len(raw)==offset+report['nrVertices']*32+report['rsVertices']*32+10554*32+168
        uv=raw[offset:offset+report['nrVertices']*32];attrs=np.frombuffer(uv,np.uint8).reshape(-1,32)
        oldattrs=np.frombuffer(oldraw,np.uint8,30717*32,50059*28).reshape(-1,32)
        assert np.array_equal(attrs[:,20:],oldattrs[keep,20:]),(path.name,'UV or skin weights changed')
        # Count orientations relative to corresponding baseline triangle lineage.
        oldtri=p[oldf[candidate['nr_lineage']]];newtri=new[newf]
        a=np.cross(oldtri[:,1]-oldtri[:,0],oldtri[:,2]-oldtri[:,0]);b=np.cross(newtri[:,1]-newtri[:,0],newtri[:,2]-newtri[:,0])
        denom=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1);cos=np.sum(a*b,axis=1)/np.maximum(denom,1e-20)
        flipped=int(np.count_nonzero((denom>1e-10)&(cos<0)))
        reversedFaces=(denom>1e-10)&(cos<0)
        oldArea=np.linalg.norm(a,axis=1)*.5;newArea=np.linalg.norm(b,axis=1)*.5
        significant=int(np.count_nonzero(reversedFaces&((oldArea>=NEAR_DEGENERATE_AREA)|(newArea>=NEAR_DEGENERATE_AREA))))
        ious=[]
        for ax in axes:
            xy=np.r_[p@ax,new@ax];lo=xy.min(axis=0);scale=760/max(np.ptp(xy,axis=0))
            x=mask(p,oldf,ax,lo,scale);y=mask(new,newf,ax,lo,scale);ious.append(float(np.count_nonzero(x&y)/np.count_nonzero(x|y)))
        # Fixed sampling of baseline vertices and face centers, both directions.
        samples=np.r_[p[::11],p[oldf[::19]].mean(axis=1)];error=distance_to_mesh(samples,new,newf)
        back=distance_to_mesh(new[::7],p,oldf)
        row={'pose':path.name,'supportPositionError':supportError,'silhouetteIoU':ious,'surfaceErrorP99':float(np.quantile(error,.99)),'surfaceErrorMax':float(error.max()),'newSurfaceErrorP99':float(np.quantile(back,.99)),'flippedVsLineage':flipped,'significantReversals':significant,'maximumReversedFaceArea':float(newArea[reversedFaces].max(initial=0.)),'vertexDisplacementMax':float(np.linalg.norm(new-p[keep],axis=1).max())}
        results.append(row);print(path.name,'IoU',round(min(ious),5),'p99',round(row['surfaceErrorP99'],4),'flipped',flipped,flush=True)
    assert len(results)==88,len(results)
    final={'cases':len(results),'physicsBitIdentical':True,'uvAndWeightsBitIdentical':True,'maximumSupportingPositionError':max(r['supportPositionError'] for r in results),'minimumSilhouetteIoU':min(min(r['silhouetteIoU']) for r in results),'maximumSurfaceP99':max(r['surfaceErrorP99'] for r in results),'maximumNewSurfaceP99':max(r['newSurfaceErrorP99'] for r in results),'maximumFlippedVsLineage':max(r['flippedVsLineage'] for r in results),'maximumSignificantReversals':max(r['significantReversals'] for r in results),'maximumReversedFaceArea':max(r['maximumReversedFaceArea'] for r in results),'nearDegenerateAreaThreshold':NEAR_DEGENERATE_AREA,'poses':results}
    (HERE/'geometry-audit.json').write_text(json.dumps(final,indent=2));print(json.dumps({k:v for k,v in final.items() if k!='poses'},indent=2))
    # Reduction can add a slightly stricter local placement safety constraint.
    # Bound its effect to .05 model units; render silhouettes have a separate
    # >=99% test; non-negligible reversals are never permitted by this audit.
    assert final['maximumSupportingPositionError']<=.05
    assert final['minimumSilhouetteIoU']>=.99
    assert final['maximumSignificantReversals']==0

if __name__=='__main__':main()
