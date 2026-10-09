"""Offline resolution experiment using an adapter-supplied inspection export."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.meridian_surface import cloth_surface


def read_obj(path):
    vertices=[];groups=[];faces=[]
    for line in path.read_text().splitlines():
        s=line.split()
        if not s:continue
        if s[0]=='o' and faces:groups.append(np.array(faces));faces=[]
        elif s[0]=='v':vertices.append(list(map(float,s[1:])))
        elif s[0]=='f':faces.append([int(i.split('/')[0])-1 for i in s[1:]])
    if faces:groups.append(np.array(faces))
    vertices=np.array(vertices)
    return vertices,groups


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--inspection',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    j=json.loads((args.inspection/'collision-model.json').read_text());g=j['tautGuides']
    vertices,groups=read_obj(args.inspection/'collision-model.obj')
    meshes=[(vertices[np.unique(f)],np.searchsorted(np.unique(f),f)) for f in groups]
    reference,_=read_obj(args.inspection/'white-cloth.obj');tree=cKDTree(reference)
    results=[]
    for rows,subdivisions in [(16,1),(24,1),(16,2),(24,2),(32,2),(16,4)]:
        began=time.perf_counter()
        try:
            p,f,proof=cloth_surface(g['paths'],g['orangeOutline'],g['adaptiveDensity']['originFractions'],
                j['dome']['apex'],g['meridians']['polarAxis'],j['normals'][-1],meshes,rows,subdivisions,end_density=True)
            distance=tree.query(p)[0]
            proof.update(authoringSeconds=time.perf_counter()-began,
                nearestReferenceVertexP95=float(np.quantile(distance,.95)),
                nearestReferenceVertexMaximum=float(distance.max()),
                distanceMeaning='Conservative vertex-sample distance; not exact Hausdorff or visual acceptance')
            name=f'cloth-{rows}x{40*subdivisions}'
            np.savez(args.output/(name+'.npz'),points=p,faces=f)
            with (args.output/(name+'.obj')).open('w') as out:
                for q in p:out.write('v '+' '.join(map(str,q))+'\n')
                for face in f:out.write('f '+' '.join(str(int(i)+1) for i in face)+'\n')
            proof['asset']=name
        except ValueError as error:proof=dict(rows=rows,columns=40*subdivisions,rejected=str(error))
        results.append(proof);print(json.dumps(proof),flush=True)
    (args.output/'lod-comparison.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
