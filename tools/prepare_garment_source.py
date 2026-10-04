"""Prepare measured Wolverine garment donors and SDK-free fixture data.
No installation. Canonical source arrays remain read-only.
"""
import argparse,hashlib,json,re,struct
from pathlib import Path
import numpy as np
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from garment_source_routes import measure_wolverine_routes
from malemod_base.garment_regions import reference_anatomy_semantics,anatomy_regions,anatomy_regions_header,SEMANTIC_VERSION,full_source_root_boundary,ROOT_BOUNDARY_VERSION
def table(text,name):
 m=re.search(r'\b'+name+r'\s*\[[^]]*\]\s*=\s*\{(.*?)\};',text,re.S)
 if not m: raise ValueError(name)
 return np.array([int(v,0) for v in re.findall(r'0x[\da-fA-F]+|\d+',m[1])],dtype=np.uint32)
def blend(a,b,t):
 d={}
 for rows,w in ((a,1-t),(b,t)):
  for k,v in rows.items():d[k]=d.get(k,0)+v*w
 return {k:v for k,v in d.items() if v>1e-12}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--runtime',type=Path,required=True);ap.add_argument('--evaluated',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--recipe',type=Path);args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
 body=np.concatenate([np.fromfile(args.evaluated/'surface.xyz.body0','<f4').reshape(-1,3),np.fromfile(args.evaluated/'surface.xyz.body1','<f4').reshape(-1,3)])
 ids=np.concatenate([np.arange(17449,17449+15168),np.arange(41435,41435+5615)])
 text=(args.runtime/'menu_retarget_body_data.h').read_text();faces=np.concatenate([table(text,'menuRetargetBodyIndices0').reshape(-1,3),table(text,'menuRetargetBodyIndices1').reshape(-1,3)+15168])
 # Measured torso cross section below the arms, above the source graft. Both
 # resources are intersected, retaining exact original-edge sparse donors.
 height=94.75;sections=[]
 for tri in faces:
  hits=[]
  for a,b in zip(tri,np.roll(tri,-1)):
   pa,pb=body[a],body[b]
   if (pa[2]-height)*(pb[2]-height)<0:
    t=float((height-pa[2])/(pb[2]-pa[2]));p=pa*(1-t)+pb*t
    if np.linalg.norm(p[:2])<25:hits.append((p,{int(ids[a]):1-t,int(ids[b]):t}))
  if len(hits)==2:sections.append(hits)
 waist=[]
 for k in range(48):
  theta=k*2*np.pi/48;direction=np.array([np.sin(theta),-np.cos(theta)]);best=None
  for (a,da),(b,db) in sections:
   edge=b[:2]-a[:2];matrix=np.column_stack([direction,-edge])
   if abs(np.linalg.det(matrix))<1e-8:continue
   r,t=np.linalg.solve(matrix,a[:2])
   if r>0 and -1e-6<=t<=1+1e-6 and (best is None or r>best[0]):best=(r,blend(da,db,float(np.clip(t,0,1))))
  if best is None:raise ValueError('waist contour ray missing '+str(k))
  waist.append(best[1])
 z=np.load(ROOT/'assets/wolverine-reference/geometry.npz');anatomy=np.fromfile(args.evaluated/'surface.xyz','<f4').reshape(-1,3)
 native_triangles=np.fromfile(args.evaluated/'surface.xyz.indices','<u2').reshape(-1,3);opening_ids=full_source_root_boundary(anatomy,native_triangles).tolist()
 # Calibrated same-side lateral hip/under-glute paths follow actual body edges.
 straps,routeProof=measure_wolverine_routes(body,ids,faces,waist)
 if args.recipe:
  saved=json.loads(args.recipe.read_text());
  if saved.get('routeRecipeVersion')!=3 or saved.get('rootBoundaryVersion')!=ROOT_BOUNDARY_VERSION or not saved.get('measuredStrapRoutes'):raise ValueError('Obsolete route or coarse root recipe; regenerate the measured complete opening')
  waist=[{int(k):v for k,v in row.items()} for row in saved['waist']];saved_opening=[int(next(iter(row))) for row in saved['opening']];
  if saved_opening!=opening_ids:raise ValueError('Measured complete root opening differs from recipe')
  straps=[[{int(k):v for k,v in row.items()} for row in path] for path in saved['rearStraps']];routeProof=saved.get('measuredStrapRoutes',[])
 recipe={'schema':3,'rootBoundaryVersion':ROOT_BOUNDARY_VERSION,'rootBoundaryVertices':len(opening_ids),'routeRecipeVersion':3,'sourceCoordinates':{'forward':'+X','lateral':'-Y','up':'+Z','unit':'uncalibrated source model unit'},'waistPlane':height,'waist':waist,'opening':[{i:1.} for i in opening_ids],'rearStraps':straps,'measuredStrapRoutes':routeProof,'sourceHashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.runtime/'menu_retarget_body_data.h',args.runtime/'neck_render_data.h',Path(__file__),Path(__file__).with_name('garment_source_routes.py'),ROOT/'assets/wolverine-reference/geometry.npz']}}
 regions=anatomy_regions(reference_anatomy_semantics(z))
 recipe['sourceHashes']['garment_routes.py']=hashlib.sha256((ROOT/'malemod_base/garment_routes.py').read_bytes()).hexdigest()
 recipe['sourceHashes']['garment_regions.py']=hashlib.sha256((ROOT/'malemod_base/garment_regions.py').read_bytes()).hexdigest()
 recipe['anatomySemantics']=dict(version=SEMANTIC_VERSION,groupCounts={k:len(v) for k,v in regions.items()},fullAnatomyVertices=len(anatomy),nativeDrawIndicesChanged=False)
 (args.out/'wolverine-jockstrap-donors.json').write_text(json.dumps(recipe,indent=2)+'\n',newline='\n')
 # Flat arrays permit the canonical runtime to consume donor recipes without
 # JSON/Python or copying shared garment implementation into the adapter.
 lines=['#pragma once','// Measured original-edge/source-vertex donors; generated by Base tools/prepare_garment_source.py.','struct JockstrapMeasuredDonor {unsigned vertex; double weight;};','struct JockstrapMeasuredSample {JockstrapMeasuredDonor donors[4];};']
 for name,rows in [('waist',waist),('opening',recipe['opening']),('strapLeft',straps[0]),('strapRight',straps[1])]:
  lines.append('static const JockstrapMeasuredSample jockstrap_'+name+'[]={')
  for row in rows:
   entries=list(row.items())+[(0,0)]*(4-len(row));lines.append(' {{{'+ '},{'.join(str(k)+','+format(v,'.17g') for k,v in entries)+'}}},')
  lines.append('};')
 (args.out/'jockstrap_measured_data.h').write_text('\n'.join(lines)+'\n',newline='\n')
 (args.out/'jockstrap_regions_data.h').write_text(anatomy_regions_header(regions),newline='\n')
 (args.out/'anatomy-regions.json').write_text(json.dumps(dict(semanticVersion=SEMANTIC_VERSION,fullAnatomyVertices=len(anatomy),fullAnatomyTriangles=len(np.fromfile(args.evaluated/'surface.xyz.indices','<u2'))//3,groups={k:v.tolist() for k,v in regions.items()},sourceAssetSHA256=hashlib.sha256((ROOT/'assets/wolverine-reference/geometry.npz').read_bytes()).hexdigest(),exporterSHA256=hashlib.sha256((ROOT/'malemod_base/garment_regions.py').read_bytes()).hexdigest(),nativeDrawIndicesChanged=False),indent=2)+'\n',newline='\n')
 # Portable test fixture protocol: counts + fully measured sample positions,
 # normals and 4 lineage donors, followed by uint32 triangles. No game layouts.
 def sample(p,normal,rows,surface):
  return struct.pack('<6d',*p,*normal)+b''.join(struct.pack('<IId',surface,k,v) for k,v in list(rows.items())+[(0,0)]*(4-len(rows)))
 def body_rows(rows):
  lookup={int(k):i for i,k in enumerate(ids)};out=[]
  for row in rows:
   p=sum(body[lookup[k]].astype(float)*v for k,v in row.items());n=np.array([p[0],p[1],0]);n/=np.linalg.norm(n);out.append(sample(p,n,row,0))
  return out
 tri=np.fromfile(args.evaluated/'surface.xyz.indices','<u2').astype('<u4').reshape(-1,3)
 normals=np.fromfile(args.evaluated/'surface.xyz.normals','<f4').reshape(-1,3)
 data=b'JGFX0001'+struct.pack('<6I',len(waist),len(opening_ids),len(anatomy),len(straps[0]),len(straps[1]),len(tri))
 data+=b''.join(body_rows(waist))+b''.join(sample(anatomy[i],normals[i],{i:1},1) for i in opening_ids)+b''.join(sample(p,normals[i],{i:1},1) for i,p in enumerate(anatomy))+b''.join(body_rows(straps[0]))+b''.join(body_rows(straps[1]))+tri.tobytes()
 (args.out/'source-garment.fixture').write_bytes(data)
 print(json.dumps({'waistSamples':len(waist),'opening':opening_ids,'straps':straps,'anatomyVertices':len(anatomy),'triangles':len(tri),'fixtureBytes':len(data)}))
if __name__=='__main__':main()
