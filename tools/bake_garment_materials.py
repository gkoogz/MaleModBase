"""Bake deterministic garment material maps without generated images.
The analytical rib/weave fields are shared with garments::Knit.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from PIL import Image

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--size',type=int,default=512);a=p.parse_args()
 if a.size<256 or a.size>2048 or a.size&(a.size-1):raise ValueError('size must be a power of two in [256,2048]')
 a.out.mkdir(parents=True,exist_ok=True);y,x=np.mgrid[0:a.size,0:a.size]/a.size
 detail=.018*np.cos(x*2*np.pi*128)+.004*np.cos(y*2*np.pi*256);rgb=np.clip(np.array([.94,.94,.925])+detail[...,None],0,1)
 # Fine ribs are surface detail on thin cloth, not swollen displacement.
 slope_x=.16*np.sin(x*2*np.pi*128);slope_y=.035*np.sin(y*2*np.pi*256);normal=np.stack([slope_x,slope_y,np.ones_like(x)],axis=-1);normal/=np.linalg.norm(normal,axis=-1)[...,None]
 Image.fromarray(np.uint8(rgb*255+.5),'RGB').save(a.out/'white-ribbed-color.png');Image.fromarray(np.uint8((normal*.5+.5)*255+.5),'RGB').save(a.out/'white-ribbed-normal.png');Image.fromarray(np.full((a.size,a.size),round(.88*255),dtype=np.uint8),'L').save(a.out/'white-ribbed-roughness.png')
 rows=[]
 for slot,color in enumerate([[.94,.94,.925],[.96,.96,.94],[.62,.035,.045],[.035,.09,.45]]):rows+=['newmtl garment'+str(slot),'Kd '+' '.join(map(str,color)),'Ka .15 .15 .15','Ks .02 .02 .02','Ns 4','d 1'];rows+=['map_Kd white-ribbed-color.png'] if slot==0 else []
 (a.out/'jockstrap.mtl').write_text('\n'.join(rows)+'\n');files=sorted(a.out.glob('*'));manifest={'schema':1,'method':'deterministic analytical knit and weave; no image generator','colorSpace':'color sRGB; normals and roughness linear','normalConvention':'tangent +Y; adapt convention in spoke','materialSlots':['WhiteRibbed','WhiteElastic','RedStripe','BlueStripe'],'files':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file() and f.name!='materials.json'}};(a.out/'materials.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest))
if __name__=='__main__':main()
