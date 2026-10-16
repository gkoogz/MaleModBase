"""Render measured dynamic-mount exports with matched orthographic cameras."""
import argparse,inspect,json,sys
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_anatomy import vertex_normals

def render(directory,adapter,calibration):
    sys.path.insert(0,str(adapter/'tools'))
    import preview_collar_pairs as renderer
    exec(inspect.getsource(renderer.render).replace('elevation=np.deg2rad(7)','elevation=0'),renderer.__dict__)
    cal=np.load(calibration);reports=json.loads((directory/'audit.json').read_text());size=540
    canvas=Image.new('RGB',(size*2,(size+30)*((len(reports)+1)//2)),'#eef0f2');draw=ImageDraw.Draw(canvas);font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',17)
    for i,row in enumerate(reports):
        label=row['label'];p=np.fromfile(directory/(label+'-points.bin'),'<f8').reshape(-1,3);f=np.fromfile(directory/(label+'-faces.bin'),'<u4').reshape(-1,3)
        p=(p-cal['sourceRoot'])@cal['basis'].T*float(cal['scale'][0])+cal['targetRoot'];normals=vertex_normals(SimpleNamespace(points=p,faces=f))
        center=(np.array([24,0,85])-cal['sourceRoot'])@cal['basis'].T*float(cal['scale'][0])+cal['targetRoot']
        image=renderer.render((p,f,normals),0,size=size,span=.78,center=center);x=i%2*size;y=i//2*(size+30)
        canvas.paste(image,(x,y+30));draw.text((x+8,y+4),label+' | true side / OFFLINE',font=font,fill='#24313b')
    canvas.save(directory/'side-review.png')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['directory','adapter','calibration']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();render(a.directory,a.adapter,a.calibration)
