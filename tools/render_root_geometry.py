"""Matched, untextured close-ups from exported triangles, without image edits."""
import argparse,inspect,sys
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw

def main():
    parser=argparse.ArgumentParser();parser.add_argument('before');parser.add_argument('after');parser.add_argument('output');parser.add_argument('--adapter',required=True);args=parser.parse_args()
    sys.path.insert(0,str(Path(args.adapter)/'tools'))
    import preview_collar_pairs as renderer
    exec(inspect.getsource(renderer.render).replace('elevation=np.deg2rad(7)','elevation=0'),renderer.__dict__)
    canvas=Image.new('RGB',(1200,1890),'#eef0f2');draw=ImageDraw.Draw(canvas)
    for col,prefix in enumerate([args.before,args.after]):
        p=np.fromfile(prefix+'-points.bin','<f8').reshape(-1,3);f=np.fromfile(prefix+'-faces.bin','<u4').reshape(-1,3)
        n=np.zeros_like(p);face=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
        for k in range(3):np.add.at(n,f[:,k],face)
        n=renderer.unit(n)
        for row,(az,label) in enumerate([(90,'True side'),(0,'Front'),(45,'Oblique')]):
            panel=renderer.render((p,f,n),az,size=600,span=28,center=[9,0,89])
            canvas.paste(panel,(600*col,630*row+30))
            draw.text((600*col+8,630*row+8),('Before' if col==0 else 'Candidate')+' | '+label+' | geometry only / offline',fill='#24313b')
    canvas.save(args.output)

if __name__=='__main__':main()
