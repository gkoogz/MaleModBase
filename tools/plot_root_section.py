"""Intersect real exported triangles with the sagittal plane (no shading)."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

def segments(points,faces):
    result=[];owners=[]
    for i,face in enumerate(faces):
        tri=points[face];hits=[]
        for k in range(3):
            a,b=tri[k],tri[(k+1)%3]
            if (a[1]<0)!=(b[1]<0):
                t=-a[1]/(b[1]-a[1]);hits.append((a+(b-a)*t)[[0,2]])
        if len(hits)==2:result.append(hits);owners.append(i)
    return np.array(result),np.array(owners)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);parser.add_argument('body',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    stock=np.load(args.body);original,_=segments(stock['points'],stock['faces'])
    fig,axs=plt.subplots(1,3,figsize=(15,7),layout='constrained')
    for ax,stage,angle in zip(axs,[609,600,612],[15,50,90]):
        prefix=args.directory/str(stage);p=np.fromfile(str(prefix)+'-points.bin','<f8').reshape(-1,3);f=np.fromfile(str(prefix)+'-faces.bin','<u4').reshape(-1,3);domains=np.fromfile(str(prefix)+'-face-domains.bin','<u4');lines,ids=segments(p,f)
        ax.add_collection(LineCollection(original,colors='#a2a6ac',linestyles='dashed',linewidths=1,label='Original pelvis'))
        for domain,color,label in [(0,'#286ca2','Retained pelvis'),(2,'#1b9c7f','Connecting ramp'),(1,'#454545','Retained module')]:
            ax.add_collection(LineCollection(lines[domains[ids]==domain],colors=color,linewidths=2,label=label))
        meta=json.loads(Path(str(prefix)+'-mesh.json').read_text());anchor=meta['anchor'];ax.set(xlim=(-3,23),ylim=(anchor[2]-16,anchor[2]+16),title=f'Angle slider {angle} | exact triangle section',xlabel='Forward (source units)',ylabel='Height (source units)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
    fig.savefig(args.output,dpi=170)

if __name__=='__main__':main()
