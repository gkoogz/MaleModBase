"""Draw the actual wire6 solver partition over its source surface (offline)."""
import argparse,struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

def read(path):
    raw=Path(path).read_bytes();offset=0
    def values(dtype,count):
        nonlocal offset
        a=np.frombuffer(raw,dtype,count,offset).copy();offset+=a.nbytes;return a
    def integer():return int(values('<u4',1)[0])
    if integer()!=6:raise ValueError('Expected wire6 source output')
    surfaces=[]
    for _ in range(3):
        n=integer();surfaces.append(values('<f4',n*3).reshape(-1,3));values('<f4',n*9)
    triangles=values('<u4',integer()).reshape(-1,3)
    radius,length=values('<f4',2);rod=values('<f4',36).reshape(12,3);rest=values('<f4',36).reshape(12,3)
    centers=values('<f4',6).reshape(2,3);anchors=values('<f4',6).reshape(2,3);radii=values('<f4',6).reshape(2,3);axes=values('<f4',18).reshape(2,3,3)
    return surfaces[0],triangles,rod,centers,anchors,radii,axes

def render(source,output):
    skin,tri,rod,centers,anchors,radii,axes=read(source)
    fig,plots=plt.subplots(1,2,figsize=(12,7),layout='constrained');fig.patch.set_facecolor('#f3f5f7')
    fig.get_layout_engine().set(rect=(0,.06,1,.94))
    for ax,projection,title in zip(plots,[(0,2),(1,2)],['True side: hidden rod continues through attachment','Front: two separate suspended contents']):
        ax.set_facecolor('#f3f5f7');ax.add_collection(PolyCollection(skin[tri][:,:,projection],facecolor='#99a0a5',edgecolors='none',alpha=.035,rasterized=True))
        for side in range(2):
            for latitude in np.linspace(-1.3,1.3,7):
                t=np.linspace(0,2*np.pi,90);local=np.stack([np.cos(t)*np.cos(latitude),np.sin(t)*np.cos(latitude),np.full_like(t,np.sin(latitude))],axis=1)*radii[side]
                pts=centers[side]+local@axes[side];ax.plot(pts[:,projection[0]],pts[:,projection[1]],color='#23865c',alpha=.65,lw=.8)
            ax.plot([anchors[side,projection[0]],centers[side,projection[0]]],[anchors[side,projection[1]],centers[side,projection[1]]],color='#bf741c',ls='--',lw=2)
            ax.scatter(*anchors[side,list(projection)],c='#bf741c',s=35,zorder=5)
        ax.plot(rod[:,projection[0]],rod[:,projection[1]],'-o',color='#1262c5',lw=3,ms=5,zorder=6)
        ax.scatter(*rod[0,list(projection)],c='#073b7c',marker='s',s=85,zorder=7)
        ax.set_title(title,fontsize=11);ax.set_aspect('equal');ax.autoscale();ax.margins(.12)
        ax.set_xlabel(['Forward','Lateral'][projection[0]==1]+' (source units)');ax.set_ylabel('Vertical (source units)');ax.grid(alpha=.15)
    fig.suptitle('Mechanical ownership: one continuous 12-node shaft + two suspended bodies\nGrey: shared visible skin | Blue: rod centerline | Green: contents supports | Amber: suspension',fontsize=13)
    fig.text(.5,.005,'Actual offline solver snapshot, before the adapter angle presentation. Wireframe supports omit the small pressure deformation.',ha='center',fontsize=8)
    fig.savefig(output,dpi=160);plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args();render(a.source,a.output)
