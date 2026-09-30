"""Render actual old/new runtime captures with matched orthographic cameras."""
from pathlib import Path
import ctypes,json
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from compact_runtime import HERE,ROOT,Header,capture
from audit_compact import points

OUT=ROOT.parents[1]/'outputs/compact-runtime-review-20260929'
LIB=HERE/'raster.dll'
raster=ctypes.CDLL(str(LIB)).raster
raster.argtypes=[ctypes.c_int]+[ctypes.c_void_p]*5+[ctypes.c_int]*2+[ctypes.c_void_p]*2
FONT='C:/Windows/Fonts/segoeui.ttf'
def font(n):return ImageFont.truetype(FONT,n)
def load(name,new):
    report=json.loads((HERE/'compact-report.json').read_text());d=np.load(HERE/'candidate.npz');rem=np.load(HERE/'remaps.npz')
    if new:
        raw=(ROOT/'captures/surface-audit-compact'/name).read_bytes();off=report['ucNodes']*28
        p=points(raw,report['nrVertices'],off);mapping=np.full(30717,-1,int);mapping[rem['nrKeep']]=np.arange(len(p));faces=mapping[d['nr_faces']]
        uc=Header('unified_collar_data.h');body=np.ndarray((report['ucNodes'],3),np.float32,buffer=raw,strides=(28,4)).astype(float)
        # Read compact generated UC indices without loading a minified line.
        uc.text=(ROOT/'src/runtime/unified_collar_data.h').read_text();uc.matches={m[3]:m for m in __import__('compact_runtime').ARRAY.finditer(uc.text)}
        uf=uc.a('ucFaces').reshape(-1,3);um=uc.a('ucMap')[:len(p)]
    else:
        raw=(ROOT/'captures/surface-audit-revised'/name).read_bytes();p,_=capture(ROOT/'captures/surface-audit-revised'/name)
        faces=Header('neck_render_data.h').a('nrIndices').reshape(-1,3);body=np.ndarray((50059,3),np.float32,buffer=raw,strides=(28,4)).astype(float)
        uc=Header('unified_collar_data.h');uf=uc.a('ucFaces').reshape(-1,3);um=uc.a('ucMap')[:len(p)]
    bodyFaces=uf[~np.all(np.isin(uf,np.unique(um)),axis=1)]
    # Crop context to the adjacent pelvis; anatomy is not clipped or modified.
    bodyFaces=bodyFaces[(body[bodyFaces].mean(axis=1)[:,2]>55)&(body[bodyFaces].mean(axis=1)[:,2]<108)]
    return p,faces,body,bodyFaces

def camera(direction,top=False):
    d=np.array(direction,float);d/=np.linalg.norm(d)
    x=np.array([1.,0,0]) if abs(d[0])<1e-9 else np.cross([0,0,1],d);x/=np.linalg.norm(x)
    y=np.cross(d,x);return np.array([x,y,d]).T

def render(p,f,B,limits,W=840,H=470,wire=False,color=(.50,.65,.72)):
    q=p@B;n=np.zeros_like(q);fn=np.cross(q[f[:,1]]-q[f[:,0]],q[f[:,2]]-q[f[:,0]])
    for j in range(3):np.add.at(n,f[:,j],fn)
    n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
    lo,hi=limits;center=(lo+hi)/2;scale=min((W-54)/(hi[0]-lo[0]),(H-42)/(hi[1]-lo[1]))
    x=(q[:,0]-center[0])*scale+W/2;y=H/2-(q[:,1]-center[1])*scale
    depth=np.full((H,W),-1e5,np.float32);attrs=np.zeros((H,W,3),np.float32)
    arrays=[np.ascontiguousarray(f,np.int32)]+[np.ascontiguousarray(a,np.float32) for a in [x,y,q[:,2],n]]
    raster(len(f),*[a.ctypes.data for a in arrays],W,H,depth.ctypes.data,attrs.ctypes.data)
    attrs/=np.maximum(np.linalg.norm(attrs,axis=2)[:,:,None],1e-12)
    light=np.array([-.3,.55,.78]);shade=.35+.6*np.abs(attrs@light)
    rgb=np.clip(shade[:,:,None]*color,0,1);rgb[depth<-1e4]=[.97,.98,.99]
    rgb=(rgb*255).astype(np.uint8)
    if wire:
        edges=np.unique(np.sort(np.r_[f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]],axis=1),axis=0)
        lengths=np.sqrt((x[edges[:,1]]-x[edges[:,0]])**2+(y[edges[:,1]]-y[edges[:,0]])**2)
        count=np.maximum(2,np.minimum(180,np.ceil(lengths).astype(int)+1));idx=np.repeat(np.arange(len(edges)),count)
        starts=np.repeat(np.r_[0,np.cumsum(count)[:-1]],count);t=(np.arange(len(idx))-starts)/(count[idx]-1)
        a=edges[idx,0];b=edges[idx,1];xx=np.rint(x[a]*(1-t)+x[b]*t).astype(int);yy=np.rint(y[a]*(1-t)+y[b]*t).astype(int);zz=q[a,2]*(1-t)+q[b,2]*t
        valid=(xx>=0)&(xx<W)&(yy>=0)&(yy<H);xx=xx[valid];yy=yy[valid];zz=zz[valid]
        visible=zz>=depth[yy,xx]-.025;rgb[yy[visible],xx[visible]]=[35,69,87]
    return Image.fromarray(rgb)

def sheet(name,filename,wire=False,detail=False):
    old=load(name,False);new=load(name,True)
    cams=[('Top',[0,0,1]),('Side',[0,-1,0]),('Oblique',[.15,-.86,.49]),('Underside',[0,-.5,-.866])]
    if detail:cams=[('Collar / ventral oblique',[.15,-.86,-.49]),('Head / upper oblique',[.15,-.86,.49])]
    im=Image.new('RGB',(1680,105+len(cams)*525),'#f7f9fc');draw=ImageDraw.Draw(im)
    draw.text((25,15),'BEFORE  |  61,378 triangles',font=font(27),fill='#23384a');draw.text((865,15),'AFTER  |  35,000 triangles',font=font(27),fill='#23384a')
    draw.text((25,58),name.replace('.bin','')+'  |  identical controls, pose, camera and scale  |  actual runtime meshes; pelvis shown translucent',font=font(20),fill='#586c7c')
    for row,(label,dir) in enumerate(cams):
        B=camera(dir);joined=np.r_[old[0]@B,new[0]@B];lo=joined[:,:2].min(axis=0);hi=joined[:,:2].max(axis=0)
        if not detail:
            context=old[2];selected=(context[:,0]<14)&(context[:,2]>65)&(context[:,2]<103)&(np.abs(context[:,1])<15)
            xy=(context[selected]@B)[:,:2];lo=np.minimum(lo,xy.min(axis=0));hi=np.maximum(hi,xy.max(axis=0))
        if detail:
            if row==0:region=np.r_[old[0],new[0]];region=region[(region[:,0]<29)&(region[:,2]>68)]
            else:region=np.r_[old[0],new[0]];region=region[region[:,0]>np.quantile(region[:,0],.82)]
            xy=(region@B)[:,:2];lo=xy.min(axis=0)-1;hi=xy.max(axis=0)+1
        for col,data in enumerate([old,new]):
            pic=render(data[0],data[1],B,(lo,hi),wire=wire)
            if not detail:
                context=render(data[2],data[3],B,(lo,hi),color=(.88,.89,.91))
                bg=Image.new('RGB',pic.size,(247,249,252));context=Image.blend(bg,context,.20)
                a=np.array(pic);b=np.array(context);foreground=np.any(a!=[247,249,252],axis=2);b[foreground]=a[foreground];pic=Image.fromarray(b)
            im.paste(pic,(col*840,145+row*525));draw.text((col*840+25,110+row*525),label,font=font(23),fill='#203f50')
    im.save(OUT/filename)

if __name__=='__main__':
    OUT.mkdir(exist_ok=True,parents=True)
    sheet('static-0-50-50.bin','neutral-angles.png')
    sheet('static-2-100-100.bin','large-angles.png')
    sheet('static-0-50-50.bin','feature-wireframes.png',wire=True,detail=True)
    sheet('motion-2-60.bin','bent-angles.png')
    print(OUT)
