"""Bind the reconstructed display surface to independent mechanical objects.

Produces shared native-readable geometry, never a game installation. Coordinate
calibration, skin palettes and native material contracts belong to the adapter.
"""
import argparse,json,struct,sys,hashlib
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def reference(path):
    raw=path.read_bytes();offset=0
    def take(dtype,n):
        nonlocal offset
        a=np.frombuffer(raw,dtype,n,offset).copy();offset+=a.nbytes;return a
    def u():return int(take('<u4',1)[0])
    if u()!=6:raise ValueError('Expected measured wire6 mechanics')
    for _ in range(3):
        n=u();take('<f4',11*n);take('<u4',n)
    take('<u4',u());radius,length=take('<f4',2)
    guide=take('<f4',36).reshape(12,3).astype(float);take('<f4',36)
    lobes=take('<f4',6).reshape(2,3);take('<f4',6);radii=take('<f4',6).reshape(2,3)
    axes=take('<f4',18).reshape(2,3,3)
    frames=np.tile(np.r_[np.zeros(3),np.eye(3).ravel()],(15,1))
    for i in range(12):
        tangent=guide[min(11,i+1)]-guide[max(0,i-1)];tangent/=np.linalg.norm(tangent)
        side=np.array([0.,1,0])-tangent*tangent[1];side/=np.linalg.norm(side)
        frames[1+i]=np.r_[guide[i],tangent,side,np.cross(tangent,side)]
    for i in range(2):frames[13+i]=np.r_[lobes[i],axes[i].ravel()]
    return frames,float(radius),float(length),radii


def bind(points,frames,ids,weights):
    local=np.empty((len(points),2,3))
    for j in range(2):
        selected=frames[ids[:,j]]
        local[:,j]=np.einsum('nij,nj->ni',selected[:,3:].reshape(-1,3,3),points-selected[:,:3])
    return ids,weights,local


def shaft_owners(points,frames):
    guide=frames[1:13,:3];a=guide[:-1];delta=guide[1:]-a
    t=np.clip(np.einsum('nki,ki->nk',points[:,None,:]-a,delta)/np.sum(delta*delta,axis=1),0,1)
    distance=np.linalg.norm(points[:,None,:]-(a+t[:,:,None]*delta),axis=2)
    i=distance.argmin(1);weight=t[np.arange(len(points)),i]
    return np.c_[i+1,i+2],np.c_[1-weight,weight]


def save_asset(path,points,normals,faces,frames,dimensions,owners,lofts,mount=None):
    ids,weights,local=owners
    with path.open('wb') as f:
        f.write(b'MMMOD002'+struct.pack('<III',len(points),len(faces),len(lofts)))
        f.write(np.asarray(dimensions,dtype='<f8').tobytes());f.write(np.asarray(frames,dtype='<f8').tobytes())
        def binding(i):
            for j in range(2):f.write(struct.pack('<Id3d',int(ids[i,j]),weights[i,j],*local[i,j]))
        for i,(p,n) in enumerate(zip(points,normals)):
            f.write(struct.pack('<6d',*p,*n));binding(i)
        f.write(np.asarray(faces,dtype='<u4').tobytes())
        for vertex,start,end,t,first,last in lofts:
            f.write(struct.pack('<IIId9d',vertex,start,end,t,*first,*last,*points[vertex]));binding(start);binding(end)
        f.write(struct.pack('<I',int(mount is not None)))
        if mount is not None:
            f.write(np.r_[mount['origin'],mount['basis'].ravel(),mount['step'],mount['station'],mount['length']].astype('<f8').tobytes())
            f.write(struct.pack('<II',mount['side'],len(mount['outer'])))
            for key in ['height','bodyWeight','shaftWeight']:f.write(np.asarray(mount[key],dtype='<f8').tobytes())
            for key in ['outer','inner']:f.write(np.asarray(mount[key],dtype='<u4').tobytes())
            f.write(np.asarray(mount['theta'],dtype='<f8').tobytes())


def build(assembly,module,source,output):
    if output.exists():raise ValueError('Use a new runtime binding output directory')
    mesh=np.load(assembly/'body-union.npz');points=mesh['points'];tree=cKDTree(points)
    frames,radius,length,radii=reference(source);attachment=np.load(module/'attachment.npz')
    ids,weights=shaft_owners(points,frames)
    donors=sparse.load_npz(assembly/'body-union-donors.npz')
    body_count=json.loads((assembly/'manifest.json').read_text())['sourceBanks'][0]['count']
    body=np.asarray(donors[:,:body_count].sum(1)).ravel()>1-1e-10
    ids[body]=0;weights[body]=[1,0]
    neck=np.load(module/'neck.npz');sack=np.load(module/'scrotum-attached.npz')
    error,neck_ids=tree.query(sack['points'][neck['rings']])
    if error.max()>1e-8:raise ValueError('Assembly cut through a required scrotal connection')
    retained=int(neck['retained_vertices']);error,sack_ids=tree.query(sack['points'][:retained])
    if error.max()>1e-8:raise ValueError('Assembly removed retained scrotal tissue')
    from malemod_base.motion_binding import source_lobe_partition
    ids[sack_ids]=[13,14];weights[sack_ids]=source_lobe_partition(points[sack_ids,1])
    lofts=[]
    def append(rings,first,last):
        t=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
        for row in range(1,len(rings)-1):
            for j,index in enumerate(rings[row]):
                lofts.append((int(index),int(rings[0,j]),int(rings[-1,j]),float(t[row]),first[j],last[j]))
    append(neck_ids,neck['start_derivative'],neck['end_derivative'])
    ramp=np.load(assembly/'pelvic-ramp.npz');rings=ramp['rings'];q=points[rings]
    t=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
    h00=2*t**3-3*t*t+1;h01=-2*t**3+3*t*t
    design=np.c_[t**3-2*t*t+t,t**3-t*t]
    derivatives=np.linalg.lstsq(design,(q-h00[:,None,None]*q[0]-h01[:,None,None]*q[-1]).reshape(len(t),-1),rcond=None)[0].reshape(2,len(rings[0]),3)
    append(rings,*derivatives)
    # Generated interior rows are solved from their two independent endpoints.
    interior=np.array([row[0] for row in lofts]);ids[interior]=0;weights[interior]=[1,0]
    if len(np.unique(interior))!=len(interior):raise ValueError('Overlapping connecting-skin ownership')
    output.mkdir(parents=True)
    dimensions=np.r_[radius,length,attachment['root'],radii.ravel()]
    from malemod_base.modular_mount import prepare_mount
    body_input=json.loads((assembly/'manifest.json').read_text())['inputs'][0]['path']
    original_body=np.load(body_input)
    axis=ramp['mount_axis'];side=np.array([0.,1,0]);side-=axis*np.dot(side,axis);side/=np.linalg.norm(side)
    basis=np.array([axis,side,np.cross(axis,side)])
    shaft_mask=np.any((ids>=1)&(ids<=12)&(weights>0),axis=1)
    mount=prepare_mount(points,body,shaft_mask,original_body['points'],original_body['faces'],attachment['root'],basis,ramp,length)
    save_asset(output/'display.modular',points,mesh['normal_field'],mesh['faces'],frames,dimensions,bind(points,frames,ids,weights),lofts,mount)
    shaft=np.load(module/'shaft.npz');sid,sw=shaft_owners(shaft['points'],frames)
    from malemod_base.modular_anatomy import vertex_normals
    from types import SimpleNamespace
    normals=vertex_normals(SimpleNamespace(points=shaft['points'],faces=shaft['faces']))
    save_asset(output/'shaft.modular',shaft['points'],normals,shaft['faces'],frames,dimensions,bind(shaft['points'],frames,sid,sw),[])
    np.savez_compressed(output/'ownership.npz',frames=ids,weights=weights,neck_rings=neck_ids,ramp_rings=rings,body=body)
    report=dict(schema='malemod.modular-runtime-authoring/1',sourceSHA256=hashlib.sha256(source.read_bytes()).hexdigest(),
        vertices=len(points),triangles=len(mesh['faces']),completeShaftVertices=len(shaft['points']),
        connectionVertices=len(lofts),retainedSuspendedVertices=len(sack_ids),sourceUnits='unchanged measured source coordinates',
        nativeObserved=False,installed=False,limitations=['Native materials and skin palette are separate adapter bindings.',
        'Live body recruitment needs numerical and native visual validation.'],
        extendedChartCells=mount['extendedChartCells'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ['assembly','module','source','output']:parser.add_argument('--'+key,type=Path,required=True)
    a=parser.parse_args();build(a.assembly,a.module,a.source,a.output)
