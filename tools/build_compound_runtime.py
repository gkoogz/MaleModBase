"""Prepare a complete dynamic support while retaining the existing physics graph.

The old loft dependencies remain deformation-only inputs. The dynamic support
renders the complete shaft; the compound phase supplies the retained suspension
and its new skin. Nothing here writes a game installation.
"""
import argparse, hashlib, json, struct, sys, io
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from build_modular_runtime import bind,shaft_owners,save_asset
from export_compound_reference import export


def read_asset(path):
    raw=path.read_bytes();offset=8
    if raw[:8]!=b'MMMOD002':raise ValueError('Unsupported mechanical artifact')
    def take(dtype,count):
        nonlocal offset
        result=np.frombuffer(raw,dtype,count,offset).copy();offset+=result.nbytes;return result
    def binding():
        entries=[(take('<u4',1)[0],take('<f8',4)) for _ in range(2)]
        return [v[0] for v in entries],[v[1][0] for v in entries],[v[1][1:] for v in entries]
    nv,nf,nl=map(int,take('<u4',3));dimensions=take('<f8',11);frames=take('<f8',180).reshape(15,12)
    points=[];normals=[];ids=[];weights=[];local=[]
    for _ in range(nv):
        points.append(take('<f8',3));normals.append(take('<f8',3));a,b,c=binding();ids.append(a);weights.append(b);local.append(c)
    faces=take('<u4',nf*3).reshape(-1,3);lofts=[]
    for _ in range(nl):
        vertex,start,end=map(int,take('<u4',3));t=take('<f8',1)[0];first=take('<f8',3);last=take('<f8',3);rest=take('<f8',3);binding();binding()
        if not np.allclose(rest,points[vertex],rtol=0,atol=1e-12):raise ValueError('Loft reference differs from its vertex')
        lofts.append((vertex,start,end,t,first,last))
    if take('<u4',1)[0]!=1:raise ValueError('Compound runtime requires a measured body mount')
    values=take('<f8',15);side,nr=map(int,take('<u4',2))
    mount=dict(origin=values[:3],basis=values[3:12].reshape(3,3),step=values[12],station=values[13],length=values[14],side=side)
    mount['height']=take('<f8',side*side).reshape(side,side)
    mount['bodyWeight']=take('<f8',nv);mount['shaftWeight']=take('<f8',nv)
    mount['outer']=take('<u4',nr);mount['inner']=take('<u4',nr);mount['theta']=take('<f8',nr)
    if offset!=len(raw):raise ValueError('Mechanical artifact has trailing data')
    return dict(points=np.array(points),normals=np.array(normals),faces=faces,ids=np.array(ids),weights=np.array(weights),local=np.array(local),frames=frames,dimensions=dimensions,lofts=lofts,mount=mount)


def build(previous,support,module,candidate,body,output):
    if output.exists():raise ValueError('Use a fresh compound runtime directory')
    a=read_asset(previous/'runtime-01/display.modular');old=np.load(previous/'assembly-01/body-union.npz');target=np.load(support/'body-union.npz')
    od=sparse.load_npz(previous/'assembly-01/body-union-donors.npz');td=sparse.load_npz(support/'body-union-donors.npz')
    manifest=json.loads((previous/'assembly-01/manifest.json').read_text(encoding='utf-8'));bank=manifest['sourceBanks'][0]['count']
    if od.shape[1]!=td.shape[1]:raise ValueError('Compound source banks differ')
    nv=len(a['points']);nf=len(a['faces']);points=np.vstack([a['points'],target['points']]);faces=np.vstack([a['faces'],target['faces']+nv])
    bw=np.asarray(td[:,:bank].sum(1)).ravel();shaft=bw<1e-10
    ids,weights=shaft_owners(target['points'],a['frames']);ids[~shaft]=0;weights[~shaft]=[1,0]
    _,_,local=bind(target['points'],a['frames'],ids,weights)
    owners=(np.vstack([a['ids'],ids]),np.vstack([a['weights'],weights]),np.vstack([a['local'],local]))
    mount=a['mount'];mount['bodyWeight']=np.r_[mount['bodyWeight'],np.zeros(len(target['points']))];mount['shaftWeight']=np.r_[mount['shaftWeight'],shaft.astype(float)]
    # Socket clearance uses the new complete-shaft boundary. Its mechanical
    # stations are still the same measured source frames and dimensions.
    ramp=np.load(support/'pelvic-ramp.npz');mount['station']=float(ramp['inner_station'])
    out=output/'runtime-01';assembly=output/'assembly-01';out.mkdir(parents=True);assembly.mkdir()
    save_asset(out/'display.modular',points,np.vstack([a['normals'],target['normal_field']]),faces,a['frames'],a['dimensions'],owners,a['lofts'],mount)
    oldowner=np.load(previous/'runtime-01/ownership.npz')
    np.savez_compressed(out/'ownership.npz',frames=owners[0],weights=owners[1],neck_rings=oldowner['neck_rings'],ramp_rings=oldowner['ramp_rings'],body=np.r_[oldowner['body'],~shaft])
    np.savez_compressed(assembly/'body-union.npz',points=points,faces=faces,corner_uv=np.concatenate([old['corner_uv'],target['corner_uv']]),source_faces=np.r_[old['source_faces'],target['source_faces']],generated=np.r_[old['generated'],target['generated']],normal_field=np.vstack([a['normals'],target['normal_field']]),visible_faces=np.arange(nf,dtype=np.uint32))
    sparse.save_npz(assembly/'body-union-donors.npz',sparse.vstack([od,td],format='csr'))
    # Dynamic exporter consumes only the appended complete support, not the old
    # neck or its hidden mechanics. Keep its exact source face IDs in the asset.
    raw=output/'dynamic-01.mount';section=np.load(module/'attachment.npz');b=np.load(body)
    from malemod_base.modular_anatomy import vertex_normals
    from types import SimpleNamespace
    normal=vertex_normals(SimpleNamespace(points=b['points'],faces=b['faces']))
    module_faces=np.flatnonzero(shaft[target['faces']].all(1))+nf
    with raw.open('wb') as f:
        f.write(b'MMMDYN01'+hashlib.sha256((out/'display.modular').read_bytes()).hexdigest().encode())
        f.write(struct.pack('<5I',len(b['points']),len(b['faces']),len(section['section_radial']),len(module_faces),len(ramp['rings'][-1])))
        for value,dtype in [(b['points'],'<f8'),(normal,'<f8'),(b['faces'],'<u4'),(section['section_radial'],'<f8'),(section['root'],'<f8'),(np.eye(3),'<f8'),(module_faces,'<u4'),(ramp['rings'][-1]+nv,'<u4')]:f.write(value.astype(dtype).tobytes())
    detached=np.load(module/'scrotum-detached.npz');error,source_ids=cKDTree(a['points']).query(detached['points'])
    if error.max()>1e-8:raise ValueError('Retained suspension does not match the previous mechanical graph')
    old_face_map={tuple(f):i for i,f in enumerate(a['faces'])};source_faces=[]
    for f in source_ids[detached['faces']]:
        matches=[old_face_map.get(tuple(np.roll(f,j))) for j in range(3)];matches=[i for i in matches if i is not None]
        source_faces.append(matches[0] if matches else -1)
    # Old authoring refined the retained boundary. Material mapping must use
    # exact original source face lineage rather than assume face identity.
    contact=np.load(candidate/'contact.npz');neck=np.load(candidate/'neck.npz')
    export(support,module,candidate,output/'contact-reference.bin')
    shaft_mesh=np.load(module/'shaft.npz');rim=np.einsum('ij,ijk->ik',section['barycentric'],shaft_mesh['points'][shaft_mesh['faces'][section['target_faces']]])
    root=section['root'];axis=section['section_center']-root;axis/=np.linalg.norm(axis)
    q=rim-root;down=(q-np.outer(q@axis,axis)).mean(0);down/=np.linalg.norm(down);side=np.cross(axis,down)
    q=b['points']-root;station=q@axis;body_chart=np.c_[-np.linalg.norm(q-np.outer(station,axis),axis=1),np.arctan2(q@side,q@down)]
    angular_limit=min(np.pi-.15,np.max(abs(contact['baseline'][:,1]))+.5)
    # Every retained source corner keeps the original face's own material
    # chart, including aliases along old refinement seams.
    old_by_source={}
    for fi,sf in enumerate(old['source_faces']):
        if sf>=len(b['faces']):old_by_source.setdefault(int(sf)-len(b['faces']),[]).append(fi)
    corners=[];detached_points=detached['points']
    for sf,face in zip(detached['source_faces'],detached['faces']):
        candidates=old_by_source.get(int(sf),[])
        if not candidates:raise ValueError('Missing retained source material lineage')
        tri=a['points'][a['faces'][candidates]];basis=np.stack([tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]],axis=2)
        inverse=np.linalg.pinv(basis)
        for point in detached_points[face]:
            bary=np.einsum('nij,nj->ni',inverse,point-tri[:,0]);w=np.c_[1-bary.sum(1),bary]
            residual=np.linalg.norm(np.einsum('nij,nj->ni',basis,bary)+tri[:,0]-point,axis=1)
            score=np.maximum(-w.min(1),0)+residual
            best=score.argmin()
            if score[best]>1e-8:raise ValueError('Retained material corner left its source chart')
            corners.append((candidates[best],w[best]))
    payload=io.BytesIO();payload.write(b'MMCDYN01'+struct.pack('<3I',len(points),len(b['points']),len(source_ids)))
    payload.write(np.r_[neck['source_translation'],axis,angular_limit].astype('<f8').tobytes())
    tq=target['points']-root
    complete_chart=np.c_[tq@axis,np.arctan2(tq@side,tq@down)]
    np.savez_compressed(output/'contact-bindings.npz',source_vertices=source_ids,source_faces=source_faces,asset_chart=np.vstack([np.zeros((nv,2)),complete_chart]),support_vertex_offset=nv,support_face_offset=nf,source_translation=neck['source_translation'])
    payload.write(np.vstack([np.zeros((nv,2)),complete_chart]).astype('<f8').tobytes());payload.write(body_chart.astype('<f8').tobytes());payload.write(source_ids.astype('<u4').tobytes())
    payload.write((output/'contact-reference.bin').read_bytes())
    for parent,w in corners:payload.write(struct.pack('<I3d',parent,*w))
    encoded=payload.getvalue();original=raw.read_bytes();raw.write_bytes(b'MMMDYN02'+original[8:]+struct.pack('<I',len(encoded))+encoded)
    manifest['compoundRuntimeInputs']=[dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [previous/'runtime-01/display.modular',support/'body-union.npz',candidate/'neck.npz']]
    manifest['hiddenDeformationVertices']=nv;manifest['nativeRuntime']=False;manifest['installed']=False
    (assembly/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps(dict(vertices=len(points),faces=len(faces),supportFaces=len(module_faces),sourceVertices=len(source_ids),sourceFacesUsingCornerDonors=sum(i<0 for i in source_faces),nativeObserved=False,installed=False),indent=2)+'\n',encoding='utf-8')
    print((output/'manifest.json').read_text(encoding='utf-8'))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['previous','support','module','candidate','body','output']:p.add_argument('--'+name,type=Path,required=True)
    build(**vars(p.parse_args()))
