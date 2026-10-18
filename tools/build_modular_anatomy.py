"""Build source-derived independent shaft and scrotum authoring objects.

An exact evaluated source snapshot is required. Output is not an installable
game package. No inferred bones, remapped textures or engine topology claims.
"""
import argparse,hashlib,json,struct,sys
from pathlib import Path
import numpy as np
from scipy import sparse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from malemod_base.modular_anatomy import Mesh,clip,directed_boundary,complete_shaft,radial_attachment,attach_neck,trumpet_neck,vertex_normals,shaft_surface_normals,export_obj


def read_source(path):
    raw=path.read_bytes();offset=0
    def take(dtype,count):
        nonlocal offset
        result=np.frombuffer(raw,dtype,count,offset).copy();offset+=result.nbytes;return result
    def integer():return int(take('<u4',1)[0])
    if integer()!=6:raise ValueError('Expected evaluated wire6 source')
    surfaces=[]
    for _ in range(3):
        count=integer();points=take('<f4',3*count).reshape(-1,3)
        take('<f4',6*count);uv=take('<f4',2*count).reshape(-1,2);ids=take('<u4',count)
        surfaces.append((points,uv,ids))
    faces=take('<u4',integer()).reshape(-1,3);radius,length=take('<f4',2)
    guide=take('<f4',36).reshape(12,3);rest=take('<f4',36).reshape(12,3)
    centers=take('<f4',6).reshape(2,3);anchors=take('<f4',6).reshape(2,3);radii=take('<f4',6).reshape(2,3)
    p,uv,ids=surfaces[0]
    if not np.array_equal(ids,np.arange(len(ids))):raise ValueError('Source order requires explicit remapping')
    return Mesh.source(p,faces,uv),guide,centers,radii,float(radius)


def save(mesh,prefix):
    export_obj(mesh,prefix.with_suffix('.obj'))
    np.savez_compressed(prefix.with_suffix('.npz'),points=mesh.points,faces=mesh.faces,corner_uv=mesh.corner_uv,
                        source_faces=mesh.source_faces,generated=mesh.generated,
                        **({} if mesh.normal_field is None else dict(normal_field=mesh.normal_field)))
    sparse.save_npz(prefix.parent/(prefix.name+'-donors.npz'),mesh.donors)


def build(source,output,shaft_cut,scrotum_cut,lobe_protection,root=None,neck_flare=None,side_wrap_degrees=None,weld=False,refine_shaft=False,proximal_fill=0.,station_bounds=None,rear_extension=0.):
    if weld and neck_flare is None:raise ValueError('Visible union requires an explicit trumpet neck')
    output.mkdir(parents=True,exist_ok=False)
    original,guide,centers,radii,radius=read_source(source)
    distal=clip(original,[1,0,0],shaft_cut)
    root=guide[0] if root is None else np.asarray(root,dtype=float)
    shaft,rings,center,radial=complete_shaft(distal,root,refine=refine_shaft)
    signed_volume=np.einsum('ij,ij->i',shaft.points[shaft.faces[:,0]],np.cross(shaft.points[shaft.faces[:,1]],shaft.points[shaft.faces[:,2]])).sum()/6
    reverse=signed_volume<0
    if reverse:shaft=shaft.reversed()
    detached=clip(original,[0,0,-1],-scrotum_cut)
    loops=directed_boundary(detached)
    if len(loops)!=1 or np.ptp(detached.points[loops[0],2])>1e-6:
        raise ValueError('Scrotal cut must isolate one complete planar neck boundary')
    if reverse:detached=detached.reversed()
    binding=radial_attachment(detached,loops[0],shaft,root,center,flare=neck_flare or 1.,side_wrap_degrees=side_wrap_degrees,proximal_fill=proximal_fill,station_bounds=station_bounds,rear_extension=rear_extension)
    if neck_flare is None:
        scrotum,fixed=attach_neck(detached,binding,shaft,lobe_protection)
        neck={}
    else:
        # Cut-edge vertices inherit the original smooth field. Recomputing
        # normals on the clipped one-sided triangles creates alternating flat
        # and averaged normals along the cut and corrugates the new surface.
        source_normals=detached.donors@vertex_normals(original)*(-1 if reverse else 1)
        if weld:
            from malemod_base.modular_union import trumpet_union
            union,scrotum,binding,neck,union_report=trumpet_union(shaft,detached,binding,rings,root,center,source_normals)
            save(union,output/'visible-union')
            (output/'union-report.json').write_text(json.dumps(union_report,indent=2)+'\n')
        else:scrotum,binding,neck=trumpet_neck(detached,binding,shaft,source_normals=source_normals,shaft_normals=shaft_surface_normals(shaft,rings,root,center))
        fixed=np.r_[detached.points[:,2]<=lobe_protection,np.zeros(len(scrotum.points)-len(detached.points),bool)]
        np.savez_compressed(output/'neck.npz',**neck)
    save(shaft,output/'shaft');save(detached,output/'scrotum-detached');save(scrotum,output/'scrotum-attached')
    np.savez_compressed(output/'attachment.npz',vertices=binding.vertices,target_faces=binding.target_faces,
                        barycentric=binding.barycentric,protected_lobes=fixed,shaft_rings=rings,
                        root=root,section_center=center,section_radial=radial,lobe_centers=centers,lobe_radii=radii)
    gap=np.linalg.norm(scrotum.points[binding.vertices]-binding.targets(shaft),axis=1).max()
    def folded(a,b):
        na=np.cross(a.points[a.faces[:,1]]-a.points[a.faces[:,0]],a.points[a.faces[:,2]]-a.points[a.faces[:,0]])
        nb=np.cross(b.points[b.faces[:,1]]-b.points[b.faces[:,0]],b.points[b.faces[:,2]]-b.points[b.faces[:,0]])
        return int(np.sum(np.einsum('ij,ij->i',na,nb)<=0))
    report=dict(schema='malemod.modular-anatomy-authoring/1',sourceSHA256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source=str(source.resolve()),shaft=shaft.validate(True),scrotum=scrotum.validate(),attachmentGap=float(gap),
        unchangedProtectedLobeVertices=int(sum(fixed)),protectedLobeError=float(np.max(abs(scrotum.points[:len(detached.points)][fixed[:len(detached.points)]]-detached.points[fixed[:len(detached.points)]]))),
        neckFacesReversed=folded(detached,scrotum) if neck_flare is None else int(neck['reversed_faces']),
        neckMinimumOutwardCosine=None if neck_flare is None else float(neck['outward_minimum_cosine']),
        neckFlare=neck_flare,sideWrapDegrees=side_wrap_degrees,proximalFill=proximal_fill,retainedSackError=float(np.max(abs(scrotum.points[:len(detached.points)]-detached.points))),
        shaftCut=shaft_cut,scrotumCut=scrotum_cut,lobeProtection=lobe_protection,
        stationBounds=None if station_bounds is None else list(map(float,station_bounds)),rearExtension=rear_extension,
        neckHandleScale=None if neck_flare is None else float(neck['handle_scale']),
        sourceRadius=radius,refinedProximalShaft=refine_shaft,pelvisModified=False,nativeRuntime=False,installed=False,visibleUnion=union_report if weld else None,
        limitations=['Generated proximal UV chart needs source-material transfer before native adoption.',
                     'Independent scrotum stays open at its mounted rim; a separate closed visible union is exported only with --visible-union.',
                     'Pelvic opening/ramp, live physics mapping, native topology and LOD adoption are not implemented by this exporter.'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--shaft-cut',type=float,required=True);p.add_argument('--scrotum-cut',type=float,required=True);p.add_argument('--lobe-protection',type=float,required=True)
    p.add_argument('--root',type=float,nargs=3)
    p.add_argument('--neck-flare',type=float,help='Build tangent-matched trumpet extension with this footprint expansion')
    p.add_argument('--side-wrap-degrees',type=float,help='Circumferential reach from underside; 90 reaches the sides, over 90 reaches above the sides')
    p.add_argument('--visible-union',action='store_true',help='Cut and weld the display surface while preserving the complete independent shaft')
    p.add_argument('--refine-shaft',action='store_true',help='Uniform section refinement and bounded smooth proximal profile; distal source stays exact')
    p.add_argument('--proximal-fill',type=float,default=0.,help='Broaden the rootward neck shoulder while preserving the reserved pelvic ramp')
    p.add_argument('--station-bounds',type=float,nargs=2,help='Measured attachment interval along the shaft, reserving the pelvic socket')
    p.add_argument('--rear-extension',type=float,default=0.,help='Rootward extension in measured source units; leaves the distal half exact')
    a=p.parse_args();build(a.source,a.output,a.shaft_cut,a.scrotum_cut,a.lobe_protection,a.root,a.neck_flare,a.side_wrap_degrees,a.visible_union,a.refine_shaft,a.proximal_fill,a.station_bounds,a.rear_extension)
