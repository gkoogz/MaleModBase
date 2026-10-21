"""Recruit a measured contact footprint across body, root ramp and shaft.

Consumes a complete-shaft pelvic union and the existing anatomical source
module. Produces authoring geometry and compound-surface donors, never a game
package. The front half is fixed; only the rear half supplies the added area.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy import sparse

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assemble_modular_pelvis import load
from build_modular_anatomy import save
from malemod_base.modular_anatomy import SurfaceAttachment,directed_boundary,radial_attachment
from malemod_base.modular_union import charted_trumpet_union
from malemod_base.surface_footprint import ContactArea,transfer_chart,underside_rear_footprint


def build(body, support, module, output, maximum_extension, ratio=2., suspension_shift=None, pelvic_half_angle=75.):
    if output.exists():raise ValueError('Use a fresh compound-contact output directory')
    original,body_donors=load(body)
    target,target_donors=load(support/'body-union.npz')
    shaft,shaft_donors=load(module/'shaft.npz')
    source,source_donors=load(module/'scrotum-detached.npz')
    attached,_=load(module/'scrotum-attached.npz')
    ramp=np.load(support/'pelvic-ramp.npz',allow_pickle=False)
    attachment=np.load(module/'attachment.npz',allow_pickle=False)
    recipe=json.loads((module/'manifest.json').read_text())
    root=attachment['root'];axis=attachment['section_center']-root
    axis/=np.linalg.norm(axis)
    old_rim=SurfaceAttachment(attachment['vertices'],attachment['target_faces'],attachment['barycentric']).targets(shaft)
    q=old_rim-root;down=(q-np.outer(q@axis,axis)).mean(0);down/=np.linalg.norm(down)
    side=np.cross(axis,down)

    def coords(points):
        q=points-root;s=q@axis
        return s,np.linalg.norm(q-np.outer(s,axis),axis=1),np.arctan2(q@side,q@down)

    source_loop=directed_boundary(source)[0]
    baseline_binding=radial_attachment(source,source_loop,shaft,root,attachment['section_center'],
        flare=recipe['neckFlare'],side_wrap_degrees=recipe['sideWrapDegrees'],
        proximal_fill=recipe['proximalFill'],station_bounds=recipe['stationBounds'],
        rear_extension=recipe['rearExtension'])
    ss,sr,sa=coords(shaft.points);bs,br,ba=coords(original.points)
    shaft_chart=np.c_[ss,sa]
    baseline=np.einsum('ij,ijk->ik',baseline_binding.barycentric,shaft_chart[shaft.faces[baseline_binding.target_faces]])
    bank=original.donors.shape[1]
    if target.donors.shape[1]!=bank+shaft.donors.shape[1]:raise ValueError('Support donor banks differ')
    bw=np.asarray(target.donors[:,:bank].sum(1)).ravel()
    _,_,angle=coords(target.points)
    angular_limit=min(np.pi-.15,np.max(abs(baseline[:,1]))+.5)
    chart=np.zeros((len(target.points),2))
    for selected,mesh,field in [
        (np.flatnonzero((bw>1-1e-12)&(abs(angle)<angular_limit+.05)),original,np.c_[-br,ba]),
        (np.flatnonzero((bw<1e-12)&(abs(angle)<angular_limit+.05)),shaft,shaft_chart)]:
        chart[selected]=transfer_chart(target.points[selected],mesh.points,mesh.faces,field,periodic_axes=(1,))
    rings=ramp['rings'];t=.5*(1-np.cos(np.pi*np.arange(len(rings))/(len(rings)-1)))
    for row,ring in enumerate(rings[1:-1],1):
        chart[ring]=chart[rings[0]]*(1-t[row])+chart[rings[-1]]*t[row]
    xy=chart[target.faces];tri=target.points[target.faces]
    normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    limit=float(maximum_extension+np.ptp(baseline[:,0]))
    eligible=(abs(angle[target.faces]).max(1)<angular_limit)&(xy[:,:,0].min(1)>-limit)&(xy[:,:,0].max(1)<baseline[:,0].max()+1)
    eligible&=(bw[target.faces].min(1)<1-1e-12)|(normal@axis>0)
    measure=ContactArea(target.points,target.faces,chart,eligible)
    # Finish the sector restriction before the closest measured body boundary.
    # Existing shaft-side/front contact stays put; only newly recruited rear
    # contact narrows as it crosses the root ramp onto the lower pelvis.
    outer=chart[rings[0]]
    valid_outer=(outer[:,0]<0)&(abs(outer[:,1])<angular_limit)
    transition_end=float(outer[valid_outer,0].max())
    half_angle=np.deg2rad(pelvic_half_angle)
    footprint,area=measure.expand_rear(baseline,ratio,maximum_extension,
        proposal=lambda extension:underside_rear_footprint(baseline,extension,transition_end,half_angle))
    area['pelvicHalfAngleDegrees']=float(pelvic_half_angle)
    area['sectorTransitionEnd']=transition_end
    pelvic_offset=measure.minimum_projection(footprint,root,down,(bw[target.faces].min(1)>1-1e-10))
    if pelvic_offset is not None and pelvic_offset<=0:
        raise ValueError('Compound contact includes pelvis above the underside half-space')
    area['minimumPelvicContactUndersideOffset']=pelvic_offset
    before=measure.centroid(baseline);after=measure.centroid(footprint)
    shift_mode='explicit source-coordinate displacement'
    if suspension_shift is None:
        suspension_shift=max(0.,float((before-after)@axis))
        shift_mode='contact-area centroid change, bounded by local attachment integrity'
        bounded_shift=True
    else:bounded_shift=False
    source.normal_field=attached.normal_field[:len(source.points)].copy()
    source.donors=sparse.hstack([sparse.csr_matrix((len(source.points),bank)),source.donors],format='csr')
    # Move the retained sac rigidly with its new area centroid. Do not distort
    # the lobes or move the fixed front rim.
    requested_shift=float(suspension_shift)
    union,neck,binding,info,report=charted_trumpet_union(target,source,source_loop,chart,footprint,eligible,
        source_shift=(axis,requested_shift))
    suspension_shift=report['appliedSourceShift']
    if not bounded_shift and abs(suspension_shift-requested_shift)>1e-10:
        raise ValueError('Explicit source shift violates attachment integrity; use centroid-derived placement or a smaller override')
    end=np.einsum('ij,ijk->ik',info['support_barycentric'],target.points[info['support_triangles']])
    error=float(np.max(np.linalg.norm(end-neck.points[info['rings'][-1]],axis=1)))
    if error>1e-8:raise ValueError('Compound rim lost its exact supporting triangles')
    fixed=np.flatnonzero(baseline[:,0]>=.5*(baseline[:,0].min()+baseline[:,0].max()))
    front_error=0.;baseline_points=baseline_binding.targets(shaft)
    for index in fixed:
        knot=int(np.argmin(abs(info['source_curve_parameter']-index)))
        if abs(info['source_curve_parameter'][knot]-index)>1e-7:
            raise ValueError('Compound cut omitted a fixed front knot')
        front_error=max(front_error,float(np.linalg.norm(end[knot]-baseline_points[index])))
    if front_error>1e-7:raise ValueError('Compound recruitment moved the front boundary')
    area['frontMaximumPositionError']=front_error
    support_weights=np.einsum('ij,ij->i',info['support_barycentric'],bw[info['support_triangles']])
    body_knots=support_weights>1-1e-10
    body_down=(end[body_knots]-root)@down
    if len(body_down) and body_down.min()<=0:
        raise ValueError('Compound attachment recruits pelvis above the underside half-space')
    area['minimumPelvicRimUndersideOffset']=float(body_down.min()) if len(body_down) else None
    output.mkdir(parents=True)
    save(union,output/'body-union');save(neck,output/'scrotum-attached')
    np.savez_compressed(output/'neck.npz',**info)
    np.savez_compressed(output/'attachment.npz',vertices=binding.vertices,target_faces=binding.target_faces,barycentric=binding.barycentric)
    np.savez_compressed(output/'contact.npz',chart=chart,eligible=eligible,baseline=baseline,footprint=footprint,source_loop=source_loop)
    inputs=[body,body_donors,support/'body-union.npz',target_donors,support/'pelvic-ramp.npz',
            module/'shaft.npz',shaft_donors,module/'scrotum-detached.npz',source_donors,
            module/'scrotum-attached.npz',module/'attachment.npz',module/'manifest.json']
    report.update(schema='malemod.compound-contact-authoring/1',area=area,
        supportBindingError=error,suspensionShift=float(suspension_shift),
        requestedSuspensionShift=requested_shift,suspensionShiftMode=shift_mode,contactCentroidBefore=before.tolist(),contactCentroidAfter=after.tolist(),
        supportKnots=dict(body=int(sum(support_weights>1-1e-10)),
                          shaft=int(sum(support_weights<1e-10)),
                          ramp=int(sum((support_weights>=1e-10)&(support_weights<=1-1e-10)))),
        inputs=[dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in inputs],
        artifacts={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.glob('*.npz'))},
        nativeRuntime=False,installed=False,attachmentGatePassed=False,
        limitations=['Whole-surface intersection audit is separate.',
            'Native adoption must cut the rebuilt support before grafting, evaluate body/root before the neck, and retain source material donors.',
            'This authoring result does not certify live controls, motion, materials or LODs.'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='inputs'},indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['body','support','module','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--maximum-extension',type=float,required=True)
    p.add_argument('--ratio',type=float,default=2.)
    p.add_argument('--suspension-shift',type=float,help='Explicit source-unit override; default follows the measured contact centroid')
    p.add_argument('--pelvic-half-angle',type=float,default=75.,help='Rear pelvic sector about the underside; must be below 90 degrees')
    build(**vars(p.parse_args()))
