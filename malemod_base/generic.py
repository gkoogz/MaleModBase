"""Deterministic 3D male reference proxy; independent of any game assets."""
import numpy as np
from .contracts import validate_preferences

def build(preferences):
    validate_preferences(preferences);c={name:.5 for name in ['height','shoulders','chest','glutes','hips','limb_length']};c.update(preferences.get('body',{}))
    height=.85+.30*c['height'];shoulder=.8+.4*c['shoulders'];chest=.75+.5*c['chest'];hips=.85+.3*c['hips'];glutes=.65+.7*c['glutes'];limbs=.9+.2*c['limb_length']
    skeleton=[];lookup={}
    def joint(name,parent,p):
        lookup[name]=len(skeleton);skeleton.append({'id':name,'parent':lookup[parent] if parent else -1,'position':(np.asarray(p)*height).tolist()})
    joint('root',None,[0,0,0]);joint('pelvis','root',[0,0,.93]);joint('spine_lower','pelvis',[0,0,1.07]);joint('spine_upper','spine_lower',[0,0,1.25]);joint('chest','spine_upper',[0,0,1.43]);joint('neck','chest',[0,0,1.56]);joint('head','neck',[0,0,1.70])
    for side,sign in [('left',1),('right',-1)]:
        joint('clavicle_'+side,'chest',[sign*.12*shoulder,0,1.44]);joint('shoulder_'+side,'clavicle_'+side,[sign*.20*shoulder,0,1.43]);joint('elbow_'+side,'shoulder_'+side,[sign*(.20*shoulder+.18*limbs),0,1.43-.27*limbs]);joint('wrist_'+side,'elbow_'+side,[sign*(.20*shoulder+.27*limbs),-.015,1.43-.49*limbs]);joint('hand_'+side,'wrist_'+side,[sign*(.20*shoulder+.29*limbs),-.015,1.43-.57*limbs])
        joint('hip_'+side,'pelvis',[sign*.095*hips,0,.89]);joint('knee_'+side,'hip_'+side,[sign*.10*hips,0,.48]);joint('ankle_'+side,'knee_'+side,[sign*.10*hips,0,.11]);joint('foot_'+side,'ankle_'+side,[sign*.10*hips,-.055,.045])
    positions=[];normals=[];uv=[];joints=[];weights=[];indices=[];parts=[]
    def ellipsoid(name,center,radii,primary,secondary=None,start=None,end=None):
        center=np.asarray(center)*height;radii=np.asarray(radii)*height;rotation=np.eye(3)
        if start is not None:
            a=np.asarray(start)*height;b=np.asarray(end)*height;center=(a+b)*.5;axis=(b-a)/np.linalg.norm(b-a);reference=np.array([0,1,0]);x=np.cross(reference,axis);x/=np.linalg.norm(x);rotation=np.column_stack([x,np.cross(axis,x),axis]);radii[2]=np.linalg.norm(b-a)*.5+.025*height
        first=len(positions);rings=12;sides=16
        for row in range(rings+1):
            theta=np.pi*row/rings
            for col in range(sides+1):
                phi=2*np.pi*col/sides;unit=np.array([np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)])
                p=center+rotation@(unit*radii);n=rotation@(unit/radii);n/=np.linalg.norm(n)
                blend=(1-row/rings if start is not None else row/rings) if secondary else 0
                positions.append(p.tolist());normals.append(n.tolist());uv.append([col/sides,row/rings]);joints.append([lookup[primary],lookup[secondary] if secondary else 0,0,0]);weights.append([1-blend,blend,0,0])
        for row in range(rings):
            for col in range(sides):
                a=first+row*(sides+1)+col;b=a+sides+1
                # Skip collapsed pole triangles; keep outward winding.
                if row>0:indices.append([a,b,a+1])
                if row<rings-1:indices.append([a+1,b,b+1])
        parts.append({'id':name,'firstVertex':first,'vertexCount':len(positions)-first})
    ellipsoid('pelvis',[0,0,.93],[.16*hips,.115,.14],'pelvis')
    ellipsoid('abdomen',[0,0,1.13],[.145*hips,.105,.25],'spine_upper','pelvis')
    ellipsoid('ribcage',[0,0,1.34],[.19*shoulder,.115*chest,.20],'chest','spine_upper')
    ellipsoid('neck',[0,0,1.56],[.055,.055,.08],'neck')
    ellipsoid('head',[0,0,1.70],[.080,.085,.12],'head')
    for side,sign in [('left',1),('right',-1)]:
        ellipsoid('pec_'+side,[sign*.077*shoulder,-.085*chest,1.35],[.085*shoulder,.044*chest,.085],'chest')
        ellipsoid('glute_'+side,[sign*.065*hips,.073,.92],[.092*hips,.063*glutes,.105*glutes],'pelvis')
        point=lambda key:np.asarray(skeleton[lookup[key+'_'+side]]['position'])/height
        ellipsoid('upper_arm_'+side,[0,0,0],[.045,.045,.1],'shoulder_'+side,'elbow_'+side,point('shoulder'),point('elbow'))
        ellipsoid('forearm_'+side,[0,0,0],[.035,.038,.1],'elbow_'+side,'wrist_'+side,point('elbow'),point('wrist'))
        ellipsoid('hand_'+side,point('hand'),[.038,.025,.07],'hand_'+side)
        ellipsoid('thigh_'+side,[0,0,0],[.074*hips,.075,.1],'hip_'+side,'knee_'+side,point('hip'),point('knee'))
        ellipsoid('shin_'+side,[0,0,0],[.047,.055,.1],'knee_'+side,'ankle_'+side,point('knee'),point('ankle'))
        ellipsoid('foot_'+side,point('foot'),[.052,.115,.045],'foot_'+side)
    mesh={'positions':positions,'normals':normals,'uv':uv,'joints':joints,'weights':weights,'indices':indices,'parts':parts}
    sockets=[{'id':'pelvis.anatomy','joint':'pelvis','position':(np.array([0,-.11,.98])*height).tolist(),'direction':[0,-1,0]},
             {'id':'waist.garment','joint':'pelvis','position':[0,0,1.02*height],'direction':[0,-1,0]},
             {'id':'chest.garment','joint':'chest','position':[0,0,1.35*height],'direction':[0,-1,0]},
             {'id':'head.face','joint':'head','position':[0,-.085*height,1.70*height],'direction':[0,-1,0]}]
    return {'contractVersion':1,'id':'generic-male-proxy','units':'meters','up':[0,0,1],'forward':[0,-1,0],
            'skeleton':skeleton,'sockets':sockets,'mesh':mesh,'controls':c,'quality':'overlapping reference surfaces; not a welded production body'}
