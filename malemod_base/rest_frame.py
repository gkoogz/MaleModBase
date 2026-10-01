"""Source rest-centerline measurement on caller-supplied prepared geometry.

Coordinates remain Wolverine source coordinates. This does not perform the
fairing, root-profile, logical surface or glans stages that prepare that input.
"""
from dataclasses import dataclass
import numpy as np
from .authored_shape import SOURCE_ROOT
from .collar import smoother


def _unit(value):
    length=np.linalg.norm(value,axis=-1,keepdims=True)
    return np.divide(value,length,out=np.broadcast_to(np.array([1.,0,0],dtype=value.dtype),value.shape).copy(),where=length>1e-6)


def _smoother32(value):
    v=np.clip(np.asarray(value,dtype=np.float32),np.float32(0),np.float32(1))
    return v*v*v*(v*(v*np.float32(6)-np.float32(15))+np.float32(10))


def sample(centers,flex):
    """Source interpolation and tangent stencil; flex is clamped to [0,1]."""
    dtype=centers.dtype
    t=np.clip(np.asarray(flex,dtype=dtype),0,1)
    u=t*(len(centers)-1);i=np.minimum(len(centers)-2,u.astype(int));q=np.asarray(u-i,dtype=dtype)
    center=centers[i]*(1-q[...,None])+centers[i+1]*q[...,None]
    tangent=_unit(centers[np.minimum(len(centers)-1,i+2)]-centers[np.maximum(0,i-1)])
    return center,tangent


def closest_flex(centers,points):
    start=centers[:-1];span=np.diff(centers,axis=0)
    denominator=np.sum(span*span,axis=1)
    q=np.divide(np.sum((points[:,None]-start)*span,axis=2),denominator,
                out=np.zeros((len(points),len(span)),dtype=centers.dtype),where=denominator>1e-8)
    q=np.clip(q,0,1)
    nearest=start+span*q[...,None]
    distance=np.sum((points[:,None]-nearest)**2,axis=2)
    segment=np.argmin(distance,axis=1)  # First minimum, as in the source loop.
    return (segment.astype(centers.dtype)+q[np.arange(len(points)),segment])/len(span)


@dataclass(frozen=True)
class RestFrame:
    centers:np.ndarray
    flex:np.ndarray
    root_follow:np.ndarray
    body_radius:float
    rest_length:float
    uses_previous_length:bool


class SourceRestFrame:
    """Cached reference weights, with independent outputs for every evaluation."""
    def __init__(self,bank):
        def get(prefix,name):return np.array(bank[prefix+'__'+name],dtype=np.float32,copy=True)
        self.shaft=np.maximum(get('physics_weights','phys_shaft_weight'),
                              get('physics_weights','phys_attachment_weight'))
        ball=np.minimum(1,get('physics_weights','phys_scrotum_weight'))
        self.suspension=get('suspension_weights','suspensionWeight')
        self.root_follow=get('pelvic_root_binding','pelvicRootFollow')
        self.root_mask=get('pelvic_root_binding','pelvicRootMask')
        flex=get('physics_weights','phys_flex_coordinate')
        if any(v.shape!=(2388,) or not np.isfinite(v).all() for v in
               [self.shaft,ball,self.suspension,self.root_follow,self.root_mask,flex]):
            raise ValueError('Expected finite versioned 2388-point source bindings')
        target=np.arange(18,dtype=np.float32)/17
        sigma=np.float32(.082);inv=np.float32(1)/(np.float32(2)*sigma*sigma)
        q=flex[None,:]-target[:,None]
        self.weights=(self.shaft*self.shaft*(1-ball)*(1-ball))[None,:]*np.exp(-q*q*inv)
        self.weights[:,(self.shaft<.20)|(self.suspension>0)]=0

    def evaluate(self,positions,*,angle_degrees,overall,width,physics_state,
                 previous_length,pelvic_ramp_blend):
        p=np.array(positions,dtype=np.float32,copy=True)
        values=[angle_degrees,overall,width,previous_length,pelvic_ramp_blend]
        if (p.shape!=(2388,3) or not np.isfinite(p).all() or
            not np.isfinite(values).all() or overall<=0 or width<=0 or previous_length<=0 or
            physics_state not in (0,1,2) or not 0<=pelvic_ramp_blend<=1):
            raise ValueError('Invalid prepared source geometry, mode, dimensions or prior length')
        root=SOURCE_ROOT.astype(np.float32)
        a=np.float32(angle_degrees)*np.float32(np.pi)/np.float32(180)
        axis=np.array([np.cos(a),0,-np.sin(a)],dtype=np.float32)
        # Preserve the source's sequential float32 accumulation, including
        # pathological short profiles where tiny center changes alter flex.
        total=np.cumsum(self.weights,axis=1,dtype=np.float32)[:,-1]
        weighted=np.cumsum(self.weights[:,:,None]*p[None,:,:],axis=1,dtype=np.float32)[:,-1]
        centers=np.divide(weighted,total[:,None],out=np.zeros((18,3),dtype=np.float32),where=total[:,None]>1e-5)
        fallback=total<=1e-5;used_previous=bool(fallback.any())
        targets=np.arange(18,dtype=np.float32)/17
        centers[fallback]=root+axis*(np.float32(previous_length)*targets[fallback,None])
        centers[:,1]=0;centers[0]=root
        for _ in range(6):
            old=centers.copy()
            centers[1:-1]=old[1:-1]+((old[:-2]+old[2:])*.5-old[1:-1])*.34
        centers[0]=root
        if physics_state==0:
            previous=0.
            for i in range(1,18):
                axial=max(np.float32(previous)+np.float32(.10),np.dot(centers[i]-root,axis));previous=axial
                line=root+axis*axial
                t=np.float32(i)/np.float32(17)
                follow=np.float32(1)-_smoother32(max(np.float32(0),(t-np.float32(.07))/np.float32(.53)))
                centers[i]=centers[i]*(1-follow)+line*follow
        else:
            span=centers[-1]-root;span[1]=0
            if np.linalg.norm(span)<8:
                used_previous=True;span=axis*previous_length
            centers=root+targets[:,None]*span
        flex=closest_flex(centers,p)
        original=_smoother32(flex/np.float32(.18))
        follow=original+(self.root_follow-original)*(pelvic_ramp_blend*self.root_mask)
        selected=(self.shaft>=.72)&(self.suspension<=0)&(flex>=.30)&(flex<=.70)
        center,tangent=sample(centers,flex[selected]);offset=p[selected]-center
        radial=offset-tangent*np.sum(offset*tangent,axis=1)[:,None]
        weights=self.shaft[selected]**2
        mass=np.cumsum(weights,dtype=np.float32)[-1] if len(weights) else 0
        radius=float(np.cumsum(np.linalg.norm(radial,axis=1)*weights,dtype=np.float32)[-1]/mass) if mass>1e-5 else 2.52*(overall/1.5)*(width/1.15)
        length=float(np.clip(np.cumsum(np.linalg.norm(np.diff(centers,axis=0),axis=1),dtype=np.float32)[-1],8,60))
        return RestFrame(centers,flex,follow,radius,length,used_previous)
