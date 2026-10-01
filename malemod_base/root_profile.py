"""Source angular collar/shaft regularization on prepared source coordinates.

This stage does not construct a body weld or replace the final UnifiedCollar
solve. Callers supply the measured rest frame and explicit source collar growth.
"""
from dataclasses import dataclass
import numpy as np
from .rest_frame import sample, _unit, _smoother32

F=np.float32


@dataclass(frozen=True)
class RootProfile:
    positions: np.ndarray
    sector_radius: np.ndarray
    corrected: np.ndarray


class SourceRootProfile:
    def __init__(self,bank):
        def get(prefix,name):return np.array(bank[prefix+'__'+name],dtype=np.float32,copy=True)
        self.shaft=np.maximum(get('physics_weights','phys_shaft_weight'),get('physics_weights','phys_attachment_weight'))
        self.ball=np.minimum(F(1),get('physics_weights','phys_scrotum_weight'))
        self.suspension=get('suspension_weights','suspensionWeight')
        if any(x.shape!=(2388,) or not np.isfinite(x).all() for x in [self.shaft,self.ball,self.suspension]):
            raise ValueError('Expected finite versioned 2388-point source fields')

    def evaluate(self,positions,frame,*,collar_growth):
        p=np.array(positions,dtype=np.float32,copy=True)
        centers=np.asarray(frame.centers,dtype=np.float32)
        t=np.asarray(frame.flex,dtype=np.float32)
        follow=np.asarray(frame.root_follow,dtype=np.float32)
        if (p.shape!=(2388,3) or centers.shape!=(18,3) or t.shape!=(2388,) or follow.shape!=(2388,) or
            not all(np.isfinite(x).all() for x in [p,centers,t,follow]) or not np.isfinite(collar_growth) or
            not 0<=collar_growth<=1.5 or not np.isfinite(frame.body_radius) or frame.body_radius<=0):
            raise ValueError('Invalid source positions/rest frame/radius/collar growth')
        center,tangent=sample(centers,t)
        offset=p-center
        radial=offset-tangent*np.sum(offset*tangent,axis=1,dtype=np.float32)[:,None]
        radius=np.linalg.norm(radial,axis=1)
        lateral=np.zeros_like(tangent);lateral[:,1]=1
        lateral=_unit(lateral-tangent*np.sum(lateral*tangent,axis=1)[:,None])
        vertical=_unit(np.cross(tangent,lateral))
        angle=np.arctan2(np.sum(radial*vertical,axis=1),np.sum(radial*lateral,axis=1))
        angle=np.where(angle<0,angle+F(6.283185307),angle)
        u=angle*F(24/F(6.283185307))
        sector=np.minimum(23,u.astype(int))
        stable=(t>=F(.52))&(t<=F(.72));collar=(t>=F(.025))&(t<=F(.16))
        eligible=(self.shaft>=F(.62))&(self.ball<=F(.05))&(radius>=F(1e-4))
        weights=self.shaft*self.shaft*(1-self.ball)*(1-self.ball)
        sums=np.zeros(24,dtype=np.float32);mass=sums.copy();collar_sums=sums.copy();collar_mass=sums.copy()
        # Source order matters: sectors are sparse, and their fallback average
        # must use the same original vertex order rather than a parallel reduce.
        total=F(0);total_mass=F(0)
        for i in np.flatnonzero(eligible&(stable|collar)):
            s=sector[i];value=radius[i]*weights[i]
            if stable[i]:
                sums[s]+=value;mass[s]+=weights[i];total+=value;total_mass+=weights[i]
            if collar[i]:collar_sums[s]+=value;collar_mass[s]+=weights[i]
        fallback=total/total_mass if total_mass>F(1e-5) else F(frame.body_radius)
        profile=np.divide(sums,mass,out=np.full(24,fallback,dtype=np.float32),where=mass>F(1e-5))
        for s in range(24):
            if mass[s]>F(1e-5):continue
            value=F(0);weight=F(0)
            for distance in range(1,12):
                for neighbor in ((s-distance+24)%24,(s+distance)%24):
                    if mass[neighbor]>F(1e-5):value+=profile[neighbor]/F(distance);weight+=F(1)/F(distance)
                if weight>0 and distance>=3:break
            if weight>0:profile[s]=value/weight
        owner=follow*(1-_smoother32((t-F(.80))/F(.06)))*(1-self.suspension)
        owner[(self.shaft<F(.08))|(self.suspension>=1)|(t<0)|(t>F(.86))]=0
        active=(t<=F(.60))&(self.shaft>=F(.08))&(self.ball<F(.50))&(owner>F(.0001))&(radius>=F(1e-4))
        ids=np.flatnonzero(active)
        a=np.floor(u[ids]).astype(int)%24;b=(a+1)%24;q=u[ids]-np.floor(u[ids]);q=q*q*(3-2*q)
        reference=profile[a]*(1-q)+profile[b]*q
        ca=np.divide(collar_sums[a],collar_mass[a],out=reference.copy(),where=collar_mass[a]>F(1e-5))
        cb=np.divide(collar_sums[b],collar_mass[b],out=reference.copy(),where=collar_mass[b]>F(1e-5))
        growth=_smoother32(F(collar_growth)/F(1.5))
        root_reference=np.maximum(reference*(1+(F(.012)+F(.030)*growth)),(ca*(1-q)+cb*q)*F(.985))
        desired=reference+(root_reference-reference)*(1-_smoother32((t[ids]-F(.055))/F(.505)))
        change=np.clip(desired-radius[ids],-reference*F(.12),reference*F(.12))
        blend=owner[ids]*_smoother32(t[ids]/F(.14))*(F(.42)+F(.54)*growth)*(1-_smoother32((t[ids]-F(.50))/F(.10)))
        p[ids]+=radial[ids]*(change*blend/radius[ids])[:,None]
        return RootProfile(p,profile,np.any(p!=np.asarray(positions,dtype=np.float32),axis=1))
