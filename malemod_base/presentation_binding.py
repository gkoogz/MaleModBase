"""Material-frame presentation bindings from preserved mechanical lineage."""
import numpy as np


def presentation_bindings(fields,points,lobe_centers,amount):
    fields=np.asarray(fields,dtype=float);points=np.asarray(points,dtype=float);lobes=np.asarray(lobe_centers,dtype=float);amount=np.asarray(amount,dtype=float)
    if fields.shape!=(len(points),3) or points.shape!=(len(points),3) or lobes.shape!=(2,3) or amount.shape!=(len(points),) or not all(np.isfinite(x).all() for x in [fields,points,lobes,amount]) or np.any((fields<0)|(fields>1)) or np.any((amount<0)|(amount>1)):
        raise ValueError('Invalid material presentation fields')
    frame=1+np.floor(fields[:,2]*11+.5).astype(np.uint32)
    # One material frame for the distal cap, preventing a cross-section from
    # shrinking or shearing when displayed between complete solver surfaces.
    frame[fields[:,2]>=.82]=12
    side=np.linalg.norm(points[:,None,:]-lobes[None,:,:],axis=2).argmin(1)
    ball=fields[:,1]>.15;frame[ball]=13+side[ball]
    frame[amount==0]=0
    return frame,amount.copy()
