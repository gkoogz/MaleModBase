"""Reference-field transfer for small native motion cages.

This preserves the reference's interpolation lineage, including support points
that are not rendered. It is an authoring operation, not the Wolverine solver.
"""
import numpy as np
from scipy.sparse import csr_matrix

def sample_mechanical_guide(points, coordinate):
    """Source C1 guide and tangent; the first two guide points are kinematic."""
    p=np.asarray(points,dtype=float);t=np.asarray(coordinate,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<3 or not np.isfinite(p).all() or not np.isfinite(t).all():
        raise ValueError('Invalid mechanical guide')
    u=np.clip(t,0,1)*(len(p)-1);s=np.minimum(u.astype(int),len(p)-2);q=u-s;q2=q*q;q3=q2*q
    p0=p[s];p1=p[s+1]
    m0=np.where((s==0)[...,None],p1-p0,(p[s+1]-p[np.maximum(s-1,0)])*.5)
    m1=np.where((s+1==len(p)-1)[...,None],p1-p0,(p[np.minimum(s+2,len(p)-1)]-p[s])*.5)
    c=p0*(2*q3-3*q2+1)[...,None]+m0*(q3-2*q2+q)[...,None]+p1*(-2*q3+3*q2)[...,None]+m1*(q3-q2)[...,None]
    d=p0*(6*q2-6*q)[...,None]+m0*(3*q2-4*q+1)[...,None]+p1*(-6*q2+6*q)[...,None]+m1*(3*q2-2*q)[...,None]
    length=np.linalg.norm(d,axis=-1,keepdims=True)
    if np.any(length<1e-8):raise ValueError('Collapsed guide tangent')
    return c,d/length


def reference_fields(bank):
    """Transfer coarse mechanical fields through R14/support/final bindings.

    Signed geometric interpolation can overshoot a bounded weight. Clamp only
    the final scalar field and report that policy; do not modify geometry or
    discard the original signed donor tables.
    """
    source = np.column_stack([bank['physics_weights__phys_'+k] for k in
                              ('shaft_weight', 'scrotum_weight', 'flex_coordinate')])
    rows = bank['r14_asset__r14Offsets']
    r14 = csr_matrix((bank['r14_asset__r14Weight'], bank['r14_asset__r14Sources'], rows),
                     shape=(len(rows)-1, len(source))) @ source
    indices = bank['rounded_render_data__rsFineSource'].reshape(-1, 3)
    bary = bank['rounded_render_data__rsFineBary'].reshape(-1, 3)
    support = np.vstack([r14, (r14[indices]*bary[:, :, None]).sum(1)])
    direct = bank['neck_render_data__nrDirect']
    final = csr_matrix((bank['neck_render_data__nrWeights'],
                        bank['neck_render_data__nrSources'],
                        bank['neck_render_data__nrRows']), shape=(len(direct), len(support))) @ support
    retained = direct != 65535
    final[retained] = support[direct[retained]]
    if not np.isfinite(final).all():
        raise ValueError('Nonfinite reference motion binding')
    return np.clip(final, 0., 1.)


def linear_chain_weights(coordinate, knots):
    """Nonnegative two-bone partition on explicitly authored ordered knots."""
    x = np.asarray(coordinate, dtype=float)
    k = np.asarray(knots, dtype=float)
    if (x.ndim != 1 or k.ndim != 1 or len(k) < 2 or
            not np.isfinite(x).all() or not np.isfinite(k).all() or np.any(np.diff(k) <= 0)):
        raise ValueError('Expected finite coordinates and strictly increasing knots')
    x = np.clip(x, k[0], k[-1])
    left = np.minimum(np.searchsorted(k, x, side='right')-1, len(k)-2)
    alpha = (x-k[left])/(k[left+1]-k[left])
    weights = np.zeros((len(x), len(k)))
    weights[np.arange(len(x)), left] = 1-alpha
    weights[np.arange(len(x)), left+1] = alpha
    return weights


def reference_cage_weights(fields, lateral, half_width, seam_distance, blend_distance):
    """Eight axial joints and two lobes; distances use the caller's measured frame.

    The caller supplies signed lateral coordinates and exact seam distances.
    Zero seam distance gives zero new influence. Original donor bindings remain
    authoritative for UV aliases and should be applied again after quantization.
    """
    fields=np.asarray(fields,dtype=float);lateral=np.asarray(lateral,dtype=float)
    distance=np.asarray(seam_distance,dtype=float)
    if (fields.ndim!=2 or fields.shape[1]!=3 or lateral.shape!=(len(fields),) or
            distance.shape!=(len(fields),) or half_width<=0 or blend_distance<=0 or
            not np.isfinite([half_width,blend_distance]).all() or
            not np.isfinite(fields).all() or not np.isfinite(lateral).all() or
            not np.isfinite(distance).all() or np.any(distance<0)):
        raise ValueError('Invalid motion cage frame or fields')
    fields=np.clip(fields,0,1)
    shaft=linear_chain_weights(fields[:,2],np.linspace(0,1,8))*fields[:,0,None]
    side=np.clip((lateral+half_width)/(2*half_width),0,1)
    weights=np.column_stack([shaft,fields[:,1]*(1-side),fields[:,1]*side])
    weights/=np.maximum(weights.sum(1)[:,None],1)
    fade=np.clip(distance/blend_distance,0,1);fade=fade*fade*(3-2*fade)
    return weights*fade[:,None]


def reference_cage_centres(points, fields, root, lateral, lobe_exclusion):
    """Author small native-rig rest centres from transferred reference fields.

    These centres are an approximation for native secondary-motion backends;
    they do not replace the authored Wolverine simulation cage.
    """
    points=np.asarray(points,dtype=float);fields=np.asarray(fields,dtype=float)
    lateral=np.asarray(lateral,dtype=float);root=np.asarray(root,dtype=float)
    if (points.shape!=(len(fields),3) or fields.shape!=(len(points),3) or
            lateral.shape!=(len(points),) or root.shape!=(3,) or lobe_exclusion<0 or not np.isfinite(lobe_exclusion) or
            not all(np.isfinite(x).all() for x in [points,fields,lateral,root])):
        raise ValueError('Invalid reference cage inputs')
    centres=[root]
    masks=[(abs(fields[:,2]-t)<.075)&(fields[:,0]>.5) for t in np.linspace(0,1,8)[1:]]
    masks += [(fields[:,1]>.8)&(lateral*side>lobe_exclusion) for side in [-1,1]]
    for mask in masks:
        if mask.sum()<8:raise ValueError('Insufficient field support for motion cage')
        centres.append(points[mask].mean(0))
    return np.asarray(centres)
