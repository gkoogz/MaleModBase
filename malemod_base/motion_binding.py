"""Reference-field transfer for small native motion cages.

This preserves the reference's interpolation lineage, including support points
that are not rendered. It is an authoring operation, not the Wolverine solver.
"""
import numpy as np
from scipy.sparse import csr_matrix


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
