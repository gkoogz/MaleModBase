"""Measured source generalized garment force ownership, through native lineage.

This is a force-distribution adapter, not the full derivative of the nonlinear
surface/collar deformation. It retains the authored shaft station/membership
and suspended-lobe fields through R14/rs/nr interpolation; no live bone names,
new anatomy, or guessed source length units are introduced.
"""
import numpy as np
from scipy.sparse import csr_matrix
from .motion_binding import source_lobe_partition

REACTION_BINDING_VERSION = 1

def reference_reaction_bindings(bank):
    source = np.asarray(bank['r14_asset__r14Reference']).reshape(-1, 3)
    shaft = np.maximum(bank['physics_weights__phys_shaft_weight'],
                       bank['physics_weights__phys_attachment_weight'])
    lobe = np.clip(bank['physics_weights__phys_scrotum_weight'], 0, 1)
    active = shaft.copy()
    active[(lobe < .05) & (shaft > .02)] = 1
    active = np.clip(active*(1-lobe), 0, 1)
    original = np.zeros((len(source), 14))
    coordinate = np.clip(bank['physics_weights__phys_flex_coordinate'], 0, 1)*11
    lower = np.floor(coordinate).astype(int)
    upper = np.minimum(lower+1, 11)
    fraction = coordinate-lower
    for vertex in range(len(source)):
        original[vertex, lower[vertex]] += active[vertex]*(1-fraction[vertex])
        original[vertex, upper[vertex]] += active[vertex]*fraction[vertex]
    original[:, 12:] = lobe[:, None]*source_lobe_partition(source[:, 1])
    rows = bank['r14_asset__r14Offsets']
    r14 = csr_matrix((bank['r14_asset__r14Weight'], bank['r14_asset__r14Sources'], rows),
                      shape=(len(rows)-1, len(source))) @ original
    # The logical authored shaft overrides mixed original skin ownership. Its
    # actual r14Flex and r14ShaftMask already drive the existing source surface.
    mask = np.clip(bank['r14_asset__r14ShaftMask'], 0, 1)
    coordinate = np.clip(bank['r14_asset__r14Flex'], 0, 1)*11
    r14 *= (1-mask[:, None])
    lower = np.floor(coordinate).astype(int)
    upper = np.minimum(lower+1, 11)
    fraction = coordinate-lower
    for vertex in range(len(r14)):
        r14[vertex, lower[vertex]] += mask[vertex]*(1-fraction[vertex])
        r14[vertex, upper[vertex]] += mask[vertex]*fraction[vertex]
    fine = bank['rounded_render_data__rsFineSource'].reshape(-1, 3)
    bary = bank['rounded_render_data__rsFineBary'].reshape(-1, 3)
    support = np.vstack([r14, (r14[fine]*bary[:, :, None]).sum(1)])
    direct = bank['neck_render_data__nrDirect']
    final = csr_matrix((bank['neck_render_data__nrWeights'], bank['neck_render_data__nrSources'],
                        bank['neck_render_data__nrRows']),
                       shape=(len(direct), len(support))) @ support
    retained = direct != 65535
    final[retained] = support[direct[retained]]
    return bounded_reaction_bindings(final)

def bounded_reaction_bindings(values):
    out = np.asarray(values, dtype=float).copy()
    if out.ndim != 2 or out.shape[1] != 14 or not np.isfinite(out).all():
        raise ValueError('Invalid generalized garment force lineage')
    # Sparse signed geometric interpolation may extrapolate scalar ownership.
    # Bound the transferred ownership, leaving unowned static tissue unforced.
    out = np.maximum(0, out)
    out /= np.maximum(1, out.sum(1))[:, None]
    return out

def reaction_bindings_header(values):
    values = bounded_reaction_bindings(values)
    lines = ['#pragma once', '// Generated measured source force-distribution binding version1.',
             'static const double jockstrapReactionBindings[][14]={']
    lines += ['{'+','.join(format(float(x), '.17g') for x in row)+'},' for row in values]
    lines += ['};']
    return '\n'.join(lines)+'\n'
