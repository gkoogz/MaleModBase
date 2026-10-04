"""Anatomical guide membership from the preserved authored surface lineage.

This describes the Wolverine source asset, not a cloth shape law or engine
skeleton. Adapters map these scalar memberships through their measured native
lineage once; all walks use the complete current anatomy surface afterwards.
"""
import numpy as np
from scipy.sparse import csr_matrix
from .motion_binding import source_lobe_partition

REGIONS = ('shaft', 'glans', 'leftLobe', 'rightLobe')
SEMANTIC_VERSION = 1
ROOT_BOUNDARY_VERSION = 1


def full_source_root_boundary(positions, triangles):
    """Ordered existing native IDs on the complete welded source opening.

    Positional aliases are exact preserved float32 source vertices. This is
    source asset extraction; no cap vertices, geometry or current-axis test
    is generated. The old thirteen coarse collar support IDs omit forty-one
    refined native boundary vertices required for exact volume closure.
    """
    points = np.asarray(positions)
    faces = np.asarray(triangles)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Invalid source root positions')
    if faces.ndim != 2 or faces.shape[1] != 3 or not np.issubdtype(faces.dtype, np.integer):
        raise ValueError('Invalid source root topology')
    if not len(faces) or faces.min() < 0 or faces.max() >= len(points):
        raise ValueError('Source root triangle out of bounds')
    _, first, aliases = np.unique(points, axis=0, return_index=True, return_inverse=True)
    edges = {}
    for face in aliases[faces]:
        if len(set(map(int, face))) != 3:
            continue
        for a, b in zip(face, np.roll(face, -1)):
            a, b = int(a), int(b)
            key = tuple(sorted((a, b)))
            directed = edges.setdefault(key, [])
            directed.append((a, b))
            if len(directed) > 2 or (len(directed) == 2 and directed[0] != (b, a)):
                raise ValueError('Source root nonmanifold or inconsistent winding')
    next_vertex, incoming = {}, set()
    for rows in edges.values():
        if len(rows) == 1:
            a, b = rows[0]
            if a in next_vertex or b in incoming:
                raise ValueError('Source root boundary branches')
            next_vertex[a] = b
            incoming.add(b)
    if not 8 <= len(next_vertex) <= 128:
        raise ValueError('Source root boundary outside measured opening budget')
    start = min(next_vertex, key=lambda i: int(first[i]))
    ordered, seen, at = [], set(), start
    while at not in seen:
        seen.add(at)
        ordered.append(int(first[at]))
        if at not in next_vertex:
            raise ValueError('Source root boundary is open')
        at = next_vertex[at]
    if at != start or len(seen) != len(next_vertex):
        raise ValueError('Source root has multiple or incomplete boundary loops')
    return np.asarray(ordered, dtype=np.uint32)


def reference_anatomy_semantics(bank):
    """Return final native-source memberships [shaft,glans,lobe0,lobe1].

    The 92x96 authored axial rings begin at R14 vertex1721; crown ring34
    starts the glans. These are existing r14_asset.h identifiers, not a
    threshold inferred from a deformed mesh. Lobe ownership retains the
    original scrotal physics field and its source suspension side partition.
    Signed geometric interpolation is preserved, then only scalar membership
    is clamped, as in the existing reference_fields authoring contract.
    """
    base = np.asarray(bank['r14_asset__r14Base']).reshape(-1, 3)
    rows = bank['r14_asset__r14Offsets']
    original = np.column_stack([bank['physics_weights__phys_shaft_weight'],
                                bank['physics_weights__phys_scrotum_weight']])
    mechanical = csr_matrix((bank['r14_asset__r14Weight'],
                             bank['r14_asset__r14Sources'], rows),
                            shape=(len(rows)-1, len(original))) @ original
    if len(base) != 10554 or len(rows) != len(base)+1:
        raise ValueError('Authored R14 semantic topology differs')
    r14 = np.zeros((len(base), 4))
    r14[:, 0] = mechanical[:, 0]
    r14[1721:, 0] = 1.
    r14[1721+34*96:, 1] = 1.
    r14[:, 2:] = mechanical[:, 1, None] * source_lobe_partition(base[:, 1])
    fine = bank['rounded_render_data__rsFineSource'].reshape(-1, 3)
    bary = bank['rounded_render_data__rsFineBary'].reshape(-1, 3)
    support = np.vstack([r14, (r14[fine]*bary[:, :, None]).sum(1)])
    direct = bank['neck_render_data__nrDirect']
    final = csr_matrix((bank['neck_render_data__nrWeights'],
                        bank['neck_render_data__nrSources'],
                        bank['neck_render_data__nrRows']),
                       shape=(len(direct), len(support))) @ support
    retained = direct != 65535
    final[retained] = support[direct[retained]]
    if not np.isfinite(final).all():
        raise ValueError('Nonfinite anatomical guide lineage')
    final = np.clip(final, 0., 1.)
    # Glans is a distinct authored region of the axial surface, not a second
    # shaft label. This changes membership only; every original vertex remains.
    final[:, 0] *= 1-final[:, 1]
    return final


def anatomy_regions(fields):
    """Stable disjoint guide index groups; full mesh/triangles stay intact."""
    values = np.asarray(fields, dtype=float)
    if (values.ndim != 2 or values.shape[1] != 4 or
            not np.isfinite(values).all() or np.any(values < 0) or
            np.any(values > 1)):
        raise ValueError('Invalid anatomical guide memberships')
    dominant = values.argmax(1)
    measured = values.max(1) > .15
    groups = {name: np.flatnonzero(measured & (dominant == i)).astype(np.uint32)
              for i, name in enumerate(REGIONS)}
    if any(len(indices) < 8 for indices in groups.values()):
        raise ValueError('Anatomical guide region has insufficient source support')
    return groups


def anatomy_regions_header(groups):
    """Native adapter data only; no solver or geometry code is generated."""
    lines = ['#pragma once', '// Generated from exact authored R14/rs/nr semantic lineage; version1.']
    for name in REGIONS:
        values = [str(int(i)) for i in groups[name]]
        lines.append('static const unsigned jockstrap_region_'+name+'[]={')
        lines.extend(','.join(values[first:first+24])+',' for first in range(0, len(values), 24))
        lines.append('};')
    return '\n'.join(lines)+'\n'
