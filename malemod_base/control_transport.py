"""Normalized size transport for a small semantic bone cage.

This is an explicit cage approximation, not the source authored surface solver.
Defaults leave an already fitted rest asset unchanged. Native axes, names and
the assignment of these factors to joints belong to each adapter.
"""
from .controls import defaults, mapped, validate

SIZE_CONTROLS=('overall','length','width','glans','scrotum')


def normalized_samples(key):
    if key not in SIZE_CONTROLS:raise ValueError('Unsupported cage size control')
    neutral=mapped(key,defaults()[key])
    low=0 if key in ('length','glans') else 1
    return tuple(mapped(key,ui)/neutral for ui in range(low,101))


def size_factors(values=None):
    ui=defaults()
    for key,value in (values or {}).items():
        if key not in SIZE_CONTROLS:raise ValueError('Unsupported cage size control')
        ui[key]=validate(key,value)
    ratios={key:mapped(key,ui[key])/mapped(key,defaults()[key]) for key in SIZE_CONTROLS}
    return dict(axial=ratios['overall']*ratios['length'],
        radial=ratios['overall']*ratios['width'],head=ratios['glans'],
        lobes=ratios['overall']*ratios['scrotum'])
