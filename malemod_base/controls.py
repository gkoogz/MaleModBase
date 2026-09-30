"""Versioned, engine-neutral controls from Wolverine's accepted live preset.

Values are source control parameters, not SI units or engine capabilities.
Adapters must explicitly report which controls have a working output path.
"""
import math

VERSION = 1
SHAPE = (
    ('overall', 'Overall size', .85, 1.2, 2.5),
    ('length', 'Length', .40, 1.6, 2.4),
    ('width', 'Width', .95, 1.59, 2.0),
    ('scrotum', 'Scrotum size', 1.0, 1.53, 2.0),
    ('angle', 'Rest angle', -80.0, 30.0, 120.0),
    ('forward', 'Forward offset', -2.0, -.7, 6.0),
    ('vertical', 'Vertical offset', -3.0, .400001, 3.0),
)
PHYSICS = (
    ('shaft_stiffness', 'Shaft stiffness', -100.0, 78.0, 400.0),
    ('shaft_weight', 'Shaft weight', 0.0, 86.0, 100.0),
    ('shaft_bounce', 'Shaft bounce', 0.0, 12.0, 100.0),
    ('shaft_velocity', 'Shaft motion response', 10.0, 62.0, 200.0),
    ('scrotum_stiffness', 'Scrotum stiffness', 0.0, 28.0, 100.0),
    ('scrotum_weight', 'Scrotum weight', 0.0, 94.0, 100.0),
    ('scrotum_bounce', 'Scrotum bounce', 0.0, 18.0, 100.0),
    ('scrotum_velocity', 'Scrotum motion response', 10.0, 72.0, 200.0),
)
ORDER = ('state', 'overall', 'length', 'width', 'glans', 'scrotum',
         'hang', 'angle', 'forward', 'vertical') + tuple(x[0] for x in PHYSICS)
STATES = ('rigid', 'intermediate', 'flexible')


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Control must be a finite number')
    return float(value)


def limits(key):
    if key not in ORDER:
        raise KeyError(key)
    return (0, 2) if key == 'state' else (0 if key in ('length', 'glans') else 1, 100)


def validate(key, value):
    value = finite(value)
    low, high = limits(key)
    if not low <= value <= high or (key == 'state' and value != int(value)):
        raise ValueError('Control outside declared range: ' + key)
    return value


def smoother(value):
    t = max(0.0, min(1.0, value))
    return t*t*t*(t*(t*6.0-15.0)+10.0)


def centered(ui, low, neutral, high):
    ui = max(1.0, ui)
    return low + (neutral-low)*(ui-1.0)/49.0 if ui <= 50 else neutral+(high-neutral)*(ui-50.0)/50.0


def mapped(key, ui):
    """Match source mapping, including length extension and glans v5 response."""
    ui = validate(key, ui)
    if key == 'state':
        return int(ui)
    if key == 'length':
        if ui < 50:
            return .40 + 1.20*smoother(ui/50.0)
        t = (ui-50.0)/50.0
        return 1.6 + .4*t + .4*smoother(t)
    if key == 'glans':
        old = ui*.70 if ui <= 50 else 35.0+(ui-50.0)*1.20
        return .82+.18*smoother(old/50.0) if old < 50 else centered(max(0.0, (old-50.0)*2), 1.0, 1.40, 1.60)
    if key == 'hang':
        # Source consumes Hang as normalized UI; do not invent physical units.
        return ui
    for name, _, low, neutral, high in SHAPE + PHYSICS:
        if name == key:
            return centered(ui, low, neutral, high)
    raise KeyError(key)


def defaults():
    return {key: 2 if key == 'state' else 50.0 for key in ORDER}


def read_preferences(document):
    if document.get('format') != 'malemod.controls' or document.get('version') != VERSION:
        raise ValueError('Unsupported control preference version')
    values = document.get('values')
    if not isinstance(values, dict) or set(values) - set(ORDER):
        raise ValueError('Unknown control preference')
    result = defaults()
    for key, value in values.items():
        result[key] = validate(key, value)
    return result


def catalog():
    labels = {x[0]: x[1] for x in SHAPE + PHYSICS}
    labels.update(state='Mechanical state', glans='Glans size', hang='Rest hang')
    return {'format': 'malemod.controls', 'version': VERSION,
            'sourceControlScaleVersion': 5,
            'sourceUnits': 'uncalibrated source parameters; not SI',
            'controls': [dict(id=k, label=labels[k], minimum=limits(k)[0], maximum=limits(k)[1],
                              default=defaults()[k], step=1,
                              mapping=[mapped(k, i) for i in range(limits(k)[0], limits(k)[1]+1)])
                         for k in ORDER]}
