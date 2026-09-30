"""Wolverine's authored coarse-shape stage, before its coupled fairing.

This is deliberately a stage, not the complete prepared/live surface. Native
adapters must not treat this result as a fully faired, collision-safe mesh.
The final UnifiedCollar evaluator remains in collar.py.
"""
from dataclasses import dataclass
import numpy as np
from .controls import SHAPE, defaults, mapped, read_preferences
from .collar import smoother

# These are the morph table's reference coordinates, NOT UI-50 preferences.
MORPH_REFERENCE = (
    (.5, 1.5, 2.5), (.6, 1., 2.), (.6, 1.15, 2.),
    (.6, 1.25, 2.), (-80., 0., 120.), (-2., 1., 6.), (-3., 0., 3.))
SOURCE_ROOT = np.array([9., 0., 84.3])


def shape_values(preferences):
    ui = read_preferences({'format': 'malemod.controls', 'version': 1, 'values': preferences})
    return ui, np.array([mapped(key, ui[key]) for key, *_ in SHAPE])


def collar_growth(overall, width):
    return float(np.clip((overall / 1.5) * (width / 1.15) - 1., 0., 1.5))


def precursor_collar(points, distances, growth, module_side):
    """Exact earlier ramp law; the final coupled solve must still follow it.

    Coordinates/distances are explicitly uncalibrated SOURCE coordinates.
    A target adapter must transform into this frame using its measured fit.
    """
    p = np.asarray(points, dtype=float).copy()
    distance = np.asarray(distances, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or distance.shape != (len(p),) or
            not np.isfinite(p).all() or not np.isfinite(distance).all() or
            np.any(distance < 0) or not np.isfinite(growth) or not 0 <= growth <= 1.5):
        raise ValueError('Invalid source collar coordinates, distances or growth')
    ramp = smoother((growth - .15) / 1.)
    lift = .46 + (.75 + .10 * ramp) * growth
    lateral = (.14 + .41 * ramp) * growth
    radius = (5. if module_side else 6.) + (3. if module_side else 5.) * growth
    influence = 1 - smoother(distance / radius)
    active = influence > .0001
    p[active, 0] += lift * influence[active]
    p[active, 1] *= 1 + lateral * influence[active]
    p[active, 2] = 84.3 + (p[active, 2] - 84.3) * (1 + .14 * growth * influence[active])
    upper = smoother((p[:, 2] - 80.8) / 8.2)
    fill_radius = (6.75 if module_side else 8.75) + (3.75 if module_side else 6.) * growth
    support = smoother(growth / 1.5) * upper * (1 - smoother(distance / max(.5, fill_radius)))
    active &= support > .0001
    p[active, 0] += .25 * growth * support[active]
    p[active, 1] *= 1 + .07 * growth * support[active]
    p[active, 2] = 84.3 + (p[active, 2] - 84.3) * (1 + .360 * growth * support[active])
    return p


def hang_offset(ui, overall, scrotum):
    # Hang is applied AFTER logical rest-section construction in the source.
    u = (ui - 50.) / (49. if ui < 50. else 50.)
    scale = overall / 1.5 * scrotum / 1.25
    return -u * (1. if u < 0 else 4.5) * np.sqrt(max(.35, scale))


@dataclass(frozen=True)
class AuthoredStage:
    coarse: np.ndarray
    growth: float
    hang_displacement: np.ndarray
    mapped_shape: np.ndarray
    omitted_stages: tuple = ('coarse coupled fairing', 'rest-frame regularization',
        'logical surface construction', 'egg rest fitting', 'rounded/raphe surfaces',
        'final UnifiedCollar', 'posed dynamics/contact', 'lighting rebuild')


class AuthoredShape:
    """Instance-owned source table evaluator. Does no per-frame work or I/O."""
    def __init__(self, bank):
        get = lambda prefix, name: np.array(bank[prefix + '__' + name], dtype=float, copy=True)
        self.base = get('morph_targets_faired', 'morph_base').reshape(-1, 3)
        n = len(self.base)
        if n != 2388:
            raise ValueError('Expected versioned 2388-point authored cage')
        self.targets = np.array([[get('morph_targets_faired', 'morph_ow_' + a + '_' + b).reshape(n, 3)
                                  for b in ['lo', 'def', 'hi']] for a in ['lo', 'def', 'hi']])
        self.morphs = {key: [get('morph_targets_faired', 'morph_' + key + '_' + side).reshape(n, 3)
                            for side in ['lo', 'hi']] for key, *_ in SHAPE}
        self.shaft = np.maximum(get('physics_weights', 'phys_shaft_weight'),
                                get('physics_weights', 'phys_attachment_weight'))
        self.ball = np.minimum(1., get('physics_weights', 'phys_scrotum_weight'))
        self.flex = get('physics_weights', 'phys_flex_coordinate')
        self.distances = get('collar_fairing', 'graftCollarDistances')
        self.angle_follow = get('pelvic_root_binding', 'pelvicAngleFollow')
        self.suspension = get('suspension_weights', 'suspensionWeight')
        if not all(np.isfinite(v).all() for v in [self.base, self.targets, self.shaft,
                    self.ball, self.flex, self.distances, self.angle_follow, self.suspension]):
            raise ValueError('Nonfinite authored shape bank')

    def overall_width(self, overall, width):
        indices, weights = [], []
        for v, spec in [(overall, MORPH_REFERENCE[0]), (width, MORPH_REFERENCE[2])]:
            low, neutral, high = spec
            i = 0 if v < neutral else 1
            a, b = (low, neutral) if i == 0 else (neutral, high)
            indices.append(i); weights.append((v - a) / (b - a))
        i, j = indices; a, b = weights
        return ((self.targets[i, j] * (1-b) + self.targets[i, j+1] * b) * (1-a) +
                (self.targets[i+1, j] * (1-b) + self.targets[i+1, j+1] * b) * a)

    def evaluate(self, preferences=None):
        ui, values = shape_values(preferences or defaults())
        overall, _, width, scrotum, angle, _, _ = values
        pouch = smoother((self.ball - .18) / .6) * smoother((self.ball - self.shaft + .18) / .7)
        p = (self.overall_width(overall, width) * (1-pouch[:, None]) +
             self.overall_width(overall, 1.59) * pouch[:, None])
        for index in [1, 3, 5, 6]:
            key = SHAPE[index][0]; value = values[index]
            low, neutral, high = MORPH_REFERENCE[index]
            if value < neutral:
                delta = (self.morphs[key][0] - self.base) * (neutral-value) / (neutral-low)
            else:
                delta = (self.morphs[key][1] - self.base) * (value-neutral) / (high-neutral)
            p += delta * (pouch[:, None] if index == 3 else 1)
        growth = collar_growth(overall, width)
        p = precursor_collar(p, self.distances, growth, True)
        membership = self.shaft * (1-self.ball)
        follow = membership.copy()
        selected = (self.ball < .05) & (self.shaft > .02)
        follow[selected] = 1 + (self.angle_follow[selected] - 1) * smoother((growth-.15)/1.)
        delta = np.deg2rad(angle) * follow
        c, s = np.cos(delta), np.sin(delta)
        x, z = p[:, 0]-9., p[:, 2]-84.3
        selected = follow > .001
        p[selected, 0] = (9. + c*x + s*z)[selected]
        p[selected, 2] = (84.3 - s*x + c*z)[selected]
        # Source's shallow bell precursor uses the POSED shaft frame.
        a = np.deg2rad(angle); c, s = np.cos(a), np.sin(a)
        x, z = p[:, 0]-9., p[:, 2]-84.3
        axial, radial = c*x-s*z, s*x+c*z
        influence = smoother(np.minimum(1., membership*2.5)) * (1-smoother(self.flex/.7))
        dorsal = smoother(np.clip(radial / max(.5, 2.52*(overall/1.5)*(width/1.15)), 0, 1))
        scale = 1 + (.042 + .035*growth*(1-.82*dorsal)) * influence
        selected = (membership > .01) & (influence > .001)
        radial *= scale
        p[selected, 1] *= scale[selected]
        p[selected, 0] = (9. + c*axial + s*radial)[selected]
        p[selected, 2] = (84.3 - s*axial + c*radial)[selected]
        hang = np.zeros_like(p)
        hang[:, 2] = hang_offset(ui['hang'], overall, scrotum) * self.suspension
        return AuthoredStage(p, growth, hang, values)
