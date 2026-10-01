"""Source-derived material coefficients for the active coupled solver.

These are uncalibrated Wolverine parameters, not REDengine dangle properties.
The caller supplies the smoothed mechanical mode and measured rest length.
Contacts, rest guides, mass distribution and a numerical solver remain separate.
"""
from dataclasses import dataclass
import math
from .controls import read_preferences, mapped, smoother, finite


@dataclass(frozen=True)
class PhysicsControls:
    shaft_mass: float
    lobe_mass: float
    shaft_drag: float
    lobe_drag: float
    shaft_motion_response: float
    lobe_motion_response: float
    shaft_gravity: float
    lobe_gravity: float
    shaft_bend_compliance: float
    shaft_bend_damping_ratio: float
    suspension_compliance: float
    suspension_shear_compliance: float
    suspension_damping_ratio: float
    lobe_torsion: float
    root_mass: float
    root_stiffness: float
    root_damping: float
    root_droop: float
    root_motion_drive: float
    root_pitch_limit: float


def mode_value(mode, rigid, intermediate, flexible):
    mode = finite(mode)
    if not 0 <= mode <= 2:
        raise ValueError('Smoothed mechanical mode must be between zero and two')
    return (rigid + (intermediate-rigid)*smoother(mode) if mode <= 1 else
            intermediate + (flexible-intermediate)*smoother(mode-1))


def evaluate(preferences, *, mode, rest_length):
    """Evaluate the source's raw-UI and mapped-value laws independently.

    Rest length must be in the same explicit source space as the rest guide.
    An adapter must apply a documented unit conversion before using these laws
    in another physical coordinate system. This function infers no SI scale.
    """
    ui = read_preferences(preferences)
    rest_length = finite(rest_length)
    if rest_length <= 0:
        raise ValueError('A measured positive rest length is required')
    mv = lambda a, b, c: mode_value(mode, a, b, c)
    shaft_weight = mapped('shaft_weight', ui['shaft_weight'])
    lobe_weight = mapped('scrotum_weight', ui['scrotum_weight'])
    stiffness = ui['shaft_stiffness']/100
    lobe_stiffness = ui['scrotum_stiffness']/100
    root_mass = (1+shaft_weight*.016)*max(.65, math.sqrt(rest_length/24))
    root_stiffness = mv(38, 22, 10)
    return PhysicsControls(
        shaft_mass=.75+shaft_weight*.0125,
        lobe_mass=.75+lobe_weight*.0125,
        shaft_drag=.9+(100-ui['shaft_bounce'])*.018,
        lobe_drag=1+(100-ui['scrotum_bounce'])*.025,
        shaft_motion_response=.65+mapped('shaft_velocity', ui['shaft_velocity'])*.009,
        lobe_motion_response=.65+mapped('scrotum_velocity', ui['scrotum_velocity'])*.009,
        shaft_gravity=mv(5, 42, 110), lobe_gravity=72,
        shaft_bend_compliance=mv(.00000001, .00008, .0015)*math.exp((.5-stiffness)*3),
        shaft_bend_damping_ratio=.10+.40*(1-ui['shaft_bounce']/100),
        suspension_compliance=.000053*math.exp(-3*lobe_stiffness),
        suspension_shear_compliance=.000022*math.exp((.5-lobe_stiffness)*3),
        suspension_damping_ratio=.10+.60*(1-ui['scrotum_bounce']/100),
        lobe_torsion=20*math.exp((lobe_stiffness-.5)*2),
        root_mass=root_mass, root_stiffness=root_stiffness,
        root_damping=2*math.sqrt(root_stiffness*root_mass)*mv(.60, .72, .86),
        root_droop=mv(.015, .11, .28), root_motion_drive=mv(7, 9, 12),
        root_pitch_limit=mv(.42, .62, .92))


def suspension_limits(measured_rest_distance, measured_lobe_radius_y):
    """Source tether limits using caller-observed rest geometry."""
    distance = finite(measured_rest_distance)
    radius = finite(measured_lobe_radius_y)
    if distance < 0 or radius <= 0:
        raise ValueError('Invalid measured suspension geometry')
    rest = max(2.45, distance)
    return rest, rest*1.12 + radius*.15
