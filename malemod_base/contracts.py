"""Validate portable preferences and explicit target-character observations."""
import math

BODY_CONTROLS={'height','shoulders','chest','glutes','hips','limb_length'}
ANATOMY_CONTROLS={'length_m','radius_m','compliance','damping'}

def finite(value):
    return type(value) in (int,float) and math.isfinite(value)

def validate_preferences(profile):
    if profile.get('contractVersion')!=1: raise ValueError('Unsupported preference contract')
    for section,allowed in [('body',BODY_CONTROLS),('anatomy',ANATOMY_CONTROLS)]:
        values=profile.get(section,{})
        if not isinstance(values,dict) or set(values)-allowed: raise ValueError(f'Unknown {section} controls')
        for name,value in values.items():
            if not finite(value): raise ValueError(f'{section}.{name} must be finite')
            if section=='body' and not 0<=value<=1: raise ValueError(f'{name} must be 0..1')
            if section=='anatomy' and value<0: raise ValueError(f'{name} must be nonnegative')
        if section=='anatomy' and (values.get('length_m',.2)<=0 or values.get('radius_m',.02)<=0):
            raise ValueError('Anatomy dimensions must be positive')
    if not isinstance(profile.get('clothing',[]),list) or any(not isinstance(x,str) for x in profile.get('clothing',[])):
        raise ValueError('Clothing must be a list of module IDs')
    return profile

def validate_character(profile, required_joints=()):
    """Missing observations remain missing rather than guessed from joint names."""
    if profile.get('contractVersion')!=1: raise ValueError('Unsupported character contract')
    if profile.get('metersPerUnit') is None: raise ValueError('Character unit calibration is unresolved')
    if not finite(profile['metersPerUnit']) or profile['metersPerUnit']<=0: raise ValueError('Invalid unit calibration')
    mapping=profile.get('jointMapping',{})
    if not isinstance(mapping,dict) or any(not isinstance(v,str) or not v for v in mapping.values()):
        raise ValueError('Joint mappings must name observed native joints')
    missing=set(required_joints)-set(mapping)
    if missing: raise ValueError('Unresolved required joints: '+', '.join(sorted(missing)))
    native=profile.get('observedNativeJoints',[])
    if not native or set(mapping.values())-set(native): raise ValueError('Mapping references an unobserved native joint')
    capabilities=profile.get('capabilities',{})
    for name,state in capabilities.items():
        if state not in {'verified','unavailable','unknown'}: raise ValueError(f'Invalid capability status: {name}')
    return profile

def classify_feature(feature):
    """Mechanical allocation rule for an adapter intake, not a policy oracle."""
    shared={'geometry','morph','preferences','garment','physics','timeline','fluid','attachment'}
    engine={'asset_import','native_joint_mapping','material_translation','collision_query','pose_sampling','packaging','input_binding'}
    if feature in shared:return 'base'
    if feature in engine:return 'adapter'
    raise ValueError('Classify the proposed behavior explicitly before adding an unknown subsystem')
