# Generic adult male reference

`python tools/build_generic.py` produces `assets/generic-male/reference.glb`
and a matching JSON contract. The GLB contains a neutral adult male proxy,
25 semantic joints, four attachment sockets and normalized skin weights.
It is a real 3D asset made from overlapping ellipsoid surfaces, not a welded,
finished character. The pelvic socket is an attachment transform, not a hole.

Authoring coordinates are right handed, meters, +Z up and -Y forward. The GLB
writer converts to glTF's +Y up coordinate system, including inverse bind
matrices. Do not apply the authoring axis transform twice.

The procedural body supports height, shoulders, chest, glutes and hips. The
`limb_length` prototype currently changes arm reach only. Facial controls and
garments are reserved, not implemented. Preferences live in
`profiles/default-preferences.json`; the anatomy module describes its own
dimensions separately from the proxy. `modules/wolverine-anatomy.json` points
to the preserved anatomy geometry and records unresolved source calibration.

## Shared evaluation

- `character.hpp`: linear skinning and semantic attachment transforms.
- `collision.hpp`: analytical capsule projection for adapter-supplied hitboxes.
- `physics/xpbd_kernels.hpp`: extracted Wolverine distance and damped bending
  formulas with explicit per-instance state; extraction recipe and hashes are
  in `tools/extract_physics.py` and `provenance/physics-kernels.json`.
- `physics/chain.hpp`: a generic fixed-step chain built from these kernels.
  It preallocates state and supports cached contact projections. It is not the
  original coupled lobe, collar, suspension and seam solver.
- `clinical/`: shared CPU sequence, fluid, deposition and cue timing.
- `surface/collar_field.hpp` and `malemod_base/collar.py`: source-derived
  recruitment metric and offline coupled body/attachment solve with hard
  original-edge seam constraints. See PELVIC-COLLAR.md for calibration, source
  omissions and adoption back to each game spoke.

Kernel arithmetic is checked against the archived source over randomized
fixtures. Fixed-step chain checks cover frame rates, hitches and instance
isolation. These are offline checks, not proof of Witcher gameplay performance.

## Adapter boundary

A new game must supply observed native joints and bind transforms, a measured
coordinate/scale conversion, actual mesh/material export and import, collision
capabilities, input integration and packaging. Native C++ headers cannot simply
be loaded into WitcherScript. Record a feature as unknown until an implemented
bridge is tested. A baked mesh is a useful first path, but does not run the
shared fluid solver or offer arbitrary live vertex edits by itself.

Keep the proprietary stock character and full depot local. Commit the recipes,
source hashes, observed profile and our authored adapter. Place common body
changes or new garment work in Base before implementing engine translation.
