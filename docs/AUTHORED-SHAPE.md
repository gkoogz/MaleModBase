# Source shape stages and the Witcher slider port

This work supports the full 18-control port requested September 30. It is not a
claim that all controls work in Witcher. Animation, fluid and audio remain deferred.

`malemod_base/authored_shape.py` evaluates the beginning of Wolverine's active
`BuildPreparedShape`: the 3x3 overall/width authored grid, pouch ownership, the
length/scrotum/forward/vertical morphs, early pelvic ramp, angle and shallow
attachment flare. Hang displacement is returned separately because the source
applies it after rest-section construction. No game SDK or graphics API is used.

Do not confuse UI-50 preferences with morph table reference coordinates. In
particular, the grid reference overall/width is 1.5/1.15; UI-50 maps to 1.2/1.59.
The early collar growth uses the former denominators. The original length table
reference is 1.0 even though UI-50 maps to 1.6. These differences are intentional
source behavior, verified by an original-code oracle rather than guessed scaling.

The `AuthoredStage.omitted_stages` field is part of the API. Coarse fairing,
logical sections, egg fitting, glans/refined render transport, final UnifiedCollar
integration and posed physics are not silently replaced with approximate outputs.
Never upload this early stage alone and describe it as complete runtime parity.

## Reconstructing a fitted character's coupled pelvis

`malemod_base/graft_collar.py` builds a combined body/module domain from the
versioned rest graft bindings. It preserves original body-edge donors, welds
their fine body and module aliases, uses the source-derived final `CollarPlan`,
and locks caller-supplied native part boundaries. The adapter supplies an explicit
measured coordinate transform and actual logical frame/guide samples.

`solve_guided` requires real guide samples; none are fabricated. `solve_checked`
adds the continuous conservative projected-area correction bound from Wolverine's
`surface_limit.h`. Limiting the entire coupled correction retains the hard linear
seam equations and boundary locks. Its returned fraction must be recorded: zero
flipped triangles does not prove the desired shape was reached. Local refinement
or better targets may be required when that fraction is small.

The Witcher synthetic target probe preserves seams and existing waist/ankle
boundaries in both fitted LODs. The raw solve initially flipped triangles; the
checked solve removes them but accepts only about 17–36% of that synthetic
correction. This is an offline fixture, not the real Raphe-driven source target,
not evidence of live expansion, and not an acceptable full-range port envelope.

## Verification and reproducibility

`tests/authored_shape_oracle.cpp` uses a verbatim source loop and source functions
extracted by `tools/extract_authored_shape.py`. The checked-in CSV contains 1,300
samples covering single endpoints and combined extremes. Compile the oracle
using the CMake target `authored_shape_oracle`, then regenerate its CSV:

```powershell
python tools/build_shape_fixture.py build/path/to/authored_shape_oracle.exe
python tools/extract_authored_shape.py
python tools/extract_collar.py
python -m unittest discover -s tests -v
python tools/verify.py
```

`provenance/authored-shape.json` hashes the immutable source, implementation,
fixture and generation tool. Collar provenance includes the graft-domain wrapper,
boundary locks and Python area limiter. Imported source files remain unchanged.

## Spoke adoption

- Wolverine can adopt the original-code oracle, control/reference-coordinate
  distinction, protected-boundary extension and continuous seam tests first.
  Keep its active full solver authoritative until full posed comparison passes.
- Witcher consumes these modules through its exact Base pin. Native graph authoring,
  pose inheritance, small-cage output, skinning, inputs/menu and packaging remain
  in the adapter. The isolated native scale graph cooks; gameplay is unverified.
- Future spokes retain the same control IDs and source stages, supplying observed
  character bindings and reporting supported output capabilities explicitly.

The eight source physics sliders remain tied to the original coupled suspension,
rod, lobe and contact solver. Three REDengine gravity/damping/speed properties do
not reproduce those eight controls. Do not declare parity by renaming them.
