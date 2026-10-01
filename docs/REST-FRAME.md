# Measured rest centerline and source dimensions

`malemod_base/rest_frame.py` measures caller-supplied prepared source geometry
using the active `BuildShaftRestFrame`, `ClosestRestShaftFlex` and
`SampleRestShaftFrame` rules. It caches source membership weights, fits 18
centerline samples, preserves the anatomical root, smooths and regularizes the
line, computes surface flex and pelvic root-follow, measures the pure shaft
radius, and clamps the measured arc length to the source 8–60 range.

Coordinates and dimensions are explicitly source units. The adapter supplies
mapped overall/width/angle, discrete source physics mode, pelvic ramp blend and
the previous measured length. That last input is a real dependency: short or
under-supported profiles can fall back to the preceding rest length. Preserve
`uses_previous_length` in shape-cache invalidation. Do not replace the anatomical
root `[9,0,84.3]` with an unrelated fitting alignment origin.

Float32 weights, interpolation and sequential accumulations preserve the source
measurement on short profiles; changing those to a different reduction order can
change closest flex even when center positions differ only slightly. The shared
implementation has no global character state, native joint names or engine API.

## Verification and scope

The SDK-free C++ oracle uses verbatim source functions. It consumes identical
caller-supplied float32 positions, isolating measurement from prior shape stages.
3,240 fixture rows cover 20 shape profiles, all three modes and previous lengths
12/24/48. Centerline, radius, arc length, sampled flex, root follow and fallback
flag comparisons pass. The fixture geometry comes from the separately verified
early authored stage; it is not the complete prepared source surface.

```powershell
python tools/build_rest_frame_fixture.py build/path/to/rest_frame_oracle.exe
python tools/extract_rest_frame.py
python -m unittest discover -s tests -p test_rest_frame.py -v
python tools/verify.py
```

This module does not supply preceding fairing/root-profile corrections, the
logical/glans surface, final coupled pelvis, complete dynamics or native output.
The existing early-stage omissions still apply. Do not present a measured frame
as a complete rest-shape port.

## Adoption across spokes

- Witcher: reverse its recorded fitting transform to measure source geometry,
  then translate the resulting rest cage through observed native bindings.
  Graph-to-visible-output and moving waist alignment still require gameplay proof.
- Wolverine: adopt the original-code fixture and cache-fallback checks first;
  keep its active complete preparation/runtime until posed parity is established.
- Future engines: consume this revision together with the source geometry and
  bindings, retain prior-length cache dependencies, and own native pose delivery.
