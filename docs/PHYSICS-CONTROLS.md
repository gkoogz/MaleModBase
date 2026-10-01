# Active physics control laws

`malemod_base.physics_controls` extracts the scalar laws used by the active
Wolverine coupled solver. It preserves both raw UI and mapped parameters. For
example, raw stiffness controls bend compliance, while mapped weight controls
mass. One engine gravity/damping/speed triple cannot represent these eight laws.

The evaluator requires a versioned portable preference document, the current
smoothed mechanical mode and a measured rest-guide length in explicit source
space. It returns separate shaft/lobe masses, drag and motion response, shaft
bend compliance/damping, suspension compliance/shear/damping, torsion and root
suspension coefficients. Measured tether distance and lobe radius determine the
rest/hard suspension limits. No source-to-SI conversion is inferred.

## Verification

The oracle includes exact scalar source statements extracted from the immutable
reference. It builds without Windows, graphics or game SDK headers. A checked-in
fixture contains 600 float32 profiles covering every physics slider, five modes
including transitions, and three measured lengths. Python output matches those
native statements. This verifies coefficients, not full coupled simulation or
native Witcher output. Contacts, actual rest guides and solver timing remain
separate responsibilities; clinical modulation is deferred.

Regenerate with:

```text
python tools/extract_physics_controls.py
# Compile tests/physics_controls_oracle.cpp with a C++17 compiler.
python tools/build_physics_controls_fixture.py <oracle executable>
python tools/extract_physics_controls.py
python -m unittest tests.test_physics_controls -v
python tools/verify.py
```

## Adoption in spokes

- Witcher: pin a tested Base revision, retain independent raw UI values, calibrate
  rest-guide space, and feed these coefficients to a supported numerical output
  bridge. Native dangle tuning must keep its own names and capability evidence.
- Wolverine: compare the existing active laws against this oracle first, then
  replace their definitions through its Base adapter once full runtime parity is
  established. Its installed solver remains authoritative and unchanged.
- Future spokes: reuse the evaluator and fixture. Own only measured character
  frames, collisions, native pose output and menu bindings in each adapter.

Do not claim a slider works merely because the coefficient changes. A working
slider also needs visible, verified output and stable behavior across its range.
