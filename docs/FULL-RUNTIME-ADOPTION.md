# Full shared runtime adoption

The user authorizes the full source solver, surface reconstruction, all 17
anatomy/physics sliders and the 3-state selector in Witcher, with a separate
overlay. Wolverine remains unchanged. Sequences, fluid and audio are deferred.

## Hub and spoke boundary

Base owns the source numerical pipeline, preference contract, sparse transfer,
coordinate mathematics and coupled target collar. The spoke supplies measured
Geralt axes/scale, stock bone names/indices, source vertex lineage, original-edge
donors, render aliases, protected body-part boundaries and native output.

The collar is evaluated on Geralt's fitted topology. It does not reuse
Wolverine's source collar vertex indices or assume equal skeletons. The native
waist and ankle boundaries stay fixed; expansion beyond those boundaries
requires shared bindings that drive both native resources together.

`surface/binding.hpp` separates position, direction and length conversion and
applies explicit sparse displacement donors. `surface/graft_runtime.hpp`
implements Base's existing Python `CollarPlan.solve_displacement` in C++ with
cached factorization, independent original-edge masters, seam elimination and
protected exterior rows. Render UV aliases expand from the same solved rows.
The native adapter must rebuild normals/tangents on its final surface while
retaining material/skin/UV classes. This output path is not yet implemented.

`surface::Frame::collision` supplies measured thigh radii and a measured pelvis
capsule, together with observed thigh endpoints. The pelvis contact history is
interpolated on the source simulation clock; a render-frame displacement is not
misinterpreted as a 240-Hz one-step velocity. Missing calibration deliberately
uses the reference collision envelope and must not be labeled Geralt contacts.
Native motion sampling, gravity/inertia frame conversion and calibration of the
actual Geralt pelvis envelope still require adapter integration and validation.

## Verified development checkpoint

- The strict Win32 numerical Session passes 37 shape cases, 33 physics cases
  and the 180-frame original-source trace without relaxing the 1e-4 gate.
- Supplying the reference contact calibration has zero anatomy difference.
  Increasing measured thigh radii produces a distinct contact response;
  invalid calibration is rejected before state changes.
- The C++ target collar matches the established Geralt Python solve in 22
  cases, both LODs and Overall 1 through 100. Maximum discrepancy is below
  1e-8 source units. Original-edge seams and protected boundaries have zero drift.
- The versioned wire codec preserves full geometry, lighting, UVs, source IDs,
  topology and mechanical state across the 32/64-bit process boundary. Both
  architectures reject malformed packets. Transport belongs to the adapter.

These are source/offline checks. No full runtime, overlay or vertex backend is
installed or observed in Witcher. Its .31 Overall output works according to the
user; its reduced physics is rejected and remains the installed implementation.

## Performance and failed gates

The original thread-isolated evaluator serializes independent geometry. A
process-isolated experiment restores the bounded source scheduler, but state
relocation changes the extreme scrotum case by 0.00011677211 source units,
exceeding 0.0001. Serial scheduling in that process variant has the same error;
retaining sequential simulation caches in TLS did not resolve it. The variant
requires explicit diagnostic opt-in and must not be adopted or deployed.

The complete development process bridge measured roughly 40-50 ms per frame,
excluding target fitting, game upload and rendering. This is not a native game
benchmark. Cached collar solve cost must be measured separately from its plan
construction. Do not refactor every rendered frame, present an approximation as
source parity, or hide these costs behind the existence of a menu.

## Adoption by every spoke

Witcher pins the committed Base revision before cooking/deployment; it does not
maintain copied numerical source. Its engine probe uses the actual game
executable hash and observed RVAs, not different-build REDkit addresses.

Wolverine is still authoritative and has not yet consumed Base. A later explicit
adoption keeps its existing draw/input/material adapters, substitutes the tested
Session and versioned bindings, and runs source trace, native output, installer
rollback and observed gameplay gates. Existing source calibration remains the
default; optional character contacts are enabled only by measured adapter data.
Future spokes follow the same coordinate, lineage, collar, native-output and
gameplay gates. Shared numerical additions never install into another game.
