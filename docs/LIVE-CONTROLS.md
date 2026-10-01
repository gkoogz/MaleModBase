# Shared size and physics controls

`malemod_base.controls` owns the 18 rest-shape/mechanical controls from the
accepted Wolverine UI v5 preset. `tools/export_controls.py` writes the versioned
catalog and source hashes. Adapters may generate native declarations/tables from
`modules/live-controls.json`; generated output must be pinned and reproducible.
They must not edit a second independent copy of the mappings.

The catalog preserves the source midpoint, length's lower smooth curve and
upper extension, and the v5 glans remapping. Most sliders span 1–100; length and
glans include zero. The default mechanical state is flexible. These numbers are
source controls, not medical measurements or SI parameters. The active source
solver consumes both UI values and mapped values. Reusing the mappings alone
does not reproduce its suspension, contacts, skinning or final unified collar.

Preferences use `format: malemod.controls`, `version: 1`, and a `values` object
keyed by stable IDs. Missing known values receive defaults. Unknown IDs, unknown
versions, nonfinite numbers, booleans and out-of-range values are rejected.
Adapters should preserve portable preferences separately from their supported
native subset and visibly identify unavailable controls. A functional menu
requires a verified deformation output; storing a slider value is insufficient.

## Runtime and performance requirements

- Simulate a small motion cage; skin render geometry through cached bindings.
- Allocate buffers during setup or topology changes, not each solver iteration.
- Bound substeps, reset history across teleport/load/equipment changes, and
  suspend work when the attachment is inactive. Avoid catch-up after suspension.
- Apply parameter changes on user input; persistence is debounced.
- Keep pelvic seam aliases and the original body-part boundary constraints.
- Record compilation, native resource round-trip, runtime behavior and observed
  gameplay independently. Record actual timing before claiming an FPS result.

## Spoke adoption

Witcher owns native bones/graphs, script declarations, UI/input and packaging.
The 0.4.20 layer now has observed attached boot and animated parent following.
0.4.21 introduces the first five live size controls through a normalized cage
preview; new-control gameplay and persistence remain pending. See
CONTROL-TRANSPORT.md and the adapter's handoff/provenance. The remaining controls
and full authored surface/physics parity are not established by the catalog.

Wolverine can adopt the catalog and preference validation first, then compare
all control samples against its current executable mapping before replacing
its menu mapping. Retain its active full solver until posed parity is checked.
Other spokes consume the same IDs and declare their calibrated capabilities.
Animation sequences, fluids and audio are outside this control pass.
