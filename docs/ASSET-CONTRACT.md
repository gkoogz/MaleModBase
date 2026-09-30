# Asset contract version 1

Portable arrays preserve original vertex order and source float32 values.
`geometry.npz` keys have the form `header_stem__array_name`; metadata records
type, shape and original declaration. It is numerical authoring data, not an
engine asset format. Never treat original palette indices as semantic joints.

Reference OBJ output keeps source coordinates, units and index order, but
reverses triangle winding to match the source's `Cross(edge2, edge1)` normals.
Source units and full bind skeleton are not established. No centimeter-to-meter
conversion is inferred. The final reference surface interpolates `cpReference`
with `nrDirect` and sparse `nrRows/nrSources/nrWeights`. This is not the final
live evaluator, which also applies shape, contacts, pouch and collar solves.
UVs come from each final vertex's packed little-endian half floats; direct
vertices use the source support attributes. The exact packed attributes also
remain in the array archive. Texture assignment and skin bindings are pending.

Character profiles declare a positive finite scale, an orthonormal basis,
translation and optional joint mapping. Reflections reverse faces so winding
remains correct. Transforming the asset does not retarget animation or physics.
Empty joint mappings are explicitly unresolved, never automatic matches.

Future authored assets should use a calibrated common unit (meters), an
explicit handedness and up/forward basis, named joints and attachment landmarks,
inverse bind matrices, material slots and per-channel morph definitions.
Source-specific geometry must retain its conversion profile until calibrated.

Morph arrays describe source cage positions or authored deltas according to
their original names/code; they are not all automatically glTF blend shapes.
Do not attach them to final render vertices without the original transport.
