# Modular surface contract

The shared module supplies a complete mechanical shaft, independently driven
suspended skin, an exact cut in the original body, and a tangent-matched joining
strip. Game buffer formats, materials, palettes and installation remain adapter
responsibilities. No game or graphics SDK is required by these headers.

The geometry and mechanical ownership artifact is `MMMOD002`. `MMMDYN01`
binds the measured body and opening section to its SHA-256. `MMXFER02` binds
retained anatomical position/normal donors and axial morphology ownership to
the same geometry identity. Version 1 is a superseded diagnostic artifact.
Changing topology requires regenerating all three and the adapter's bindings.
Never infer ownership from a moving vertex's height.

Rod frames own shaft length. Transferring legacy axial positions into that
complete shaft imports a second length/collar correction and can reverse local
station order at short, wide settings. Transfer its cross-sectional residual
instead. The independently authored crown-to-tip region retains full axial
morphology through its original R14 ring lineage and final-surface donors. Four
pre-crown material rings provide a C2 handoff, fixed in the authoring topology.
Rebuild the shaft differential after this transfer; preserve the independently
moving suspended surface and its original normal field.

For new connecting skin, `modular_shading.boundary_uv_field` extends the exact
retained boundary texture coordinates using positive intrinsic cotangent
weights. Apply it independently to split UV charts. Nearest-point transfer from
the excised surface can switch atlas sides across adjacent vertices; that is
not a continuous material map. This authoring operation never moves geometry,
changes donors or replaces the licensed textures. Adapters provide measured
atlas coordinates and validate their native normal-map appearance.

Control/rest frames determine the opening. Physics frames move the independent
objects and their connecting skin without recutting the body. Original body
points, parent-triangle donors, native resource boundaries and UV aliases are
preserved. Aperture placement finds the lowest valid fit on the supplied body;
the adapter must record its calibration and supported angle range.

Run the independent shared regression project with `cmake -S tests/modular`.
The Python authoring tests are `tests.test_modular_anatomy`,
`tests.test_modular_union` and `tests.test_modular_shading`. These checks do not
certify native materials, gameplay or the attachment-integrity gate.

Witcher adopts this module through a separately pinned shared-module revision,
while its existing numerical worker/profile retains its own verified revision.
This avoids relabeling an old binary as a new Base build. Wolverine and future
spokes consume these same headers and versioned artifacts after supplying their
own measured body/binding/material adapters and passing their native attachment
matrix. Their accepted installations are unchanged by this opt-in adoption.
