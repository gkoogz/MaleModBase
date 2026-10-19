# Generated skin material continuity

`malemod_base.loft_material` supplies a closed, arclength-based UV chart for
generated skin strips. Retained boundary-only faces keep their original chart.
The caller owns source selection, atlas allocation, native encoding and imports.

The bake inherits both boundary colors and tangent-space normals, blends pigment
over the strip and reuses measured retained skin detail. Detail scale comes from
area-weighted retained surface density, not the length of a jagged cut in UV
space. Circumferential texture repeats are integral so the wrap closes. A narrow
boundary blend avoids stretching one-dimensional boundary samples into long
visible streaks. The retained skin's visible creases must survive the high pass.

This does not change geometry, mechanical ownership, skinning, source UVs on
retained surfaces or numerical physics. Geometry and shading acceptance remain
separate. The adapter must reject atlas overlap, preserve untouched pixels,
transform boundary normals into the new chart frame, rebuild tangents and bind
the material artifact to its exact native texture package.

Run `python -m unittest tests.test_loft_material tests.test_modular_shading`.
The tests cover closed charts, retained boundary triangles, density under scale,
exact boundary colors, normalized normals and periodic texture endpoints.

Witcher adoption uses `prepare_neck_material.py` and `build_neck_material.py` with
its modular dependency pin. Its optional texture package participates in native
preview identities, attachment acceptance, installation and managed rollback.
Native acceptance is recorded in the adapter handoff, never inferred from these
tests. Wolverine and other spokes can consume the shared chart/bake with their
own measured material islands and formats; none is automatically installed or
claimed validated by Witcher evidence.
