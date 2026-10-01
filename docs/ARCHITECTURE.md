# Shared foundation and game adapters

MaleModBase owns source anatomy, garments, semantic attachment definitions,
deformation bindings, reusable numerical algorithms and conversion tooling.
Game repositories should eventually pin a base revision and contain their
character profiles, format exporters, integration code and installation logic.
Use separate adapter repositories with pinned dependencies rather than
long-lived branches that duplicate the shared source.

## Boundaries

1. Authoring data: geometry, morphs, UVs, material inputs and source lineage.
2. Shared evaluation: character-local morphology, contact and deformation.
3. Character profile: proportions, bind transforms, attachment landmarks,
   semantic-to-native joint mapping and material choices.
4. Engine adapter: animation pose sampling, collision query implementation,
   buffer uploads, material translation, timing, input and packaging.

The adapter supplies character-local transforms and body collision geometry;
shared evaluation returns character-local surfaces and semantic events. Render
and collision surfaces must share topology IDs and revisions. Contact anchors
are triangle ID plus barycentric coordinates, never a draw-call offset.

## What was actually separated

The surface correction limiter now lives in a standalone C++ header. Geometry,
morph and binding arrays can be read without Direct3D. Character fitting runs
offline with validated transforms. The clinical sequence, CPU viscous-fluid
solver, mesh generation, deposition and audio cue timing are now portable,
with per-character state and adapter callbacks. See CLINICAL-SEQUENCE.md.
The distance and damped-bending XPBD kernels now have explicit instance state,
and drive a separate generic fixed-step chain. Skinning, sockets and cached
capsule projection are shared too; see GENERIC-BASE.md. The complete authored
anatomy suspension/contact/collar solver now also has an extracted SDK-free
32-bit surface session; see SOURCE-SURFACE.md. The immutable snapshot remains
its provenance input, with a hash-checked declaration recipe.

The active final collar's growing support metric and a source-derived offline
coupled solve are now shared, with exact donor-edge seam elimination. The full
posed runtime, guide inputs and graphics uploads remain separate integration
work. See PELVIC-COLLAR.md for the feature contract and spoke adoption matrix.

The original XPBD solver uses globals for rod nodes, lobe transforms, previous
poses, contact impulses and tuning. The complete surface extraction isolates
mutable globals and function-local caches on each session's owned worker thread.
It exposes explicit controls, timestep, motion forces and optional thigh inputs.
Simultaneous sessions and changing original-source traces pass offline gates.
64-bit arithmetic, target fitting and engine uploads remain unresolved; the
existing authoritative Wolverine runtime has not migrated to this interface.

## Lessons retained from Wolverine

- Keep the original motion cage independently of final render resolution.
- Reduce triangles, packed attributes, UV aliases and all bindings together.
- Keep non-triangulated support points needed by pressure/diffusion stencils.
- Preserve accumulation order where numerical parity matters.
- Rebuild lighting from final deformed geometry; explicitly handle winding.
- Enforce seam constraints and distinguish contact from skin attachment.
- Keep pose simulation, collision and rendering in explicitly declared spaces.
- Report geometry tests separately from material appearance and game FPS.

## Shared garments

A new garment requires its own mesh, semantic attachment IDs, material slots,
body fit envelope and supported deformation/collision requirements. Each
character profile supplies fit and native joints; each engine adapter exports
and packages it. Publishing one garment can trigger all adapters' builds once
those exporters exist. Different proportions and cloth systems may require
authored fit corrections. A thong or loincloth is not presently implemented.
