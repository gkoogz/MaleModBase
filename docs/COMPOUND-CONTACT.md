# Rear-only compound contact

The current request is to double the **surface area covered by the attachment**,
keeping its front edge fixed and adding the area toward the pelvis. The user
explicitly authorized including the root transition and pelvic surface. The
previous Witcher `rearExtension=0.25` change did not meet that requested size.

This implementation is shared authoring geometry and an SDK-free motion
evaluator. It is **not yet adopted by Witcher's renderer or installed**. The
existing native attachment gate has not been run for this topology.

## Geometry and measurement

`surface_footprint.ContactArea` integrates clipped source triangles using their
true 3D area, rather than flattened chart area, axial length or the exposed neck
skin. A monotone rear-only map and bisection solve the requested area ratio.
The distal half stays exact. Missing coverage or overlapping chart regions
within the footprint are errors, never reasons to silently clamp the area.

`tools/build_compound_contact.py` starts from a complete shaft already joined
to the measured original pelvis. The chart spans shaft, generated root ramp
and retained body. Inserted points interpolate original triangle charts;
recomputing nonlinear polar coordinates after refinement had created overlapping
needle triangles. No retained pelvic vertex is moved.

The cutter refines the retained sac boundary before building a tangent-matched
patch and welding shared vertices. Both ends retain exact source triangle IDs
and barycentric donors. Existing body boundaries and per-corner UV aliases
survive. The complete mechanical shaft remains a separate input.

The old endpoint-normal interpolation test is not an outward field when the
support turns onto the pelvis. A smoothly curved tube can fail that test even
with consistent winding. Compound lofts instead check actual adjacent facets,
collapsed triangles and both prescribed boundary orientations. The selected
patch also requires adjacent-facet cosine >= 0.5. These local checks do **not**
replace a whole-surface intersection audit. Tests contain both the legitimate
large-turn case and deliberately corrupted interior rows.

The sac's requested rootward translation is derived from the change in the
physical contact-area centroid. It is a rigid source displacement, preserving
the lobe shape. A bounded search limits that displacement if it would violate
the local attachment criteria. An explicit source-unit override is available.
Area recruitment and fixed-front coordinates do not change during that search.

## Coupled data and motion

`malemod.compound-contact-authoring/1` couples geometry and bindings in one
manifest with input/output hashes. `neck.npz` contains revision 1, row indices,
endpoint derivatives, retained-source refinement, source/support triangle
donors, source curve parameters and the applied source translation. Consumers
must evaluate the translated source and finish support recruitment before
evaluating the connecting patch.

`CompoundGraftPlan` in `include/malemod/surface/compound_graft.hpp` transports
endpoint tangents through their actual triangles and preserves the authored
interior residual at rest. This permits body-owned, ramp-owned and shaft-owned
rim points without assigning all of them to a shaft frame. Moving the suspended
surface does not feed back into the shaft or body. Unit tests cover independent
endpoint motion, rigid rotations, uniform scales and collapsed-support rejection.
This evaluator is not a collision solver or an observed game integration.

## Current evidence and remaining adoption

The measured Witcher candidate covers 240.771487 source square units versus
120.385397 before: ratio 2.00000576. All 192 fixed front samples differ by at
most 6.4e-14 source units. The rim has 210 body, 104 ramp and 703 shaft knots.
The retained pelvis has zero displacement and retains its five original open
boundary loops. Private generated evidence lives under
`D:/MaleMod/Work/witcher/rear-attachment/expanded-support`.

The final offline preview moves the retained sac 2.112795 source units toward
the pelvis, bounded from a centroid target of 3.933640. Its minimum adjacent
facet cosine is 0.500270 and its minimum boundary-normal cosine is 0.935665.
The full-surface audit of this preview found no new crossings;
77 distal source crossings were identical to the untouched complete-shaft
support. Its 95,510 triangles have no degenerate faces under the auditor's
recorded tolerance. True side, front and oblique offline views were reviewed;
the lateral shoulder shape remains visible and is not a native acceptance
claim. This does not certify the glans or waive native visual review. All 53
installed files still match the current receipt. Later candidates must receive
their own audit and renders.

The Witcher renderer currently rebuilds a complete circular root join and
assumes every neck endpoint belongs to the shaft. It must adopt a compound
support-cut/graft phase, with source corner attributes, before this candidate
can be installed. Evaluate the body/root first, then the compound rim and sac
skin. Measure suspension equilibrium once per rest/control rebuild; do not
cancel live inertial motion. Preserve both resource palettes, UV aliases, live
controls and supported LODs. Rerun the full grey-room and campaign attachment
gate. The prior installed runtime and exact rollback remain authoritative.

Wolverine and other spokes should pin a tested Base revision and supply their
own measured complete support and source bindings to this same contract.
Do not copy the algorithm into a game-specific fork or change those installed
runtimes as part of this authoring work.

## Reproduction

1. Build the existing source module from its adapter-owned recipe.
2. Run `assemble_modular_pelvis.py` using **shaft.npz**, not the prior visible
   scrotum union, with the adapter's measured root margins.
3. Run `build_compound_contact.py --body <original body NPZ> --support <complete
   support directory> --module <source module directory> --maximum-extension
   <measured chart capacity> --output <fresh Work directory>`. Ratio defaults to 2.
4. Audit the whole result with `audit_surface_intersections`; compare unchanged
   source crossings explicitly. Inspect true side, front and oblique renders.
5. Native adoption and its acceptance evidence are separate steps.

The focused Python suite covers 29 tests. The SDK-free modular CMake suite
covers 10 tests, including the intersection auditor fixtures. General
`tools/verify.py` still fails the existing motion-filter hash for the root
`CMakeLists.txt`; this work has not changed that file or waived that mismatch.
All 516 imported Wolverine files still match their recorded provenance hashes.
