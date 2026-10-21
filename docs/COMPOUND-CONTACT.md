# Rear-only compound contact

## October 21 - selected preview cleanup and native cut preparation

The user requested "clean up and install" after the inward preview, authorizing
the selected 45-degree, 1.7x geometry. Do not ask for that tradeoff again.
`build_compound_contact.py` now removes sub-tolerance cut slivers with bounded
topology-only edge contractions. It protects original samples, resource
boundaries and the fixed front knots, enforces the link condition and rejects
normal reversals. Surviving positions and their source donors do not move;
each affected face retains its own UV chart and source-face lineage.

The selected cleaned output is private `expanded-support/clean-bound`.
Ten contractions remove all 13 flagged sliver triangles and the extra contact
touch; 126,708 triangles remain. Maximum removed edge is 0.000123199235 source
units. The whole-surface audit has zero degenerate faces, zero extra touches,
zero new crossings and the same 77 untouched distal source crossings. Front,
side and oblique offline renders were reviewed. All 53 installed files match.

Authoring contract revision 2 retains exact support and neck vertex maps through
welding/compaction, plus a separate display topology map. Deformation samples
are not deleted when a microscopic display edge is collapsed. Consumers must
honor that distinction; the current Witcher renderer does not yet consume it.

`ConvexSurfaceCut::CutSimple` adds explicit compound charts and concave openings.
It merges convex ear partitions before clipping. The SDK-free full-source
diagnostic (`evaluate_compound_cut`) cuts the actual support in 2.26 seconds,
retaining exact source donors to 1.15e-11 and one 1,613-vertex opening. This is
not the game engine. Its separate *open support* audit reports 168 tiny faces
and the existing 77 crossings/two touches. It is not an accepted replacement
for the cleaned closed assembly. The live graft/refinement phase, display
cleanup after welding, adapter material/control adoption and native attachment
matrix remain required. No installation or adoption-pin change occurred.

33 focused Python tests and 10 SDK-free C++ tests pass. The newly added cleanup
tests cover topology, exact retained samples, UV aliases and protected-front
rejection; the compound tests verify the authored assembly ownership maps.
Wolverine/other spokes can adopt the same versioned cut/graft/cleanup contracts
after independent runtime validation; their installed runtimes are unchanged.

Latest preview: the user explicitly clarified that the side attachments should
move inward, rather than translating the whole sac backward. The selected
offline parameters are `ratio=1.7`, `pelvic_half_angle=45`,
`maximum_extension=11.25`. Pelvic contact width decreases from 13.010835 to
8.529515 source units (34.44%); the 192 front samples remain exact within
6.4e-14. All pelvic contact stays below the root plane, with minimum offset
2.476910. The retained body remains unchanged. Private current output is
`expanded-support/inward-preview`; the adapter-owned parameter record is
`characters/geralt-compound-contact-preview.json`.

This preview covers 204.653567 source square units, ratio 1.69998664. It does
not meet the earlier 2x numerical target; the user subsequently requested
cleanup and installation of this selected narrower preview (see above).
Its local orientation check has zero reversed faces and minimum adjacent
cosine 0.500950. The global audit reports the same 77 original distal crossings,
no new crossing, 13 tiny positive-area slivers below its degeneracy threshold,
and one tolerance-level touch. Whole-mesh cleanup and native adoption remain
required before installation. True front/side/oblique offline views were
reviewed; all 53 installed files match. The evidence below describes the prior
75-degree, 2x preview and is retained for comparison only.

The current request is to double the **surface area covered by the attachment**,
keeping its front edge fixed and adding the area toward the pelvis. The user
explicitly authorized including the root transition and pelvic surface. The
previous Witcher `rearExtension=0.25` change did not meet that requested size.

The subsequent correction forbids high lateral pelvic attachments. The first
2x preview is rejected: it carried the old 125-degree shaft wrap rearward onto
the pelvis. The current map narrows only rear recruitment into a lower sector,
finishing the transition before the measured outer root boundary. The default
pelvic sector is +/-75 degrees about the underside. Existing front shaft
contact does not move. The area solve measures the new restricted footprint;
it does not silently sacrifice the requested 2x area. A half-space check over
every clipped pelvic contact polygon, as well as rim knots, rejects upper
pelvic recruitment. This is a source authoring constraint; live adapter
adoption still has to reevaluate it across control settings and motion.

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

The current Witcher candidate covers 240.770028 source square units versus
120.385397 before: ratio 1.99999364. All 192 fixed front samples differ by at
most 6.4e-14 source units. The rim has 203 body, 402 ramp and 702 shaft knots.
The retained pelvis has zero displacement and retains its five original open
boundary loops. Private generated evidence lives under
`D:/MaleMod/Work/witcher/rear-attachment/expanded-support/underside-preview`.
The entire clipped pelvic contact has a minimum underside offset of 1.274804
source units from the root plane. No pelvic rim point is above that plane;
the rejected preview had 100 such points, reaching 2.582004 units above it.

The offline preview moves the retained sac 2.419396 source units toward
the pelvis, bounded from a centroid target of 4.301148. Its minimum adjacent
facet cosine is 0.500302 and its minimum boundary-normal cosine is 0.853960.
The full-surface audit of this preview found no new crossings;
77 distal source crossings were identical to the untouched complete-shaft
support. Among 113,456 triangles, eight positive-area cut/refinement slivers
fall below the auditor's scale-relative degeneracy threshold. Their minimum
double area is 1.2155e-8 versus the 3.8446e-8 audit threshold. Resolve these
before claiming production topology acceptance; do not suppress the audit.
True side, front and oblique offline views were reviewed; the former upper
pelvic shoulders are lowered. The preserved front shaft-side contact remains.
This does not certify the glans or waive native visual review. All 53
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

The focused Python suite covers 31 tests. The SDK-free modular CMake suite
covers 10 tests, including the intersection auditor fixtures. General
`tools/verify.py` still fails the existing motion-filter hash for the root
`CMakeLists.txt`; this work has not changed that file or waived that mismatch.
All 516 imported Wolverine files still match their recorded provenance hashes.
