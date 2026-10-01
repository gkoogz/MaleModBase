# Shared rest-surface attachment

`surface.rest-graft` is the first fitting feature developed through the Witcher
spoke and implemented in Base. It complements `surface.pelvic-collar`: this
feature builds a fitted rest surface; the collar controls coupled deformation.
Neither offline evaluator establishes a game's live deformation capability.

The caller supplies an observed native body, an explicitly aligned reference
module and an interior body patch. Existing body boundaries must be protected.
The fitter finds the two open loops, subdivides their original edges using a
common angular correspondence and records sparse vertex/face lineage. A local
harmonic fit brings the module boundary onto the body. A bounded local repair
requires positive signed area relative to the original face normals while
keeping seam vertices fixed. It rejects an unsatisfied repair budget.

The resulting boundary is a continuous topological surface. Separate render
vertices may remain for UV aliases, but their positions and transferred skin
weights must agree. Harmonic attribute extension produces target skin/color/UV
fields from exact boundary values and an explicit far-field value. Original
module UVs remain in the binding artifact. Original source support points and
bindings remain in the source bank, including untriangulated pressure support.

Skin influence limiting is deterministic and preserves each input row's sum.
This matters for native byte-quantized weights that do not sum to exactly one.
The adapter declares its supported influence limit, records discarded weight
and validates native import/export. No bone names or unit conventions live here.

## Bodies made from separate native parts

Witcher's torso and legs are separate resources sharing native skeleton weights
at their waist. An attachment confined to the pelvis must leave existing waist
positions, skin weights, UVs and shading attributes unchanged. Do not replace
that native join merely because the assets are separate files.

A future deformation that reaches a part boundary requires a shared boundary
binding: source part/topology revision, matched vertex or original-edge donors,
canonical position, skin field and shading policy. Evaluate both parts from
that same boundary state, including each LOD, then let each adapter upload its
own buffers. A game-specific resource split belongs in its adapter; the common
boundary constraint belongs in Base. Until a spoke implements that path, lock
the boundary and report the deformation envelope rather than moving one side.
This coordinated runtime boundary system is a requirement, not implemented
support claimed by the rest fitter.

## Verification and adoption

Tests cover exact body-edge donors, existing boundary preservation, UV aliases,
skin continuity under independent affine bone transforms, local orientation
repair and rejection, and deterministic influence limiting. Native round-trip
and observed gameplay are separately recorded by the Witcher adapter.

An extended skeleton also requires coverage by every active pose-update LOD.
Valid names, parent indices and bind transforms do not prove that an engine
updates an added joint during animation. Each spoke must verify that all joints
referenced by its skin weights fall inside the native update range, or supply
a tested LOD remapping. Test idle, gait and detail-level transitions separately.
This is a shared adoption check for Wolverine and future spokes; native LOD
counts and resource changes remain in the adapter. Witcher's private 104-joint
rig inherited a 40-joint reduced-detail cutoff; the adapter is testing complete
coverage. Its runtime outcome remains separate from this requirement.

Wolverine can adopt the fitting tools for future character authoring, and the
seam/skin/orientation checks can strengthen its export regression gates. Offer
that backport when this first spoke has passed user testing. Do not replace its
working final unified solver with this rest fit or alter the legacy snapshot.

Current limits: star-shaped boundary correspondence, no global intersection
proof, no new dynamics, and no automatic LOD simplification. The Witcher first
attachment retains the reference module topology in both native LODs; detailed
material transfer and secondary motion remain separate follow-up work.
