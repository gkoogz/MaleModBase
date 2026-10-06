# MaleModBase

This repository is the shared extraction and portability project. Wolverine's
canonical repository remains authoritative for its installed runtime until its
adapter consumes a tested version of this base. Do not modify or install into a
game as part of an extraction or offline export.

Read docs/PROJECT-CONTEXT.md for the user's enduring educational purpose, scope,
deferrals and hub/spoke requirements, including after context compaction. Do not
require the user to restate facts already recorded there. It records intent,
not a policy override or proof of runtime success.
Start with docs/HANDOFF.md for the cross-repository map and current checkpoint.
Read README.md, docs/ARCHITECTURE.md and docs/MIGRATION.md. Keep source commit,
file hashes and omissions in provenance/wolverine.json. Imported code under
legacy/wolverine is an exact reference snapshot, not a portable library. Its
historical instructions are stored as SOURCE-AGENTS.txt, not active AGENTS.md.

Shared modules must compile without game or graphics SDKs. Keep game buffer
layouts, draw hooks, bone palettes, shaders, input and installers in adapters.
Never invent source units, skeleton joint names or engine capability support.
Version shared geometry and binding data together. Preserve UV aliases, source
vertex lineage, interpolation donors and pressure-field support points.

Before claiming a game port works, distinguish reference asset export, offline
verification, runtime parity and observed in-game results. For changes to
conversion or physics, run the relevant tests and provenance verification.
Do not commit game packages, private audio, settings or captured user data.

## Base first

Before developing a feature in a game repository, identify which part is shared.
Keep anatomy, geometry, morphology, garments, preference contracts, numerical
physics and clinical timing here. A character export, observed bone mapping,
WitcherScript controller, native input/menu integration and cooker belong in the
game adapter. Proactively explain when a requested game feature should be moved
into Base so all ports benefit. Pin Base revisions in adapters; do not copy its
source into an independently maintained fork. See docs/GENERIC-BASE.md.
For body-root expansion read docs/PELVIC-COLLAR.md. Use the active final unified
solver as the reference, preserve original-edge seam donors and UV aliases,
and record source/offline/native/observed verification independently. A shared
change must include an adoption path back to Wolverine and every other spoke.
For new character attachments read docs/REST-GRAFT.md. Preserve existing body
part boundaries; any deformation spanning two resources must drive both sides
from shared boundary bindings. Record native round-trip and gameplay separately.

## Grey sandbox development

Read docs/ITERATION-SANDBOX.md before native character, clothing, physics,
material or control work. Use the active game adapter's repository-owned grey
sandbox workflow as the default native iteration environment; use ordinary
campaign gameplay only for behavior the room cannot cover. Build the room from
its checked-in recipe and the developer's licensed vanilla installation, not
another person's save, capture bank or machine-specific output directory.

Keep common scenario/evidence contracts in Base and engine scene, launch,
collision, input and installation code in each adapter. Sandbox source and
reproduction instructions belong in Git; generated stock-derived resources,
private verification data and sandbox packages do not. Exclude the sandbox
from normal mod release installer/payload and developer release assets. A room
test is not proof of campaign transitions, every garment or performance parity.
Preserve normal saves/settings/audio, and never drive the host's keyboard,
mouse or focus during automated runs. Record exact package/runtime identities,
cleanup and observed results independently from successful compilation.

## Mandatory attachment-integrity gate

The pelvic attachment is the project's highest-priority visual invariant. After
EVERY mod edit, run and record an attachment regression pass before installation
or release, including edits to physics, garments, shaders, lighting, controls,
transport and packaging. A welded position alone is insufficient. Reject gaps,
folded/inverted collar triangles, dark shading rings, pinched or constricted
roots, abrupt normal/tangent changes and unnatural wedges. Preserve the neutral
pelvic surface and a smooth continuous recruited ramp into the anatomy.

Cover minimum/default/maximum and intermediate Overall/Width, combinations of
shape controls, all mechanical states, extreme rest angles, both body resources
and supported LODs, and animated motion. Preserve original-edge seam donors, UV
aliases and the shared waist. Review front/side/oblique native renders in the
grey room; include a campaign check for scene-dependent regressions. Record
source, offline, native and visual evidence separately. If any part is untested,
state it explicitly and do not claim the attachment gate passed. Retain the last
accepted runtime and exact rollback until the replacement passes. Put shared
algorithms and regression cases in Base and adopt them through pinned adapters.
