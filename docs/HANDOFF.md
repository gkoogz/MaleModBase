# Resume here: cross-repository handoff

Latest correction: the user rejected Witcher .29's protected head/lobe binding
as a workaround and requested investigation of a possibly missing motion axis.
The opt-in protected binding API and tests are removed; motion_binding.py and
its provenance are restored to the pre-change contract. Witcher has restored
installed .28 and its original uniform render stations. The adapter's read-only
native instruction audit finds full-rank XYZ translation and rotation on all
ten joints, but still reproduces nonuniform surface distortion. Tangent-only
orientation has no independent material roll; the reference also uses shortest
rotation, so this is not established as the observed defect. See the adapter's
MOTION-AXIS-AUDIT.md for exact evidence and runtime limits. Wolverine is unchanged.

Latest user observation: Witcher .26 shape and form are approved, but its motion
slides sideways and feels wavy and light. Preserve that evaluated mesh/cage and
all source default material coefficients. The shared secondary-rig contract 2
adds relative-frame integration and inertial terms; Witcher measures pelvis
motion, rotates gravity/capsules into pelvis coordinates, and publishes local
points. Constant world translation must not cause drag or render-frame drift.
This is an explicit adapter motion calibration, not full Wolverine parity.
Wolverine stays unchanged; sliders/toggles remain absent. See adapter handoff
for native cook, installation and observed gameplay status.

Latest clarification: the Witcher first physics build moves in game, but does
not match Wolverine reset defaults. Shared `surface::Output` now exports the
source mechanical state alongside geometry, and `export_default_baseline.py`
exports state 2/all-50 after 120 reference frames. Base adds C1 guide sampling
for independently mapped rendering joints. Witcher is rebuilding this evaluated
default instead of retaining the posed large reference. Full native/gameplay
parity remains a separate gate; do not revive sliders or capture hotkeys.

Checkpoint: 2026-10-01. This file describes repository state and recorded
observations, not a promise that the current machine still matches them.
Chat history is not required to resume. Verify Git and local inputs first.
Read [PROJECT-CONTEXT.md](PROJECT-CONTEXT.md) for the user's stated postgraduate
educational purpose and enduring requirements before interpreting this project.

October 1 scope reset supersedes slider reconstruction: remove Witcher sliders,
retain the initial unit scale and observed working skeleton attachment, and
adapt Base physics/collision for rod and suspended lobes. Wolverine remains
unchanged. See `SECONDARY-RIG-PHYSICS.md` for SDK-free point-mass adaptations,
source support extraction, limitations and adoption gates. Numerical tests and
native compilation do not establish observed Witcher secondary motion.

## Repository map

| Repository | Responsibility | Current relationship |
| --- | --- | --- |
| [MaleModBase](https://github.com/gkoogz/MaleModBase) | Shared assets, anatomy, morphology, garments/preferences, numerical physics and clinical timing | Hub; this checkout is named `MaleMod` locally |
| [TheWitcher3MaleMod](https://github.com/gkoogz/TheWitcher3MaleMod) | Geralt bindings, REDengine resources, native runtime/input/menu, cooking and deployment | Spoke; pins Base in `dependencies/base.lock.json` |
| [XMenOriginsWolverineMaleMod](https://github.com/gkoogz/XMenOriginsWolverineMaleMod) | Existing authoritative Wolverine runtime and its engine integration | Original implementation; migration to a Base-consuming adapter is still pending |

These are independent repositories, not game branches of Base. A shared change
is committed here, then each adapter deliberately updates its dependency pin,
translates supported capabilities, verifies native behavior and deploys its own
package. Updating Base does not automatically change an installed game.
`legacy/wolverine/` is an immutable reference snapshot, not a second maintained
runtime. Its source commit is `dc64bdc44e75fd5521f066cdb2975277e9c34302`;
`provenance/wolverine.json` records hashes and omissions.

## Current shared work

The October 1 07:54 screenshot reports Witcher 0.4.23 is better but still fails
shape quality. First-principles reconstruction found that the static final
reference is a posed authoring mesh, not evaluated neutral. The complete source
numerical surface pipeline is now extracted into an SDK-free per-character
`surface::Session`: 17,528 vertices, original 35,000 triangles, lighting/UVs and
both coupled body sections. Read [SOURCE-SURFACE.md](SOURCE-SURFACE.md) and
`provenance/source-surface.json` before using it. 37 shape and 33 physics fixtures
and a changing 180-frame trace pass against the original 32-bit source. Strict
arithmetic is required. The 64-bit parity gate fails and remains blocked; no
complete Witcher surface upload, new installation or gameplay success is claimed.
Wolverine remains authoritative until its adapter adopts a verified Base pin.

`ROOT-PROFILE.md` adds the active source angular regularization stage: 24 sectors,
original sparse filling, shaft/pouch ownership, quintic seam fade and bounded
root correction. 143,280 original C++ vertex comparisons pass. It consumes caller
positions and RestFrame; preceding fairing, logical/glans/egg construction,
final UnifiedCollar and native full-control output remain separate gates.

New observed Witcher result: **0.4.9 resizes visibly at 0.8 and 1.2** and greatly
improves torso/leg tracking. Screenshot: active=true, accepted=true, 27 callbacks,
requested/readback=0.8, frozen=false. Some animations still open a temporary
vertical waist gap. Grey menu rows are intentional diagnostic readouts.
**0.4.18-boot-recovery-test FAILED user-reported following:** the user reports
continued static anatomy and collar stretching and requests a fresh examination
of Witcher animation. The installed Witcher package is
`publish/20261001-031309-f1ed02` pins Base `4b4719f` and retains cage
`ca78de0`. The adapter now reports graph boot exits and retries transient player
root/skeleton readiness for at most five seconds. Native cooking, 47 adapter
tests, eight exact unpacked resources/buffers and installed hashes pass. This
does not establish hip following; see the spoke handoff and F6 boot diagnostics.
The new adapter investigation traces native name-to-provider skinning mapping:
missing bone names use identity transforms, without falling back to their authored
parent. Geralt's observed movement animation set uses the 94-joint stock rig.
An offline stock-palette control preserves the graft seam and pelvis-local
binding under observed idle/walk/run clips (streamed tail uses native fallback).
The 05:05 user screenshot confirms that the live player still has 94 bones.
Witcher **0.4.19 FAILED observed loading-screen CTDs** twice. Windows records
access violations at witcher3.exe RVA 0x1e06862; SDK source-cache preservation
was not game-loader compatibility and is now blocked.
**0.4.20-shipped-player-load-test has observed working load/pose**, package
`publish/20261001-061552-504edb`, built against Base `9966388` and cage `ca78de0`.
The concrete gameplay and Geralt appearance templates are now extracted from
the shipped game bundle through official unbundle. Only rig imports and CRCs
change; all shipped cooked flags, embedded data and other bytes are preserved.
Native inspection compares against those same shipped bytes, not the SDK's
different loaded source view. Native template/rig/graph checks, 56 adapter tests,
ten exact unpacked resources/buffers and five installed hashes pass. The 06:21
user screenshot confirms attached first-attempt boot, active/accepted graph,
60 samples, 104 parent entries and max parent-follow error 0.000116.
See the spoke's `docs/ANIMATION-FOLLOW.md` and provenance for the archived CTD.

**0.4.21-size-controls-test is installed, new controls gameplay pending**,
package `publish/20261001-063505-7a41a8`, built against Base `15758e5` and unchanged
`ca78de0` cage. The new shared `control_transport` module supplies normalized
mapping for five size controls; native UI/storage/joint assignments remain in
the adapter. Defaults preserve the fitted rest asset; this cage preview is not
source morph, refined glans, coupled pelvis or physics parity. Three shared
transport tests, Base provenance, 58 adapter tests, native cook/binding checks,
ten exact unpacked resources and five installed hashes pass. 0.4.20 is now the
managed working rollback. See CONTROL-TRANSPORT.md for every-spoke adoption.
Future cage adoption in
Wolverine and every spoke must validate effective runtime bone mapping and
animated parent inheritance, in addition to resource names and rest frames.
**0.4.10-root-pose-test FAILED observed gameplay:** waist separation persists and
the user also reports an ankle gap. Its historical package is
`publish/20260930-224402-e6260b`, built against Base `81fe047` with unchanged
`ca78de0` cage. Its graph preserves observed bone-zero identity, matching the
native attachment reset instead of reintroducing animated Root; 93 stock
alignments, ten vector scales and 60 scalar channels survive native cooking
and connected-output/packed-byte verification. 35 adapter tests pass.
The root-only hypothesis did not resolve it. Historical
**0.4.12-model-pose-test improved but FAILED gait parity**, package
`publish/20260930-231057-0fb1c4`, built against Base `c7f78e3`. It reads parent model-space bone matrices instead of
the local animation sample context. The user reports a smaller gap that closes
at idle but bobs open with every step. Latest scale/ankle readouts were not
provided. The 0.4.11 secondary-motion package is held,
never installed, because it retains the failing local pose path. Consult the
spoke's current installation receipt/handoff. Native connected graph traversal,
six exact packed resources, five installed hashes and 37 adapter tests pass.
Witcher has **0.4.14-player-stack-test installed, FAILED authored joint follow**, package
`publish/20261001-005951-7981b3`, built against Base `0af9e5c` with unchanged
`ca78de0` cage. It appends an InputNode graph to Geralt's existing animation stack
instead of sampling another lower-body pose. Native checks retain all 94 stock
bones/rest frames and ten authored joints, stock player behavior/ragdoll/steering
bindings, 21 connected pose nodes, eight exact unpacked resources and five
installed hashes. 44 adapter tests pass. The user confirms scaling but reports
the anatomy stays unnaturally steady during idle sway, stretching the base.
Waist/ankle gait parity was not independently reported in that feedback.
Native name/parent/rest checks alone do not establish animation inheritance.
No per-frame script solver was added.
The next installed candidate is **0.4.15-authored-rest-test**, package
`publish/20261001-014011-2cda1b`, built against Base `466aebb` with the unchanged
`ca78de0` cage. **FAILED observed animation following:** the user reports that
it follows WASD movement but stays steady during idle sway, stretching the base. It restores reference local
transforms only on the ten authored joints before scale; all stock animated
bones and root motion retain the preceding player graph output. Native checks
verify the exact full-weight bone mask, 23 connected pose nodes, eight unpacked
resources and five installed hashes; 46 adapter tests pass. An earlier build
was rejected before packaging because integer Float JSON tokens were silently
ignored by the vendor converter. Float token authoring and a binary round-trip
regression repair that serialization defect. The native rest-mask mechanism
belongs in Witcher; every future spoke should separately verify attachment
following under movement and non-default numeric values through its serializer.
This does not establish full controls, pelvis deformation or active physics.
**0.4.17-full-joint-lod FAILED observed gameplay**, package
`publish/20261001-021142-cf7e81`, built against Base `3b10594` and cage `ca78de0`.
Review found the private 104-joint rig retained a 40-joint reduced-detail update
limit. Native CalcTransforms limits model-space computation by this count,
excluding all added joints at indices 94..103 when that LOD is selected.
The private rig now retains all 104 joints in that update range. Native cook,
explicit LOD coverage, 23 connected pose nodes, eight unpacked resources, five
installed hashes and 47 adapter tests pass. These offline/native gates did not
predict the runtime failure. The user's 2026-10-01 02:25 screenshot shows
`Graph active: false`, `accepted: false`, `Slider changes: 0`, requested scale
1.0, graph readback 0.0, zero pose samples, zero pelvis/root motion, and default
bone indices/counts (pelvis 0, root 0, parent entries 0). The displayed UI thumb
is 0.94. Thus the six-second measurement did not run; its zero motion/error
values are unavailable measurements, not evidence of successful tracking.
User reports the attachment follows WASD roughly but remains anchored away from
Geralt's animated hips/idle pose. The full-joint-LOD gameplay hypothesis is
not confirmed and this build is a failure, not a working fix. 0.4.16 was
measurement-only and never installed.
See REST-GRAFT.md for the shared LOD coverage requirement and adoption path.
The first-graph helper InputNode candidate 0.4.13 is held/uninstalled because
native PrepareForSample resets its input to reference pose. The private player
and parent template redirects belong only in Witcher; shared rig resources for
other characters are not overridden. See the spoke's docs/PLAYER-STACK.md.
Full controls
and dynamic pelvis remain incomplete; this newer shared stage is not in that
installed runtime. Root-pose policy belongs in Witcher; the numerical regularizer
belongs here and has an explicit adoption path for every spoke.

`REST-FRAME.md` documents the source rest-centerline/radius/length measurement
stage, with cached reference weights and the explicit prior-length fallback.
3,240 original C++ fixture rows and all 46 Base Python tests pass. It consumes
caller-supplied prepared source geometry; preceding fairing, logical/glans
construction and native output remain separate gates.

Witcher 0.4.7 FAILED despite variable accepted=true, and 0.4.8 FAILED too:
torso idle motion did not reach the replacement legs, and no resizing occurred.
Native inspection isolated disconnected cooked pose/scale inputs. The adapter
supplied compiled graph tables but omitted sourceDataRemoved=true, so REDkit
rebuilt inputs from absent editor sockets and cleared them. This is an adapter
serialization defect; the shared source shape/material laws were unchanged.

Historical **Witcher 0.4.9-connected-graph-test** native
cook retains all 94 stock pose alignments and ten named scale inputs connected
to the output. Full path/rig-name checks, six exact unpacked resources/buffers,
33 adapter tests and five installed-file hashes pass. Its menu has one scale
probe and three disabled diagnostic rows; full source controls and dynamic
pelvis remain incomplete. Installed Base revision is `95da934`, with native cage
geometry from `ca78de0`. See the spoke's docs/HANDOFF.md and
[the native graph failure/fix](https://github.com/gkoogz/TheWitcher3MaleMod/blob/main/docs/NATIVE-POSE-GRAPH.md).
Recovery baseline remains its verified 0.4.4 package. Do not reinstall earlier
deformation candidates: the strengthened native output gate rejects them.

The shared lesson for every spoke: distinguish parameter acceptance, resource
presence, a fully connected native output path and observed rendered results.
For proposed Wolverine backports, adopt the Base shape/rest/material oracles and
seam/boundary tests first; its authoritative full runtime remains unchanged.
Do not copy a REDengine serialization flag into another engine's adapter.

`PHYSICS-CONTROLS.md` documents the active eight-control material-law evaluator.
Its 600 original C++ float32 profiles and all 43 Base Python tests pass. It is a
shared numerical input layer; a complete coupled simulator and native output
remain incomplete. The Witcher 0.4.5 graph/dangle test failed observed gameplay:
the test scale did not visibly resize the mesh, and the lower body was offset.
Do not reuse that candidate as a proven deformation backend. The spoke records
the restoration and next isolated pose/output investigation.

Read `AUTHORED-SHAPE.md` for the new source-derived early coarse-shape evaluator,
1,300 original-code fixture samples, coupled graft collar domain and protected
native part boundaries. The checked correction preserves both sides of the
seam and prevents inverted triangles. An actual-Geralt synthetic stress probe
passes those invariants in both LODs but accepts only 17â€“36% of its requested
correction: the full-range deformation envelope is NOT established. Actual
guide/rest-stage completion and native output remain required. The spoke has a
native-cooked isolated scale graph; pose/dangle compatibility is not verified.

Native cage authoring now has shared donor-field transfer and two-influence
chain binding in `malemod_base/motion_binding.py`; read `MOTION-BINDING.md`.
It is an offline authoring approximation, not a replacement anatomy solver.

The next physics/menu pass has added the 18-control catalog and preference
validation in `malemod_base/controls.py`. Read `docs/LIVE-CONTROLS.md`.
`tools/export_controls.py --check` and `tools/verify.py` enforce its source and
implementation provenance. These are control mappings, not a completed native
deformation bridge. The user supplied a Witcher screenshot confirming the
installed fitted attachment's appearance; motion remains unverified. The
adapter's `docs/RUNTIME-PHYSICS.md` records native compiler probes and next gates.

The first new spoke now uses Base's `surface.rest-graft` implementation. Read
`docs/REST-GRAFT.md`, `modules/rest-graft.json` and `provenance/rest-graft.json`.
It fits an actual body opening, preserves original-edge donors and UV lineage,
transfers skin fields and repairs local orientation while locking the seam.
The Witcher adapter owns the observed Geralt frame, native FBX and rig checks.
Its preliminary official import/export preserved triangles, bones and the
continuous seam. See the adapter handoff for its latest installation and user
test state; native round-trip does not establish gameplay.

Geralt's torso/legs are separate native resources. Preserve their existing waist
join for this pelvis-only fit. If a later deformation reaches that boundary,
implement shared two-part constraints before moving either side. Wolverine can
adopt the reusable fitter and seam verification for future authoring; its current
runtime remains unchanged. Secondary motion/live collar control remain pending.

## Previous feature checkpoints (historical; not current implementation)

- Base feature checkpoint: `cb86e7edd6907f7301cd1f7bd0d25a0c40191fe4`.
  Shared generic proxy, source geometry/bindings, extracted physics/clinical
  modules and the source-derived pelvic collar are available. The collar has
  growing support and an offline coupled solve with exact donor-edge seams.
- Witcher feature checkpoint: `93ade78d5749917bfa787c4b8267c2ff4cf31fd7`.
  Official headless REDkit pipeline and a stock bare-body override were built,
  verified and installed. The user reported: "we get a barbie state. Half way
  there". Bare appearance is observed; movement, seams and armor transitions
  remain unconfirmed. Read that repository's `docs/HANDOFF.md`.
- No attached anatomy, fitted Geralt weld, live dilation, fluid runtime bridge,
  hotkey or live body editor has been implemented in Witcher. Its
  `features/pelvic-collar.json` explicitly leaves scale/frame calibration,
  seam bindings and native deformation bridge unresolved.
- Wolverine's installed runtime and canonical source were not changed by the
  extraction. Full posed runtime parity with Base remains pending.

The checkpoints above identify implementation commits. Later documentation
commits may exist; the adapter's lock file is authoritative for its build.

## Verification already recorded

Base: 27 Python tests passed; 1,024 historical native C++ collar metric cases passed;
source/provenance verification passed. Witcher: 11 adapter tests passed;
official export/import/compile/cook/pack/unbundle and package integrity passed.
These are historical results, not a substitute for rerunning relevant checks
after changes. See `docs/PELVIC-COLLAR.md` and the adapter's provenance reports
for scope and limitations. Offline math does not establish native gameplay.

## Fresh agent or another machine

1. Clone Base as `MaleMod` and Witcher as sibling `TheWitcher3MaleMod`, or supply
   an explicit Base path in the adapter's ignored `local/config.json`.
2. Read each repository's `AGENTS.md`, this file, architecture/migration docs,
   and Witcher's handoff. Inspect `git status`, HEAD and dependency lock. Resolve
   the exact pinned Base commit without discarding local work.
3. Install Base Python requirements; run `python tools/verify.py` and
   `python -m unittest discover -s tests -v` from Base. For collar work also run
   `python tools/extract_collar.py --check` and the CMake C++ collar test.
4. Follow Witcher's `config/local.example.json`, then run
   `python tools/mod.py doctor` from its root. New machines require licensed
   local game, uncooked depot and official REDkit installations. Git does not
   contain those inputs or game-derived FBX/package outputs.
5. On this machine inspect Witcher's ignored `local/installation.json` and
   compare installed files before making any deployment claim. On a new
   machine rebuild using the adapter workflow; an installation receipt is
   not recovered by cloning Git.

Base tracks the reusable reference assets. Missing source captures, skeleton
palettes, private audio, SDKs, release binaries, user settings and captured user
data are not supplied by Git; consult provenance before promising reproduction.
Another account needs repository access and its own tool/Git credentials.

## Next implementation gates

Rest fitting and native round-trip are complete for Geralt's first reference
shape. Continue with the independent motion/size output bridge and moving-pose
seam tests, then maximum expansion. Shared deformation algorithms belong here;
native Geralt bindings belong in Witcher. A compiler-accepted property does not
prove a live output. Return shared improvements to Wolverine through an explicit
adapter migration and parity checks, not by editing the legacy snapshot.

Update this checkpoint and the affected adapter's feature/provenance status
when work stops. Record completed work, remaining gates, actual verification
and installed-versus-source revisions. Keep the adapter's Base pin current for
intentional adoption; do not imply that a new pin rebuilds an installed package.

## October 1 source-backed size repair

The user reports distorted maximum Witcher size sliders in 0.4.21. Base now
provides shape_transport.py: authored coarse morph section rotations, positive
RMS dimensions and centroid offsets, with defaults and source branch knots.
See SHAPE-TRANSPORT.md for limitations and every-spoke adoption. Three section
transport tests and four existing original-C++ authored stage tests pass.
Native installation and observed gameplay belong to the Witcher checkpoint.

The next user screenshot shows 0.4.22 is closer but still has a sharp distal bend
and uneven contour. The current shared transport replaces independent shaft
section fitting with SourceRestFrame radius/span ratios in the measured export
axis. The source UI-default coarse pose and the installed large reference export
are different; raw displacement vectors between them are not valid transport.
Two crown joints now implement one similarity transform; lobe fits remain separate.
Three coherent transport tests, three source rest-frame oracle tests and four
authored-stage oracle tests pass. See SHAPE-TRANSPORT.md for remaining limitations.
