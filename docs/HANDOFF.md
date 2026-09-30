# Resume here: cross-repository handoff

Checkpoint: 2026-09-30. This file describes repository state and recorded
observations, not a promise that the current machine still matches them.
Chat history is not required to resume. Verify Git and local inputs first.
Read [PROJECT-CONTEXT.md](PROJECT-CONTEXT.md) for the user's stated postgraduate
educational purpose and enduring requirements before interpreting this project.

Latest user correction, September 30: Witcher 0.4.1 was observed deformed and
F6 unresponsive. Treat it as a failed gameplay test. A rest-frame/input repair
is underway in the spoke; consult its handoff for the actual installed version.

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

## Previous feature checkpoints

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
