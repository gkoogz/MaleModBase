# Modular native integration checkpoint - October 16

## Installed delivery, 14:55 local

The root-anchored candidate passed the registered attachment matrix and is now
installed in Witcher. Shared pin: 16e941a. DLL SHA256:
05485dfc19ef0e4fbf7603b8ca6303202eae11cfb763e510ad67c0a75fb04d66.
The installer changed 11 coupled files and verified 31 protected files. Exact
rollback: Witcher local/native-backup-modular-20261016-145452; use the adapter's
`tools/mod.py restore-native --backup <that absolute directory>` with game closed.

Private delivery/modular-acceptance.json binds the source manifest, all coupled
artifacts, 32 size/state/angle configurations (96 views), 36 same-control motion
views, 39 campaign views, and nine unobstructed LOD views. Reviewed native close
side/front/oblique appearances retain the continuous root and shared waist.
Thirty-two configuration meshes and 12 motion meshes preserve exact original
pelvis points, with zero reversed ramp triangles and zero nonmanifold edges.
Both resources/LODs completed actual GPU draws. Sessions 23, 24, 25 and 27 are
closed, each with 24 original save files verified and scoped audio restored.
The rejected obstructed far views in sessions 23/26 are replaced by session 27;
only distant cameras are level, while close profiles retain anatomical alignment.

This is sampled regression acceptance, not an exhaustive geometric guarantee.
The extreme short/wide/minimum-head combination retains distal surface wrinkles
outside the pelvic collar. Global self-intersection, every possible slider
combination, garment behavior and performance parity are not claimed. The full
Base extraction verifier's unrelated motion-header mismatch remains. Wolverine
and other game installations are unchanged.

The installed-path startup check completed 21 campaign captures (300-320), using
the actual installed DLL and normal startup helper without injection or host
input. Session 28 is closed, audio restored and 24 original saves verified.
All 49 managed installed file hashes and seven protected audio/control/settings
hashes match. No Witcher process remains. Evidence is in private
delivery/post-install-verification.json. Interactive desktop startup was not
observed. Older pending/uninstalled notes below are historical.

## Current checkpoint, 14:00 local - supersedes the historical notes below

Shared revision 16e941a anchors simulated root motion to the selected rest/control
frame. The constraint releases smoothly over two radii, capped at one quarter of
shaft length; distal frames and independent lobes remain free. A captured
maximum-size semi-rigid upward pose had seven reversed ramp triangles because
the first simulated tangent rotated against the fixed pelvic aperture.
Replaying all 32 native settings with this fix gives zero reversed ramp faces,
exact original body positions, 78 original external edges and no nonmanifold
edges (private dynamic-anchored-23/audit.json). This is offline evidence, not
native acceptance of the changed DLL. Final native tests are pending.

Shared MMXFER02 transfer now isolates cross-sectional shaft residuals from
duplicated legacy axial length/root corrections, while retaining crown-to-tip
axial morphology through R14 lineage. Use runtime-03/display-axial-v2.transfer.
The new neck uses an intrinsic harmonic UV field with exact boundary values;
this removes nearest-projection atlas jumps without replacing textures or moving
geometry. Maximum adjacent axial UV jump fell from 0.14535 to 0.00888. Use
native-material-02/geralt.modular-materials-v5. All other geometry, mount and
native binding paths below remain current. Numerical worker/profile retain
their own 431e626 revision; the adapter separately pins modular headers.

The adapter now recognizes actual engine LOD1 draws through rebased short index
views. NativeIndexFirst validates the owned allocation and normalizes offsets.
Session 20, sealed-session-20261016-133902/PID 30140, completed 96 captures at
32 settings, with both body resources and both native LODs (GPU mask 15).
It is closed with 24 saves verified and scoped audio restored. Evidence is
session-evidence-20.json in the private build root. Its pre-root-anchor DLL is
NOT accepted: case 630 contained the seven ramp reversals described above.

Three shared motion/cut/stitch tests, 16 focused Python authoring tests, native
index-view tests and nine installer-gate tests pass for their latest changes.
The full Base verifier still fails on the pre-existing source extraction/motion
header mismatch. No full-suite pass is claimed. Native build manifests bind
actual compiler inputs and all 15 coupled runtime files, including worker,
profile, bindings, geometry, materials, transfer and hash sidecars.

Remaining: final root-anchored native matrix, same-control animated motion and
campaign validation, then exact acceptance proof and transactional install.
Review the extreme short/wide/minimum-head distal surface, which previously had
distal folds distinct from the collar. No global self-intersection or continuous
slider-sweep proof is recorded. Installation remains gated on actual reviewed
native appearance, both LODs and verified save/audio cleanup. The accepted DLL
and rollback below are unchanged. No earlier snapshot is final acceptance.

Wolverine and future spokes adopt these same shared contracts through their own
measured bindings and native attachment gates; their runtimes remain unchanged.

## Historical checkpoints (superseded)

## Follow-on checkpoint, 11:10 local

Session 24588 closed successfully. Angular clearance improved the shape but
rigid lobe blending visibly folded retained suspended skin. New shared
modular_transfer.hpp and tools/build_modular_transfer.py transfer actual solver
surface deformation onto 7159 retained shaft and 8400 suspended vertices.
Generated connecting skin remains independently evaluated. Native angle metadata
rotates shaft source positions around the exact same root as its guide. This
also restores source shape detail (including glans) instead of only radius and
length. Artifact runtime-03/display.transfer SHA256
88615aec6caeec0a0e48ca489eb59d98e6e3998e7aaf29715618bcae6a5be926.

Session 9796, sealed-session-20261016-105847, completed eight live side captures
with this transfer. Retained skin no longer showed the large folded flap seen
in 24588. It is closed with saves/audio restored. Visible material/normal seams
at the neck and root remained; no installation or acceptance.

Latest edits additionally transfer measured normals, project connecting-skin
endpoint derivatives into those tangent planes, and use the module's proximal
UV chart across the root ramp (radial body-chart sampling still stretched).
The aperture now follows the requested source anchor before its fitting shift,
so Forward/Vertical changes cannot leave the cut at the old position. A final
rebuild/test/private launch is running via launch-dynamic-06.py; paths will be
in session-dynamic-06.txt and preview-dynamic-06.txt when launched. Inspect those
files/process ownership before launching anything else. New changes are not
yet visually accepted. The earlier 57-frame matrix predates these fixes.

Status: active development, not accepted and not installed. The user requires
a playable implementation, original pelvis preservation, a compact seamless
root ramp, independently moving suspended skin, and live evidence. Continue
implementation rather than presenting this checkpoint as delivery.

## Shared implementation

`modular_motion.hpp` separates complete-shaft and lobe ownership. Connecting
skin uses endpoint-driven Hermite lofts. A control/rest frame now owns the
aperture independently of moving physics frames; movement does not recut the
body. `convex_surface_cut.hpp` clips the measured original body without moving
its points, retains parent-triangle barycentric donors, conforms shared edges,
and rejects branching/nonmanifold openings. `surface_stitch.hpp` refines unequal
loops onto common angular knots and produces a manifold connecting strip.

`pelvic_aperture.hpp` fits the actual section to the measured front surface.
The lowest fitting anchor is selected, excluding nearly tangent perineal
surfaces. `modular_dynamic.hpp` uses angular clearance rather than translating
the entire root by the worst compressed-arc clearance. A periodic nonnegative
support envelope bounds the required clearance while leaving the opposite arc
free. The former fixed-opening body-stretch approach is rejected.

Private data root: D:/MaleModBuilds/modular-anatomy-20261015. Geometry is
runtime-03/display.modular (11c2d8fb5fd301b475d6f38caa45274c92a88397a15489546aa18b9a2109def0),
dynamic mount aperture-02/dynamic.mount, native carrier material binding
native-material-02/geralt.modular-native. Current material fields are
native-material-02/geralt.modular-materials-v3. Earlier material files are
diagnostic artifacts, not accepted replacements.

## Native adapter

Witcher renders replacement topology through the observed native pipeline with
32-bit indices and both body resources. Measured native winding is opposite
the authoring winding. Cross-resource ramp faces use an observed palette that
contains their source joints, including upper-body joints at maximum size.
Both LOD layouts are populated; this is not a claim of distant-LOD validation.

The modular source path uses its own root pivot, no longer the dorsal pivot or
angle feasibility limit of the superseded collar. The original adapter path
is retained when modular artifacts are absent. Root UV transfer now follows
the original radial body chart, with actual ramp UV-derived tangents, rather
than extrapolating each body triangle along the extrusion. Completed shaft UVs
have axial extent; new neck UVs interpolate their independent endpoint charts.
Native appearance still requires review, especially material transitions.

## Evidence

Three direct MSVC shared tests pass: modular_motion, convex_surface_cut and
surface_stitch. Full CMake configure remains blocked by the pre-existing
motion-header/source-extraction mismatch; do not erase unrelated motion work.

dynamic-04/audit.json contains four focused width/angle cases: zero reversed
ramp faces, zero moved original vertices, 78 original boundary edges, no
nonmanifold edges. dynamic-native-04/audit.json repeats this for two actual
native source captures at maximum size. These are local strip/original-surface
tests, not global intersection or visual acceptance.

Native test 23840 (sealed-session-20261016-101002) completed eight side views;
rejected material stretching. Native test 4272
(sealed-session-20261016-102459) completed all 57 repair-review captures across
size, state, angle and locomotion, with both resources rendering and about
22-37 ms modular CPU evaluations. This used the older global clearance;
its bulky ramp and material seams are not accepted. Both sessions were closed
with their exact receipts, restoring all 24 original saves and scoped audio.

Current native session 24588 (sealed-session-20261016-104528) is testing the
angular clearance and radial chart/tangent changes. Its exact preview and
receipt paths are in private preview-dynamic-04.txt and session-dynamic-04.txt.
Close it using run_iteration_world.py --close and its receipt before another
launch. Never use physical desktop input. Native CPU test against captured
maximum-size source passed independent lobe movement and invariant topology
under physics; cached evaluation was 23.3 ms, shape build 1.53 seconds.

## Remaining acceptance work

Review current native captures and correct material/normal seams. Extend
control transfer: the new display presently consumes guide/length/radius/lobe
data, so source shape detail such as the glans morph needs explicit transfer.
Check Forward/Vertical anchoring, shape-cache stability, moving connecting
skin, all supported LODs and campaign behavior. Root placement currently finds
a fitting anchor in discrete search steps; examine slider continuity. Validate
binary input fields and package the coupled contracts with pinned Base adoption.
Wolverine and other spokes adopt the same shared code after their own native
binding and attachment gates; do not copy it into a separate implementation.

Installed DLL remains
2fde715c2f34635e77723d88d25a7948f94635637eb52cb5aa1aa5e47fcd49c4.
Exact rollback remains in Witcher local/native-backup-root-20261014-194227.
No candidate has passed the full attachment-integrity gate or been installed.
