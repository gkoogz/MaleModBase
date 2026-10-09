## October 8 - installed local walking and whole-garment material follow-up

The user confirmed that white/grey switching affects the whole jockstrap.
Installed developer runtime: 25dc08422d5bc6585d704ccd937a648356d3d0e10b134ba59bd94dbaff6786f9,
only in %LOCALAPPDATA%/MaleMod/MeridianPrototype. The prior 29ebe88c runtime,
manifests and native evidence are preserved in rollback-performance-20261008-183418.
Rollback validation passed; current WolverineLive.ini and TeachingFluid.ini
hashes are unchanged. Normal retail runtime remains 4f4900a5. No commit or push.

Base now refines the two curved testicle solids with 48 virtual longitudinal
intervals and curvature-weighted resampling of the existing surface rows.
The fixed 32 rays, 24 render rows, 2471 vertices and 4816 triangles are unchanged.
No proxy rendering or cadence reduction. Exact 16-pose six-repeat CPU replay:
mean total 5.63073 -> 5.78195 ms; same fallback counts, no outer failures.
This is post-skinning CPU timing, not game FPS. A new SDK-free sub-row curved
obstacle regression passes and fails against the previous installed header.
Existing clearance, seam, continuity and runtime tests plus 17 Python guide/
surface tests passed. Base import/provenance verification passed.

Wolverine owns explicit U/V/W clamp, zero mip limit/bias, fixed filters and
linear sampler state, visible-face normal handling, and native inverse scene
depth in gameplay alpha. The measured gameplay vertex/pixel disassembly confirms
the clip-w inverse-depth contract. This supersedes the earlier gameplay-opacity
exception. These fixes cover the waistband and straps as well as the pouch.
They identify material-contract defects, not a proven cause for every reported
flicker. Native diffuse shading still changes with pose/light; human-play
confirmation of the complete flicker repair remains pending.

Final native run 20261008-182045-e3cc25 completed all 18 size/width/angle/state/
naked cases with zero reported draw rejections and exit0. Front, side, actual
front/rear oblique and walk/jump/stop captures were reviewed. Last cumulative
sample: 1921 attempts, 1838 wraps, 83 transported, 82 uncertified; Draw mean
5.9645 ms, max27.451 ms. The earlier five-case sweep stopped on a missing queued
capture log despite a completed native capture; the writer now accepts the
completion receipt. The complete rerun is the sampled evidence.

Tank diagnostic run 20261008-183018-8df021 audited 7921 material passes with zero
state mismatches. Its private FOV130 inspection preset was restored byte-exactly
to FOV90 afterward. Face maps remain aligned. Title overlays obscure the pouch,
so these captures do NOT establish tank cloth visual acceptance. This also
clarifies the earlier 29ebe88c note: its cited title images showed face/body,
not a clear unobstructed pouch inspection. All owned runs exited0; no host
keyboard/mouse/focus or human-play process was driven.

The full attachment gate remains incomplete: extreme fit/faceting, uncertified
transported contact, all coupled controls, body resources/LODs and campaign
transitions remain unverified. No accepted runtime promotion. Source, offline,
native and human verification are separate. Witcher remains paused; future
spokes must consume a tested committed Base pin and measured binding revision5.

Private build/evidence/install receipts:
Base build/fidelity-20261008/{candidate2,native-evidence.json,install-receipt.json}.
Shared implementation: Base docs/MERIDIAN-CURVATURE.md.
Adapter implementation: Wolverine docs/NATIVE-CLOTH-FIDELITY.md.

## October 8 - posterior trapping, garment continuity, and even room lighting

The user tested the October 7 developer prototype and reported persistent
posterior shaft trapping and frequent garment disappearance during movement.
They also requested more even lighting in the grey room.

The shared anterior-envelope query now represents the connected tissue between
the two independently colliding lobes. Its bounded, nonimpulsive correction and
mass-weighted relative-velocity projection preserve tangential swing. The native
adapter suppresses an opposing posterior contact only where this anterior web
owns that contact, and includes the previously omitted terminal rod segment.
Existing bend, spring, gravity, damping, and animation parameters are unchanged.
The opt-in candidate consumes the new Base header. An older pinned Base keeps
its original path; this must be explicitly adopted and pinned before a normal
release. The immutable reference snapshot was not edited.

The pouch now retains a valid material surface in a body-boundary frame and
transports it with current hem and rig controls when the meridian optimizer
fails. It attempts bounded current-pose contact repair instead of skipping the
draw. Initial frames use the authored skinned cloth until a valid surface exists.
Contact-certificate failures are counted separately: continuous visibility is
NOT proof of perfect cloth collision, self-intersection freedom, or skin coverage.

Native candidate fix4 recorded 1,561 draws for 1,561 evaluated garment attempts,
including 127 transported poses; 115 fallback poses lacked a complete collision
certificate. Its 18-case size/width/angle/state/naked capture sweep had zero
reported draw rejections. Combined maximum size still produces poor cloth shape.
The command writer hit a Windows replacement sharing race during cleanup; it
now uses atomic File.Replace with bounded retry, and cleanup was completed.
This is a test-control change, not a gameplay input override.

The SDK-free recovery fixture reproduces a shaft remaining behind the lobes
without the anterior guard, and validates recovery plus a separate sustained
motion-forcing case. The tracked Wolverine recipe is
tools/iteration/verify_anterior_recovery.py. Its output explicitly separates
numerical proof from native/visual acceptance.

The final lighting recipe is --studio --even-lighting: upper/lower sky fill .85,
a weak .035 directional key and four shadowless .10 azimuth fills. Native front
and rear inspection showed much more even illumination; the intermediate sky-
only variant still left the back too dark and was not selected. The ordinary
studio recipe still regenerates its original exact SHA256. The new layer's
SHA256 is 10adb25651b10b14db1ee490feeb17b0ae300ed558c2b507d16f9c2a5789d5eb.
Each prototype gets its own mutable layer; shared assets are immutable hard
links on E:, reached through a private CookedPC junction. The accepted room's
package is not overwritten.

Final candidate: build/bugs-20261008/fix5, runtime SHA256
9e42b4d47c36b972945c4f991e3708cfee78638d75633f0bf6eb6ff1fe98e5a5.
It exports Wolverine 16affd49da5a122df776969574ca62a984b59425 and Base
99ff741ea95f18ed84526c35a6c3f38a37d857b6, with explicitly hashed current
Base/adapter overlays. Unrelated dirty worker changes remain excluded.
Final exact-build verification completed: all 18 native cases captured with zero
reported draw rejections; walking/jumping/stopping and default appearance were
reviewed. Last cumulative sample: 1,921 evaluated attempts and 1,921 draws,
88 transported poses, 76 without a complete collision certificate. Draw mean
14.72 ms, maximum 56.89 ms in the sealed room; this is not a campaign FPS claim.
The final SDK-free fixture reproduced posterior trapping with the guard disabled
and passed enabled recovery and independent motion-forcing tests. The enabled
recovery ends 7.39 units anterior with .185 units of lobe drift.

The Desktop Wolverine Jockstrap Prototype now selects fix5 and even-v3.
Its settings hash is unchanged; exact prior runtime, stage, manifest and a
validated rollback script are retained in MeridianPrototype/rollback-20261008.
Launch-contract validation passed. Human play of this new revision has not
been observed. The prior user-play report belongs to candidate p.

Attachment status remains incomplete: default native naked/front views and
the candidate matrix are evidence, but not a pass for every coupled control,
body resource/LOD, moving front/side/oblique view or campaign transition.
No regular retail/accepted-human runtime promotion, Git commit, push, or release.
Witcher remains paused. Shared numerical/continuity modules are ready for its
future pinned adapter adoption; no Witcher gameplay claim is made.

## October 7 - textured developer-play checkpoint; NOT accepted

The user requested in-game play, soft ribbed white pouch fabric and one thin red
and one thin blue waistband stripe. Selected developer candidate: native-build-p,
runtime SHA-256 a83dcb200e44b7313550b7ce89fdc693a90072891fa3ac3f733c77f9ef8627a4.
Source headers were restored to that exact tested candidate after rejecting
later experiments. The accepted retail/human runtimes remain unchanged.

A separate human-play prototype is prepared and was launched at
%LOCALAPPDATA%/MaleMod/MeridianPrototype. Wolverine's
tools/iteration/Play-MeridianPrototype.ps1 verifies its exact runtime, executable,
authored floor and private checkpoint. It uses physical input without the agent
input shim, timer or automatic pause. It does not change the guarded accepted
Play-NativeSandbox.ps1. Start at Overall/Width/Angle 50. This is developer review,
NOT a passed attachment gate. Human interaction must be recorded separately.

Native evidence: 18 cases captured at 1920x1440 in
live-room/runs/20261007-183518-437274/material-p. Three cases contain reported
rejection lines: combined-max, return-default and mechanical-2. Logging is
throttled; this is not a failure-frame count. The default frontal appearance was
inspected: white pouch/trim with thin continuous red and blue stripes.
Combined-max is visually REJECTED: jagged/distorted cloth even when some frames
satisfy proxy separation. Full motion, LOD and campaign checks are NOT accepted.

Base owns shared wrapping, analytical proxy supports, whole-triangle separation
and meridian_material.hpp fabric intent. Wolverine owns the observed two-layer,
seven-row waistband mapping, native buffers and meridian_material.h. Its
derivative-filtered rib shader uses reflected LightDirection, LightColor and
AmbientColorAndSkyFactor, plus seven WorldIncidentLighting vectors and the two
native SH basis cube samplers. SH evaluation follows measured native shader
disassembly. This implements scene directional/SH diffuse lighting, not full
spotlight/shadow parity. Close-up weave review remains incomplete.

Skinning now interpolates individually skinned donors plus transported offsets;
the prior method introduced cross-donor terms by skinning an interpolated point.
Current HDR palettes, per-section actor/view matrices and matching targets are
required; all donor points are converted into anatomy actor space before fitting.
There are 3623 draw vertices, 7072 triangles, 1801 native skin samples; cloth alone
has 1921 vertices/3760 triangles. Collision guides are numerical only. No overall
frame-rate improvement is established. Runtime adaptive origins are still pending.

Later q/r repairs attempted cap-directed seam fitting, repeated convex resampling
and preserved contact knots. They exposed nonconvergence, costly iterations and
invalid extreme output. Projection and oval-loft trials did not justify replacing
p. These experiments remain only in ignored build output. Do not call them fixes.
Complete actual-skin containment by the proxies must be checked; proxy separation
alone is insufficient. Strict geodesics, full self-intersection and the mandatory
attachment gate remain unproven. Witcher adoption requires a tested pinned Base
revision and its own measured bindings/material adapter; no Witcher install.

### Selected checkpoint validation

On October 7 the selected p developer runtime was opened in a normal visible
classic-D3D9 human process, without agent input. Its title render reported
successful cloth draws (7072 triangles); no user-controlled gameplay action was
automated or claimed observed. The desktop shortcut is Wolverine Jockstrap
Prototype. Enter, Continue, F6 controls. This launch is not visual acceptance.

The 21 Python collision/outline/taut-guide/surface tests passed. Selected shared
clearance tests passed on optimized MSVC Win32 and x64; the x64 donor transport
test passed. tools/verify.py passed all 516 imported files/materials, 185 source
arrays and geometry/graft/collar/physics/clinical provenance. Both repository
diff whitespace checks passed. No commit, push or normal installation was made.

## October 7 - live meridian wrapping candidate, not an accepted installation

The current request is to make the jockstrap work in Wolverine. The previous
native donor-only mesh was insufficient: it did not rewrap or check collision,
and independent interior skin donors produced conflicting deformations. Its
previous screenshots are not acceptance evidence for this new implementation.

New shared code is in `meridian_clearance.hpp` and `meridian_rig.hpp` beside
`meridian_runtime.hpp`. Wolverine's `meridian_adapter.h` supplies actual body
and anatomy bindings, current bone palettes, and the existing CPCenter/CPRadii/
cpBasis physics values. Interior cloth is now rebuilt from the current rig and
sewn outline, not independently skinned to nearby anatomy triangles. Seven
perfect circular sections share six tapered links and an aligned rounded dome.
The two 3-percent-expanded tapered ovoids use the current physics center, axes
and radii. A common native skin mapping per ovoid preserves concentricity.

The runtime fits the sewn outline against both point and complete-edge contact,
applies the same bounded displacement to its hem/nearby trim, repairs small
longitude-order reversals, constructs outward meridian envelopes, and certifies
every cloth triangle against every convex support solid. Analytical support
functions replace scans of tessellated collision objects. Near attachment and
tip, correction can move axially within the same longitude plane; radial-only
correction incorrectly rejected valid cap-adjacent paths. The cloth terminal
point may advance for thickness/clearance; the blue dome's apex is unchanged.
The strict global geodesic and universal convex-turn claims are NOT established.

Geometry/binding contract revision 5: 80 columns, 24 rows, 1,921 welded cloth
vertices and 3,760 cloth triangles. Twenty-four render-only UV seam aliases share
positions and summed normals with the welded grid. Total draw: 3,623 vertices,
7,072 triangles, one opaque pass. Only 1,801 samples need native skinning;
collision tessellation is reconstructed numerically and never rendered.
Covered anatomy is hidden only after THIS FRAME's cloth draw succeeds. Failed
buffer locks release the incomplete buffer. Failed numerical poses retain the
full anatomy draw instead of claiming a valid garment. The original source
anatomy, collar donors, body resources and numerical mechanics remain intact.

Current native builds and captures are private under
`build/meridian-runtime-20261007`. Build derivatives still export accepted
Wolverine 16affd49 / Base 99ff741e and add individually hashed experimental
headers; unrelated dirty worker/solver changes are excluded. No normal retail
or accepted human sandbox runtime has been replaced. Run the shared
`profiles/meridian-regression.json` through Wolverine's repository-owned
`Test-MeridianCandidate.ps1`; it records requested controls, native captures,
reported numerical errors and explicit unreviewed visual status. Optional
RenderWidth/RenderHeight on Open-NativeSandbox allow higher-resolution native
captures without touching host input or focus.

The full attachment gate, campaign, supported LOD matrix, runtime adaptive
origin redistribution and blanket self-intersection acceptance remain pending.
Do not promote this candidate merely because compilation or support-plane tests
pass. Source/offline/native/visual evidence are separate. See
`docs/MERIDIAN-RUNTIME.md` for the latest measured run and limitations.

## October 7 - meridian surface engine performance prototype

The user's current request is native performance iteration, including lower
cloth resolution and omitting hidden anatomy/inspection guides. Shared Base
meshing now supports cosine-spaced rows clustered at BOTH seam and rounded tip.
Uniform coarse grids either failed pinned-seam clearance or ballooned locally;
40 angular columns cut the attachment boundary and were rejected. The selected
80-column / 24-row cloth has 1,921 vertices and 3,760 triangles (85.2% fewer
cloth triangles). All static triangles clear the original nine blue proxies.
Only the new hem ribbons are reduced to 33 rows. Original waistband/glute
geometry is retained. Cloth plus all fixed trim: 3,599 vertices / 7,072 triangles.

Base `include/malemod/garments/meridian_runtime.hpp` provides SDK-free source
triangle donor transport with rotating offsets and rebuilt mesh normals.
Wolverine owns donor IDs, bone palettes, source extraction and Direct3D buffers.
The private native candidate uses one opaque pass, a reusable dynamic VB with
DISCARD, a static IB, and no blue guide draw. It suppresses 31,998 classified
anatomy triangles only after cloth drawing succeeds, retaining 3,002 collar/
noncovered triangles. It preserves full numerical anatomy, seam donors and
pressure support. It bypasses the retired garment worker/solver entirely.

Observed in an isolated native grey-room copy: cloth drawing, acknowledged
Overall 1 / 50 / 100 changes, camera orbit, walking, and return to default50.
Twenty-one Python geometry tests and the standalone Win32 C++ transport test
pass. Actual 2,000-update optimized Win32 CPU benchmark: mean .557 ms,
p95 .673 ms, max 1.280 ms. Native cumulative update mean reached 1.536 ms
(max 2.363), draw/pose/upload/submission mean .812 ms (max 1.220). These are
inclusive CPU measurements, not GPU timings or an FPS gain. Native complete
frame samples remained roughly 70-73 ms; the existing full anatomy surface/
physics pipeline remains the main cost. There is no controlled before/after
FPS claim. No normal retail or human sandbox runtime changed.

This is donor-driven kinematic cloth, not a new dynamic cloth solver. The
adaptive ray distribution and collision certificate are authored offline.
Live path redistribution, dynamic whole-surface collision/self-contact and
fresh rewrapping after morphology changes remain unresolved. Wide native views
are partial visual evidence, not the full attachment gate: Width combinations,
all mechanical states, extreme rest angles, close front/side/oblique collar
views, body-resource/LOD coverage and campaign transitions remain untested.
The candidate is explicitly unaccepted; retain the previously accepted runtime.

Native tested candidate SHA256:
B09DF57440807E1161EAFE5E10BD9C5047C41B7FC7D3065D87CD50CDBAE31556
Base output: `build/meridian-runtime-20261007/` (private generated assets/evidence).
The explicit private overlay uses accepted Wolverine16affd49/Base99ff741e
plus individually hashed new shared header, adapter and recipe. Other dirty
cloth work is excluded. Native child ended at the configured 300-second limit
(status124); owned audio restoration returned HRESULT0 and the child is absent.
No host keyboard, focus or audio settings were changed. Normal runtime hashes
were rechecked unchanged. There is no release or commit in this iteration.

Adoption: author/validate a versioned geometry+donor recipe, commit/pin the
shared Base revision when requested, bridge it through each adapter, and pass
its full native attachment/motion/campaign gate before normal installation.
Witcher remains unchanged; it must supply its own measured donors and export/
renderer bridge rather than copy Wolverine buffer layouts or source IDs.

Reproduction (Base working directory, Python310):

```powershell
python tools/benchmark_meridian_lods.py --inspection build/collision-chain-white-cloth-20261007 --output build/meridian-lods
python <Wolverine>/tools/author_meridian_runtime.py --base . --inspection build/collision-chain-white-cloth-20261007 --lod build/meridian-lods/cloth-24x80.npz --body-input <measured-body-input.json> --anatomy <paired-surface.xyz> --output build/native-meridian-recipe
python <Wolverine>/tools/iteration/build_private_runtime.py --base . --source-ref 16affd49da5a122df776969574ca62a984b59425 --meridian-recipe build/native-meridian-recipe --output build/fresh-meridian-runtime
```

The recipe needs the developer's matched source export, not somebody else's
capture bank. Generated geometry, licensed native inputs and verification data
stay private. This experimental runtime is opt-in only through
`MALEMOD_MERIDIAN_CANDIDATE=1` in an owned isolated room. Ordinary installers,
Base pins and human sandbox launch contracts are unchanged. Use the native
room Open/Close/Capture tools; never launch human play over the user.

The static authoring check certifies the convex blue proxy union only. Donor
transport is deliberately cheap and does not carry that certificate through
arbitrary deformation. Publication must follow a successful complete update;
failed donor geometry rejects the candidate and restores ordinary anatomy
visibility. GPU submissions are asynchronous, so CPU draw times are not GPU
cost. See Microsoft's [D3D9 buffer guidance](https://learn.microsoft.com/en-us/windows/win32/direct3d9/performance-optimizations).

The private clone used for this iteration is reproducible with
`tools/iteration/Create-MeridianTestRoom.ps1 -SourceWorkspace <fresh-room>
-Destination <new-owned-path> -CandidateRuntime <candidate-d3d9.dll>`.
Use a room regenerated from the licensed vanilla installation and repository
recipe. This helper copies small program/config files and the engine-generated
fixture checkpoint, junctions read-only content, gives the child its own logs/
mutable saves, and leaves the source room untouched. Activate the opt-in env
flag only for the owned Open-NativeSandbox process. Captures and generated
recipes remain private outputs, never release payloads or source dependencies.
The tested candidate builder's individually hashed overlay is recorded apart
from its accepted source/base commits; unrelated legacy dirty work is excluded.

Current canonical Wolverine main remains 16 commits ahead of origin/main;
both repositories retain pre-existing dirty work. Nothing was committed/pushed.
