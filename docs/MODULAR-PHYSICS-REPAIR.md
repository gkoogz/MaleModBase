# October 16: post-install modular motion repair (in progress)

User reports walking compression, sluggish updates, misshapen glans, visible
rings and a rigid proximal segment. Installed repair DLL is745c52e8...; exact pre-repair05485dfc rollback is retained.
Installed-path startup was observed and its private session closed cleanly.

Shared Base commits e499d2a, f2038f5, 06f995b, 7398eac separate a dynamics-free
material shape from legacy moving skin, use continuous section frames, preserve
section diameter and segment length, retain a rigid crown, and shorten the root
constraint from up to a quarter of length to one guide interval. Root clearance
is applied before material deformation. Numerical worker remains 431e626.

Witcher caches fresh zero-motion source morphology per controls. Modular body
replacement bypasses the superseded target/body reconstruction. ModularService
runs geometry separately; PresentModular uses portable material-frame temporal
transport. Native LOD shading is cached across identical aliases. Original body,
UVs, maps and scrotal skin source are preserved.

Measured baseline walking: 5.7 solver publications/s, 9.4 display publications/s,
median source110 ms + target59.64 ms. Session29 removed target cost (~0.1 ms),
reached14.4 solver/s but retained stepped bends. Rejected. Session30 addressed
material station remapping and root-offset transport; then31 separated expensive
geometry/display, reaching about28 display publications/s (partial run metric).
All29/30/31 sessions closed, original saves verified, audio restored.

Session32 was closed after87 captures (29 configurations), before all96; it is
not a complete matrix pass. Audio was restored and the installed DLL remains
05485dfc. Its tip showed a flattened dome and artificial upright rim.

The resumed investigation isolated three interacting causes:

- Rest morphology incorrectly went through live axial suppression.
- Rigid ownership began inside the authored distal component, separating its
  rim and tip. The field now covers that entire authored component and fades
  across the adjacent shaft using original surface distance and source lineage.
- Initialization transferred the surface between nonmatching shaftGuide and
  restGuide stations. Late intervals were shortened before animation began.
  Cache the zero-motion surface with its own matching guide frames instead.

Base77b3750 fixes rest transfer and adds its regression test. Base8350f0b adds
selected-material station binding, complete distal ownership and regressions.
Witcher consumes8350f0b; numerical worker/profile remains431e626. The shader,
maps, UVs and accepted body geometry are retained. No template substitution or
experimental canonical fit is active. Native preview now permits validated
coupled artifacts to replace their managed counterparts in the private copy.

Replay40 passes29 captured size/angle cases: zero reversed collar faces,
unchanged original body vertices,78 boundary edges and no nonmanifold edges.
The matched-guide diagnostic removes the distorted rim while preserving source
shape. This is offline replay, not a new native attachment gate pass.
Motion/transfer/ownership regressions and the native temporal90-degree diameter
test pass. Candidate41 DLL SHA256 is
745c52e8a81e6a5f7eddc2e1547e8dee4b96baf4d3e754b24874962017eb4fe6.
Its 36 walking/settling captures were reviewed. Twelve captured material guides
retain interval lengths to 1.34e-15 relative error; the first free interval moves
1.82 to 40.72 degrees relative to rest. No accordion compression was observed.
Measured display publication rate is 31.49 Hz, numerical publication rate13.28 Hz
in this screenshot workload; these are not60 Hz or general performance claims.
Median source55.08ms, target0.10ms, preparation16.03ms and modular37.41ms.

Full42 completed96 captures/32 configurations. All contact sheets were reviewed;
close cases0-28 retain the collar. Cases23/24 retain distal creasing at extreme
short-length/head-size combinations. Far cases29-31 are camera-obstructed and
are replaced by distant43: all9 level front/side/oblique views reviewed at4/8/15m.
Both body resources and both LODs completed GPU draws. All32 captured geometry
audits preserve original body points, have zero reversed ramp faces,78 boundary
edges and zero nonmanifold edges. Motion41, full42 and distant43 are closed;
audio restored and24 original saves verified for each. Campaign44 completed39
captures, all reviewed, then closed with audio/saves restored. The hash-bound
registered attachment gate passed and the candidate was installed with31 protected
files verified. All49 managed hashes match. Exact rollback is adapter-local
native-backup-modular-20261016-173954. Installed-path startup45 produced six
reviewed campaign images at the two angle extrema, without a preview override.
Session sealed-session-20261016-174037 is closed; scoped audio restored,24
original saves and7 settings/control files verified. No Witcher process remains.
This is installed-path startup in a private desktop, not observed user-interactive
startup. Proofs are physics-repair/delivery/modular-acceptance.json and
physics-repair/delivery/post-install-verification.json.
Source manifest build-source-41.json records110 verified compiler inputs.

Private evidence/scripts: D:/MaleModBuilds/modular-anatomy-20261015/physics-repair.
Current private launch helpers pass the full coupled artifact set. Native preview
classifies validated managed artifacts as replacements inside the private copy.
Close grey-room runs with tools/run_iteration_world.py --close <receipt>.
Campaign close additionally restores its campaignSaveTransaction. Never use host
input/focus or install over a running game. Handoffs from prior installation
remain valid for the accepted baseline, not this pending replacement.

Wolverine and other spokes should adopt the shared material/ownership contracts
through their own pins and native gates. Their installed runtimes are unchanged.
