# Root articulation and stable junction ownership

The educational purpose remains in PROJECT-CONTEXT.md. The user rejected the
previous spatial hinge and asked for rotation at the actual root, no upper
saddle, greater upward/downward travel, and real side renders before installing.

## Causes established

The old hinge suppressed rotation for approximately one shaft diameter beyond
the seam. Its stationary half-space also stopped upper tissue moving back as
the shaft rose. A root-located numerical origin therefore still produced a
distributed bend and a stationary neck. Merely shortening the field compressed
the lower transition instead of fixing its ownership.

The first replacement chose moving rows from their current height. The live
flexible state could move shaft vertices below that cut, incorrectly returning
them to the junction solve. Initializing such a cut once was also wrong: the
default neutral source is Full Floppy and its distal guide already droops.

## Shared solution

`root_articulation.hpp/.cpp` implements rigid rotation about an explicit measured
root and a separate junction displacement extension. Rigid handles preserve
their full cross sections; neither a spatial angle ramp nor a pelvis smoothing
mask is applied to those handles. Body/core rows are exact fixed constraints.

The remaining connected material minimizes the squared degree-normalized graph
Laplacian of displacement. Boundary rows on both sides participate in the energy,
so the extension continues the boundary displacement gradient. It retains the
reference surface detail; it does not smooth the complete anatomy or shrink the
rigid shaft. Positive inverse-edge-length weights define the material metric.
Existing exact-donor endpoint constraints reject new folds and collapsed faces.
A difficult endpoint first uses continuation along the same rigid angle path;
only junction repairs are carried forward between stages. This is not proof of
global intersection freedom or a repair of all pre-existing defects.

Both modules remain SDK-free. The optional build uses the existing Eigen
dependency. Tests exercise root invariance, equal proximal/distal rotation,
exact girth, fixed boundaries, identity, live vertices crossing their original
height, continuation endpoints, and conflicting ownership rejection.

## Witcher calibration and data

The adapter uses the measured dorsal attachment vertex, with the lateral axis
centered on the source guide. It is an attachment-edge hinge, not a mid-shaft
bend or a claim of medical validation. The source/collar still evaluate at
reference control 35; requested control 50 presents that canonical pose.

MMPREC03 adds unsaturated source lobe participation to the existing versioned
binding/longitudinal chart. Source interpolation can extrapolate slightly;
classification scores are bounded to [0,1], with raw ranges/counts retained in
artifact provenance. MMPREC02 remains readable for earlier recruitment consumers;
the new articulation explicitly requires MMPREC03. Checksums, binding revision,
source-header hashes and both LOD counts remain mandatory.

Body rows, original seam donors, protected boundaries and lobe participation
>= .9 stay fixed. Rigid shaft rows use original material ownership: station
>= .15 with lobe participation < .2, or station >= .3. The remainder is the
junction. These sets are immutable across size, angle, animation and mechanical
state. No live height or forward-coordinate comparison can reclassify the shaft.

Normal/large geometry requests +55 degrees upward. The downward endpoint is
20 degrees for smaller sockets and rises continuously to 30 with clearance at
larger radius. Very short attachments use a smoothly reduced nonzero range.
Feasibility fallback is still explicit telemetry, synchronized across LODs;
it must not be described as full requested travel. The complete matrix must
check progressive travel rather than accepting a no-op or backward slider step.

## Evidence and adoption

Private artifacts and real captures live in
`D:/MaleModBuilds/witcher-fixed-collar-20261014`. Rejected alternatives are kept
there. Session 184159 completed 57 native views and was closed, with saves/audio
restored; it exposed the live ownership problem and is not the final candidate.
Final candidate DLL SHA256:
`2fde715c2f34635e77723d88d25a7948f94635637eb52cb5aa1aa5e47fcd49c4`.
Session `sealed-session-20261014-190815` completed 57 real side/front/oblique
views across sizes, all three mechanical states, both endpoints and walking.
All captured angle gains were [1,1]. The session is CLOSED; original saves and
audio were restored and temporary room/driver files removed. All three native
contact sheets and the six enlarged true-side views were inspected. The upper
stationary neck/saddle is absent in these sampled views. The downward endpoint
retains underside compression; the tiny-size inherited shape defects are not
certified repaired. Idle arms occlude part of the shaft in some captures.

The final matrix has 189 cases / 378 LOD outputs, no new folded/collapsed faces,
no backward slider steps, exact fixed rows, rigid error <=5.685e-14 and seam
error <=2.491e-13. Every case retained full requested angle gain. All 18 canonical
outputs at control 50 are byte-identical to the prior reference. Root/junction,
endpoint and MMPREC02/03 validation tests passed. This candidate has not repeated
the older candidate's separate interpolation sweep or captured-transient replay.

Private evidence: `ownership-final-proof.json`, `ownership-bounded-matrix/`,
`ownership-polygon-trace.json`, and `ownership-native-side-angles.png` under the
directory above. Native source/target medians were 46.05/131.23 ms, presentation
interval 150 ms: these results do not establish performance parity.

The user subsequently selected this reviewed candidate for installation. On
October 14 the exact DLL and its two matching data files were installed, with
39 managed files, 31 protected user files and 37 rollback files verified.
Rollback: Witcher `local/native-backup-root-20261014-194227/Restore.ps1`.
No release or dependency pin changed; normal campaign startup was not observed
after this installation. Full
attachment acceptance includes the broader shape/LOD/motion/campaign matrix,
garment interaction, shading and performance; a focused room pass is distinct.
Source physics still runs at the canonical angle; inverse garment-force/contact
parity remains unverified. Garments and clinical simulation are disabled here.

Production adoption requires a clean tested Base revision, an explicit Witcher
pin and the matching ownership artifact. Wolverine and other spokes consume
the same shared rotation/junction API through their own pins and measured
attachment/material bindings; Geralt's calibration and limits are not defaults
for another character.
