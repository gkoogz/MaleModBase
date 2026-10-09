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

## October 7 - white surface from adaptive meridians

Verified: 20 geometry tests pass and `tools/verify.py` passes. The actual
12,801-vertex / 25,440-triangle reference mesh has one 160-vertex open seam,
zero boundary displacement, positive triangle areas, and whole-triangle
separation from all nine collision proxies (minimum support-plane margin
4.4743e-6 source units). This certificate concerns the supplied static convex
proxies, not source-body clearance, self-contact, moving cloth or gameplay.
Rejected development attempts: unconstrained plane corrections oscillated;
freezing support choices diverged; choosing support solely by signed distance
produced excessive radial displacement. The accepted method exits each proxy
along its longitude and selects the common support plane requiring the least
outward correction, with exact pinned-boundary checks. It converges in three
passes. The generated reference is a new surface, not the retired pouch.


The user requested using the existing rays as the new white cloth surface.
Base owns `malemod_base/meridian_surface.py`: cyclic neighboring meridians
form narrow panels, resampled by arclength and interpolated by polar radius
and height. This accommodates the small axial backtrack at the rounded tip.
The 40 adaptive origins remain construction guides; four angular subdivisions
and 80 lengthwise rows create the surface. Its single open boundary follows
the complete attachment outline. One cloth apex closes the distal end, offset
.18 uncalibrated source units from the collision apex to allow cloth thickness.

Every complete triangle must lie outside each convex collision proxy on at
least one proxy support plane. This is a conservative whole-face certificate,
not merely a vertex/centroid sampling check. Corrective projections are static
reference meshing, not fabric dynamics. No retired pouch solver is restored.
The Wolverine offline adapter exports `white-cloth.obj` with UV seam aliases
and `white-cloth-inspection.png`; the original blue/ray inspection is retained.
Output: Base `build/collision-chain-white-cloth-20261007/`.

No game installation or runtime has changed. Native attachment integrity,
motion, morphology/LOD and campaign acceptance remain untested. Shared meshing
is in Base; measured inputs/rendering remain in Wolverine. Future spokes adopt
through a tested pinned Base revision, not copied independently maintained code.

## Rounded glans dome - current preview

Use output `build/collision-chain-dome-20261007` with the same adapter command.
The final circle and seventh tapered link are removed. The existing crown circle
is the dome rim; the old advanced tip center is its exact single apex. Samples
follow radius*cos(phi) and axial-height*sin(phi), giving a hemisphere scaled
along its axis to preserve both specified landmarks. A perfect hemisphere is
produced by the same shared function when height equals radius.
The dome has 24 latitude rows, 64 longitude segments, one apex and a closed base.
Rim vertices are copied exactly from the sixth link's end. It is used in all
collision queries and exported as `glans_dome`. No active eighth circular rim
remains. Schema 2 records seven radii/plane normals, eight centers and explicit
dome shape metadata. The old frustum fit is clearly marked historical.
The adapter verifies all measured surface vertices distal to the crown inside
the actual tessellated dome, rather than relying only on the ideal equation.
Forty adaptive ordered meridians meet at the actual apex, and both smooth fixed
hem straps remain. Seventeen unit tests pass. Native attachment/motion/LOD and
campaign verification remain untested; no runtime is installed by this export.

## Smooth fixed hem straps - current preview

Use output `build/collision-chain-fixed-hems-20261007` with the same adapter and
body-input arguments. Each hem is a single cubic fitted near the measured guide.
A bounded six-variable handle fit preserves both endpoints and enforces .20
source-unit clearance from every convex blue proxy at 161 curve samples. Shape
is moved through handles, not by per-sample projection. Bounds reject stretched
solutions. Closed ribbons are .30 wide, .04 thick, fixed geometry. The frame is
continuous, and centerline endpoints coincide with the 80-degree band endpoints
and existing shared under-strap junction. Ribbon edges are checked separately
from successful curve fitting. Meshes are added to the retained trim and exported
in `fixed-straps.obj`; discarded pouch and hems are not reimported.
Adaptive meridians use the new smooth boundary and retain cyclic order/common
pole. Current center curves need no local angular-order adjustment.
Fifteen tests cover endpoint preservation, bounded clearance fitting, exact
ribbon dimensions, closed topology and nondegenerate triangles in addition to
the previous meridian/density tests. This is fixed offline source-pose geometry,
not observed live gameplay or certified full attachment gate. Native motion,
LOD/morphology/attachment matrix and campaign acceptance remain untested.

## Adaptive meridian preview - current

Use output `build/collision-chain-adaptive-meridians-20261007` with the same
adapter command and body input. The shared adaptive function evaluates 160
candidate paths against current closed blue proxies, computes physical bending
demand (total turning + half maximum turn + twice normalized detour), cyclically
smooths the density and retains a nonzero floor. Forty origins are placed by
weighted arclength quantiles around the entire orange attachment outline.
No origin positions, testicle locations or per-ray weights are baked in.
Calling again with changed geometry recomputes the distribution; the fixture
compares obstacle-free and box-obstructed geometry to verify that behavior.
This has not been integrated into native game update/lifecycle transport.

The forty rays are longitude meridians, not parallel Cartesian lanes. Fixed
outward half-planes with distinct ordered angles prevent intersections except
at the shared pole. Local attachment-order reversals are minimally repaired
and cleared radially from the proxies; the exact rendered outline includes
every adjusted origin. Shortest convex-turn routing stays within each half-plane.
The pole is just outside the enlarged tip cap, because the physical anatomy tip
lies inside that collider. 3D intersection guarantees do not imply that a
transparent camera projection cannot overlay front and rear strands.
JSON records candidate density, chosen origins, adjustments, half-plane errors,
positive radial support and shared endpoint; OBJ stores the forty polylines.
Thirteen unit tests pass. Anatomy, enlarged tip proxy, straps and waistband are
retained. No fabric surface or runtime installation. Native attachment/motion
and campaign acceptance remain untested. Older lane/radial notes below are history.

## Parallel lane correction - current preview

Use output `build/collision-chain-parallel-inspection-20261007` with the same
adapter arguments. `parallel_taut_paths` preserves the supplied measured lateral
axis for every point of every ray and supplies separate terminal points in a
row at tip height. The row retains the full starting width. This implements the
user's no-lateral-movement instruction; it does not converge to a single point
or guarantee that every endpoint lies on the narrow tip circumference.
The actual 40 lane coordinates are distinct, with zero lateral drift and
minimum separation 0.00410851 uncalibrated source units in this export. This
proves no 3D intersection between strands. Projection overlap in side/oblique
wireframe views can still show lines in separate depth planes together.
Per-plane collision avoidance and convex-turn shortest routing remain active.
Nine geometry tests pass, including a box-wrap analytical length fixture and
parallel-lane invariants. Native and full attachment gates remain untested.

## Forty taut guides revision

Use the same adapter command with `--body-input` and change the output to
`build/collision-chain-taut-inspection-20261007`. The final tip circle is measured
from all current vertices distal to the crown, enlarged to enclose the bulged
glans in the actual 64-sided frustum, and advanced past the distal surface.
Forty starts are spaced by arclength around the complete orange closed outline.
Each guide ends just outside the new tip cap center. Actual closed collision
meshes are intersected with each ray's radial plane. A visibility graph finds
the shortest free route, with convex-turn-constrained A* when necessary.
No segment may pass through a blue object; all bends have a consistent sign.
This is shortest routing in each selected plane, not a global 3D geodesic.
`malemod_base/taut_guides.py` and the enclosure helper in `collision_chain.py`
are shared. SciPy is already a declared Base dependency.

Embedded orange hem points are moved outward off the root proxy, so those local
portions differ from the prior pure body trace. Original and lifted curves and
all exact starts are preserved in private JSON. The rendered outline explicitly
contains every ray anchor. `taut-guides.obj` exports the forty polylines; no
fabric surface is created. Eight numerical geometry tests pass. Native gameplay,
attachment matrix and dynamic wrapping remain untested; no runtime installed.

## Strap/hem inspection revision

Pass `--body-input build/cloth-face-current-adapter5/baseline8/rest-native-input.json`
and `--output build/collision-chain-hem-inspection-20261007` to the adapter command
below for the latest preview. The body file supplies actual measured triangle
positions and normals. The lower strap routes are projected onto pelvis/upper
inner-thigh surfaces and share an identical terminal cross-section posterior to
the testicles. The upper 44 percent of each strap and waistband stay unchanged.
Two orange conceptual lines run from the arc endpoints along the measured body
surface to the common strap endpoint; these are guide curves, not sewn hems.
Base owns curve and exact closest-triangle/barycentric projection, while this
Wolverine adapter supplies the measured region, source units and fitting targets.
Body-surface donors and source hashes are recorded in the private output JSON.
Anatomy and the blue collision model are unchanged. Native/motion acceptance
remains untested. The earlier preview is retained separately.

# Collision-only inspection, October 7

The user rejected the pouch and will guide subsequent garment construction.
The failed `PouchSession` implementation and its executable build target were
removed. Historical reference cloth and the optional standalone numerical
library are not used by this inspection. Existing installed games are unchanged.

`malemod_base/collision_chain.py` owns the portable geometry: eight perfect
circles with shared centers, seven tapered links, concentric tapered ovaloids,
and a front arc cut from a nonlevel waistband bottom edge. There is no fabric
state, simulation, pressure, support or cloth fitting in this path.

Wolverine adapter `tools/render_collision_chain.py` is the offline extraction/inspection
adapter. It reads a paired evaluated anatomy/mechanics export and retains only
the saved garment's fixed waistband and straps. It drops the entire pouch and
both hems. Wolverine's authored glans ring lineage locates the narrow connection,
maximum crown and near-tip section. Intermediate shaft circles fit measured
cross-sections near the mechanical guide. The root circle uses the existing root
boundary, including its flare. These first-pass fit choices remain reviewable.

The two ovaloids use the exact exported lobe centers, axes, radii and source
ovoid taper `(1 - .13*z) * sqrt(1-z*z)`. The first preview enlarges their radii
uniformly by 3 percent. This is not a claim of complete skin enclosure.
Close glans sections share the measured glans axis to avoid crossing tilted rims.
OBJ exports have seven separately closed links and two closed ovoids.

Reproduce this private source-pose render on this workstation:

```powershell
python <Wolverine-checkout>/tools/render_collision_chain.py --base . --baseline build/wolverine-default-guide-v2 --trim build/cloth-face-current-adapter5/baseline8/cloth-frame-0.bin --output build/collision-chain-inspection-20261007
python -m unittest tests.test_collision_chain
```

The saved anatomy and mechanics are a matching historical offline reset-state
export at 120 source steps; the fixed trim comes from a separate identity-pose
capture. This is explicitly an offline geometry inspection, not a current native
gameplay capture. The generated JSON records exact source file hashes and circle
parameters. All source-derived outputs stay under ignored `build/`.

Checks: eight circular sections/seven closed links; shared section centers;
closed ovoid topology and exact normalized source shape; 80-degree arc on a
nonlevel bottom edge. Three unit tests pass. Front, side and oblique renders were
reviewed. Anatomy vertices and triangles are unchanged. Native attachment matrix,
moving-chain skin enclosure and campaign tests have not run; no installation or
attachment-integrity pass is claimed. Witcher adoption remains future adapter work.
