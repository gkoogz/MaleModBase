# Measured garments

`include/malemod/garments/jockstrap.hpp` owns reusable, SDK-free garment geometry.
Naked is the default; WhiteJockstrap preserves the internal anatomical surface.
The adapter supplies actual evaluated body/anatomy surfaces, semantic body axes,
ordered waistband/opening contours, two under-glute paths, donor identities,
measured gravity and optional body capsules. No skeleton names, native vertex
layouts, assumed meters, input hotkeys or material SDKs are shared here.

## Current walking cloth candidate (October 4)

The active revision-2 material is one continuous walked sheet. The approximately
80-degree front waistband arc supplies its broad top stitch; two short underside
stitches meet the under-glute straps. The waistband follows naturally seated
joined-body contour cuts with common angular row correspondence; the earlier
transverse planes remain an independent preliminary API. See
[WAISTBAND-CONTOURS.md](WAISTBAND-CONTOURS.md). Two continuous
elastic hems reinforce the sheet's left and right edges. Original anatomical
and body meshes remain present and unchanged by the clothing toggle.

Lower-side coverage now grows with measured panel aspect: complete native
forward reach is divided by the broad top seam's lateral width. Compact panels
retain their original terminal approach; extended panels smoothly gain
lower-middle fabric. The quarter-width transition is an authored pattern
choice, not a universal anatomical rule or a game/UI size table. Both sewn
endpoint tangents stay fixed. `sideCoverageExtra` is the requested maximum;
layout exports additionally record measured aspect and actual effective growth.
The retained candidate38 eight-case rest proof passes native clearance, sheet
consistency, authored seams and individual-component route/self-crossing gates.
The requested maximum is .45; the two compact canonical cases receive zero
growth and retain the earlier clean source geometry byte for byte. Largest
Geralt sampled lateral coverage is88.4/87.1 percent, versus87.1/85.8 previously.
Its closer opening is modest, and complete sewn construction, moving cloth,
all72 actual controls and native optics/gameplay remain separate unmet gates.
Both current game spokes consume this same Base constructor through their
measured inputs; future clean pinned adoption must not duplicate the rule.

Initial draping uses authored shaft, distal and left/right suspended-volume
regions and real triangle contacts. A convex rest tension envelope retains every
measured anatomy sample and the original body pressure samples underlying the
broad front stitch, including supplied aliases. Derived garment arc samples are
sewn boundary conditions rather than extra pressure prominences. One coherent
angular chart of that envelope prevents independently fanning lanes from
crossing. The lower stitch strip stays on the same exterior chart instead of
returning inward across its final row. The envelope is only an initialization
tool: full native tissue triangles remain the physical collision geometry.
These are rest-only construction operations, not per-frame skin targets. The generated material
has persistent particle positions, velocities, stretch/shear/bend constraints,
sewn connections and unilateral measured body/anatomy contact. Ordinary animated
waist changes never trigger a new walk; explicit morphology revisions do.

`MaterialLayout::jointRevision == 1` declares finite authored endpoint joints.
Hem centers have a thickness-derived outward offset from their sheet boundary;
underside strap cap centers share the authored narrow sheet strip with zero
center offset. Rest contact projection moves these sewn aliases together. The
runtime underside stitch uses the same declared weighted sheet points rather
than a post-fit nearest-face gap. Static joint receipts, persistent constraint
residuals and final native collision checks are separate verification gates.

The runtime waistband now has one physical material midsurface. Its original
inner and outer vertices, stripe UVs, native donors and complete collision faces
remain present; `band_material.hpp` reconstructs their authored finite offsets
from the current angular/cross-row material director. Both layer offsets use the
same analytic rendered-contact derivatives, including every director node.
This avoids treating fabric thickness as an independently extensible spring.
The rest fitter still owns actual skin seating, thickness and pelvic-ramp tilt;
the runtime director does not correct invalid static band geometry. The SDK
test verifies both layers, rest aliases, exact reconstruction, severe tilt,
finite differences, virtual work and linear/angular contact sums. It does not
certify the current source-motion matrix.

`sewn_contact.hpp` supplies an independently tested, currently **inactive**
contact operator for linear weighted stitches. It removes displacement modes
that would open those stitches, using their actual free-node inverse masses.
Rank-revealing orthogonalization handles duplicate rows; cached projector
columns retain every affected sewn node, including support larger than a raw
rendered face gradient. Effective mass, free-node impulse and moment come from
the applied complete gradient. No small coefficient or pressure ancestry is
discarded. Mass or pinning changes require rebuilding the operator.
This helper does not replace the final physical, rendered-seam, fine-strain or
material self-intersection gates. Its production adoption and matching friction
velocity response remain pending. A frozen prior-chart default50 experiment
passes external contact/material and actual rendered stitch allowances, but
independent inspection finds 246 sheet crossings already in its initial fit;
that prototype is rejected for garment shape and remains far too slow.

A separate frozen19 diagnostic starts with zero proper sheet crossings in both
the static fit and runtime zero-time reconstruction. With the private full
sewn-contact operator, 48 actual canonical default50 source-feedback frames
and 60 Geralt default50 gravity states pass the external tissue contact,
material, fine-edge and rendered-stitch measurements. Their measured internal
particle plus prescribed-support reaction sums also conserve impulse and moment.
However, source motion creates four new proper sheet crossings at frame1 and
reaches166; gravity creates18 at state7 and reaches180. These are newly generated
cloth self-intersections, not an invalid starting chart. Those runs are rejected.
Median solver times are480.8ms per source frame and70.17ms per120Hz gravity step,
which fail the required cadence. A private full fine-surface continuous
vertex-face/edge-edge self-contact prototype is under development; it is not
enabled in the shared runtime or accepted as working. Complete moving surface
audits remain separate from external tissue-clearance flags.

A later private frozen19 self-contact smoke retains every fine face and swept
vertex-face/edge-edge crossing witness, including crossings away from a shared
material vertex. Its admissible local thickness is capped at each primitive's
actual fitted rest separation; it does not discard a whole connected one-ring.
The transported contact normal has complete seven-point analytic derivatives,
and the manifold is rediscovered after deformation. The first actual canonical50
source frame has zero independently measured sheet crossings, overlaps or
degenerate faces and fine-edge stretch of 1.06479, but native tissue contact
fails. It costs 46.30 seconds for that source frame. The requested two-frame smoke
therefore aborts after one frame and is rejected; further replay expansion was
stopped. This is neither production dynamic support nor a cadence pass.

`Parameters::mechanics` declares cloth mass fraction, stretch/bend compliance,
extension limit, damping and friction separately from appearance. Cloth mass is
relative to an adapter-supplied measured generalized tissue mass; source values
are not represented as kilograms without a calibration. The optical contract
is soft matte ribbed/elastic white with **0.95 opacity**, plus narrow red and blue
bands. `Material.opacity` and the baked opacity map define optical coverage;
they do not set mechanical stiffness. Native support for that optical property
requires a separately verified adapter material and cannot be inferred from a
CPU preview.

Source lineage has sixteen exact bounded donors. Interpolation coalesces equal
identities and rejects overflow; it never silently truncates support points.
Classification-only closing/refinement faces establish volume membership but
never replace physical contact triangles or native body resources.

Reciprocal support now records impulses from actual solved unilateral cloth
contacts, with physical triangle/barycentric lineage and force moments. The
adapter maps these to measured local generalized tissue degrees of freedom.
There is no proximity-based gravity compensation in the current reaction path.
This interface and its conservation tests are distinct from observed native
coupled motion, which remains unverified for the current candidate.

### Current verification boundary

The earlier plane-walk convergence blockers are resolved. Independent frozen
candidate13 replay passes all eight actual source cases (Geralt and Wolverine,
Overall25/50/75/100): original tissue clearance, zero proper sheet crossings,
overlaps, degenerates or winding conflicts, and all six authored endpoint joints.
The maximum joint residual is 3.71e-16 circumference units; strap turns stay below
17.69 degrees and side-hem turns below17.41 degrees. Adjacent facet folds still
require visual review. This static result does not accept the moving garment.
The extreme-state joint/contact matrix remains required.
An exploratory full-grid integration completes 60 captured default50 gravity
steps and48 actual canonical default50 feedback frames with the unchanged
material/physical gates satisfied. Integrated contact now revisits shared-donor
physical pairs and rediscovers newly violated fine faces before reconstructing
velocity; each correction retains its measured reciprocal impulse. Those runs
precede subsequent side-coverage and waistband-density changes, cover only one
state, and fail the worker cadence requirement. Sustained gravity, source-coupled
motion and worker cadence remain required gates on one stable final source set.
Every final fine sheet edge is checked against
its authored rest length, separately from control constraints; the maximum
published extension remains 15 percent. Current changes are not an approved
installed or gameplay-verified garment build.

## Historical installed radial-shell checkpoint

The following sections retain the earlier installed/reference checkpoint and
its receipts. Their radial construction, four-donor contract, one-way proximity
support and timing numbers are historical; they do not describe the active
walking cloth candidate or prove its native success.

## Geometry and materials

The waistband has an inner and outer surface, closed thickness edges, and narrow
red/blue geometric bands. The small elastic hem and two under-glute straps are
three-dimensional solid ribbons. Each strap ends at the live lower pouch hem.
The front panel joins the actual front waistband to the upper pouch arc; the
side/rear openings are intentional jockstrap construction and can widen at
large shapes. Cloth is an opaque thin surface; adapters must render both sides.

The pouch uses a fixed angular grid fitted to **every actual live surface
vertex**. Each vertex supplies the radial extent of all neighboring grid corners;
empty directional cells represent suspended fabric between measured supports.
Bounded diffusion fills these gaps, and outward smoothing bridges sharp valleys.
It retains distinct shaft, glans and lobe bulges rather than fitting one bounding
ellipsoid. Chord allowance and measured clearance are added after fitting.
`coverageMargin` is the sampled radial clearance certificate; it is **not** a
proof that every conceivable self-contact or non-star-shaped concavity is solved.

All vertices carry up to four normalized Body/Anatomy source donors. Normals,
UV tangents and handedness are rebuilt from the final geometry; UV seams retain
identical physical positions and reconciled lighting. Material slots are white
ribbed cloth, white elastic, red and blue. `Knit()` and
`tools/bake_garment_materials.py` provide analytical cloth detail and color,
normal and roughness maps without image generation. Native material conversion
and lighting are adapter work.

## Contacts and numerical support

Body capsules form one union, with escape along the measured garment/body
outward direction. This prevents nearest-surface projections from oscillating
between overlapping thigh and pelvis volumes. They are validated and resolved
against vertices **and complete triangles**; a capsule intersecting a large
triangle is detected even when all three vertices lie outside it. Intact band,
hem and strap cross sections are fitted with bounded early-exit passes and
equal-arclength material resampling. Both strap ends meet actual fabric surfaces,
and the panel joins the current band. See GARMENT-CONTACT-REPAIR.md for the
current algorithm and material integrity gates. Pouch points are returned
to their anatomical radial envelope after body projection. Impossible or
unresolved contacts set `contactBudgetSatisfied=false`; they are not silently
accepted, hidden by deleting geometry or labeled successful.

`Output.support` contains at most 64 nearby lower-cloth support samples. Each has
exact source lineage, world/caller acceleration, measured radial separation
and normalized influence. Influence fades with separation and the upward
support component. The maximum requested acceleration is 15% of the supplied
measured downward gravity; default fraction is 5.5%. Naked and failed contact
budgets supply no numerical support. `garments/numerical_support.hpp` aggregates
these into shaft/lobe source acceleration; the production source solver applies
that bounded acceleration. This is slight pouch support, not a full two-way
cloth finite-element simulation.

## Character recipe and adoption

`tools/prepare_garment_source.py` measures the canonical Wolverine source body
triangles and exact final `nrPacked` surface. It creates original-edge sparse
waist donors, the final unified graft opening's 13 source donors, measured
under-glute paths and an SDK-free test fixture. The reference source body is
+X forward, +Y lateral, +Z up, in uncalibrated source units. Its observed waist
cross section at source Z=94.75 is a **character recipe**, not a universal Base
constant. The generic right-handed garment frame uses lateral=-Y, forward=+X,
up=+Z. It must not use the live shaft pitch as body vertical.

The character recipe is in `assets/garments/wolverine-jockstrap-donors.json`
with source hashes. Generate a neutral recipe once, then use `--recipe` for
other evaluated shapes so donor identities remain stable. Canonical Wolverine
`src/runtime/jockstrap_adapter.h` consumes that recipe and Base via its pinned
include path; it does not copy the geometry implementation. Its draw adapter
blends all actual body/anatomy donor skin influences using current-frame palettes
and restores the native D 3 D 9 state. Witcher provides its own character contours,
calibration, native carrier and mesh/material renderer. Every future spoke can
use the same Session and numerical-support contract.

## Verification and limits

Build the C++ `garment_test` target without game SDKs. It covers 240 animated
synthetic 1..12x shapes, a dense 40x extreme, Naked/reset, unchanged topology and
capacity, translation/unit covariance, invalid inputs, material/lineage/TBN
contracts and capsule/whole-triangle collision. Use `--fixture FILE [OBJ]` for
actual evaluated source states; the test animates 32 measured mesh states and
rejects a uniform-ellipsoid replacement. The observed UI50 default and UI100 max
fixtures have 17,528 final anatomy vertices and 35,000 triangles. Current CPU
measurements after material correction: synthetic mean 8.78 ms, UI50/UI100
without contacts means 27.93/29.16 ms; source canonical thigh contacts mean 57.67 ms.
The actual Geralt fit gate passes nine controls plus 32 coherent articulated
body-donor/capsule poses with 4,068 vertices and 7,764 triangles, including contact,
material strain, cross-section and both physical sewing-surface gates. Latest
worst update 77.13 ms under concurrent load. This is why both adapters generate
cloth on a CPU worker and keep GPU pose/draw on the render thread.
These are offline measurements, not
in-game frame rates. Native screenshots, collision walkthroughs, actual draw
cost and garment appearance remain separate adapter verification gates.

### Measured lateral hip route correction

`tools/garment_source_routes.py` is the one SDK-free Wolverine character export
recipe called by the Base reference generator and canonical adapter generator.
It consumes measured source coordinates and original triangles/donor identities;
it contains no native packed layout or universal character-unit assumption.
The old target sign selected opposite-side candidates near the centerline.
The corrected 21/20-point paths begin on the actual lateral waist intersections,
follow connected original edges around the posterior hip and beneath the glute,
then return medially to the inner thigh/pouch. Exact positional UV aliases join
the original body part weld. Waist and opening recipes, body boundaries and draw
indices are retained. These are reference donor/export checks; working dynamic
fit, collision, performance and native gameplay remain separate unfinished gates.

### Authored anatomy guide regions

`malemod_base/garment_regions.py` exports stable shaft/glans/lobe0/lobe1
membership from existing R14 ring/crown identifiers and original scrotal
mechanical fields, transported through exact R14/rs/nr source lineage. Final
scalar membership is clamped; signed geometry donors and the complete surface
are retained. The source guide sets contain 5,448/2,993/3,851/3,771 vertices
respectively within the unchanged 17,528-vertex, 35,000-triangle anatomy.
Lobe0/lobe1 retain the original negative-Y/positive-Y material ordering.

The optional `Input.anatomyRegions` array indexes the full current anatomy
samples. These are guide regions for the shared mesh walks, not a substitute
mesh or a runtime classifier based on current pose, engine axes or bones.
Character/native exports map the same scalar fields through their recorded
lineage once. `prepare_garment_source.py` writes region JSON/header sidecars
with source/helper hashes. JGFX0001 fixtures remain unchanged until explicitly
adopted by a new guide-aware fixture reader; historical radial-fit results do
not establish the new single-sheet walk or persistent fabric dynamics.

`python tests/test_garment_regions.py` verifies authored direct glans lineage,
disjoint full-surface bounds, original lobe sides and invalid-input rejection.
Wolverine embeds the generated native index sets. Witcher MMGRB003 appends four
count/index arrays after its measured MMGRB002 body fields; its previous fields
are byte-identical. Native packet/capture tests verify complete anatomy counts,
lineage preservation and malformed bounds/overlap/truncation rejection. Neither
these authoring gates nor an offline native build claims observed gameplay.


### Complete source root opening

The refined native source opening has54 welded boundary vertices. The old
13 coarse unified-collar support IDs omit41 refined boundary points and cannot
prove whole-source volume closure. `full_source_root_boundary` in the shared
source semantic/export helper follows existing consistently wound native edges
after exact positional-alias welding. It returns only existing source IDs;
no physical or render triangles, skin donors, UVs or source coordinates change.
Recipe schema3/rootBoundaryVersion1 records the measured complete opening;
old saved coarse recipes are rejected. The canonical generated native opening
consumes all54 IDs. Root-cap triangles remain classification-only.

The Python gate verifies full54 source boundary, bounds, ordering under unit
scaling/translation, inconsistent/open topology rejection. Native adapters must
separately prove the supplied full boundary closes at all evaluated states;
this exporter correction does not certify the pending sheet contact/dynamics.
