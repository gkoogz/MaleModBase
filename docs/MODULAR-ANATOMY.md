# Independent shaft and scrotal attachment

The October 15-16 redesign supersedes the rejected pelvic dilation candidate.
Preserve the original pelvis and construct a compact attachment ramp. Keep a
complete, capped shaft with its ventral profile independently of the scrotum.
Reuse the existing distal shaft and lower sack. The sack's upper surface must
rise up both sides of the shaft and flatten into it through a broad transition.
The supplied stylized adult references guide shape, not anatomical measurements
or clinical validity. Reference images remain outside Git.

## Shared implementation

`malemod_base/modular_anatomy.py` provides exact triangle clipping with sparse
source donors, separate per-corner UVs, shaft completion from a measured section,
triangle-bound attachments and a tangent-matched scrotal transition. Longitudinal
footprint expansion and circumferential side reach are independent parameters.
Ninety degrees reaches either side; the current authoring candidate uses 100
degrees measured from the underside. This does not enlarge the shaft.

The neck inherits the original smooth normal field through cut-edge donors.
Recomputing normals from clipped triangles produced corrugations where flat
cut-edge normals alternated with averaged original vertex normals. Cosine
sampling resolves both tangent joins. Original retained sack vertices stay exact.

The later fold audit found that manifold closure was insufficient: the former
`trumpet-09-refined` neck contained 877 locally reversed triangles. Nearly
coincident section knots received different polygon normals from their widely
spaced neighbors. Using those normals as Hermite derivatives amplified a
tessellation artifact into folded geometry. The target field now comes from
the differential of the measured shaft profile, with angular weighting. Source
tangent detail is confined to the start of the new patch while preserving its
exact endpoint derivative. Corner-angle normals also avoid area-weight bias
under coplanar subdivision. A whole-loft local orientation check rejects folds.

`malemod_base/modular_union.py` creates a separate visible union. It cuts the
covered shaft patch in its longitudinal/circumferential chart using constrained
triangulation, preserving triangle barycentric donors and corner UV aliases.
Every intersection becomes a shared boundary knot. Refine the detached sack
boundary **before** constructing the transition: snapping a finished fine strip
to a newly subdivided curved shaft can reverse triangles. The complete mechanical
shaft is never cut or moved. The visible union is a separate closed surface.

The current export is authoring geometry. Its shared seam positions are welded.
`modular_shading.py` now preserves the retained surfaces' normal fields through
cuts and interpolation, supplies differential normals on the new strips, and
matches both boundary fields exactly. Normals belong to geometric vertices;
UV aliases and mirrored tangent handedness remain separate corner attributes.
The NPZ and OBJ exports carry these normals instead of asking a downstream
importer to rebuild them from one-sided cut triangles. `corner-frames.npz`
provides the explicit draw-corner mapping and reports singular UV faces.
Generated charts still need material transfer and native tangent validation.
Closed manifold topology does not prove absence of self-intersections or clean
native animated appearance. This is not a finished native attachment.

## Original-body ramp

`malemod_base/modular_pelvis.py` cuts and welds an actual display union with the
original body. It removes only the connected hidden root, preserving the sack
behind the same cutting plane. Removing retained source vertices is an error.
Body points outside the covered patch stay exact; the original open boundaries
must retain their exact edge identities. Both seams have shared vertices and
per-corner UV/source donors, rather than coincident overlapping surfaces.

The body mount frame and shaft orientation are separate. Tilting every body
intersection ray with the shaft can send the underside through empty space
between the legs. The new opening keeps that underside supported and recruits
the compressed arc from measured oblique cylinder/body intersections. Its
projected height can increase at steep angles; its lateral width tracks shaft
width plus the explicit margin (12 percent in the current recipe). This is a
change in which original triangles are covered, not a body displacement field.

The requested trumpet shape uses a 12 percent radial margin, plus an additional
8 percent on the ventral midline for the raphe connection. This extra support
fades with the fourth power of the ventral direction cosine and is zero on the
upper half and both lateral extrema. It changes neither shaft diameter nor
the lateral opening width. The first ramp derivative uses the same local margin
so the extra underside support forms a continuous flare, not a separate bump.
Both parameters are explicit authoring inputs; they are not game-slider values.

The retained-body tangent points into the opening, independent of the tilted
target. Ramp orientation uses a periodic surface differential: linearly blending
endpoint normals incorrectly rejects a valid rapidly turning arc. Regression
tests also deliberately reverse interior cells to check that the differential
audit detects real short folds. Neither audit proves global nonintersection.

## Reproduction

Install `requirements-authoring.txt` into a development environment. The observed
private build used Shapely 2.1.2 without changing the system Python installation.
Run `tools/build_modular_anatomy.py` on an evaluated wire6 source snapshot:

```
python tools/build_modular_anatomy.py --source SOURCE --output NEW_DIRECTORY --shaft-cut 24 --scrotum-cut 76 --lobe-protection 74 --root 6.45 0 84.3 --neck-flare 1.3869573755569748 --side-wrap-degrees 100 --visible-union --refine-shaft
```

These coordinates are measured for the current source snapshot. They are not
portable engine units or automatic slider calibration. The root is based on a
ray intersection with the original Witcher pelvis; automate that calibration
before runtime adoption.

The current longitudinal flare reserves the measured pelvic-ramp interval;
circumferential reach remains 100 degrees on each side. The complete independent
shaft preserves distal positions and measured transverse bounds. Only its newly
constructed proximal profile receives the bounded Fourier refinement.

For a measured body in the same coordinate frame, exported as a Mesh NPZ and
sparse donor sidecar, assemble the visible surface using:

```
python tools/assemble_modular_pelvis.py --body BODY.npz --module NEW_DIRECTORY/visible-union.npz --attachment NEW_DIRECTORY/attachment.npz --output NEW_ASSEMBLY_DIRECTORY --margin 0.12 --underside-margin 0.08
```

The body and module source banks stay distinct. A posed attachment contract may
also supply `mount_axis` and `mount_radial` in addition to its shaft frame.
Adapter extraction/calibration is required; do not relabel engine coordinates.

Outputs include complete shaft, detached/attached sack, visible union, sparse
donors, exact attachment triangle IDs/barycentrics and a manifest. Generated
source-derived meshes stay private. Run:

```
python -m unittest tests.test_modular_anatomy tests.test_modular_union tests.test_modular_shading -v
```

## Evidence and adoption

Current private artifacts: `D:/MaleModBuilds/modular-anatomy-20261015`.
Current independent anatomy: `trumpet-21-shared-shading`. The completed body
assembly is `trumpet-22-ventral-ramp`; its matched render is
`seamless-ramp-review.png`, with `seamless-root-detail.png` for a closer view.
The 6,451 protected lower sack vertices and all
retained anatomy remain exact. The neck has zero reversed faces in its local
audit and 1,122 shared seam knots. The independent shaft remains closed.

`collar-matrix-22` contains seven collar-only geometry checks: tilts -25, 0,
+25 and +50 degrees; widths 0.65x, 1x and 1.35x in the documented combinations.
All have zero locally reversed ramp faces and preserve the body's original
open boundaries. `seamless-ramp-angles.png` shows six true side views.
These are geometric tilts/width factors, not native slider positions. The sack
is omitted from this isolation matrix; its posed dynamics are not tested here.

Fifteen focused shared tests pass, covering closure, source preservation,
subdivision-independent normals, near-coincident knots, donor fields, tangent
joins, bilateral reach, independent shaft binding, original body boundaries,
tilted/width-varied ramps, deliberate fold detection, exact inherited shading,
normal continuity across mirrored UV splits and explicit singular-UV reporting.
The assembled body
is a high-resolution authoring surface (about 131,000 triangles), not an
established native topology, quantization or performance budget.

The measured stock upper/lower waist has 34 matching knots. The offline body
preview uses the original lower positions as canonical, moving only matching
upper waist aliases by at most 1.284 mm; the original pelvis is unchanged.
Source aliases/correspondence are recorded privately. Upper source UV0 is
retained, while lower body UVs remain an explicit clay-preview placeholder.
Do not treat this as material-transfer or native body-boundary validation.
The full repository verifier still stops at the pre-existing `Motion header
differs from source extraction` mismatch; this work does not overwrite that
separate motion change to make a broad verification claim.

The prior dilation/waist candidates are superseded, not accepted. The installed
Witcher runtime and rollback remain unchanged. No modular geometry has been
cooked, installed, or observed in the native grey room or campaign.

Before native adoption, complete material transfer, immutable morphology/physics ownership, both native body
resources and supported LODs. Version topology and binding data together; old
17528-source-vertex bindings cannot consume this replacement. Test the full
attachment matrix and publish matching native front/side/oblique images.

Witcher owns measured body calibration, cooker, runtime buffers and sandbox
driver. Wolverine and future spokes adopt the same Base module and versioned
binding contract after their own calibration/visual gates. Do not independently
copy shared algorithms or replace an accepted runtime from this authoring export.
