# Measured garments

`include/malemod/garments/jockstrap.hpp` owns reusable, SDK-free garment geometry.
Naked is the default; WhiteJockstrap preserves the internal anatomical surface.
The adapter supplies actual evaluated body/anatomy surfaces, semantic body axes,
ordered waistband/opening contours, two under-glute paths, donor identities,
measured gravity and optional body capsules. No skeleton names, native vertex
layouts, assumed meters, input hotkeys or material SDKs are shared here.

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
