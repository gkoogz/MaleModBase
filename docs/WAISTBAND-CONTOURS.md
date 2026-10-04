# Measured waistband attachments

## Natural seated contour (October 4 development)

The user has refined the preliminary horizontal-plane design. The visible band
must follow the waist naturally rather than form a level belt. Shared
`natural_band.hpp` supplies a smooth front dip, hip rise and rear transition,
expressed in fractions of the measured fabric width. These are styling choices
inferred from the supplied classic garment reference, not a universal male
waistline or prescribed medical fit.

`band_rest.hpp` first measures a seated middle contour on the actual joined body
surface. Its local tangent and measured skin normal define the direction across
the strip. The top, bottom and five material stripe rows then intersect the
supplied native triangles at adjusted offsets to retain a consistent surface
width. `waist_contours.hpp` interpolates the scalar seating field along actual
triangle edges; every attachment still reconstructs from exact body donors.
All seven rows share the same transverse angular material columns. Each ray
intersects the original cut segments rather than interpolating an independently
resampled contour. This preserves four supplied-surface donors and avoids the
phase shift caused by a large pelvic graft extending one row farther forward.
The exact native skin cut API retains its original arc-length default.

### Recruited pelvic ramp

When morphology recruits the pelvis into the root ramp, the cut surface includes
both actual body resources and the full current anatomical mesh. The front rows
therefore follow the measured slope. Finite band thickness uses the complete
3D material-column director, including its vertical component. This is measured
from the complete lower-to-upper cut profile in each angular radial/vertical
plane. All seven stripe rows share that director; independently changing it
at the graft weld twisted the thickness layers through adjacent rows. The
original cut positions and native donor weights remain exact.
Flattening that normal into
the transverse plane would cut the cloth into the ramp at large sizes.

The rest fitter corrects any remaining chord-clearance shortfall locally against
complete body/anatomy triangles and closed-volume classification. Both band
layers and positional aliases share each correction, retaining finite thickness
and continuous stripes. It does not push the whole waistband radially because
one front face is close to tissue. Cloth attachments retain the measured mixed
body/anatomy donors plus their local offset so the same ramp remains active as
the underlying source moves. Whole-face clearance, motion and installed visuals
remain separate verification gates.
There is no vertical displacement of an already fitted horizontal shell, and
no substitute body surface. The old two-plane API remains available as a
reference and regression contract.

The shared numerical test checks nonlevel front/hip fit, seven continuous rows,
exact native donor reconstruction, approximate surface width and rotated,
translated 100-times unit covariance. Actual character cuts, fabric contact,
covered renders, motion, cooking and installed adoption remain separate gates.
The natural contour development has not been installed.

Earlier 37/48-column independently exported band meshes at overall sizes 25,
50, 75 and 100 for both characters passed zero proper triangle crossings,
positive-area coplanar overlaps and degenerate faces. Densifying the exact cuts
to at least 96 material columns exposed finite-thickness crossings at the
Geralt100 lower front edge. A local angular/cross-row normal variant had 19
raw crossings and 33 after clearance correction. Frozen candidate18 uses the
common column director above and independently passes zero crossings, overlaps
and degenerates in all eight raw and locally corrected complete band meshes
(2,688 faces each). Receipts are in Witcher's
`build/previews/current/walking-sheet-candidate18-{precontact,corrected-band}`.
This self-consistency proof does not independently certify final external
clearance, moving cloth, the entire garment or native gameplay. Native route
sample counts do not limit rendered band resolution.

For pattern-design context, [In the Folds' shaped waistband tutorial](https://inthefolds.com/blog/2015/11/25/how-to-draft-a-shaped-waistband)
uses separate upper/lower measurements and smooth joined curves. Its garment
pattern dimensions are not used as character dimensions in this implementation.

## Preliminary two-plane contract

The user's waistband design defines two transverse planes at the band's top
and bottom edges. The body intersections, rather than an enlarged circular
proxy, guide a snug elastic fit. This geometry belongs in Base; every adapter
supplies its actual current body surface in one coherent character frame.

`include/malemod/garments/waist_contours.hpp` implements the SDK-free
`malemod::garments::waist::Intersect` contract. It accepts body samples,
indexed triangles, a validated frame, ordered local bottom/top heights and a
bounded attachment count. The planes are perpendicular to the frame's local
up axis. No source units, joint names or native formats are assumed.

The implementation cuts actual triangle edges, welds positional aliases,
requires closed unbranched intersection loops, selects the largest torso
cross section and resamples it by arc length. Each attachment carries up to
four weighted indices into the supplied body surface. These weights exactly
reconstruct its surface position and allow the adapter to retain native
vertex lineage. Neither original part boundaries nor UV aliases are rewritten.

Both contours have positive winding and a consistent forward-side starting
point. Top and bottom circumferences are independent: a tapered waist is not
forced into a cylinder. Elastic pressure and thickness clearance belong to the
garment solver consuming the contours. The cut positions are on the skin;
fitting must keep the fabric outside it while allowing gentle elastic tension
and the requested limited forward pull under pouch load.

The strap anchors sit at the lateral hip midpoint in side profile. Their
measured skin routes pass behind the hip, cup the underside of each glute and
return through the medial thigh to the pouch. The front pouch suspension has
a broad sewn attachment to the waistband. These are shared garment design
requirements; character-specific surface donors and native topology remain
in each adapter. Posterior-only starting points and thigh garters do not meet
the requested fit.

At large shapes the graft can enter either transverse plane. A body-only cut
then becomes open. `waist_graft.hpp` builds the actual joined cut surface from
complete supplied body and anatomy triangles, refining the original coarse
body boundary onto existing anatomical root vertices. The numerical edge-match
tolerance is measured in circumference units. The original native draw and
contact surfaces stay unchanged; no classification cap enters the cut.
Attachments retain exact mixed Body/Anatomy lineage. Cutting a fabricated cap
to force a closed body-only loop is not an acceptable substitute.

The pouch is one continuous material sheet. Its upper edge is sewn to an
80-degree total arc along the lower front waistband edge. Surface walks start
from this broad arc, pass over the retained shaft and glans, then beneath both
scrotal lobes, and return to two short underside strap seams. Authored anatomy
regions select those supports; current axis guesses do not identify tissue.
The walks establish the initial drape and contact guides. Interior fabric is
not rigidly pinned to anatomy: persistent tension, bending and unilateral
collision let it respond to motion. Clothing adds this sheet over the complete
existing anatomy; it never removes or replaces that anatomy.

### Taut spans across concavities

The walk initially contains measured prominent tissue supports and exact
physical-contact detours. Exterior string pulling can replace a run of these
supports with one chord only when that chord clears the physical body/anatomy
union and lies on the outward side of every removed support. For a removed
point `p`, its actual outward normal `n` and closest chord point `q`, require
`dot(q - p, n) >= -tolerance`. The tolerance scales with measured clearance.
This retains the front of a protruding glans even if a shorter chord behind it
happens to be collision free, while bridging genuine inward valleys.

These walks define the rest material sheet. During motion, unilateral contact
can push the sheet out of tissue; there is no attractive interior contact or
per-frame snap into skin recesses. Persistent stretch and bending constraints
maintain tension between the sewn boundaries. Cross-lane triangles must also
pass complete physical face checks: clear individual paths alone do not prove
that the web connecting them clears a curved anatomy surface.

The sheet has two continuous side hems, one on each walked side edge. They
share the exact boundary particles and material transport of the sheet rather
than moving as independent trims. Elastic side-edge tension should mostly close
the side openings while retaining a small intentional gap. The upper waistband
arc and the two lower strap seams remain the actual sewn attachments; a closed
trim around all four sheet edges does not describe the requested garment.

## Verification boundary

The standalone `waist_contours` CTest passes exact tapered-frustum cuts,
independent circumference/area expectations, original-edge part/UV aliases,
four-donor reconstruction, triangle-order invariance, rotated/transformed
frames, explicit 100x unit conversion and malformed/open-surface rejection.
This proves the numerical intersection contract. Native Geralt/Wolverine
integration, animated fitting, cloth contact/performance and observed game
appearance are separate gates. A static preview does not establish those.

Both Witcher and canonical Wolverine consume the shared helper with their
measured body topology and current poses. Updating the Base pin and verifying
each adapter is the adoption path; the immutable Wolverine reference snapshot
is not a development target.

## Bounded contact reuse

`proximity_cache.hpp` reuses an outside separation certificate only while its
old clearance, minus cloth travel and the cumulative maximum surface-vertex
travel, still exceeds the requested margin. Topology changes invalidate the
certificate. The initial outside classification and clearance must include the
measured root's virtual classification closure; an uncapped opening alone does
not certify volume membership. Classification caps never become fabric contact
obstacles or rendered geometry. The cached value is a conservative lower bound,
not a newly measured minimum distance.

The current body collider also accepts a previous triangle as a search seed.
It still traverses the exact BVH and can select any closer current triangle.
Normals and triangle bounds are recomputed on every body update. SDK-free
checks compare cached certificates to independent moving-plane distances,
exact current deformed-body queries and exhaustive unpruned face distances.
These checks establish safe reuse; sustained worker throughput and live cloth
quality remain separate adoption gates.

`BodyCollider::ClosestFaceCached` also retains a conservative neighborhood of
physical triangles. It collects every triangle whose bounds overlap an expanded
query with travel padding 1.25 times the contact radius. That list
is reused only while maximum
cloth-vertex travel plus cumulative maximum surface-vertex travel leaves enough
padding for the current query. Surface identity or topology changes rebuild it.
Every retained candidate is tested against current exact geometry; this cache
does not supply volume membership or omit a triangle that can enter contact.
Persistent neighborhoods retain at most 512 triangles. A complete narrow padded
set replaces the previous four/two/1.25-times attempts: wider sets incurred
additional exact triangle work and cannot rescue an overflowing narrow set.
Its certified travel padding can expire sooner; the requested physical contact
radius stays unchanged. If complete collection exceeds the bound, the exact
full BVH query runs instead. A partial
candidate list is never used as a collision surface. Dense-source tests require
both a complete smaller neighborhood and the fallback to retain the exact
closest face, including a formerly excluded triangle moving into contact.
Moving narrow-neighborhood tests compare to exact current BVH queries, including
entry of previously excluded faces, topology changes and surface-owner changes.

When distant source motion exhausts that global travel bound, a complete cached
neighborhood may also be recertified against bounded exclusion witnesses. Each
source triangle is either retained as a candidate or covered by a recorded
excluded BVH subtree/triangle. Every excluded current bound must remain outside
the current padded query. Any overlapping witness forces a complete rebuild;
overflow of either bounded list uses the existing full BVH fallback. This
certifies candidate coverage only and does not replace closed-volume membership.
The regression includes remote vertex motion and a formerly excluded face
entering contact. A two-frame actual source comparison preserved numerical
outputs and encoded source bytes, but its 585/806 ms cloth costs still fail the
active-time performance budget; it is not a throughput or gameplay approval.
