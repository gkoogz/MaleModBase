# October 13 collar support development checkpoint

Latest: see [stock-pelvis anchor investigation](STOCK-PELVIS-ANCHOR-20261013.md).
A measured surface candidate places the pivot 25 mm inward and 9 mm down.
Seven-angle offline trials support coordinated refitting, not a pivot-only
repair. Even the angle-50 numerical pass has visible attachment defects. No
new native session or installation occurred; the attachment gate has not passed.

The opening-only cap below does not constrain the whole transition. A captured
upper transition row overshot the measured section by 27.8% of section height.
The new `convex_support.PlanarSupportProjector` can enforce an outer planar
envelope alongside support floors, but has only synthetic tests so far and is
not yet applied to these real-mesh fits or adopted by the native adapter.

## Latest user constraint: close width, no unintended upper growth

The user explicitly rejected making the collar much wider than the shaft, then
asked whether it was getting taller. Keep the measured shaft section as the
envelope: a small transition outside it, and additional room below for the
raphe. Do not enlarge the whole opening as a workaround for fitting failures.

The offline shared `swept_collar.circumscribed_opening` now uses existing rim
edges as exact left, right and upper supporting planes. A thin convex ellipse
adds the inferior allowance; its top is zero and its side allowance is one
tenth of the requested inferior allowance. The former cubic support offset
could reverse consecutive donor edges (captured LOD0 minimum turn -0.02724).
The replacement keeps their order. Oversized lateral output, collapsed edges
and nonfinite input are rejected.

Opening-only checks on 42 captured maximum-size cases (states 0/1/2, angles
10/15/25/35/50/70/90, both LODs) retain the entire measured section, positive
corner turns and zero upper overshoot. Lateral width is 1.014903--1.016145 times
the measured shaft width. These are cross-section measurements, not complete
body-surface or native-render passes. Before the upper cap, the angle-15 opening
was about 8% taller: about 7% below and 1% above; that upper excess is now removed.

Seventeen targeted Python tests now pass, including stock-anchor binding,
convex-envelope projection, pointed-underside profiles, donor-edge
containment, width limits, upper limits, unit covariance and exact section
coordinates. Expanded standalone C++ recruitment constraint tests pass for
opposed planes, nested donors, fixed tissue and diagnostic rejection. No normal
installation or new native game session occurred. The Python surface fit is
still offline and has not been adopted into the adapter.

Full-surface studies remain rejected: `swept-lateral-s0-a15` has 12 LOD0 and
4 LOD1 area/neighbor violations after projection. Local constrained repair and
ARAP studies did not establish a replacement pass. Do not use their outputs as
accepted geometry. The newer upper-capped study is separate; do not combine
opening-only success with any prior full-surface or live evidence.

Correction to an earlier hypothesis: LOD1's original seam parents are exactly
its nine actual boundary corners. There is no demonstrated donor/corner
classification mismatch. Original-edge donor interpolation remains mandatory.

Private evidence: `D:/MaleModBuilds/witcher-progressive-recruitment-20261012/`
`convex-capped-opening-matrix.json` and named study folders. Retail worker SHA
was rechecked and remains the accepted hash recorded below. Both body resources,
cross-LOD waist agreement, coupled sizes, motion, live views and campaign still
need the complete attachment gate before promotion.

The educational anatomy task remains active. Deliverable: a shaft-sized pelvic
collar, extra inferior raphe clearance, a small continuous ramp, neutral surface
preservation and graceful slider angles 10-90. Actual side/front/oblique native
renders are required before normal installation. Numerical passes and offline
clay renders are not visual acceptance.

## Causal findings

Captured saddle sample: unique vertex 12595, rendered row 12960. Main cage
ancestors: 2256 (0.538665) and 985 (0.461334). Source diffusion lowers its fitted
height by 2.785 source units; pelvic ramp fitting lowers it again; the raphe
collar shift lowers it another 2.345 units. Suspension weight is zero. Pouch neck
smoothing does not move this sample. A private material-ventral raphe gate was
studied but is not adopted. The original worker is retained in current studies.

The adapter adds its own pelvic pull. Growing that reach with radius spreads it
into shaft material. Its seven-unit curvature window misses crown rows
10112/10113/11725, at distances 9.731/10.694/11.778. The source collar also affects
the first guide segment. At slider 90 that segment can point backward while the
full guide remains forward/down. Support now uses the effective full guide.

Forward travel tied to dilation overshoots the lower body seam. Failed cases
often end at small seam fans 14163-14166 or 14298-14301. Original-edge donors
remain mandatory; extra study locks on their positions can make a widened corner
infeasible. The bounded study allows those extra donors to adjust with radial
support floors while preserving the actual prescribed waist and exact aliases.

## Shared implementation

`surface/collar_support.hpp` fits the current healthy shaft and actual
body/anatomy boundary. It raises only participating upper collar rows, eases the
support to zero at the seam/healthy boundary, and projects area, body winding,
neighbouring-face and support inequalities in donor space. Curvature fitting
covers the measured supported material extent. The guarded recruited body,
healthy shaft and scrotal exclusions are fixed during fitting. Neutral dilation
is exact pass-through. No game SDK, bone guesses, topology or texture changes.

`RootTransition::ArcAnteriorLimit` separates forward travel from radial girth
and releases the compressed arc with pitch. The oblique law retains full
cross-section and extra inferior allowance. The constraint queue restores fixed
masters exactly at its final continuation step and rejects nonfinite support
normals. These are source changes, not an installed port.

## Verification boundaries

Standalone collar support passes dorsal repair, exact body/healthy preservation,
neutral identity and unit covariance at 0.01/1/100 scales. Recruitment constraints
and oblique circumference/raphe/arc-budget tests pass. The private x64 probe
composes actual native vertex buffers on both observed Geralt LODs.

Private study: `D:/MaleModBuilds/witcher-progressive-recruitment-20261012`.
`shared-whole-guide-sweep` passed state 0 at 15/25/35/50/70/90. Its angle-10
floating-point negative budget was clamped. `shared-final-matrix` adds states 1/2
and records remaining seam-fan/extreme-angle rejects. `shared-boundary-guard`
passes state 0 at 10, but state 1 at 35/90 is rejected. Identities differ between
named studies; never combine them into an accepted complete matrix.

Reviewed offline side/front/oblique profiles fill the deep depression but retain
a small crease and some underside/oblique recesses. No clean native visual pass
is claimed. No new native session or retail installation occurred in this
continuation. Cold upward preparation is about 20 seconds for both LODs, so
performance is not ready for promotion. Coupled size controls, all states,
motion, native LOD views and campaign remain part of the unfinished gate.

Accepted retail DLL:
`9089e7342af9484b6f22bcd042de31e687f4e7568ddcd72c75e682da123daace`.
Worker: `979647f61040910e2e0bd20fda7af34e42ab388254ae97a3f0b5b7359a9e4be1`.
Binding and adapter pin remain 431e626; exact rollback is untouched.

## Adoption

Keep algorithms/regressions in Base. A Witcher diagnostic must explicitly
consume these headers with a matching graft archive and record implementation
identity separately from the old binding revision. Retain normal runtime until
native visual/performance checks pass. Wolverine and other spokes must adopt a
tested Base revision with their own boundary, lineage and native matrix; none
is silently upgraded. Full Base verification still has the pre-existing motion
extraction mismatch; targeted standalone checks are recorded separately.
