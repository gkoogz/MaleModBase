# Continuous shaft and separate compliant scrotal skin

## October 14: rejected preview and relative-response correction

The user rejected the initial native preview as still essentially rigid. Its
nonzero global excursions were insufficient evidence of independent deformation.
Retain those receipts as historical, unaccepted results.

The next opt-in revision addresses three specific constraints:

- Transverse suspension kept its tiny fixed compliance even after damping was
  reduced. Its new compliance is effective inverse mass divided by squared
  angular frequency (4 to 16 radians/second across the stiffness slider). This
  retains the coupled rod Jacobian, tension tether, hard extension limit and
  contacts, while permitting independent lateral travel.
- The root's nearly critical damping ignored the Bounce control. The damping
  ratio now ranges from 0.8 to 0.2 with that control. Root stiffness, prescribed
  first two rod nodes, attachment coordinate and travel stops are preserved.
  Curvature damping is reduced separately, without changing bend stiffness.
- A valid 0.15-second native request previously advanced at most 0.10 seconds.
  The opt-in solver now consumes up to 36 fixed 1/240-second steps per request.
  A three-second constant-input comparison at 50/150 ms now produces identical
  sampled rod coordinates; the rejected build differs by 1.031 source units.

`relative_dynamics_test` settles for 20 seconds, drives for four and releases
for four. `tools/compare_relative_dynamics.py` removes common root rotation and
measures lobes relative to their attachment. The default-size lateral response
increases 5.19/5.45 times; maximum-size response increases 2.43/1.67 times.
Root-relative tip excursion decreases in these particular driven tests; do not
claim every measure of shaft motion increased. The rod remains deformable,
with measured interspan angles and retained continuous hidden segments.

These are model-space response measurements, not clinical validation or native
visual acceptance. Private comparisons and native receipts are under
`D:/MaleModBuilds/relative-dynamics-20261014`. The installed angle runtime and
rollback remain intact until native review and explicit deployment. The option
remains disabled by default; Wolverine and other spokes adopt this shared
implementation through a tested Base pin and their own native attachment gate.

The user's October 14 requirement is explicit: the shaft is a continuous
mechanical object through the region obscured by the scrotal skin. The sack
hangs from that structure; it does not replace, truncate or become a shaft
segment merely because the visible mesh is connected.

`scrotal::MechanicalPartition` names the existing 12-node rod, two separate
suspended contents bodies and the source's 0.12 material attachment station.
All eleven rod distance constraints and all internal bending constraints remain.
The suspension samples the same proximal Hermite span and retains its reaction
Jacobian into the rod. Body/shaft contacts, contents volume/pressure transforms,
original seam donors, UV aliases and the angle/collar construction are unchanged.
This is an explicit ownership contract for the current model, not a claim that
an anatomical organ is literally a thin centerline or that its hidden volume
has been removed. Rendering visibility never changes mechanical membership.

The rigid appearance had two concrete contributors in the reference solver:
an upright angular spring of 20000 with fixed damping 120, and an analytic pouch
surface rebuilt directly around the contents with no secondary skin state.
The diagnostic opt-in `MALEMOD_COMPLIANT_SCROTAL_SKIN` makes angular/shear damping
respond to the existing Balls Stiffness/Bounce controls. It adds two bounded,
damped skin followers updated at each original 1/240-second physics substep.
The contents remain colliding supports, while the overlying skin can lag and
settle. This is a secondary surface model, not a finite-element tissue model.

Skin influence comes from immutable donor scrotal and shaft memberships. It
is zero at shaft membership >=0.5, ramps smoothly across scrotal tissue, and
passes to fine vertices through their original barycentric donors. It never
uses current position, angle, or proximity to classify tissue. Skin offsets are
limited to 14% of each support's smallest radius; a scale discontinuity or
teleport resets the follower. Existing support projection and a triangle-area
guard limit the resulting skin deformation. The rod is affected only by its
existing physical suspension/contact forces, not by this render-skin offset.

The immutable Wolverine snapshot is unchanged. Changes are explicit Base
extraction substitutions backed by standalone helpers and regression tests.
Default builds retain the previous behavior. Other spokes can adopt the helper
and source recipe through a tested Base revision; Witcher uses a private worker
and profile binding for evaluation. No automatic Wolverine installation or
clean dependency pin is implied.

Verification and exact artifact identities are recorded separately in
`provenance/scrotal-compliance.json`. Native previews must pass their own
attachment review; a source test does not establish campaign or garment parity.
The installed angle candidate and its rollback remain the accepted baseline.
