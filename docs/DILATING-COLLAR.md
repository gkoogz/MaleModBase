# Dilating collar development candidate

The shared opening field is `surface::DilatingCollar`; the Witcher adapter
opts into it with `MALEMOD_ENABLE_DILATING_COLLAR`. This is currently an
uncommitted development candidate, not an accepted release or a new Base pin.

The old opening could remain narrower than the scaled anatomy. The replacement
intersects the posed shaft cylinder with the measured rest pelvic surface.
Each radial direction uses the original opening polygon as its inner reference;
adjacent original-edge donors retain their radial spacing. The opening follows
the actual scaled radius, with ten percent additional ventral clearance. A
monotone quintic field recruits nearby pelvic rows and returns to the unchanged
outer surface. Original-edge donors, UV aliases and shared waist remain exact
constraints in the adapter's sewn solve. Body orientation is checked separately.

The root pivot is the first source guide point. Shaft handles rotate rigidly;
only the connecting tissue uses the junction solve. `CollarRadialSupport` then
fills a proximal dorsal radial deficit against the posed guide. It cannot shrink
existing girth or move fixed seam rows, body rows, scrotal cores or distal shaft.
This avoids a long stationary neck but is not an intersection certificate.
The diagnostic clamps the effective angle slider to 10 through 90; an earlier
angle-1 native trial failed the body orientation guard and was rejected.

`EndpointConstraints::ProjectWarm` reuses the previous correction in independent
donor coordinates while retaining the new pose and the cold target's correction
budget. It rechecks the current reference and falls back to the original solve
if the seed fails. This prevents the difficult downward case from repeatedly
performing hundreds of cold iterations. Tests cover fixed rows, interpolated
donors, rejected stale corrections and the unchanged displacement budget.

## Evidence boundaries

Private evidence is under `D:/MaleModBuilds/dilating-collar-20261014` on the
development host, not a portable source input. The first complete native run
captured 57 room views across angles, sizes, states and walking. It exposed a
downward-angle solver delay. The warm-start candidate is undergoing a second
native run and campaign validation. No installation or full attachment gate is
claimed here. See HANDOFF and the diagnostic provenance for current receipts.

The 108-case offline matrix covers three states, nine Overall/Width pairs and
angles 10, 15, 50 and 90, with Length following Overall. The native-composed
buffers include both LODs and body resources. These samples do not enumerate
every possible slider combination. The separate motion/combined-control probe
and native visual review remain necessary; numerical completion cannot certify
absence of folds, shading rings, pinching or abnormal wedges.

Body continuation checks incremental signed area, not global self-intersection.
Historical source collar fitting remains active. A trial removing those passes
and an explicit full-circle loft were rejected and are not the current model.

## Adoption

The geometry and numerical solver stay in Base. The character's measured
opening, native body ownership, render buffers and installer stay in Witcher.
After acceptance, pin a reviewed Base revision in the adapter rather than
copying the shared headers. Wolverine and future spokes must consume that pin
with their own measured boundary bindings and independently pass the attachment,
motion, LOD and campaign checks. This work does not change Wolverine's runtime.
