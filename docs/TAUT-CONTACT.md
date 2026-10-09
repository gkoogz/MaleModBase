# Certified finite-grid garment contacts

The numerical contact layer in include/malemod/garments/taut_contact.hpp refines
an already walked envelope using whole-triangle support planes. It keeps sewn
and terminal anchors exact, permits outward radial and contact-normal axial
relief within each longitude plane, bounds displacement by the existing grid,
rejects inverted faces, and publishes transactionally. Clear surfaces do not
receive unneeded correction.

WalkCertifiedTautEnvelope accounts for chord error by rebuilding a common
support cover after measuring failed primitive separation. Construction support
is bounded by both longitudinal and transverse mesh spacing. Its final surface
must separate every current primitive; refinement still passes the original
sampling budget. It returns the accepted seed so adapters can independently
check physical-space spacing after an affine chart transform. Contact coverage
is not a proof of appearance, cadence, native lighting or pelvic attachment.

Nonplanar outlines use common absolute interior height stations with a bounded
fade from each exact sewn height. The fade remains monotone; origins, source
lineage, UV aliases and topology are retained. Adapters version any denser mesh
and its measured bindings together, supply exact primitive support functions,
and independently certify the inverse-transformed surface before hiding body
faces. Failed geometry must not hide anatomy.

Wolverine adoption is through dependencies/base.lock.json and measured garment
binding revision6. Witcher and future spokes must use a tested Base pin with
their own measured primitives, pose contracts and native attachment evidence.
The current tests cover changing cube supports, all triangle corners, exact
anchors, unchanged clear surfaces, physical sampling and transactional rejection.
Five private native contact inputs were interpolated into the denser candidate
for diagnosis; that synthetic replay passed but is not observed native evidence.
Native full-matrix acceptance and performance remain required before deployment.
