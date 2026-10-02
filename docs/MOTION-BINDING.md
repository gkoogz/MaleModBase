# Motion cage authoring

`malemod_base.motion_binding` transfers the archived coarse mechanical fields
through the preserved R14, support and final-render donor tables. Geometric
interpolation uses signed coefficients; the final scalar weights are clamped to
[0, 1]. This is a new authoring approximation, not Wolverine runtime parity.
Original geometry, donor tables and unrendered supports remain unchanged.

The module also supplies a nonnegative, two-influence partition over explicitly
authored chain coordinates, reference-derived cage centres and a seam-faded
eight-joint/two-lobe binding. Lateral blend and seam distances are supplied by
the adapter in its measured authoring frame. Native bone names, measured unit transforms, joint
rest frames, dynamics resources and SDK writers belong in each adapter.

A spoke must preserve body and attachment seam weights, protect existing body
part boundaries and verify native bone/weight round-trip before installation.
This module alone does not enable motion or live dimensions in any game.

Wolverine adoption: use the field transfer and partition tests for future native
rig exports; retain its authoritative full XPBD/contact solver. Future spokes
can consume the same transfer through a pinned Base revision and provide their
own motion cage. Native secondary-motion solvers need independent calibration.
