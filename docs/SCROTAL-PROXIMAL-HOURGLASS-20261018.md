# Proximal suspension and Hang-dependent neck

The requested adult-anatomy repair moves the suspended tissue and its attachment
toward the pelvis, broadens the upper attachment, and varies the connecting skin
between a full raised shape and a waisted lowered shape. It preserves the complete
shaft, neutral pelvis, retained sac topology, UV lineage and numerical worker.

## Shared implementation

The optional `suspension_shift` authoring parameter rigidly translates retained
suspended tissue toward the root before fitting the connecting loft. Its default
is zero. Donors, UVs and relative tissue geometry remain exact. Moving only the
upper footprint while leaving the lower cut forward caused the broader loft to
fail its orientation check; moving the whole retained object first resolves that
correspondence without relaxing the fold check.

`suspension_profile.hpp` shapes only generated suspension cross-sections. The
mechanical plan selects lofts connecting suspended owners to shaft owners; pelvic
ramps and retained tissue are excluded. Each section uses its perimeter-weighted
center, avoiding bias from unevenly sampled knots. A bounded radial profile has
unit value and zero first derivative at both joins. At minimum Hang the central
section expands by 10 percent; at maximum Hang it contracts by 38 percent. The
maximum change is two thirds of the way from the retained sac to the shaft rim.
A smooth control curve makes the transition monotone. Both ends retain their
original positions and tangents.

The feature is opt-in through `State::suspensionProfile`, carried through temporal
interpolation. Existing measured `SuspensionRestOffset` centers suspended tissue
under the final mounted rim. `SuspensionLift` retains its measured low-Hang
retraction. Neither numerical forces nor solver cadence are changed.

## Verification and adoption

Eight focused C++ tests and 22 authoring/material tests pass, with no skipped
tests. New tests cover endpoint and tangent preservation, bounded monotone waist,
retained ownership and scale/translation invariance. Actual-character checks
preserve the complete shaft exactly and retained sac geometry up to a rigid
translation; local neck and pelvic-ramp orientation checks pass.

The Witcher adapter owns measured Geralt parameters, native material rebaking,
control mapping and the installation gate. Candidate evidence is separate from
installation evidence under the private `scrotal-proximal-hourglass-20261018`
folder. Native review and final installation status are recorded in the adapter
report; successful source tests alone do not establish playability.

The Witcher candidate was subsequently installed October 19 after 252 native
review stages and 68 captured-pose audits. Installed startup and movement were
observed; all 53 managed and 31 protected files verify. Native source, offline
checks, visual review and installation receipts remain separate. The adapter
report records exact identities, limitations and rollback. No other spoke was
installed or published.

Other spokes, including Wolverine, can adopt the optional authoring translation
and profile flag through a pinned Base revision. Each must calibrate its own
attachment and validate controls, materials, motion, LODs and native appearance.
Their installed runtimes are unchanged by this shared implementation.
