# Stock-pelvis anchor investigation

The user asked whether the angle pivot is making the collar unnecessarily hard
to model, and requested placement derived from the unmodified pelvis. The
measured candidate is the front surface at the existing authored centerline and
height. It moves the current mechanical pivot about 25.000 mm inward, 8.998 mm
downward and 1.300 mm toward the centerline in calibrated native model space.
This is a candidate authoring anchor, not a newly established biological
landmark or a deployed repair.

## Measurement and provenance

The original Geralt lower-body FBX matches the character profile's SHA256:
`486dcf786c56068a54625eaa4a7d1527e9dd3cf721b52a52a838f141cd5ad51d`.
No deformed capture or already-cut collar defines the new surface location.
The existing profile supplies centerline and height; a bounded inward ray
determines depth on the original stock triangles.

- Native candidate in meters: `(0, 0.070962338825, 0.975)`.
- Calibrated source candidate: `(6.562195452392, 0.126732349396, 83.422599792480)`.
- Existing source pivot: `(9, 0, 84.300003051758)`.
- LOD0 stock triangle 75; original donors `(71, 70, 65)` with weights
  `(0.009724172459, 0.641949776683, 0.348326050858)`.
- Independent LOD1 surface measurement is only 0.471057 mm farther forward.
  Use the LOD0 anchor as canonical; do not let LOD selection move the pivot.

`malemod_base.surface_anchor.measure_anchor` implements this measurement with
original triangle provenance, bounded nearest-hit selection, coordinate/unit
covariance and explicit rejection of missing or degenerate intersections.
`tools/measure_surface_anchor.py` records the input mesh hash and donor binding.
The adapter must supply the unmodified exported body; generated character
meshes and bindings remain private. Seventeen targeted Python tests pass,
including three new anchor tests and the existing section/constraint tests.

## Why it is a useful candidate

The captured source guide root is stationary across three states and seven
angle settings. The issue is not measured root drift. The root is in front of
the stock surface, so its extended centerline intersects the curved pelvis at
different heights as angle changes. At slider 15, that intersection lies
37.500 mm from the candidate surface center. At a surface hinge the line's
intersection at the hinge remains fixed by construction. This kinematic fact
does not prove that a finite-radius tube fits, or that its collar is smooth.

The adapter independently computes its collar field center by averaging LOD0
seam samples: `(5.2902708744, -0.1005438632, 81.7979288642)`. That average depends
on sample density and is not the area center of the opening. Unifying the
authoring frame is worth testing, but differing centers alone do not establish
the complete cause. Do not physically translate the whole opening by the
difference between these centers: that would unnecessarily move stock tissue.

## Actual-mesh counterfactual

An offline trial rebased the captured shaft material and complete guide by the
constant difference between the old pivot and the stock-surface candidate,
then refitted the collar. This preserves local shaft dimensions, but is NOT a
worker-level change to the angle pivot, rest chain, collision or skin bindings.
Maximum-size state-0 tests covered slider 10/15/25/35/50/70/90 and both LODs.

Matched fits with the old versus rebased root produced these remaining
area/neighbor constraint violation counts (LOD0 / LOD1):

| Angle | Old root | Rebased root |
| --- | --- | --- |
| 15 | 369 / 8 | 72 / 0 |
| 25 | 569 / 18 | 1 / 7 |
| 50 | 6 / 83 | 0 / 0 |
| 90 | 314 / 1681 | 818 / 255 |

The tradeoff at 90 disproves a universal numerical improvement claim. Actual
side/front/oblique offline views at 15 reduce the upper notch but retain a
lower defect. At 50 the local triangle guard passes both LODs, yet the render
shows a jagged upper join and a lower gap. Reject both visual candidates.
The fitting mask also still allows some healthy scrotal material to move;
that must be excluded before acceptance. Numerical convergence is insufficient.

## Coordinated adoption path

Use the measured stock binding as one canonical authoring origin for the
mechanical pivot, guide root and collar field. Preserve the existing neutral
pelvis positions, original-edge donors, waist and UV aliases. Refit the local
attachment around that origin rather than translating the stock body or
enlarging the entire collar. Circumference and inferior allowance continue to
come from the measured shaft section.

A real pivot change must consistently rebase the source rest shape, guides,
rest lengths, rotation, raphe support, collision and virtual skin binding.
Changing only the `ShaftRoot` literal can introduce an orbit or double offset.
The immutable legacy snapshot remains untouched. An opt-in shared implementation
must preserve the default source behavior, then be adopted by a pinned Witcher
adapter. Wolverine and other spokes can use the same measurement/contract with
their own observed stock surfaces; Geralt coordinates must not become shared
defaults or modify their accepted runtimes.

Source measurement and offline comparison are complete for this investigation.
No new native session, game installation or full attachment-gate pass occurred.
Various sizes, moving states, both resources/LODs, native side/front/oblique
views and campaign behavior still require observation before promotion.

Private evidence root:
`D:/MaleModBuilds/witcher-progressive-recruitment-20261012`.
Key files: `pivot-virgin-pelvis-audit.json`,
`geralt-stock-anchor-candidate.json`, `pivot-virgin-pelvis-side.png`,
`run_pivot_collar.py`, `run_centered_collar.py`, and
`swept-pelvic-pivot-s0-a{angle}/`.
