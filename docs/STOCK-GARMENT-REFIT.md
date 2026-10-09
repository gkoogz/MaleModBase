# Preserving stock clothing during refitting

Keep the original garment cut, UVs, materials, folds and donor lineage. A larger
torso needs measured fitting, not a replacement procedural cotton/denim look.
Original licensed pixel assets stay local to each adapter's build.

`garment_shell.clip_scalar_band` clips an adapter-measured material strip with
source-face barycentric lineage. `thin_shell` adds a shallow inner face and cut
walls, welding positional aliases only for topology and offset directions.
It retains separate source-corner donors and separate wall shading groups.
Thickness and strip fields are adapter inputs in measured source units.
Wolverine adopts this through a Base pin for the original full-waist leather
chart and open fly. Other spokes author their own measured strips and thickness;
Witcher remains paused. This geometry does not certify body collision.

`torso_garment_fit.curved_boundary_midpoints` rounds newly subdivided cut edges
using their original edge tangents, a measured adapter mask and a bounded
offset. Original corners remain exact. Skin/UV lineage stays on the original
edge. Wolverine uses it at the stock tank neckline/armholes and recomputes
shading normals from the actually skinned mesh, preserving UV aliases.
Tests: `python -m unittest tests.test_garment_shell tests.test_torso_garment_fit`.
Native, campaign and full attachment acceptance must be recorded independently.

`torso_garment_fit.expand_projected_sections` supports an optional smooth signed
offset field. It translates each front/back height section, vanishes at the
side join, and retains source Y/Z coordinates. The adapter supplies measured
body supports and may restrict front/back envelope constraints to outward
faces. This restriction does not certify grazing-face or full-volume clearance.
Review those surfaces, clothing cutouts and animation in the native room.

The optional `lateral_spacing` extends this to a smooth height/lateral tensor
field. Expansion needed at the center of the chest no longer translates every
point on a height section by that maximum. Original Y/Z, folds, source donors
and UV aliases remain intact. Wolverine supplies measured 3-unit lateral knots;
other adapters choose their own measured spacing and native fit validation.
The localized-envelope regression checks that a remote cut is not inflated
by a central support peak. This is not a full-volume collision certificate.

Independent vertex projection can erase stock ease and overturn fine folds.
Wolverine's previous tank revision8 had 509 faces whose normals rotated beyond
90 degrees relative to stock. Revision9's measured offline candidate has none;
maximum rotation is 25.7941 degrees. This is an offline orientation comparison,
not a collision or attachment visual pass. Its exporter refuses overturned
source faces and retains original source-pair donors and UV aliases.

`fly_panels.fold_rigid_attachment` applies one proper rotation to an accessory.
Its centroid follows the same measured flap envelope while all pairwise
distances remain unchanged. The existing cloth fly partition retains its
barycentric lineage. Wolverine uses this for the original separate belt buckle.

Wolverine owns its meshes, original texture resources, bindings, native lights
and renderer. It adopts these shared Python authoring functions through its
exact Base lock and versions the generated geometry with its binding receipts.
Witcher remains paused. Future adapters use measured local frames and their own
licensed materials; they do not copy Wolverine's coordinates, bones or shaders.

Regression command: `python -m unittest tests.test_torso_garment_fit
tests.test_fly_panels tests.test_radial_garment_coverage`.

The SDK-free `physics/bounded_hinge.hpp` adds damped angular secondary motion.
It integrates constant measured torque analytically, limits travel, dissipates
outward stop velocity and resets invalid samples/stalls. The adapter advances
once per frame, measures its own source-space motion, and resets on wardrobe
switches, teleports and device loss. It owns gravity-axis and gain calibration.
`fly_motion_bindings` preserves duplicate hinge pins with continuous response;
a measured upper band gives original belt ends their separate small response.
This bounded accessory model does not claim general cloth/body collision.

Wolverine's adoption keeps the original stock jeans/belt UVs and rigid buckle,
reduces the fly half-width from 14 to 9 measured source units, and folds it back
from 145 to 155 degrees. Denim travel is at most 3 degrees; belt travel is at
most 2 degrees. Native normals are rebuilt with position aliases while stock
UV aliases remain distinct. The closed variant is unchanged. Other spokes
adopt this header and author measured hinges/bands through their own Base pin;
Witcher remains paused. Numerical tests: `bounded_hinge_test`; binding tests:
`tests.test_fly_panels`. Every native adoption still needs the full attachment
regression and its own material/collision/motion review before installation.
