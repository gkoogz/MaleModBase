# Preserving stock clothing during refitting

Keep the original garment cut, UVs, materials, folds and donor lineage. A larger
torso needs measured fitting, not a replacement procedural cotton/denim look.
Original licensed pixel assets stay local to each adapter's build.

`torso_garment_fit.expand_projected_sections` supports an optional smooth signed
offset field. It translates each front/back height section, vanishes at the
side join, and retains source Y/Z coordinates. The adapter supplies measured
body supports and may restrict front/back envelope constraints to outward
faces. This restriction does not certify grazing-face or full-volume clearance.
Review those surfaces, clothing cutouts and animation in the native room.

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
