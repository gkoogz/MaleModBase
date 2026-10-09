# Independent torso and pelvic costumes

`controls/costume.hpp` defines Top (Naked/Tank Top) and Bottom
(Naked/Jockstrap/Jeans/Jeans open). Missing bottom keys migrate the legacy Style value; the
legacy setting never selects a top. Adapters persist both selections separately.

`torso_garment_fit.py` refits measured stock garment vertices against measured
body triangles. It preserves height, source ease, topology and UV alias ordering.
Open armhole patches use explicitly bounded closest-triangle bindings. Adapters
retain the original skinning and source vertex IDs, and transport fitted seams
with their current body donors. Numerical units are supplied by each export.

For enlarged torso fits, refine measured edges with explicit source-pair
lineage, relax internal angle/height backtracking in the tubular chart while
pinning true cut edges, then project and clear dense measured face supports.
Position aliases move together while UV aliases remain separate. Independent
projection of stock folds can collapse their depth; check actual native shading
and winding as well as a flat-color silhouette. The common-section expansion
alternative is not Wolverine's active fit: it inflated the stock shirt in native
testing and was rejected.

Wolverine's active authoring pin is 45ca775a2a4fe2e748ec98966727c1702864928b.
Its adapter keeps the stock winding sign, current body normals, material passes
and dog-tag surface contact. Eight fitting regressions and independent wardrobe
preference tests pass offline; native acceptance is recorded separately in the
adapter's docs/TANK-TOP-COSTUME.md. No equivalent Witcher native adapter exists.

Wolverine consumes this through an exact Base lock and a measured stock Alkali
tank recipe. The native shirt and accessory contact use the same fitted surface;
neither a second anatomy solver nor a reduced update cadence is required.
Witcher remains deferred: future adoption requires an observed character export,
its own native renderer/material mapping and measured accessory pose adapter.

Run `python -m unittest tests.test_torso_garment_fit`; compile
`tests/costume_test.cpp` against `include` without graphics or game SDKs.
Source/offline success does not certify native fit, pelvic attachment or gameplay.

`fly_panels.py` clips measured source triangles at an adapter-supplied fly
envelope and folds the two panels around their outer attached edges. Every
generated vertex retains source-face donors and barycentric weights; the
adapter interpolates UVs and measured skin influences from those donors. The
same source rest geometry and current skeleton drive both variants without a
new cloth solver. Wolverine implements the first measured adoption; future
spokes must supply their own axes, dimensions, source assets and native tests.

`radial_garment_coverage.py` preserves measured cutouts in an angular/height
chart, then tests radial containment. Wolverine uses this only to mask covered
bare torso draw faces, avoiding later native body passes overwriting its cloth.
The historical orthographic garment_coverage.py API and provenance are unchanged.
This authoring sample is not a universal whole-fragment containment certificate.

The October9 stock wardrobe review proposes one user-facing refit factor for
multiple garments. It is a design candidate, not an implemented universal fit.
Blend body-relative donor displacement with garment-specific ease/protected
regions; whole-model scaling would also enlarge boots, buckles and masks.
Enforce clearance independently, since partial fitting can otherwise intersect
an enlarged body. Full suit pieces need authored coupled waist boundaries and
coverage masks before independent Top/Bottom mixing. Cache bindings offline and
follow current body/skeleton pose at runtime. Shared math belongs here; native
asset segmentation, observed palettes and materials remain in each adapter.
Wolverine's docs/STOCK-WARDROBE-CANDIDATES.md records the first source review.
No runtime change or Witcher adoption is claimed by these reference renders.
