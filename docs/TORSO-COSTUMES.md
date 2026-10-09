# Independent torso and pelvic costumes

`controls/costume.hpp` defines Top (Naked/Tank Top) and Bottom
(Naked/Jockstrap). Missing bottom keys migrate the legacy Style value; the
legacy setting never selects a top. Adapters persist both selections separately.

`torso_garment_fit.py` refits measured stock garment vertices against measured
body triangles. It preserves height, source ease, topology and UV alias ordering.
Open armhole patches use explicitly bounded closest-triangle bindings. Adapters
retain the original skinning and source vertex IDs, and transport fitted seams
with their current body donors. Numerical units are supplied by each export.

Wolverine consumes this through an exact Base lock and a measured stock Alkali
tank recipe. The native shirt and accessory contact use the same fitted surface;
neither a second anatomy solver nor a reduced update cadence is required.
Witcher remains deferred: future adoption requires an observed character export,
its own native renderer/material mapping and measured accessory pose adapter.

Run `python -m unittest tests.test_torso_garment_fit`; compile
`tests/costume_test.cpp` against `include` without graphics or game SDKs.
Source/offline success does not certify native fit, pelvic attachment or gameplay.
