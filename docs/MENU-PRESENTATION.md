# Shared menu presentation

The October 9 user request is a restrained, masculine and minimal live menu,
with better typography and consistent presentation across game adapters.
`controls/menu_theme.hpp` owns the charcoal, ivory and muted brass palette,
condensed sans-serif preference, weight and concise title/navigation vocabulary.
It has no game, graphics or operating-system dependency.

Adapters use the font installed on the local machine with a native sans-serif
fallback; no font files are copied or distributed. Wolverine rasterizes a
grayscale native GDI atlas for its Direct3D9 overlay, retaining its small bitmap
fallback if font/texture allocation fails. Witcher uses its existing GDI panel.
Each adapter owns layout, renderer lifecycle, hit regions and input handling.
State, numerical controls, persisted keys and engine capabilities are unchanged.

Adoption requires an exact Base pin containing this header in each adapter.
Source compilation and panel previews do not establish observed gameplay or
the mandatory attachment gate. Native scoped menu open/selection/adjustment,
closure and reset rendering need adapter evidence; combined character and
attachment regressions remain required before installation or release.
