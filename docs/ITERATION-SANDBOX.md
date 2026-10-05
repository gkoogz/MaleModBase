# Native iteration sandboxes

The requested sandbox runs inside each actual game engine: Geralt on a grey
physical floor and Wolverine in a minimal UE3 room. Use the real gameplay pawn,
animation and installed adapter physics. A cinematic pawn, external renderer or
anatomy-only simulation does not meet this requirement.

`profiles/iteration-sandbox.json` defines shared dimensions and test coverage.
Its meters are a portable contract, not a claim about either game's source units.
Each adapter must calibrate its world units and own native maps, lights,
collision, launch/bootstrap, camera and capture code. Keep these out of Base.

## Acceptance

Record resource authoring, native load/cook, installed package and observed
gameplay independently. Observe the correct world, grounded player, advancing
animation and numerical physics, rendered anatomy and garment, the existing F6
controls, and frame/solver timing. Capture the shared cases and orthogonal views.
No automated test may read or write the user's ordinary saves, switch the
physical desktop, send host input or mute the host's audio.

## October 4 checkpoint

REDkit's reference world and game definition have been dumped with the official
tool. The measured game class is `CWitcherGameResource`; its player template is
`gameplay/templates/characters/player/player.w2ent`. The game's script API has
`RequestNewGame(gameResourceFilename)`. A dedicated game definition can select
the test world; loading a normal save would select that save's original world.

Wolverine's retail installation contains Raven cooked `.xxx` packages. Its UE3
configuration advertises map URLs, but no compatible Raven map editor/cooker
has been located. PhysX headers are not a UE3 map SDK. Public UDK compatibility
and retail direct-map loading are unverified. The existing WStart cinematic
anatomy support is not a walking gameplay sandbox.

These are discovery results. Neither sandbox has passed native gameplay.
