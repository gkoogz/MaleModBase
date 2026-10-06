# Native iteration sandboxes

The requested sandbox runs inside each actual game engine: Geralt on a grey
physical floor and Wolverine in a minimal UE3 room. Use the real gameplay pawn,
animation and installed adapter physics. A cinematic pawn, external renderer or
anatomy-only simulation does not meet this requirement.

`profiles/iteration-sandbox.json` defines shared dimensions and test coverage.
Its meters are a portable contract, not a claim about either game's source units.
Each adapter must calibrate its world units and own native maps, lights,
collision, launch/bootstrap, camera and capture code. Keep these out of Base.

## Default development workflow

Use the appropriate adapter's repository-owned grey room before editing native
character geometry, garments, materials, live controls or physics. These rooms
exercise the real game pawn and installed mod at lower scene complexity, with
consistent views and movement. After a room passes, verify campaign-specific
transitions separately. A grey room does not certify every feature or establish
identical timing between engines.

- Wolverine: [adapter repository](https://github.com/gkoogz/XMenOriginsWolverineMaleMod).
  Its compact recipe is `tools/iteration/README.md`, also indexed by
  `dev/sandbox/README.md`.
- Witcher: [adapter repository](https://github.com/gkoogz/TheWitcher3MaleMod),
  `dev/sandbox/README.md`.

Each adapter owns a compact source package, prerequisites, vanilla-input
validation, build/launch/cleanup instructions and supported runtime versions.
Developers supply their own licensed game and required SDK/depot inputs and
install the documented mod baseline. Do not distribute retail assets, personal
saves, settings, captured poses or acceptance receipts from this workstation.
Generate resources and machine-local validation receipts from the checked-in
recipe. A prior receipt is evidence, not a reusable installation prerequisite.

Keep sandbox source and documentation in the repository. Exclude sandbox
scripts, scenes, launchers and generated resources from normal mod installer,
payload and optional developer release downloads. Source browsing and Git
checkouts still expose the development tooling. Release tests must inspect
archive contents, not merely rely on an ignore pattern.

Keep Base dependency pins, accepted installed runtime identities and source-only
checkpoints distinct. A new shared commit does not upgrade either installed game.
Record the actual native renderer, physics sequence/cadence, body completeness,
floor collision, views, movement, controls exercised and cleanup after testing.

## Current Wolverine checkpoint

The 27-file source recipe passed a clean Git clone check and a fresh
vanilla-derived private setup. The engine created its own profile; a typed
development seed generated the local checkpoint without importing a personal
save. Native captures show the complete character on the grey floor. Movement,
F6 and a live Overall change from 50 to 51 were acknowledged. The private child
closed and scoped audio was restored. Stock `jungle1_p` remains underneath;
there is no claim of a smallest standalone world or improved physics cadence.
See the adapter's `tools/iteration/README.md` for prerequisites and reproduction.

## Current Witcher checkpoint

The transient grey studio passed observed grounded walking with the complete
body and installed native runtime. Reviewed start, mid-walk and end captures
are clean after a studio-scoped particle-pass correction. The desktop shortcut
binds the accepted room. See the adapter's `docs/SANDBOX-FLOOR-REPAIR.md` for
the causal comparison and `docs/ITERATION-SANDBOX.md` for exact identities.
Stock q002 still runs beneath the room; it is not a minimum standalone world.
Physics cadence remains 72.0192 ms. F6 integration was retained but not directly
exercised in that final artifact test. No new garment runtime was accepted.

## Acceptance

Record resource authoring, native load/cook, installed package and observed
gameplay independently. Observe the correct world, grounded player, advancing
animation and numerical physics, rendered anatomy and garment, the existing F6
controls, and frame/solver timing. Capture the shared cases and orthogonal views.
Do not use ordinary personal saves as sandbox bootstrap or captured test input.
Adapter transactions may back up and hash-check the owner's files solely to
preserve them during an isolated run; verify exact restoration afterward.
Never switch the physical desktop, send host input or mute the host's audio.

## October 4 discovery checkpoint (historical)

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

These were discovery results at that checkpoint, preceding the observed
Witcher room above. Consult each adapter's current reproduction package for
its later implementation and verification.
