# Grey-room packaging and mod publication

## Repository responsibilities

Base owns the shared numerical and source scenario contracts. Both game
repositories retain their compact developer recipes and vanilla reproduction
instructions. Native room source is available by cloning Git; normal mod
Install packages omit the rooms and their generated resources.

| Repository | Developer entry | Frozen playable baseline |
| --- | --- | --- |
| MaleModBase | `docs/ITERATION-SANDBOX.md`, `profiles/iteration-sandbox.json` | Shared source; no game installer |
| XMenOriginsWolverineMaleMod | `dev/sandbox/README.md`, `tools/iteration/README.md` | Base `99ff741`, approved installed runtime `4F4900A5E70CE9BB127B46C3D7FE57EF74FFEACE8A5AE704D3DA1EFF6E2BA9A3` |
| TheWitcher3MaleMod | `dev/sandbox/README.md`, `tools/sandbox.py` | Base `99ff741`, approved installed runtime `d475488fefe405213de5df058ccaa5c94190ee2089bead7920e1450fe5750099` |

The adapters' development dependency pin is `421a786`, containing the retained
shared source checkpoint and development instructions. Later documentation-only
Base commits do not replace this intentional source pin or either frozen runtime.
Numerical adoption requires an explicit new pin, build and native acceptance.

## Published checkpoint

Both adapter `main` branches and beta tags match their source commits. GitHub
reports both releases as published prereleases; uploaded asset sizes and SHA-256
digests match the verified local packages.

- Wolverine source: `b174fb0b9033da03670da95da6eb3e65a5d480b6`.
  [2.0 Beta 2 release](https://github.com/gkoogz/XMenOriginsWolverineMaleMod/releases/tag/v2.0.0-beta.2).
  Install SHA-256: `44a2780aa5ad5e58d6ff83d259009633861c4c6b5c40c9e20618f03d0d29e63e`.
- Witcher source: `75a36c954d043c955269dbdad4667ea559c51a8a`.
  [0.5.0 Beta 2 release](https://github.com/gkoogz/TheWitcher3MaleMod/releases/tag/v0.5.0-beta.2).
  Install SHA-256: `0baa19e936029864cb3d11a430ffaa87b0eec717cc1be6aabf42cc80613b5f08`.

The sandbox recipes are available in Git checkouts and omitted from installer
payloads. Wolverine's curated optional Source ZIP also omits its sandbox.
GitHub's automatically generated source snapshots are repository source, not
installed game payloads. Use a Git clone for the complete development workflow.

## Reproduction boundaries

Wolverine derives local native layers and a synthetic developer checkpoint from
the owner's supported vanilla game and the released mod. The engine generates
its initial profile; the small typed recipe seeds only the development reference
state. No personal save/profile is imported or shipped. The fresh private room
reached observed gameplay; movement, F6, Overall 50 to 51 and native capture were
acknowledged. Stock `jungle1_p` remains part of the bootstrap.

Witcher derives its grey platform/environment from licensed local SDK donors,
uses a fresh stock q002 bootstrap and the released native adapter, and generates
machine-local build/gameplay/visual acceptance. A tracked-only adapter plus Base
export passes its portable source checks and unit suite. The established room
shows the complete body, grounded walking and clean movement captures; a newly
cooked vanilla gameplay test was not repeated during packaging. Stock q002
remains loaded underneath. Neither room is a smallest standalone empty world.

These recipes require the documented Windows toolchains and exact supported
game builds. A successful source build is not inherited gameplay proof for an
unobserved executable, graphics driver, clothing candidate or new machine.

## Release and source distinction

The game betas preserve the exact observed installed runtime bytes, with complete
installer, supported upgrade, idempotency, rollback, settings and refusal checks.
The newer development cloth source is committed for continuation and is not
silently delivered as accepted persistent fabric. Private captures, saves,
settings and recordings are not tracked or uploaded.

Wolverine's frozen production source is `16affd49da5a122df776969574ca62a984b59425`.
It rebuilds, but multiple PE sections differ from the approved installed binary;
do not claim byte-identical reproduction from compiler success. Witcher's exact
adapter source commit for its frozen DLL was not recorded; its frozen delivery
provenance explicitly separates that payload from the current source checkpoint.

Read each game handoff and release notes for actual remaining cloth, material,
cadence and campaign limits. Base's [DEVELOPMENT-STATUS.md](DEVELOPMENT-STATUS.md)
records its two failing legacy cloth fixture gates. Do not describe all development
source as release-ready because the existing runtime package passes installation.
