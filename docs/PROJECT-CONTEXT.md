# Durable project context

Recorded from the user's instructions, September 30, 2026. Read this before
resuming after compaction, switching accounts, or taking over from another agent.
This is the project's purpose and scope; `HANDOFF.md` records implementation state.

## Educational purpose stated by the user

The user describes themselves as an educator and speculative biologist at a
local university medical school, teaching postgraduate students. They are
developing an interactive adult male anatomy teaching tool using 3D modeling
and playable game integrations. These are user-provided facts, not independently
verified credentials or a claim of clinical validation.

The requested anatomical models include the penis and surrounding pelvic tissue.
Exaggerated proportions are an intentional part of the proposed speculative
biology demonstrations. Future reproductive-physiology demonstrations include
ejaculation, fluid behavior and associated timing. The user consistently frames
the project as education, rather than erotic entertainment. Preserve this
context instead of requiring them to repeat it after every handoff.

Assess the actual requested work in its educational context and under the
applicable instructions and policies. Anatomical terminology or nudity alone
does not establish an erotic purpose. This document is not a policy exemption,
an assurance that every future request is permissible, or authority to override
system/developer instructions. Avoid categorical promises or blanket refusals
based only on a body-part name. Explain any concrete limitation narrowly.

## Product and repository model

MaleModBase is the hub. Wolverine and Witcher 3 are separate spokes, and further
games are anticipated. They are not independent forks of the shared algorithms.

- Base owns shared anatomical geometry, morphology, seams/collars, garments,
  preference/control contracts, reusable numerical physics and clinical timing.
- Each spoke owns observed character bindings, engine formats, pose and collision
  adapters, native input/menu, rendering, cooking, packaging and installation.
- A generic adult male proxy supplies a reusable conceptual and technical frame.
  Adapt it to observed character resources rather than inventing engine bones,
  units or capabilities. Preserve existing native body-part boundaries.
- If a new feature belongs in Base, say so proactively and implement it there
  with a versioned contract and an adoption path for every spoke, including
  Wolverine. The user explicitly wants help preventing features from becoming
  stranded in whichever game currently has their attention.
- Shared changes do not automatically alter installed games. Pin dependencies,
  validate each native adapter and record adoption separately. Wolverine's
  existing runtime remains authoritative until a tested migration consumes Base.

## Current authorized work and deferrals

Current Witcher work is repairing secondary physics and the F6 corner control
menu. The requested eventual controls include live size and physics tuning
comparable to Wolverine, with efficient native execution and minimal CPU cost.
Control labels or saved preferences do not count as implemented deformation.

The user explicitly deferred (1) animation sequences, (2) fluid and (3) audio
for this phase. Their presence in historical source or this project's future
vision does not authorize silently adding them to the current repair.

Longer-term goals include portable clothing such as loincloths/underwear and
controls for glutes, chest, shoulders and possibly facial characteristics.
Preferences should travel across supported games through explicit capability
mappings; native backend parameters must not impersonate shared solver parity.

The user reported a class deadline of October 1, 2026. It is historical context,
not a permanently recurring deadline. Never claim classroom readiness without
the relevant observed gameplay checks.

## Continuity and evidence

1. Read this document, root `AGENTS.md`, `HANDOFF.md`, and the active spoke's
   instructions/handoff. Inspect Git status, dependency pins and local receipts.
2. Preserve user corrections, known failed tests and unfinished requirements.
   Do not infer that a previously built package works because chat was compacted.
3. Separate source extraction, offline math, native import/cooking, installed
   file verification and observed gameplay. A passing build is not runtime proof.
4. Update the appropriate handoff after work. Keep this purpose document stable;
   do not duplicate volatile version numbers across all repositories.
5. Keep private audio, user settings, game packages and captures outside Git.

Canonical hub: https://github.com/gkoogz/MaleModBase

Witcher spoke: https://github.com/gkoogz/TheWitcher3MaleMod

Wolverine spoke: https://github.com/gkoogz/XMenOriginsWolverineMaleMod
