# MaleModBase

This repository is the shared extraction and portability project. Wolverine's
canonical repository remains authoritative for its installed runtime until its
adapter consumes a tested version of this base. Do not modify or install into a
game as part of an extraction or offline export.

Read README.md, docs/ARCHITECTURE.md and docs/MIGRATION.md. Keep source commit,
file hashes and omissions in provenance/wolverine.json. Imported code under
legacy/wolverine is an exact reference snapshot, not a portable library. Its
historical instructions are stored as SOURCE-AGENTS.txt, not active AGENTS.md.

Shared modules must compile without game or graphics SDKs. Keep game buffer
layouts, draw hooks, bone palettes, shaders, input and installers in adapters.
Never invent source units, skeleton joint names or engine capability support.
Version shared geometry and binding data together. Preserve UV aliases, source
vertex lineage, interpolation donors and pressure-field support points.

Before claiming a game port works, distinguish reference asset export, offline
verification, runtime parity and observed in-game results. For changes to
conversion or physics, run the relevant tests and provenance verification.
Do not commit game packages, private audio, settings or captured user data.
