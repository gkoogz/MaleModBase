# Source synchronization and release boundary

## October 5, 2026 review

Base's source checkpoint includes the retained walking cloth construction,
wire 6 reaction contracts and native sandbox scenario contract. These changes
are useful development source; they are not an accepted new cloth runtime.
The installed Witcher and canonical Wolverine adapters still use their separately
recorded Base `99ff741` baseline. Updating `main` does not deploy this checkout.

Current verification from this checkout:

- `python tools/verify.py`: passed imported source/material/185-array geometry,
  graft, collar, physics and clinical provenance checks (516 imported files).
- `python -m unittest discover -s tests -v`: 96 tests passed.
- Fresh Release CMake build: passed, including previously missing test binaries.
- Fresh `ctest --test-dir build/garment-cmake -C Release --output-on-failure`:
  37 of 39 passed. `garments` and `garment_cloth` reject their input with
  `Measured anatomy has multiple or unmatched open loops`. Those older synthetic
  fixtures do not pass the current measured root-closure contract. No passing
  actual moving cloth fit or release readiness is inferred from that diagnosis.

The exact build/test logs remain machine-local in ignored `build/`. Do not
silently remove these failing checks, weaken closure validation or relabel the
current shared garment implementation as accepted gameplay.

Sandbox packaging and instructions are development tooling in the adapters.
They belong in Git and are excluded from normal mod release downloads. Native
scene creation belongs in the adapter; common scenarios and evidence contracts
remain here. Read [ITERATION-SANDBOX.md](ITERATION-SANDBOX.md) before native work.

## Local material outside this sync

The untracked speculative teaching-doll/reference inspection work and private
voice transcript/provenance are separate local authoring inputs, not this game
mod release payload. Preserve them locally; do not stage private reference
images, captured user data, saves, audio, settings or generated game resources
while synchronizing the shared mod source.

## Future adoption

Use each spoke's exact source dependency pin for development. Keep frozen
release/runtime pins distinct. Before adopting a new shared numerical or cloth
revision, pass its source checks, measured character geometry/contact gates,
native rendering and moving gameplay acceptance in both development rooms.
Then verify the affected campaign transitions and complete release installer,
upgrade, rollback, settings and payload checks. Preserve historical failed
candidate receipts as evidence rather than shipping them as a downgrade.
