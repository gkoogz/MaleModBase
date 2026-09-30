# MaleModBase

Shared anatomy assets and portability infrastructure extracted from
[XMenOriginsWolverineMaleMod](https://github.com/gkoogz/XMenOriginsWolverineMaleMod).
The first import preserves the developed Wolverine source and creates usable
geometry exports; it does not yet replace the installed Wolverine runtime.

**Resuming without chat history? Read [the handoff](docs/HANDOFF.md) first.**
It maps all three repositories, records the current implementation and observed
game state, and explains what a fresh machine must regenerate.

## Contents

- `legacy/wolverine/`: tracked source, geometry tables, morphology, XPBD/contact
  physics, material code, existing simulation code, remeshing inputs and tools.
- `assets/wolverine-reference/`: OBJ surfaces, portable NumPy geometry/binding
  arrays and source metadata. Reference geometry is not a captured live pose.
- `assets/generic-male/`: metric adult male proxy with semantic skinning and
  attachment sockets in GLB and JSON; see [generic framework](docs/GENERIC-BASE.md).
- `include/malemod/`: standalone C++ math, triangle correction limiter and
  portable clinical sequence with fluid simulation, meshes, deposition and cues.
- `adapters/`: Wolverine dependency profile and a Witcher port profile with
  explicit unresolved requirements.
- `provenance/`: byte hashes and an inventory tying every imported file to its
  original commit; compiled artifacts and installers' payload remain external.
- `tools/`: repeatable import/export, verification and character fitting tools.
- `templates/github-actions-verify.yml`: CI template; activation requires a
  GitHub credential permitted to create Actions workflows.

## Use

```powershell
python -m pip install -r requirements.txt
python tools/export_geometry.py
python tools/extract_clinical.py --check
python tools/extract_physics.py --check
python tools/extract_collar.py --check
python tools/build_generic.py
python tools/verify.py
python -m unittest discover -s tests -v
```

Import `assets/wolverine-reference/final-reference.obj` into a DCC tool for
geometry inspection. `coarse-sculpt.obj` preserves the earlier authored R14
surface; `support-reference.obj` preserves the triangulated support reference.
`geometry.npz` retains rigging indices, weights, morphology and binding tables.
OBJ does not carry rigs, materials, animation or solver behavior.

Fit a reference asset using a character profile:

```powershell
python tools/fit_mesh.py assets/wolverine-reference/final-reference.obj profiles/reference-character.json build/fitted.obj
```

To reproduce the source import from a local Wolverine checkout:

```powershell
python tools/import_wolverine.py --source C:/path/to/wolverine
```

Read [architecture](docs/ARCHITECTURE.md), [migration status](docs/MIGRATION.md),
[asset contract](docs/ASSET-CONTRACT.md), [clinical sequence](docs/CLINICAL-SEQUENCE.md)
and [porting](docs/PORTING.md).
The source-derived [pelvic collar](docs/PELVIC-COLLAR.md) now includes a portable
metric kernel and offline coupled evaluator with hard body-edge seam constraints.
Commercial reuse rights are not established by this extraction: upstream has
no root license, and game-derived assets retain their separate provenance.
