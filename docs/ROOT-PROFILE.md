# Shared angular root-profile regularization

`anatomy.source-root-profile` version 1 ports the active
`RegularizeSharedRootProfile` and `LogicalShaftOwner` stage from the authoritative
Wolverine snapshot. Implementation: `malemod_base/root_profile.py`.

The caller supplies the complete versioned 2,388-point prepared source surface,
its measured `RestFrame`, and explicit source collar growth in [0, 1.5]. These
are Wolverine reference coordinates, not inferred SI or native game units.
Reference shaft, attachment, pouch and suspension fields are instance-owned.

The evaluator measures 24 angular sectors downstream, fills sparse sectors
using the original circular neighbor policy and carries that profile back
toward the collar. Root flare follows source growth; original ownership and
quintic seam fades bound the correction. Mixed pouch rows are excluded. The
source radial correction limit is preserved. It returns independent positions,
sector radii and a changed-vertex mask without altering source bindings/aliases.
Float32 accumulation follows original vertex order.

## Verification and limits

The extraction recipe retains the exact original C++ functions and hashes the
source, implementation, harness and deterministic NPY output. The oracle consumes
identical caller-supplied float32 points/rest frames, isolating this stage from
the separately tested early shape and rest measurement. Every vertex in 20
profiles times three mechanical modes matches: 143,280 comparisons. Tests also
check excluded pouch/downstream rows, instance isolation and invalid inputs.

```powershell
python tools/extract_root_profile.py --check
python -m unittest discover -s tests -p test_root_profile.py -v
python tools/verify.py
```

Build the `root_profile_oracle` CMake target, then regenerate fixtures with
`python tools/build_root_profile_fixture.py path/to/root_profile_oracle.exe`.
Run the extraction again after regeneration to refresh provenance.

This does not implement preceding fairing, logical/glans construction, egg rest
fitting, final UnifiedCollar, posed dynamics or native upload. It does not move
one side of an existing body-part boundary or guarantee a full expansion envelope.
Rebuild the source rest frame after this stage, as the original pipeline does.

## Adoption across spokes

- Witcher: consume this stage from the pinned Base during control-change rest
  preparation, using the observed fit frame, original donors and protected waist.
  Native connected pose/scale output is now observed, but complete independent
  controls and the dynamic pelvis remain unfinished. Avoid per-vertex frame loops.
- Wolverine: use the same fixture cases to verify a future replacement of its
  original stage behind the adapter. Keep the current runtime authoritative until
  full preparation/posed parity passes; no installation or runtime change is made.
- Future spokes: require the matching source geometry/binding revision and an
  observed target frame. Keep the common stage here and native pose policy in
  each engine adapter.
