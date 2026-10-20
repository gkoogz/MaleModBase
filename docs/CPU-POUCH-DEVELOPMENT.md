> RETIRED October 7: The user rejected this pouch and fabric logic. These are historical failure receipts, not current implementation instructions. Continue with docs/COLLISION-INSPECTION.md.

# CPU pouch development checkpoint — 2026-10-07

This is an **unaccepted offline experiment**, not a playable release or a passed
attachment-integrity gate. No installed Wolverine or Witcher runtime has been
changed by this work. The user requested a playable Wolverine prototype and
specifically asked to use established game-clothing techniques.

## Diagnosis and replacement

The supported reference solver previously mixed interpolated collision hits
with final-pose reaction barycentrics. Contact weights now come from the exact
queried collider. Its complete material/contact solve also belongs inside each
substep. The corrected captured coalesced replay passed its old contact/material
checks, but took 39.8901 seconds for 5.453 seconds of motion. That is unsuitable as
the new runtime path.

The optional `malemod_cloth` target uses the standalone NvCloth CPU solver. Base
owns the numerical wrapper, measured capsule bindings, garment fitting, cloth
pattern and render-to-simulation bindings. Public interfaces contain no engine,
graphics or NVIDIA SDK types. The adapter must still own coherent pose capture,
native rendering, input, worker lifecycle, dependency pinning and installation.

`PouchSession` is separate from the existing `Session`; no adapter has yet adopted
it. It builds only with the additional default-off option
`MALEMOD_BUILD_POUCH_EXPERIMENT=ON`. The waistband and most rear straps follow
measured body triangles. The pouch
uses a separate simulation mesh with fewer vertices at the narrow lower seam,
virtual collision samples and tethers. The final experiment uses a cropped actual
body triangle patch; the preceding backstop integration was rejected.
The lower attachments use
the measured perineal ends of the glute routes, rather than anatomy-root donors.
The short strap connectors and hems follow the sewn pouch edge.

The retained refinement rebuilds the body-supported straps from the measured
centerline, projects both width rails onto actual skin, and sews each final cap
to the pouch's actual transverse seam. Independently transported cap frames
were the source of the last strap/body intersections. The retained replay has
zero strap/body face intersections in all seven poses. This is a narrow offline
intersection result, not complete anatomy contact, native appearance or gameplay.

`material_metric.hpp` measures the intrinsic rest length of a rendered edge
across simulation triangles. A straight 3D chord across a folded rest surface
can incorrectly report unfolding as stretch. Its folded-surface, shared-edge,
reversed-direction, rigid-transform/scale and missing-coverage tests pass.

The NvCloth wrapper explicitly keeps actual triangle edges stiff. The generic
cooker's geometric classification can label an actual edge as bending on a
tapered/folded input. Per-constraint stiffness retains soft non-edge bending
without softening those material edges. No upstream dependency source changed.

Capsules retain measured triangle support ownership and source-bound axis ends.
They enclose the assigned actual source vertices. Instantaneous PCA axes were
rejected because round cross sections can switch eigenvectors and introduce
false collider motion. Capsule centers must not simply use the surface extrema:
that extends both ends by another full radius. The fitter searches tighter
enclosing capsules along the bound axis.

The current fitting experiment has 25 percent pattern ease, followed by ordinary
persistent simulation; animated frames do not rewrite material rest lengths.
This choice is not a calibrated physical fabric. Material strain, visual shape,
coverage and body contact are still failing acceptance. Do not relax acceptance
thresholds merely to turn a failed replay green.

This backend currently provides **one-way cloth contact only**. It emits no
invented tissue impulses. The existing reciprocal support interface remains
available in the reference solver, but measured coupling has not been
implemented or accepted in this experiment.

## Dependency and reproduction

NvCloth source: https://github.com/NVIDIAGameWorks/NvCloth

Pinned revision: `6e86b1838ae185c527744d475d34777ff69c1b8a` (1.1.6).
The checkout belongs in ignored build storage, not in an independently maintained
source fork. CUDA and DX11 are disabled. `cmake/nvcloth-compat/typeinfo.h` supplies
the standard header spelling needed by current MSVC; upstream source is unchanged.
The upstream NVIDIA/PxShared notices are retained in `third-party/licenses/`
and must accompany future library redistribution. This checkpoint does not
distribute a game package.

```powershell
git clone https://github.com/NVIDIAGameWorks/NvCloth.git build/dependencies/NvCloth
git -C build/dependencies/NvCloth checkout 6e86b1838ae185c527744d475d34777ff69c1b8a
cmake -S . -B build/nvcloth-cmake -A Win32 -DMALEMOD_BUILD_NVCLOTH=ON -DMALEMOD_BUILD_POUCH_EXPERIMENT=ON -DBUILD_TESTING=OFF
cmake --build build/nvcloth-cmake --config Release --target cpu_cloth_test replay_cpu_pouch
build/nvcloth-cmake/Release/cpu_cloth_test.exe
build/nvcloth-cmake/Release/replay_cpu_pouch.exe PRIVATE_RECORDING_DIRECTORY 60 --contacts
```

Recordings contain `reference.input`, `prepared.input`, and contiguous
`request-1.input` onward. They remain private and must not be committed. The replay
returns nonzero for failed contact/material checks and exports OBJ/layout files
beside the private inputs. `body_intersections` audits the complete actual body
mesh; pouch proxy clearance alone is insufficient. Fine rendered mesh strain is
measured separately from simulation edges against the authored eased pattern.
Complete anatomy/trim, volume membership and coverage auditing remain incomplete.
The optional `--contacts` writes exact intersecting garment/body face IDs next
to each private mesh export. Those witnesses located the rotating strap caps.

The final generic CPU cloth test passes on MSVC x86 and x64. Its first 600
moving-attachment/capsule frames have maximum stretch 1.00987 and minimum vertex
gap -0.0000205338 in fixture units (small numerical overlap, not exact zero).
It also checks a triangle floor, separation backstops and rejected negative time.
A tapered panel with moving lower anchors reaches maximum stretch 1.09009.
The source-bound capsule and folded material-metric fixtures pass on both builds.
These are numerical fixtures, not Wolverine gameplay results.

The final supported reference replay passes 7 coalesced requests spanning 5.453
seconds, taking 39.8901 solver seconds. Nine targeted body/contact/reaction,
root/collar/waist, lifecycle and strap tests pass. Source provenance verification
passes; the original Wolverine import remains unchanged.

The **retained CPU pouch experiment is rejected**: six of seven coalesced requests
fail contact/material checks. The structural-edge refinement took 14.8831 solver
seconds to advance 5.453 seconds. The last request has six body-intersecting faces
(one pouch, five hem, zero strap), maximum coarse stretch 1.14030 and rendered
stretch 1.13231. Earlier poses still exceed the 1.15 material limit. A fresh
retained-source receipt is `build/nvcloth-cmake/retained-final.csv/.err`.

The earlier checkpoint failed all seven requests with 75 body-intersecting faces
and rendered stretch 2.05319. Its 60 Hz-interpolated replay failed all 331 updates
and cost 52.1657 solver seconds. Timings include full body auditing and do not
establish a native frame rate. No revised 60 Hz replay has passed.

Additional rejected trials are preserved as private receipts: wider body-pinned
lower seams overconstrained the garment; identical simulation/render topology
reduced face crossings but stretched narrow lower edges excessively; area-based
particle masses worsened the tapered fixture and capture; a flat pattern with
lower-edge gathering still failed contact/material checks. None is retained as
an accepted fallback. Near-body hem rail projection did not fix the face-interior
crossings and was removed.

The upstream CPU path applies virtual collision samples to sphere/capsule
contacts, not triangle contacts (`SwCollision.cpp`). Actual body triangle
collision therefore does not certify rendered-face separation. Remaining work
includes full pouch/hem contact, stable pattern strain and collision cost.
Neither the fast proxy-only approach nor the complete-body experiment is playable.

Private development receipts are under `build/nvcloth-cmake/replay*.csv/.err`.
Earlier iterations that checked pouch proxies alone were subsequently rejected
when the complete body audit found intersections. Never cite those intermediate
proxy checks as a garment contact pass.

## Required before adoption

1. Finish fitting/contact/material/coverage checks on one stable implementation.
   Preserve the failed capture replay as a regression, plus broader sizes, states,
   angles and actual body resources. Validate complete rendered geometry and seams.
2. Extend lifecycle, initialization and morphology tests beyond the existing
   unit/frame and numerical backend fixtures. Finish measured
   reciprocal coupling or explicitly scope an accepted one-way prototype.
3. Pin a tested Base commit in Wolverine and add a reproducible optional CPU
   backend build. The old render late-latching lineage also needs review for free
   cloth versus body-bound trim. Do not just swap the worker type and claim parity.
4. Build a clean, sealed native derivative and run the repository-owned grey room
   workflow, preserving the accepted runtimes and exact rollback. Review native
   front/side/oblique motion captures and the mandatory pelvic attachment matrix.
   A campaign check is separate and still required.
5. Adopt the same pinned Base backend in Witcher's adapter only after Wolverine
   acceptance. Do not copy the solver into a second game-specific implementation.
