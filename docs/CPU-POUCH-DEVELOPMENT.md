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
checks, but took 41.50 seconds for 5.453 seconds of motion. That is unsuitable as
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
build/nvcloth-cmake/Release/replay_cpu_pouch.exe PRIVATE_RECORDING_DIRECTORY 60
```

Recordings contain `reference.input`, `prepared.input`, and contiguous
`request-1.input` onward. They remain private and must not be committed. The replay
returns nonzero for failed contact/material checks and exports OBJ/layout files
beside the private inputs. `body_intersections` audits the complete actual body
mesh; pouch proxy clearance alone is insufficient. Fine rendered mesh strain is
measured separately from simulation edges against the authored eased pattern.
Complete anatomy/trim, volume membership and coverage auditing remain incomplete.

The final generic CPU cloth test passes on MSVC x86 and x64. Its first 600
moving-attachment/capsule frames have maximum stretch 1.00893 and minimum vertex
gap -0.0000263875 in fixture units (small numerical overlap, not exact zero).
It also checks a triangle floor, separation backstops and rejected negative time.
The source-bound capsule enclosure/frame/unit/ray fixture passes on both builds.
These are numerical fixtures, not Wolverine gameplay results.

The final supported reference replay passes 7 coalesced requests spanning 5.453
seconds, taking 39.8901 solver seconds. Nine targeted body/contact/reaction,
root/collar/waist, lifecycle and strap tests pass. Source provenance verification
passes; the original Wolverine import remains unchanged.

The **final CPU pouch experiment is rejected**: its same 7 coalesced requests all
fail contact/material budgets, taking 16.0879 solver seconds. The last request has
75 body-intersecting garment faces (1 pouch, 71 strap, 3 hem), maximum coarse
stretch 1.29497 and maximum rendered stretch 2.05319. The preceding 60 Hz-interpolated
capture replay failed all 331 updates and cost 52.1657 solver seconds. These
timings include full body auditing and do not establish a native frame rate.
Body wrapping, lower connectors/hems, collision workload and strain remain
unresolved. Neither the fast proxy-only approach nor the slower complete-body
approach is playable.

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
