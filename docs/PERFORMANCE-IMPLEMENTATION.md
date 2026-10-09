# Full-rate garment optimization and 32 rays - October 8, 2026

The user requested implementation of measured performance improvements, then
explicitly requested **32 rays** (correcting an initial 30). This supersedes the
earlier resolution-preservation requirement for this specific change. It does
not authorize 30 Hz solving. The active recipe previously used 80 angular
columns, derived from 40 authored guides with subdivision. Both recipes retain
24 longitudinal samples. The new pouch has 769 vertices / 1,504 triangles,
versus 1,921 / 3,760. Complete garment rendering becomes 2,471 vertices / 4,816
triangles, versus 3,623 / 7,072. Existing waistband, glute straps, hem ribbons,
materials, anatomy masking and pelvic collar are retained.

## Shared numerical changes

- Reuse the three current signed distances within a clearance plane test.
- Cache a fallback triangle's last separating plane. Re-evaluate that plane
  against all three current vertices every pass; a failed hint falls through
  to the original complete scan. No stale collision result is reused.
- Cache seam escape routes, slopes and square roots within one FitSeam call.
  Directions and plane normals remain invariant within that solve. The cache
  never survives a pose change.
- Lazily cache material-space transport weights until Remember/Reset changes
  the reference. Preserve float summation order and exact boundary transport.

The render-rate solve, anatomy 240 Hz integrator, 24 constraint iterations,
solver budgets, collision solids and lighting are unchanged. No Rust rewrite,
new thread pool, occlusion policy or approximate arithmetic was introduced.

## Measured exact-output comparison

Private evidence: `build/performance-20261008/paired2/comparison.json`.
Three alternating pairs replay 44 private poses ten times each, after warmup.
The bank combines 24 historical successful poses with 20 newly captured poses,
including six that require fallback. Both builds report 66 fallback calls per
run including warmup, and zero outer failures. All six output streams, including
positions, normals, UVs and vertex attributes, are byte identical:
`768fee7479532e012183a3d085220f6783f65558029e8fa5b5596a347ee393e6`.

| CPU phase | Previous ms | Optimized ms |
| --- | ---: | ---: |
| Whole post-skinning cloth block | 14.9348 | 12.4194 |
| Triangle clearance | 9.8762 | 8.8330 |
| Fallback repair/transport | 30.0996 | 20.7264 |
| Seam fitting | 1.0735 | 0.8002 |

This is about 16.8% less cloth CPU work and 31.1% less fallback time with the
original 80-ray topology. Phase means have different sample counts; do not add
them. These are CPU replay results, excluding skinning, GPU and the rest of the
game. They are not FPS gains. The 32-ray topology is a separate representation
change and is not claimed byte identical to the 80-ray mesh.

## 32-ray authoring and portability

`tools/author_meridian_lod.py --rays 32 --rows 24` recomputes the existing
curvature-driven guide distribution against the measured inspection geometry.
The first coarse mesh correctly rejected a panel crossing near its pinned hem.
`tools/fit_meridian_seam.cpp` now exposes the same SDK-free C++ seam fit used by
adapters for authoring. Supply its executable through `--seam-fitter`. The
maximum fitted anchor adjustment was 0.157885 source units; no metric units are
assumed. The resulting whole-triangle minimum separation is +0.0002437 source
units. The boundary remains exactly pinned to that fitted seam.

Wolverine's `tools/author_meridian_runtime.py --columns 32` regenerates geometry,
donors, aliases, trim followers, support directions and row heights together.
Contract layout remains revision 5 with explicit counts and hashes. Regenerating
the old 80-ray input preserves every numerical table; only the position of the
band-constant declaration differs. The runtime retains ten boundary transport
anchors independently of ray count. Adaptive origin redistribution is still an
authoring operation; per-frame redistribution remains unimplemented.

Base owns the numerical optimizations and configurable authoring. Each spoke
must consume a pinned tested Base revision and author its own observed donor
recipe. Wolverine's private builder records the old accepted source/base pins
plus exact named overlays. Normal adapter pins are unchanged pending commit and
adoption. Witcher's controller, worker transport and installed files are not
modified by this work; these helpers are available for its future adoption.

## Build and verification evidence

Private build root: `build/performance-20261008`.

- `candidate2`: optimized 80-ray DLL
  `a9591974f7293cef69344b1e3c795692116b4cdb222692f8ca916ea3f487c1d3`.
- `candidate32`: 32-ray DLL
  `01446089bdede19b0d01d925ffccd385f9e8ddab65d9909e6446308b84689f42`.
- Both export Wolverine `16affd49da5a122df776969574ca62a984b59425` and
  Base `99ff741ea95f18ed84526c35a6c3f38a37d857b6`, then apply hashed overlays.
- Shared x86 optimized runtime/clearance/continuity tests passed. The cache
  regression covers moving obstacles, reordered planes and topology changes.
- 22 Python collision/guide/cloth tests passed, including a 32-ray boundary test.
- Base provenance verification passed: 516 imports and 185 source arrays.
- The 80-ray candidate completed 18 native captured cases without draw rejection.
  That is a sampled regression result, not the complete attachment gate.

### System crash during the first 32-ray native sweep

Windows recorded MEMORY_MANAGEMENT 0x1A / subtype 0x41792 (corrupted PTE).
The first sweep saved 16 cases before a state-change interruption; it did not
finish. Cause is not established. The memory dump remains local at
`C:/Windows/MEMORY.DMP`; it was not uploaded. The incident records and exact log
are private in `build/performance-20261008/incident`.

The user then explicitly instructed continuation. The same candidate was
restarted in a bounded owned room with raw-pose capture disabled. A subsequent
successful run cannot establish the cause or dismiss the earlier system crash.
Retain this incident in the handoff and do not characterize it as a proven GPU,
RAM, driver or mod fault. The playable prototype and retail DLLs were verified
unchanged immediately after the crash.

## Acceptance boundary

Complete native results and installation identity are recorded in the current
handoff and private `native-evidence.json`. The full attachment gate remains
incomplete: campaign transitions, every resource/LOD and all shape combinations
are not covered. Extreme deformation and uncertified fallback contact predate
this optimization and remain limitations. The normal retail and accepted human
sandbox must not be promoted from these sampled developer results.

Wolverine now owns `compare_meridian_profiles.py` for paired exact-output
measurement, optional bounded raw capture, and `Update-MeridianPrototype.ps1`
for hash-checked developer-only update with settings preservation and rollback.
Private captures, game assets, compiled outputs and crash data remain outside Git.

## Final native run and installation

The resumed run `room32/runs/20261008-153005-e44cbd` completed all 18 sampled
cases with zero reported draw rejection and exit code 0. Ten captures were
inspected, including front, side, oblique, saved Overall 95 / Length 100, extrema
and post-jump posture. The naked side/front/oblique captures showed continuous
attachment at the inspected default settings. No full attachment pass is claimed.
Last cumulative sample: 601 garment attempts / 601 draws; 439 wraps, 161
transported and 161 uncertified. Garment Draw mean 5.3001 ms, max 43.041 ms.
The native runs differ in pose and workload; this is not an apples-to-apples FPS
comparison with the earlier 80-ray run.

`Update-MeridianPrototype.ps1` validated, installed and hash-verified candidate32
into `%LOCALAPPDATA%/MaleMod/MeridianPrototype`. The existing desktop shortcut is
unchanged. WolverineLive.ini and TeachingFluid.ini hashes were preserved. The
prior DLL and manifests remain in `rollback-performance-20261008-153721`; its
restoration script passed ValidateOnly. The normal retail DLL remains
`4f4900a5e70ce9bb127b46c3d7fe57ef74ffeace8a5ae704d3da1eff6e2ba9a3`.
No commit, push or release was performed. Incomplete full attachment acceptance
and unresolved system-crash causation remain explicitly recorded.

## October 8 material follow-up

The subsequent Wolverine lighting repair retains this shared 32-ray/full-rate
implementation. It reuses skinned/wrapped/uploaded geometry across native light
passes and caches shaders for the light capabilities actually present. Engine
state, depth alpha, texture pool and tank HDR changes are adapter responsibilities;
see Wolverine docs/NATIVE-MATERIAL-REPAIR.md and Base HANDOFF for exact installed
identity, sampled evidence, rollback and the incomplete attachment gate. The
native CPU timings are not a measured game-FPS improvement.

## Local walking fidelity follow-up

The installed 25dc0842 developer runtime adds 48 virtual walk intervals around
the two testicle ovoids and redistributes the existing rows by current turning
angle. Rays 32, rows 24, triangle count and full-rate cadence are unchanged.
The 16-pose CPU mean rises from 5.63073 to 5.78195 ms; no FPS claim.
See MERIDIAN-CURVATURE.md and the newest HANDOFF for sampled native evidence,
exact rollback and the still-incomplete attachment/campaign gate.
