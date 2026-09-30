# 2.0 Beta 1

Compact 35k runtime, remapped collider/deformation data, collar winding correction, reinforced existing support/contact parameters and retained worker-pool optimizations. Complete offline installation/upgrade/rollback package. See releases/2.0.0-beta.1.md for evidence and known limitations.

## 1.9.0 — shared anatomy and CPU performance

- Bounded parallel skin evaluation, full-precision SIMD, cached normal decoding and bending coefficients reduce CPU cost while retaining the approved geometry and solver quality.
- Includes tank/gameplay body and anatomy harmonization, captured tank material-pass corrections, shared normal/specular maps, the approved upright flared neck and structural support model.
- Includes the recent directional surface-contact rendering, body-contact coverage and density improvements.
- Adds separate surface-stage telemetry and versioned installation/rollback with settings preservation. See docs/VALIDATION-1.9.md for measured results and live-verification limits.

# 1.8.0 — 2026-09-28

- Prebuild lossless mipmapped skin textures to eliminate PNG decode/filter work at first draw.
- Batch HUD submissions, skip absent-character loading work and prevent duplicate overlay updates.
- Isolate saved cameras from diagnostics; bound session logging and remove automatic screenshot capture.
- Restrict camera overrides and attract-movie blocking to their intended scene/content.
- Gate inactive fluid capture and retain collision topology across animated vertex updates.
- Include the complete 1.7 menu anatomy, world-space contacts and menu-return fixes.

# 1.7.0 — 2026-09-28

- Bundle the approved menu anatomy, body retarget, pelvic weld, necklace contact, camera package and blend textures.
- Keep gameplay fluid and floor marks in world coordinates using verified baked-geometry camera origin and PhysX floor scale.
- Reset scene-owned resources when returning from gameplay to the main menu.
- Include body contacts, temporary splats, current fluid defaults and saved menu camera poses.
- Preserve existing settings and private audio; add WStart installation and exact rollback coverage.

ï»¿# Release 1.6.0 — 2026-09-26

Each main pulse now samples an independent volume and feed duration for every teaching sequence. A deterministic seed within the run keeps the pose and emission variation aligned; each new run gets a fresh seed. Feed windows that overlap are grouped into a shared continuous thread, with smooth ramped flow through the handoff. Per-throb angle intensity is varied independently. The default duration variation guarantees one extended pump when the configured range can cross the 1.5-second pulse spacing. All controls are editable in `Binaries/TeachingFluid.ini`; setting a variation to zero disables it.

Validation adds seeded-repeatability, bounds, zero-variance, overlap-continuity, and angle-peak regression coverage. Clean-install/upgrade/rollback checks and camera/render tests remain in the release suite. See `docs/TEACHING-FLUID-1.6-VALIDATION.md`. No user study audio or rejected pec sculpt work is included.

---
# Release 1.5.0 — 2026-09-26

- Keep the released v1.4 body, character package, textures, and supported installer path; exclude rejected pec modeling experiments.
- Replace the particle-like tracer effect with continuous white viscous streams and an attached clear preliminary strand.
- Fix camera-motion smearing by keeping simulation state in component space and applying camera transforms only for drawing.
- Add optional random local WAV cues at 2.5 and 7 seconds; user study audio remains outside redistributable files.
- Preserve saved game settings and existing user `TeachingFluid.ini`; install the sample only when absent.
- Document limitations: visual fluid approximation, no full terrain/body collision or splash solver, and no live FPS claim.
# Release 1.4.0 — 2026-09-26

- Packages the existing installed teaching tracer demonstration and visibility update.
- Includes the existing relaxed suspension adjustment.
- Freezes the installed runtime; no further model or animation changes for this release.
- Retains prototype limitations documented in README.md and docs/TEACHING-FLUID-POC.md.
## 1.3

Packages the installed prepared-shape/geometry performance improvements, baked refinement and attachment bindings, slight lateral suspension relaxation, and large-diameter pelvic ramp correction. Retains 240 Hz physics and full-rate surfaces. Adds version-specific 1.3 installer/upgrade/rollback. Existing neighboring folds remain; recorded replay results do not establish live game FPS.

## 1.2

Packages the verified September 25 coupled-contact solver update from f8d354f: static/sliding friction, damped suspension and step-interpolated dimensions. Restores the exact DLL requested by the user; later geometry candidates are excluded. Retains the documented tight-pose fold limitations. Adds a version-specific verified installer and rollback.

## 1.0 Beta 1

Packages the currently installed runtime and 22-clip idle chatter pool. Adds an immediate in-game Idle Chatter toggle, four throb modes, and reordered controls with F6 guidance. The installer verifies and backs up all audio files. The underside ridge issue remains open.

## v0.9

Packages the current R36-based attachment and surface work, oblique resting shape, tauter central web, bounded shaft motion, animated chest necklace contact, prior collision asset adjustments, and state-specific diffuse texture variants.

The installer supports original and existing Revision 161 Natural packages, verifies all payloads, preserves settings, and restores previous files on rollback. Clean-install and upgrade fixtures passed.

Known issues: the reported underside indentation is not fixed; extreme-pose contact remains imperfect; current live visual validation is incomplete.
## 0.7.1 junction R2

Final-pose scrotal junction fairing, broader upper attachment, adjacent ventral-row
support and triangle redistribution. Original topology/package retained. Eleven
matched captures, compiled-DLL smoke and both upgrade/rollback paths tested.
Default/maximum sampled neck intersections removed; extreme-angle contacts remain.
Live-game playtest pending.

## 0.7.1 � Pelvic ramp

Final-pose constrained body/collar fairing, angular-profile radial support,
tangential vertex redistribution and bounded, orientation-checked correction.
Existing package, connectivity, UVs, rig and physical solver are preserved.
Added verified 0.7 upgrade/rollback. Both installer paths and ten matched geometry
cases passed isolated checks. Live-game visual playtest remains pending.

# Changelog

## v0.7

- Promote the installed, hash-verified runtime from artifact `v0.6t61t62t63t65t67t68t69t70`.
- Rebuild the WBX1 package delta against the exact installed Revision 161 package.
- Preserve the transactional installer, settings, checkpoint handling, and reversible backups under a version-specific v0.7 path.
- Publish neutral postgraduate medical-education documentation while retaining the runtime provenance identifier for reproducibility.

## v0.6

- Package the exact restored Revision 161 character and runtime.
- Include its rounded collar topology while preserving Weapon X handling, live controls, and physics.
- Retain the verified installer/uninstaller workflow with version-specific backups.
- Known issue: collar shimmer remains at some sizes and viewing angles.
- Exclude the rejected Revision 162 and 163 experiments.

## v0.5

- Packaged the Weapon X-safe character build with Revision 157 runtime behavior.
- Included remodeled body and anatomy, live morph controls, physics, collision constraints, and persistent settings.
- Renamed the overlay to a concise anatomy-tool label with explicit F6 show/hide text.
- Moved Reset All from Home to F8.
- Added transactional one-click installation and verified one-click uninstallation.
