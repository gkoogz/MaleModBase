# Compact runtime release — 2026-09-29

Tag: `release-2026-09-29-compact-runtime`

This is a DLL update for the existing installed mod and assets. It preserves saved settings. It is a local release snapshot, not a published GitHub release.

## Changes

- Reduce the final anatomy render/collision skin from 61,378 to 35,000 triangles (43% fewer), and the rounded support triangles from 53,444 to 32,000. Remap deformation/normal/material bindings, pressure metadata, collar topology/seams and fluid triangle references together.
- Keep the required coarse motion cage and active diffusion support points; eliminate unused stored smoothing rows and the disabled clinical pipeline. No old high-resolution anatomy safety/render/collision fallback remains in-game.
- Retain previous surface optimizations and exact worker-pool collision skinning.
- Restore the joined collar's original opposite-to-render winding so its reconstructed lighting normals point outward.
- Strengthen upright suspension by tuning existing angular inertia, restoring torque and damping. No new support solver or extra iterations.
- Enlarge the existing rod/body contact envelope to cover the visible ventral bulge and tighten its contact compliance. This prevents the reproduced post-movement hanging overlap without new triangle collision queries or solver passes.

## Validation

- Reduction: 48 static control combinations, 270 motion frames, 480 pulse frames, exact welds/fixed exterior, normalized weights and finite geometry. Across 88 captured poses, retained UV/skin weights and physics trajectories matched baseline; minimum silhouette overlap 99.15%. A few near-degenerate micro-facet orientation differences are explicitly recorded.
- Shading correction: all 88 packed collar normal sets agree with their posed geometry (cosine >0.99); UVs/weights/physics preserved and maximum position drift below 0.000008 model units.
- J sequence and hitch regressions, serial/batched collision comparisons, BVH/barycentric anchors and fluid conservation/budget checks passed on the compact build.
- Latest support/contact build: six deliberately overturned mode/size cases have no inverted settled samples. Eighteen perched/launch/angle cases passed. Contact can legitimately tilt a support; minimum suspension alignment in the latest stress fixture was 0.567.
- Actual posed mesh intersection check: after identical settle/movement/settle input, the floppy fixture went from 37 triangle-pair crossings on 21 shaft faces to zero. All three tested modes now have zero shaft/pouch-body crossings. The check excludes shared root/neck faces and coplanar contact.

## Performance and limitations

The geometry-reduction benchmark against already optimized commit `2ea2b7b` measured CPU surface time of 26.276 → 22.031 ms at typical settings and 27.155 → 22.638 ms at large settings (16–17% lower). These are surface-stage measurements before the final support/contact tuning, not current whole-game FPS. Contact-active workloads can change with the corrected envelope even though the constraint/pass count stays the same.

The captures and neutral renders validate geometry, not the complete live game's materials or every character pose/control combination. After the collar winding fix, the user reported “Not working”; whether that meant persistent discoloration or a launch/load problem was not confirmed before the subsequent physics work. Treat that report as open. Do not infer that this release has a verified fix for every shading artifact or every possible intersection.

## Binary and installation

The packaged DLL matches the currently installed test build byte-for-byte:

`SHA256 FED0695979D6DA9D0C7B21466FF9E1F36009E496687E89A552AA443737C1B71D`

The previous installed DLL is backed up locally. This package contains only the replacement DLL, these notes and checksums; it requires the existing mod's textures, audio, settings and other assets. Copy the DLL into the existing game's `Binaries` folder while the game is closed. The package does not overwrite settings or supply a fresh full mod installation.
