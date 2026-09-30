# Compact runtime mesh

The render skin and anatomy fluid collider both use **35,000 triangles / 17,528 vertices**. The supporting rounded surface uses **32,000 triangles**, including its placement and structural-skin safety tests. There is no 61,378-face runtime skin, safety copy, collision fallback, or runtime decimator.

The original coarse R14 cage remains the source for motion and authored frames. The supporting surface retains 25,597 points: pressure-field support and interpolation donors are required even when they are not corners of the reduced supporting triangles. Pruning these independently changed diffusion lengths and folded bound surfaces. Retaining these points preserves the existing field while reducing the triangulated surfaces, final vertex stream, final interpolation bindings, and joined collar system.

The joined collar domain contains **73,690 faces**: the 35,000-face anatomy plus 38,690 unchanged body faces. It is not another high-resolution anatomy mesh.

## Runtime changes

- Final indices, packed attributes, sparse deformation bindings, direct copies and material/normal bindings are compacted together. Retained UV coordinates and skin weights are unchanged.
- Fine frame bindings, rounded indices, triangle-parent metadata and placement-safety lists use the reduced supporting surface.
- Unified collar nodes, packed aliases, body joins and seam constraints are remapped. A removed UV representative is replaced by its surviving alias of the same joined node.
- The pressure/diffusion field keeps its active stencils and pass count. Only active smoothing rows are stored: **160,386 → 41,795 neighbor entries**. All 88 complete captures are bit-identical before/after this storage pruning.
- Fluid triangle IDs are generated from the same final index buffer that renders the model. BVH refits and barycentric anchors therefore reference the compact mesh directly.
- The disabled clinical collar/reference pipeline and its stale tables are removed. Earlier packed fairing, discarded-lighting pruning and worker-pool collision skinning optimizations remain included.

## Authoring

`baseline/*.h.gz` are immutable offline inputs from commit `2ea2b7b`; their hashes are recorded in `baseline/manifest.json`. They are never included or opened by the game. `candidate.npz` and `remaps.npz` are offline topology/lineage records.

Reduction uses endpoint quadrics, manifold link checks, protected body/UV boundaries and feature neighborhoods, UV/skin-weight compatibility, a 0.70-unit cluster bound, and normals checked against 88 posed fixtures. The two safety-lock JSON files preserve returning strips identified by the complete runtime audit. Redundant fine donors are substituted through local affine frames only when their fixture error is small; unsuitable donors remain in the compact supporting point cage.

From the repository root, with Python, NumPy and SciPy:

```powershell
python tools/remesh/compact_runtime.py generate
python tools/remesh/compact_runtime.py apply
```

Generation requires the 88 baseline captures in `captures/surface-audit-revised`. Applying the checked-in candidate also requires these fixtures to validate/rebake donor transport. Output headers are deterministic; no generation step occurs in-game.

For geometry review, build `tools/remesh/raster.dll` with `build_raster.cmd` from that directory. Capture the compiled model using `tools/neck/unified-fast.exe` with `SURFACE_AUDIT_CAPTURE_DIR` pointing to `captures/surface-audit-compact`, then run:

```powershell
python tools/remesh/audit_compact.py
python tools/remesh/render_review.py
```

The renderer uses real packed runtime positions with matched orthographic cameras and geometric shading. Translucent pelvis context exposes the join. These are geometry captures, not screenshots of the game's materials.

## Verification and limits

- 48 static control combinations, 270 motion frames and 480 pulse frames: finite geometry, normalized weights, exact seam constraints, unchanged fixed exterior and no repeated collar factor builds.
- 88 compared captures: bit-identical physics trajectories and retained UV/skin-weight data; at least **99.15% silhouette overlap** across five views. Sampled surface-distance P99 is at most **0.256 model units**; supporting-point displacement is at most **0.0275 units**.
- No normal-direction reversals with meaningful area. At most five nearly collinear root facets change sign in a pose; each has area below 0.005 square model units before and after reduction, and the largest resulting facet is 0.00296. These are reported explicitly, rather than claiming mathematical identity. Full texture/material rendering and actual game FPS have not been tested.
- Compact collider: 12 exact serial/batched comparisons, valid triangle/source IDs, 12 BVH refits, 2,436 moving barycentric anchors and body sweeps.
- Sequence: 600 frames to completion, two collar builds; 350–800 ms hitch checks pass.
- Production fluid budget: complete mass/contact/splat conservation at 15/30/60/120 FPS; deferred crossing recovery and 512 concurrent deposits pass.

`performance-audit.json` contains three alternating baseline/compact pairs per setting. It measures the CPU surface stage, not total game frame time or FPS. `geometry-audit.json` includes every pose, source displacement, silhouette score, sampled surface distance and near-degenerate normal change.

The reduced DLL and subsequent shading/support/contact fixes are built and installed for testing. See the repository release notes for the current binary hash and validation limits. Earlier installation status in the work log is historical.
