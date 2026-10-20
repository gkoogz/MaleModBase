# Collar coordination investigation, October 13

The request remains a complete collar repair, matched to the measured shaft
diameter, preserving surrounding tissue, with actual true-side previews before
installation. No candidate in this investigation has passed that acceptance.
The normal game installation and dependency pin have not been changed.

## Causal evidence

The private stage trace follows target vertex 12994 back to cage vertex 2335.
The source collar diffusion lowers that upper sample by about 2.2 source units;
the adapter subsequently lowers it by approximately another 3.3. Source angle
ownership leaves the proximal material behind while the shaft rises. The
adapter's seam-distance blend expands with radius and suppresses more of the
proximal shaft's displacement at large sizes. These are contributing stages;
the separate torso/legs alone do not establish a waist-constraint cause.

The rejected radial-flow diagnostic uses the already-deformed radial direction.
Some originally dorsal rows are below the posed axis and get pushed further
down. Its red-outlined hooked cleft is not an acceptable replacement.

## Current opt-in code state

The existing STOCK_ANCHOR_PREVIEW switch is temporarily reused for an isolated
coordination experiment. It is OFF by default and its name is now misleading:
the latest experiment retains the original source pivot and adapter root law.
Do not promote or describe it as a measured-anchor implementation.

* Session targetOwnsCollar currently releases dorsal angle following and blends
  dorsal cage rows back toward their pre-diffusion positions. Its newest trial
  classifies the source arc from `morph_base` relative to the authored root,
  rather than `graftBaseNormals`. Original lower ramp, raphe and final unified
  solver remain active. This is still an unaccepted experiment.
* The adapter blends between its original body-following displacement and the
  posed shaft displacement using immutable rest-normal upper-arc weights.
* Angle 50 source and both native LOD buffers are byte-identical to the baseline.
* Angle 15 and 25 true-side renders remove the deep upper notch but deform the
  lower junction and retain creases. They are rejected, not a finished repair.

The earlier ArcAnteriorLimit body-forward budget was reverted to zero after
its body bulge/cleft failure. Native game testing has not resumed for these
known-failing candidates.

## Private reproducible evidence

Root: `D:/MaleModBuilds/witcher-anchor-native-20261013`.

* `capture.py`: runs the actual source worker, target fitter and native buffer
  composer with calibrated contacts. It uses fresh output directories.
* `render.py`: actual captured LOD0, orthographic true side, neutral clay.
  These are offline buffer renders, not native grey-room captures.
* `anchor-coherent-*`: full angle ownership/no second adapter blend; upper gap
  closes but lower surface distorts.
* `anchor-dorsal-*`: material upper-arc release with measured anchor; rejected.
* `anchor-fixedroot-*`: original anchor/body field with dorsal release;
  angle 50 parity verified; angle 15/25 still visually rejected.
* `neck.py`, `arap.py`, `perimeter.py`: isolated reconstruction comparisons.
  Each retained defects; perimeter ray hits were incomplete on the lower arc.
* `orientation-qp.py` and `quality-qp.py`: ongoing constrained corotational
  experiments against the original captured mesh. Original-edge elimination,
  fixed exterior, gradual section support, intrinsic area and adjacent-normal
  constraints are simultaneous. The second adds a stricter dihedral bound.
  Check their actual report JSON before claiming convergence or quality.

## Further October 13 trials: still no accepted replacement

The 63-case `fixedroot-matrix` completed both LODs, with finite positions and
exact donor seams. That is numerical export evidence only. The newer source
material-position trial has a separate `material-matrix`; never combine its
worker identity with the earlier matrix. A fresh `original-matrix` uses the
baseline worker/probe, whose angle-15 source and both LOD buffers were verified
byte-identical to the immutable original capture before the sweep.

* `cage-lineage{0,1}.npz` maps original 2388 source cage controls through R14,
  rounded support, neck refinement and calibrated target module bindings.
* `cage-qp.py` reached approximately 0.289 of its requested upper support;
  `cage-fast.py` reached approximately 0.348 before being stopped after repeated
  iteration limits. Neither completed the clearance constraint. Their
  checkpoints are diagnostics, not deployable outputs.
* `reference-arap.py` and `reference-cage.py` deform from the continuous
  angle-50 surface to the angle-15 distal boundary. They close the upper cleft
  but crease the lower junction. The cage variant produced 43/41 new fold pairs
  against its angle-50 reference, with no collapsed faces. Both are rejected.
* `union-project.py`, `height-material.py` and `height-disk.py` are rejected
  projection/chart studies: many folded triangles and visible surface damage.
  Do not resume them as successful collar models.
* `anchor-material-*` uses the current source worker with material positions
  for the source release. Angle 15 still has 60/71 new fold pairs, and lobe-core
  displacement up to 0.872 source units. Angle 25 has 74/70 new fold pairs.
  Angle 50 remains exact. See `material-audit.json`.
* `layer.py` isolates the candidate's upper correction over the original output
  using immutable angle-50 radial coordinates. Its angle-15 side view preserves
  the lower outline better but still has 34/51 new fold pairs. Angle 25 retains
  a visible ridge and 35/42 new fold pairs. It is an offline composition, not a
  runtime feature. `layer-repair.py` did not fix all folds; even its successful
  LOD1 signed-normal solve retained 41 new fold pairs. Solver success is not a
  bending pass. `layer-cosine.py` adds actual adjacent-normal constraints; read
  its report before making any claim about completion.

The source normal is not an anatomical ownership coordinate. For example,
source cage row 552 is at approximately (12.732, -0.049, 80.155), below the
authored root, yet its normal has positive Z. However, a horizontal rest-root
plane is also only an approximation for the curved distal reference centerline.
Changing the source mask alone does not isolate its downstream rest-frame and
physics effects; the measured lobe-core motion demonstrates that limitation.

No new native grey-room run, normal installation, pin update, or accepted
attachment gate resulted from these experiments. The latest private worker and
probe differ from the older DLL in the preview build directory.

## End-of-run evidence and current source

`layer-path.py` applies the upper layer in 20 increments, repairing local
signed areas and actual adjacent-normal cosines in exact seam-donor space.
LOD0 completed the entire path at maximum size, state 0, angles 15 and 25:
zero new fold pairs and zero collapsed faces against the original pose. Actual
orthographic renders are `layer-path-side-15-25.png` and
`layer-path-front-15.png`. They are offline authoring studies, not live runtime
corrections. Angle 25 still has a ridge; neither constitutes full acceptance.

LOD1 angle 15 failed at step 6. Its `fit1.npz` contains only step 5 (25 percent
of the deformation). Its zero-fold count MUST NOT be read as a complete pass.
LOD1 angle 25 was stopped during step 13 after completing step 12; no final
candidate was saved. An adaptive angle-15 LOD1 retry was also stopped without
a completed result. No numerical jobs remain running.

Both `material-matrix` and `original-matrix` completed 63 cases / 126 LOD
buffers. The material matrix has finite outputs and maximum seam error
4.26424e-14 source units. Its audit explicitly records `visualPassed=false` and
`nativeObserved=false`. This matrix does NOT test the later layer-path authoring
correction across those cases.

An independent body-field trace exposed another contributor: the final graft
solve allows body masters within four source units of the seam to move again.
Its output can substantially contract the opening authored by the body pass.
The private `anchor-bodyheld` experiment prescribed those projected body
positions exactly. LOD0 maximum-size angle 15 then had opening width 18.393,
but 103 new fold pairs and one collapsed face; the underside was visibly bad.
Its body signed-area minimum remained approximately .02 and donor seams were
exact. The final redundant orientation projection initially rejected tiny
residual differences at the .02 target when all body coordinates were fixed;
removing that redundant projection did not make the geometry acceptable.

`anchor-sized` tested a section scale of 1, zero extra lateral thickness,
inferior thickness .14 and forward gain .35. It retained a lower opening/crease
and is rejected. The adapter's body-prescription/profile changes were restored
from `target-before-body-prescribed.cpp`, and the probe was force-recompiled
after restoration. The rejected source is preserved privately as
`target-bodyheld-sized-rejected.cpp`.

The shared `RootTransitionProfile` configuration remains source-only, with
historical defaults unchanged. New unit cases check exact lateral/upper radius,
inferior allowance, neutral identity and rejection of a constricted profile.
The standalone MSVC test passes. The existing `build/garment-cmake` configure
was blocked by an unrelated motion-filter extraction mismatch, so this is not
a full-repository build pass. The 21 targeted Python attachment tests also pass.
No adapter currently adopts the custom profile. Adoption in Witcher, Wolverine
or other spokes still requires an explicit pin and each spoke's complete gate.

Current private worker: material-position source mask. Current private probe:
original body policy plus the earlier normal-based adapter release. Neither is
the offline layer-path model. Do not install these binaries as that model.

Earlier private fits (`material`, `fillet`, `complete`, `virgin`, `height`,
`flow`, and V6/V7) were also rejected. A solver convergence result is not an
attachment pass. No current DLL was built from the latest source; the probe
and private worker were built. Do not confuse the older DLL in that build
directory with the latest candidate.

## Shared fitter changes pending validation/adoption

`PlanarHalfspaceProjector` handles unbounded coplanar support intersections.
The Python constrained fitter now projects actual visible seam samples through
the donor binding, rather than skipping dependent rows. It adapts ADMM penalty
and checks actual clearance. Twenty-one targeted Python tests passed before
this continuation; these changes are not yet a runtime collar solution.

Keep shared numerical changes in Base. Any accepted runtime requires an
intentional pinned adapter adoption, exact seam/UV aliases, both body resources
and LODs, scale/state/angle/motion coverage, native front/side/oblique review and
campaign evidence. Wolverine remains unchanged until separately adopted.
