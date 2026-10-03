# Stable pelvic recruitment frame (algorithm revision 2)

## Purpose and source finding

The shaft must follow its commanded rest angle and simulated motion. The
recruited pelvic annulus must keep its attachment orientation relative to the
character's pelvis. This is shared geometry behavior and belongs in Base.

The immutable reference `UnifiedCollar::Apply` used `SampleShaftChain(0)` for
both its metric rebuild and each radial target. That sample contains live
pitch/yaw and the rest angle command. The cached metric and per-frame target
could therefore disagree, and large angle changes rotated the recruited torso
into a forward platform. The target-character query path repeated that moving
coordinate frame.

Revision 2 uses `include/malemod/surface/pelvic_frame.hpp` for an explicitly
measured `RecruitmentFrame { root, axis, lateral, up }`. The observed reference
wrapper takes `ShaftRoot()` and the fixed `neutralShape[4]` attachment direction
(30 degrees in the reference data). It supplies observed +Y as the lateral
axis. No game skeleton names, inferred meters or graphics APIs enter Base.
The actual shaft guide, rest-angle mapping, dynamics and ventral guide contact
remain live. This is not a clamp on the shaft's controls.

The frame is the neutral collar orientation, not the body's vertical axis.
Garments use a separate measured body frame. In the reference frame +X is
forward, +Y lateral and +Z body up. A garment API that requires
`lateral cross forward = up` must use -Y as its lateral direction.

## Seam publication correction

The solve eliminates each fine seam vertex onto its original edge donors.
Previously its packed aliases were published only if a donor was free. A
fixed donor pair can still require a correction of the incoming seam slave.
Changing the recruitment mask exposed an output gap of 0.074 source units.

Revision 2 always publishes those eliminated seam slaves and their existing
UV aliases. The donor constraints, topology, factorization and source lineage
are unchanged. The 31-case final sweep has maximum rendered original-edge
residual below 8.80e-6 source units.

## Wolverine adoption

The immutable snapshot is never edited. The canonical Wolverine adapter must
include `<malemod/surface/pelvic_frame.hpp>` through its pinned Base include
path, then add this wrapper after its measured `ShaftRoot()` declaration:

```cpp
static ::malemod::collar::RecruitmentFrame StablePelvicRecruitmentFrame(){
 const auto root=ShaftRoot();const float angle=neutralShape[4]*3.1415926535f/180.f;
 return ::malemod::collar::StableRecruitmentFrame(
  {root.x,root.y,root.z},{cosf(angle),0.f,-sinf(angle)},{0,1,0});
}
```

Replace the beginning of `UnifiedCollar::Apply` with exactly the extracted
Base arithmetic:

```cpp
Topology();Read(body);const auto pelvic=StablePelvicRecruitmentFrame();
V3 root{pelvic.root.x,pelvic.root.y,pelvic.root.z},
 axis{pelvic.axis.x,pelvic.axis.y,pelvic.axis.z},
 up{pelvic.up.x,pelvic.up.y,pelvic.up.z};
float radius=logicalShaftBodyRadius,length=0;V3 previous=root;
```

In `Build`, replace the conditional seam write mask assignment with:

```cpp
write[ucSeamVertices[k*3]]=true;
```

Keep the existing loop and all subsequent alias/normal-face publication.
The Base extractor applies the same guarded source-span replacements to its
generated worker; target query evaluation uses the same stable frame.
Game resource uploads remain the adapter's responsibility.

## Witcher and future spokes

Pin the new tested Base commit for the numerical worker and rebuild it.
Do not independently reproduce the new coordinate frame in WitcherScript or
native graphics code. `Output.collarMetric` and source query displacements now
use the stable support orientation; actual shaft guide samples still carry
the moving anatomy. Target body opening, torso/lower boundary constraints,
native skin palettes and root/world transforms remain calibrated in the spoke.
Rebuild all coupled binding support fields when the adapter's binding recipe
depends on the collar metric. Preserve both sides of every resource boundary.

The collar itself does not change the wire layout. The concurrent optional
garment-load input advances the worker transport to version 5. This is a
behavior revision: workers,
binding packages and adapter adoption must be versioned and verified together.

## Verification and limits

- `pelvic_frame_test`: measured frame orthogonality, rigid-transform covariance,
  scale normalization, and rejection of zero/parallel/nonfinite inputs.
- `surface_process_test`: lifetime isolation, metric lifecycle, wire roundtrip,
  live pitch/yaw, rest-angle endpoints with stable axis/up, finite target queries.
- `tools/test_collar_angle_adoption.py`: 31 separate worker processes, 120 steps
  each, states 0/1/2, requested large shape and combined maxima, angles
  1/10/50/64/100, finite output, exact topology, original-edge seams, and area
  collapse checks. Angle 1 is the supported low endpoint; no range is extended.

The requested shape is Overall 100, Length 75, Width 100, Glans 52, Scrotum 100,
Hang 95; other controls are 50. At state 2, changing angle 1 to 100 still moves
the shaft root direction through about 160 degrees, while the two body's
maximum angle-dependent displacements decrease from 6.63/10.07 to 6.05/6.50
source units. Inner contact and earlier morphology remain angle-dependent;
body positions are intentionally not claimed to be invariant.

No previously nondegenerate face collapses in this sweep. The smallest area
ratio relative to the default is 0.008382 at an extreme; this is an area gate,
not proof of absence of self-intersection. That minimum belongs to an anatomy
crease, not the recruited body. Body-only faces have minimum area ratio
0.096862 across all 31 cases. An offline sagittal slice of the
requested state-2 shape shows the former upper torso kink removed at angles
10 and 64. These tests do not establish native game rendering or live appearance.

Old source parity records remain historical evidence of revision 1. Revision 2
is intentionally different: at the all-50 default after 120 steps, anatomy
changes by at most 0.078877 source units and body sections by 0.078877/1.026880.
Mechanical guides, rest guide, lobe mechanics and controls remain unchanged.
The difference arises because the earlier final collar followed simulated
root pitch even at default. Do not relabel the old oracle as a new parity pass.

Reproduce the numerical gate with an independently built pre-revision-2
reference executable (the tested earlier Base pin was
`2a7b38a7019d6102046085a8586e86273d5579fe`). Preserve that executable separately
when rebuilding the current worker:

```powershell
python tools/test_collar_angle_adoption.py --runtime build/collar-angle-cmake/Release/surface_runtime_cli.exe --previous-runtime <pre-revision-2-executable> --output build/collar-angle-agent/cases
```

Build outputs, captured meshes and diagnostic images are not committed.
