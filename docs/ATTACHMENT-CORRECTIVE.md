# Material-bound attachment correction

The raised-angle study traced the saddle into the root displacement blend and
absolute surface fairing. Releasing the shared waist moves neighbouring body
tissue without fixing the saddle. The correction keeps the recruited body
surface, collar donors and neighbouring tissue fixed and acts on the sewn patch.

`include/malemod/surface/attachment_corrective.hpp` supplies an SDK-free C++17
plan. Adapters supply unique topology, immutable exclusions, sparse displacement
paths, measured scales and material frame bindings. `attachment_fields.py`
supplies ownership and bounded neighbour support. Shaft-dominant material does
not inherit collar controls. Zero displacement is distinguished from a hard
anatomical boundary.

The plan interpolates reviewed knots with C2 weights and checks area and
originally acute neighbouring faces over each entire chord using quartic
Bernstein bounds. It preserves valid authored shapes exactly. Threatened chords
trigger local constraint projection and guarded continuation; unsafe remaining
travel is damped. Exclusions remain exact. These are local checks, not a global
intersection, seam-shading, native-skinning or visual-acceptance certificate.

Witcher supplies the existing angle contract, measured girth, per-LOD depth,
body anchor frame, shared waist, donors and material roles. The correction
retains the existing opening, UV aliases, maps, resource boundaries and lobe
cores. Generated geometry and correction fields remain private ignored inputs.
The artifact is bound to the exact binding bytes and geometry revision.

## Verification and adoption

Shared tests cover fixed rows, zero response, frame transport, valid endpoints
with transient collapse, curved paths, ownership and bounded support. Witcher
checks both LODs across 48 source angle/state cases and six captured native LOD
outputs against the original solver for the identical source frames. Record
source, offline, native and visual results independently. Numerical checks do
not constitute the mandatory attachment gate.

Current adoption is an explicit dirty-source diagnostic; the binding revision
and normal dependency pin remain unchanged. After visual approval and the full
gate, commit the shared API/tests as one clean Base revision and deliberately
pin it in Witcher. Wolverine must consume the same API through its Base pin,
generate its own material-bound field from licensed lineage, and repeat its
native attachment gate. Future spokes follow the same adoption path. Never
copy the algorithm into an independent fork or reuse Geralt's calibration for
another character.
