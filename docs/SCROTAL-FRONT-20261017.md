# Hanging-sac front contour, October 17-18

The user identified the front of the hanging sac, not its connecting neck.
The repair reconstructs the transition above the rounded lobes while preserving
the complete shaft, neutral pelvis, original lower sac and suspension solver.
This is part of the documented neutral adult-anatomy teaching project.

## Cause and chosen repair

The original retained sac had 8,410 vertices. Its front-quartile transfer donors
carried legacy shaft influence up to 0.52068, versus about 0.0000103 at the back.
Independent lobe bindings did not remove that influence: live surface transfer
reapplied the old combined surface, including its stiff upper-front strip.

The accepted-runtime transfer and numerical physics remain unchanged. Instead,
the authoring cut moves from source Z76 to Z72, below that strip. The rounded
lower lobes remain exact: 4,752 protected source vertices have zero error. A new
tangent-matched patch connects the retained skin to the complete shaft.

radial_attachment accepts measured longitudinal station bounds. They reserve
the pelvic socket independently of the deeper cut's outline. The character
recipe uses [4.56114258982752, 15.229930394059204] in measured source units,
preserving the old distal extent and moving the proximal extent forward 1.5.
This is character calibration, not a universal anatomical constant. The
circumferential side-wrap remains 100 degrees; the shaft is not widened.

trumpet_neck tests the complete sampled patch before emission. It reduces all
Hermite handle lengths together until no faces reverse, preserving endpoint
tangent directions and coherent neighboring columns. The measured candidate
uses scale 0.6 and has zero reversed neck faces (minimum outward cosine 0.12713).
No original lower-sac vertex, UV alias or shaft geometry is smoothed away.

## Rejected experiments

Local material transport folded the front; harmonic displacement extension
creased during walking; whole-sac material transport retained a hard upper lip
and did not establish a compliant skin replacement. Limiting only live loft
handles also retained the lip. These runtime edits and MMXFER03 were reverted.
The chosen candidate uses the accepted MMXFER02 runtime.

Deeper cuts without a reserved footprint failed orientation or crossed the
pelvic assembly cut. Those failed authoring trials were not installed. The
current recut09 candidate assembles without crossing the root socket.

## Reproduction and evidence boundaries

Use the shared builder with the licensed character source and its measured root:
shaft cut 24, scrotum cut 72, protected lobes below 70, neck flare
1.38695737555697, side-wrap 100, proximal-fill 0.65, the station bounds above,
refined complete shaft and welded visible union. Assemble against the original
joined body, then rebuild runtime, dynamic mount, transfer, native binding and
materials together. Do not mix those artifacts between revisions.

Private generated evidence is under
D:/MaleModBuilds/modular-anatomy-20261015/sac-front-20261017/recut09.
The 13 shared anatomy/union tests pass with the authoring triangulation dependency
available. A 32-case preflight replay preserved the original pelvis and found no
reversed collar faces. Native full-matrix and restart views have been captured
and reviewed; final motion, distance, campaign and installation evidence are
recorded in the adapter's delivery report when complete. Do not infer a full
attachment gate or installation from authoring, compilation or this checkpoint.

The broad dirty-checkout provenance command currently stops at the unrelated
motion-filter extraction mismatch. This repair does not alter that extraction
or its installed worker. Record exact authoring inputs and retained runtime
provenance separately rather than treating the broad command as passed.

## Adoption

The shared builder and regressions belong in Base. Witcher must pin this source
and the coupled character recipe, retain the verified runtime/worker identity,
then pass native front/side/oblique, morphology, motion, both-body-resource/LOD
and campaign checks before replacing installed assets. Wolverine and future
spokes can adopt the same builder with their own measured footprint and cut;
they must regenerate their bindings/materials and pass their native gate.
No other spoke's runtime is changed by this work.
