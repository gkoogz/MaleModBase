# Garment material and contact correction

The v2 capsule union check could succeed while material shape was unacceptable.
Actual Geralt CPU geometry contained long triangular strap fans, including
vertices below the feet. This was an authoring/contact failure, independently
of native shading. Cloth lighting normals are not body escape directions.

## Shared correction

The shared fitter uses measured capsule axis gradients and a consistent measured
route side to escape the union. Band, hem and strap cross sections translate as
intact groups, preserving width and thickness. Bounded primary projection and
two or three equal-arclength material passes redistribute a solved route over
fixed section counts. Complete triangles and vertices retain explicit signed
contact checks. An unresolved budget remains a failure.

The waistband uses the measured pelvic mean-height plane, while retaining its
original body donors. Red/blue stripes therefore do not reproduce zigzags in a
triangulated body seam. The pouch hem is rebuilt from the fitted rim, and the
panel top joins the current fitted band. Both strap ends are sewn by the closest
feasible **actual fabric-surface** displacement; nominal center distances do not
stand in for a physical connection. Sewing keeps each end section intact and
checks adjacent triangles against the calibrated capsule union. Knit UV length
follows the published material arclength. No triangles are deleted or hidden.

## Verification, October 3

- All 15 SDK-free CTest targets pass. Garment tests retain 240 animated 1..12x
  synthetic cases, a dense 40x extreme, fixed topology/capacity, lineage/TBN,
  Naked/reset, invalid inputs and unchanged translation/unit covariance bounds.
- Actual canonical Wolverine source fixtures retain 17,528 anatomy vertices and
  35,000 triangles. Default/max and the default with canonical thigh capsules
  pass 32 moving states each, including shape and whole-triangle contact gates.
  Source motion is anatomy rotation; articulated game body tests are separate.
- The persistent Witcher adapter test passes 41 actual binding cases: nine
  controls/extreme-angle combinations and 32 coherent articulated hip/knee/foot,
  body-donor and capsule poses. Topology is 4,068 vertices/7,764 triangles.
  Maximum ribbon edge is 55.586 mm, material spacing ratio 1.510 or less, band
  stretch 1.543 or less, and cross-section error below 2e-16 m. Both actual sewn
  fabric surfaces touch within 1.2e-16 m. All whole-triangle signed distances
  exceed the required clearance by at least 131 micrometres. The receipt and
  exact measured values are recorded in provenance/garments.json.
- Serialized source means are 57.675 ms with contacts and 27.931/29.158 ms without
  contacts. The source contact mean guard is explicitly 60 ms; the historical
  no-contact 30 ms guard is retained. A concurrent compile/test run exceeded the
  contact guard at 63.774 ms; the identical serialized run passed. The final
  Geralt diagnostic worst update was 77.133 ms under concurrent load. These are
  CPU worker costs, not a claimed game framerate. The shared numerical anatomy
  solver and physical laws are unchanged by this garment geometry repair.

## Adoption and evidence boundaries

Both adapters must pin the clean correction commit, rebuild the shared API and
strictly regenerate their owned native carrier topology/fingerprints. Wolverine
uses src/runtime/jockstrap_adapter.h; Witcher uses native/garment_service.hpp and
native/garment_fit_test.cpp. Existing measured character donors remain valid;
do not copy the fitter into an independently maintained adapter fork. A larger
fixed carrier is required for the refined straps.

The 41-case receipt was generated against the accepted 2062b2d body contract with
the new shared implementation in a diagnostic checkout. It proves offline
geometry/contact/material integrity. Clean adapter adoption, native cooking,
rendered appearance, collision walkthroughs and runtime scheduling remain
separate adapter gates. Public 0.5.0-beta.1 is unchanged by this repair.

## Moving sewn-end correction

The subsequent private game run exposed a transient pouch sewing failure even
though checkpoint geometry passed. The first failed raw worker input reproduced
the failure deterministically: all triangles cleared the body, but translating
the final ribbon cross section to its nearest hem point would consume the
required clearance. The nearest stitch was 5.027 mm away; translation alone
placed a far corner at 0.319 mm signed distance, below the unchanged 2.298 mm
margin. There was no local feasible translation-only sewing target. Shifting
the same input to a local origin or by plus/minus one kilometre did not fix it.
The cause was a fixed endpoint material frame under moving hem contact geometry.

When ordinary translation cannot sew a pouch end, the shared fitter now turns
that intact end using the actual nearby hem tangent and measured capsule radial
frame. Original width/thickness axis signs prevent a reversed material twist.
Candidate movement is bounded by the material width and the existing local
sewing search region. It selects the least-displacing candidate only after all
adjacent triangles pass the original clearance. No successful contact flag is
manufactured, and an unresolved joint remains a reported failure.

The exact captured case now passes contact, material dimensions and physical
sewing: signed distance remains 3.829 mm, the hem surface gap is below 4e-15 m,
maximum ribbon edge is 46.343 mm and material spacing ratio is 1.414. A fresh
session replay at local origin, plus/minus one kilometre and 100-fold caller
units also passes; maximum translation covariance error is 1.74e-11 m.
The persistent adapter capture/replay tools serialize explicit fields and run
these coordinate gates. Captured input, geometry and local receipts remain
ignored/private; only their hashes and measured conclusions are recorded here.

All 15 shared tests, 96 source fixture states and 41 coherent actual Geralt
binding cases pass again. Latest Geralt worker maximum is 62.040 ms; serialized
source contact mean is 58.177 ms. The numerical source kernel and verified
archive remain unchanged. These results prove the captured-case correction
offline; the corrected native game run remains an adapter gate.
