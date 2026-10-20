# Live morphology cadence repair

The user reported roughly one visible update every two seconds during pulses
and slider editing. This checkpoint implements and measures a repair in the
private Witcher grey room. It is not installed or adopted by a release pin.

## Cause and implementation

The adapter used live dimensions to invalidate the entire dynamic attachment,
although its complete shaft still used immutable zero-motion material. Pulses
therefore paid for aperture cutting, donor reconstruction and four native buffer
layouts without correctly applying their transient strain to that material.

Base now carries explicit `State::materialScale` relative to the immutable
control morphology. Material sections apply pulse strain, and rod length
constraints retain that requested length without compounding it. The socket
circumference remains fixed; strain releases smoothly across the first two free
intervals. This preserves the control-sized opening during transient pulses.
Ordinary size/angle controls still rebuild the properly fitted opening.

The adapter derives this ratio from the live and zero-motion dimensions and
uses neutral dimensions for cache invalidation. New semantic material or hang
controls still invalidate the cache: stale ownership is not accepted.

Rebuild optimizations preserve actual geometry and corner attributes:

- Base uses a conservative triangle-ray hierarchy for measured aperture rays.
  The original narrow-phase arithmetic, tolerances and source-order tie rule
  remain authoritative.
- Untouched triangles bypass unnecessary cut-conformance and loop-refinement
  allocation. New intersections still preserve original-edge donors.
- Zero clearance supports skip unnecessary trigonometry.
- Native bone weights use bounded sorted storage, source-corner rows are
  cached, ramp rows are computed once per vertex, and independent resource/LOD
  layouts build concurrently. Output insertion order remains deterministic.
- A fresh isolated morphology worker overlaps the ordered live source solver.
  Reusing a stepped worker was tested and rejected because its output differed
  from a fresh zero-motion morphology. Physics samples are not skipped.

The shared algorithms belong in Base. Native layout, process orchestration,
diagnostic fields and the scripted cadence scenario belong in the Witcher
adapter. The diagnostic CMake opt-in now reports unadopted modular hashes while
still validating the committed pin; normal builds continue to reject drift.

## Evidence

Private evidence root:
`D:/MaleModBuilds/modular-anatomy-20261015/root-seam-20261019/candidate-14`.

Final DLL: `c3542adcd533b14e0c3e038e015897fd2766b0a9031828104c5f4982ca480780`.
Final replay probe: `dee0dadbb96168cca4a2cac4a0b019bbe0a271b1f553eb9fc5a7103ffdfa6b76`.
Source snapshots and exact hashes are in `final-sources.json`.

Eight shared tests passed, including conservative ray lookup, material pulse
diameter, fixed socket, noncompounding length and existing attachment tests.
Forty captured-pose replays passed orientation, manifold, body-boundary and
material-ownership checks. All forty retained byte-identical positions, faces,
rings, packed native attributes, indices and secondary attributes relative to
Candidate13. Replay median full rebuild fell from 2221.5 to 412.7375 ms; these
separate replay runs are indicative timing, not a gameplay FPS claim.

The same official native scenario ran against the installed baseline and the
final private candidate, without host input or focus changes:

| Native measurement | Installed baseline | Candidate14 |
| --- | ---: | ---: |
| Distinct pulse meshes per second observed by sampling | 0.825 | 7.335 |
| Median pulse mesh evaluation | 1794.9 ms | 67.0 ms |
| Average acknowledged width-edit interval | 2.388 s | 0.958 s |
| Average acknowledged angle-edit interval | 1.811 s | 0.688 s |

The pulse phase observed 90 distinct meshes in 90 samples over 12.13 seconds,
with zero topology rebuilds and measured radial strain from approximately
0.9975 to 1.1195. Sampling is deliberately spaced by at least 100 ms, so this
is observed update cadence, not total renderer throughput. Each width/angle
phase contains twenty consecutive acknowledged edits; intervals include the
driver's minimum 100 ms hold. Slider editing still has measurable latency and
must not be described as instantaneous or frame-rate responsive.

The installed baseline lacks the new rebuild/strain telemetry. Its missing
fields are not evidence of zero rebuilds or zero strain.

Receipts, raw traces, captures and exact preview identities are linked from
`cadence-baseline.json` and `cadence-candidate.json`. Both sessions were closed
by their owned receipts and restored saves/audio. Twenty-four final native side/front/underside attachment views were reviewed
across sizes 50/100, angles 15/25/50/90, hang extremes and motion. No obvious
new root gap or fold was seen; the prior material transition remains. Both body
resources completed with LOD mask 5. These observations do not cover all native
mechanical states/LODs or campaign scenes. Attachment evidence is in
`skin-evidence.json`, separate from timing evidence. A cadence result alone is not the
complete attachment-integrity gate or campaign acceptance.

`tools/verify.py` remains blocked by the previously recorded motion-filter
snapshot newline/hash mismatch. That failure was not repaired or relabeled as
a pass as part of this performance change.

## Adoption boundary

No installation, shared revision adoption or production pin changed. The
working tree includes the uninstalled Candidate13 root-ramp geometry. On October 20 the user explicitly selected the latest collar preview together
with the cadence repair. That resolves the geometry choice; native validation
and deployment remain separate steps. Keep the currently accepted DLL and exact rollback
until the selected combination passes its native attachment and campaign gates.

When adopting, commit the shared changes and tests, include
`triangle_ray_index.hpp` in the versioned module file set, update the adapter's
modular pin intentionally, and revalidate the selected native package. The
numerical worker revision remains separate. Wolverine and other spokes should
adopt the same shared material-strain and aperture code through their own pins
and observed native gates; no other game runtime was modified here.
