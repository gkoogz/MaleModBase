# Prepared-mesh control presentation

The October 20 request is to give Witcher Wolverine's direct live-control
behavior. Wolverine deforms its prepared vertices instead of waiting for
rest-shape authoring, attachment cutting and native layout reconstruction.
Witcher now adopts that separation. Its ordered 32-bit numerical worker remains
authoritative; this is not a replacement of the solver or a claim of x64 parity.

`surface/live_controls.hpp` owns the SDK-free deformation contract. Inputs are
an immutable prepared surface, the controls that authored it, current controls,
semantic region weights, a root frame, original triangle donors and connection
lofts. Each update is evaluated from the prepared surface, never accumulated
onto the previous displayed edit. Protected stock vertices remain fixed.
Original-edge subdivision points inherit their triangle donors' displacement;
independent nonlinear evaluation at those points was rejected because it bent
edges and folded narrow triangles. Generated connection vertices follow their
loft endpoint ownership instead of placeholder physics bindings.

The adapter prepares these correspondences with its existing native layouts.
Its presentation worker receives controls independently of completed physics
sequences, deforms existing vertices and transports shading directions. Indices,
UVs, skin weights and native aliases remain unchanged. Nozzle and garment
presentation data follow the same applied deformation. Numerical requests and
contact feedback remain ordered. Preference writes wait for 700 ms of quiet
input on a separate worker, following Wolverine's persistence approach.

## Boundaries

The fast path is a bounded approximation while exact background authoring
catches up. Large changes can require a different valid attachment chart.
Triangle collapse or reversal rejects an attempted preview; backtracking tries
a smaller step and reports the controls actually displayed. It never labels a
limited preview as the requested final shape. Exact topology authoring still
exists. Removing it, claiming instant full-range edits, or claiming complete
Wolverine runtime parity would be inaccurate.

Presentation normals transport existing material detail using geometric normal
change. Valid triangles alone do not prove seamless appearance or exclude all
self-intersection. The exact-fit handoff and extreme combined edits need visual
review in addition to numerical checks.

## Evidence on October 20

- Six shared C++ regressions pass: live controls, original motion-filter
  comparison, presentation, root articulation, root transition and waist graft.
- Witcher deferred persistence and native temporal presentation tests pass.
- Actual prepared Witcher assets pass 81 bounded edits with immutable native
  indices, UVs and weights. All one-step edits from 50 to 49/51 apply fully;
  21 large jumps need a limited intermediate result. Mean evaluation 31.54 ms,
  maximum 74.23 ms in this offline run.
- Native cadence session `sealed-session-20261020-114757` observed all 40
  consecutive width/angle edits on completed GPU output. All 20 width edits
  and 19 angle edits used direct presentation ahead of the numerical worker.
  Mean acknowledged intervals: width 147.10 ms, angle 146.66 ms. The driver
  deliberately holds each edit for at least 100 ms; these are sampling
  intervals, not isolated input-to-photon latency. Median deformation work
  was 39.27/35.35 ms under game load. This is not evidence of 60 Hz controls.
- Previous Candidate14 intervals were 958/688 ms. That comparison is from
  recorded prior evidence, not a newly rerun baseline.
- Pulse cadence remains about 7.3 observed distinct meshes/sec. The ordered
  source dynamics and pulse generation were not accelerated by this change.
- Native DLL SHA256:
  `b3487f9a57e636f7ae3acf553d06a3985ab8484ba0a38f3f10ca9779864f804f`.
  Private current evidence is under `D:/MaleMod/Work/witcher/live-controls`.

The broad native attachment sweep completed 78 checkpoints, then crashed while
changing to a combined minimum-size configuration. The dump identifies an
out-of-range triangle vertex (`0x2000c6fd`) during the second geometric-normal
calculation in the exact-fit renderer. The writer responsible for that index
has not been identified. This is a failed run, not a passed attachment gate.
One hundred repeated offline evaluations of that configuration and an
AddressSanitizer build's 81 control edits did not reproduce the corruption.
The instrumented native build completed 18 focused shape checkpoints, including
the previously failing transition, with no sanitizer error. Its last screenshot
was not captured before shutdown; the review runner now waits for complete PNGs.

The final normal build checks triangle indices before the normal calculation.
That converts an invalid read into a reported geometry error; it does not
establish the corruption's cause or claim recovery. Its DLL SHA256 is
`22355b42791b1ba3b1cec157d836cd6f380915cfe46fd668b1707f1f33aa9502`.
Its offline probe passes all 81 bounded edits (21 limited large jumps), mean
30.76 ms and maximum 74.92 ms. Native follow-up results are recorded separately
under `final-shapes` and `final-cadence` in the current evidence directory.

Final native follow-up:
- `sealed-session-20261020-132622`: all 18 shape checkpoints, 19 complete
  screenshots, healthy runtime flags and both body resources; clean exit.
- `sealed-session-20261020-133959`: all 40 width/angle edits and seven complete
  screenshots; width/angle intervals 148.49/145.79 ms, direct presentation on
  20/19 samples, median deformation work 30.58/31.69 ms. The same 100 ms test
  hold applies. Pulse produced 89 distinct meshes across 90 samples in 12.11 s.
- Both sessions verified restoration of all 24 original saves and normal audio.
- Selected native views were inspected: minimum-size side/front, the final
  long/slender oblique shape, and the final direct-angle side view. The root
  remains attached in these views; existing pronounced folds/material changes
  remain. This limited review does not approve the full attachment matrix.

Compilation, complete checkpoint counts, visual review and installation are
separate states. This candidate has not been installed. Full attachment,
campaign, garment-contact and LOD certification is not claimed.

The old motion-filter verification failure was caused by generated LF data
compared with CRLF checkout bytes and outdated generated hashes. Its version 2
provenance normalizes generated text only; immutable source bytes remain exact.
The original numerical comparison passes. The broader provenance command now
reaches a different pre-existing stale graft-runtime header hash; that gate is
still not passed and has not been waived.

## Spoke adoption

Witcher pins this header alongside its shared modular geometry contract, with
its numerical worker revision unchanged. It owns native buffer mapping, ordered
publication, input, packing, visual validation and installation.

Wolverine already has a direct deformation path. A later adoption can replace
equivalent presentation math with this contract after its own observed binding,
attachment, motion and performance checks; do not replace its accepted runtime
as a side effect of this work. Other spokes can reuse the same geometry contract
and tests while supplying their own measured bindings and native publication.
