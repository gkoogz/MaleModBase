# Calibrated angular root joint candidate

This is an opt-in numerical development candidate from Base `2c154792`.
It is not an accepted attachment or gameplay replacement. The shared first-link
joint is consumed by an isolated Witcher worker; actual native adoption and
the complete attachment gate remain outstanding in both spokes.

## API and mechanics

`surface::Frame::rootContacts` defaults to false. Enabling it requires actual
`Frame::collision` calibration and observed/calibrated `thighEndpoints`.
Wire version 7 carries the flag explicitly and rejects an enabled uncalibrated
request. Older workers must be rebuilt and reverified; there is no protocol
fallback that silently enables different physics.

The fixed root position and rest command do not change. On opt-in only, the
first rod link receives an angular degree of freedom with effective tangent
point inverse mass `linkLength² / angularMass`. The angular mass is taken from
the existing source root suspension, without inventing character units. The
existing coupled distance, bend, rod/thigh and rod/lobe constraints can then
react onto root orientation rather than folding the free rod around a wholly
kinematic first link. The first link is normalized to its exact original length
after acceptance. `root_contact::FromJoint` recovers continuous pitch/yaw and
angular velocities from the actual accepted link and its velocity; the pitch
is stored relative to the unchanged rest drive.

`root_contact::Relative(total, imposedDrive)` subtracts both imposed angles and
angular rates. Recovery subtracts the actual rest-drive velocity and the sampled
clinical yaw change per fixed substep, including its inactive transition. The
prior imposed yaw resets with the source compliant state. Driven motion must
not be integrated again as contact suspension motion.

No new pelvis/rod proxy constraint is added. Neutral body capsule overlap at
the recruited welded root is expected and must not be treated as detached
surface penetration. The actual root/collar ownership, curved rod tube and
skin surface need their own native inspection and contact certification.

The source snapshot under `legacy/wolverine` is unchanged. Source integration
is an explicit checked extractor transformation and a default-off runtime
input. Geometry bindings, vertex lineage, UV aliases, original-edge donors and
shared waist boundaries are unchanged.

## Verification and limitations

The SDK-free joint test covers angle/velocity round trips across angle wrap,
radial velocity removal, link inertia scaling and invalid-state rejection.
It also verifies separation of nonzero imposed pitch/yaw and their velocities.
An enabled process-kernel regression exercises moving rest pitch, nonzero
clinical yaw and clinical deactivation; accepted guide/root direction agrees
within 2.98e-7. This numerical fixture is not a game capture.
The wire test covers exact serialization and uncalibrated opt-in rejection.
The process-isolated Win32 kernel compiles and regenerates byte exactly from
the checked source spans.

The disabled final worker replay compares 46 measured Geralt bind-contact cases
against the old wire6 worker: 138 complete buffers for both target LODs and
composed native vertices remain byte-identical. Artifact:
`TheWitcher3MaleMod/build/witcher-parity-root-relative-disabled-proof.json`.
That covers neutral, three states with extreme angles at maximum width/size,
coupled maxima/reported shapes and 30 moving-force frames. It is not the full
native/campaign acceptance matrix or proof of all unused source outputs.

`build/root-contact-source-consumption-current/proof.json` compares 70 original
control/physics inputs with the preserved exact pre-root source library. All
560 complete geometry, both-body, normals, tangents, UV, topology and mechanics
artifacts are byte-identical. All three states also preserve complete numerical
payloads across six CDF modes, including impulse consumption, retry, zero-substep
queueing and coalescing. The oldest original oracle correctly rejects the
pre-existing stable-collar revision (default maximum displacement 0.39728987);
the current proof does not relabel that older receipt as passing.

The opt-in maximum Semi/Overall/Width/Angle mesh has substantially fewer shaft
creases than the measured baseline. Its attachment ridge/lower bridge still
fails visual acceptance. Offline numerical success is not a native visual pass.
Earlier virtual straight-station angular projection and graft-only compensation
prototypes were rejected; they are retained only in private build diagnostics.

Historical `provenance/source-surface.json` deployed-worker receipts remain
wire6 evidence. They must not be relabeled to certify this wire7 candidate.
`provenance/root-contact.json` identifies the new candidate source and exact
disabled replay independently. A production source/worker consumption receipt,
full native attachment regression and campaign check are required before any
adapter installation, release or accepted new Base pin.

Run `python tools/verify_root_contact.py --kernel
build/root-contact-process/surface-generated/surface-kernel.inc` to verify
candidate file identities, immutable source closure and the generated kernel.
This audit preserves the explicit unaccepted status; it does not certify a
historical worker receipt or supply missing native evidence.

## Adoption paths

Witcher: pin the final tested candidate only after its source worker receipt is
completed. Build both architectures against the same wire7 headers; rebuild
bindings/render/profile identities for that pin. Carry an explicit calibrated
runtime opt-in into pose frames. Preserve the installed431 runtime and exact
rollback until the complete native grey-room and campaign gates pass. Do not
accidentally include the unrelated dirty garment TimeContinuity dependency.

Wolverine: consume this shared header through its tested Base pin and the same
conditional root-mass/joint-recovery integration. Enable only with validated
live contacts; fallback rest thigh constants are not live measurements. Preserve
the existing reference branch exactly when disabled. Capture actual root/guide
and body contact inputs in the private room, inspect both body resources/LODs,
and prove the full attachment gate before installing. Analytical garment ring
guides must not be substituted for solver rod nodes or clinical skin radius.

Other spokes: retain the default-off branch until character calibration, worker
protocol and native attachment ownership are verified. Do not copy an independent
physics fork or infer bone names, units or backend capability.
