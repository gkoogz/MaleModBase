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

The fixed root position and rest command do not change. The scalar point-mass
candidate at `389f6c0` was rejected by sustained native Wolverine motion: angular
rates reached 198 rad/s and the anatomy reversed into the body. Fixed root,
first-link length and finite-state checks did not detect that visual failure.
`root_contact::PointInverseMass` remains a historical compatibility helper; it
must not be used to implement the provisional coupled angular root.

The successor uses `surface/tangent_root.hpp`. Its two angular Jacobian columns
are derivatives of the existing source direction convention. Contact mobility
is the tensor `J * diag(1/Iroot,1/Iroot) * J^T`, not an isotropic translational
mass. Keep total pitch/yaw in persistent state during all position iterations.
Apply impulses to those angular coordinates with `PositionImpulse`, and rebuild
the first station from `Direction(state) * originalLinkLength` after each
correction. Root-adjacent vector bending solves the full 3x3 mobility matrix.
Radial mobility vanishes; exact link length follows from the coordinates and
is not imposed by a late normalization.

The existing source suspension was calibrated while the first link was
kinematic. Its nominal straight-rod inertia is now explicitly derived:
`Imodal = Iroot + sum(mass_i * arm_i^2)`, for original dynamic shaft stations
2 through 11 only. `mass_i = .75 + .0125 * sourceShaftWeight` and
`arm_i = i * originalRestLength / 11`; `Iroot` retains the original root
suspension expression. These are source numerical quantities, not inferred
physical game units. Neither lobes, the recruited pelvis nor source station
0/1 are assigned guessed modal masses. They retain their existing constraints.

`EquivalentLaw` multiplies source stiffness and damping by `Imodal/Iroot`.
This preserves their nominal free-response coefficients `K/I` and `C/I`.
The original pitch/yaw onset limits stay unchanged and their source soft-stop
coefficient 90 is converted to `Kstop = 90 * Imodal` by the same rule. Both
restoring/damping and soft stops enter the position solve implicitly. No new
hard angle or angular-rate clamp is added. This nominal free-response identity
does not establish full coupled-response, material or native stability parity.

After position iterations, reconstruct the first-station velocity from accepted
angular displacement over the substep, using the accepted Jacobian. Contact
velocity/friction impulses use that same tensor. Recover tangent angular rates
with `Rates`; then `root_contact::Relative(total, imposedDrive)` subtracts the
actual sampled rest-drive and clinical-yaw changes over the same substep,
including clinical deactivation. Do not subtract only the analytic root-drive
velocity: a finite-step command transition otherwise integrates twice. Previous
driven yaw resets with the original compliant state.

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
within 1.54e-6. Its lateral root excursion .957741 is large and remains
unaccepted; this checks state agreement, not stable clinical travel. This
numerical fixture is not a game capture.
The wire test covers exact serialization and uncalibrated opt-in rejection.
The process-isolated Win32 kernel compiles and regenerates byte exactly from
the checked source spans.

The disabled final worker replay compares 46 measured Geralt bind-contact cases
against the old wire6 worker: 138 complete buffers for both target LODs and
composed native vertices remain byte-identical. Artifact:
`TheWitcher3MaleMod/build/witcher-parity-tangent-gradient-final-proof.json`.
That covers neutral, three states with extreme angles at maximum width/size,
coupled maxima/reported shapes and 30 moving-force frames. It is not the full
native/campaign acceptance matrix or proof of all unused source outputs.

`D:/MaleModDiagnostics/witcher-parity-20261009/tangent-source-consumption-final/proof.json` compares 70 original
control/physics inputs with the preserved exact pre-root source library. All
560 complete geometry, both-body, normals, tangents, UV, topology and mechanics
artifacts are byte-identical. All three states also preserve complete numerical
payloads across six CDF modes, including impulse consumption, retry, zero-substep
queueing and coalescing. The oldest original oracle correctly rejects the
pre-existing stable-collar revision (default maximum displacement 0.39728987);
the current proof does not relabel that older receipt as passing.

The earlier proper fractional-curve transport diagnosis remains a separate
private experiment; this revision does not adopt it. The existing Geralt
maximum-angle source/target skin still has folds and a defective lower bridge.
Source-only numerical success is not a native visual pass. Earlier virtual
straight-station angular projection, root-only graft compensation and skipping
the logical-surface reassertion were rejected.

The new tangent helper independently tests finite-difference Jacobians, radial
nullspace, reciprocal mobility/virtual work, fixed link length, SPD bending,
source-station modal inertia and 960-step nominal free-response equivalence.
It tests implicit stop symmetry/release and compilation after real Win32 headers.
An optional `MALEMOD_BUILD_ROOT_DIAGNOSTICS` target takes an external measured
case file and preserves the complete source solver across eight moving/settling
episodes. Captured case data is excluded from Git. Its finite/root/link/hemisphere
diagnostics are not an angular stability or visual acceptance certificate.

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

Wolverine: consume `tangent_root.hpp` through the tested provisional Base pin
and mirror the checked `tools/angular_root_kernel.py` integration. Do not retain
the rejected scalar root-mass/late-normalization path. `Jacobian`, `Response`,
`EffectiveMass`, `Mobility`, `PositionImpulse`, `ModalInertia`, `EquivalentLaw`,
`Spring`, `SoftStop`, `Solve` and `Rates` define the shared API; the adapter owns
its source station arrays, persistent total angular state and measured drive. Enable only with validated
live contacts; fallback rest thigh constants are not live measurements. Preserve
the existing reference branch exactly when disabled. Capture actual root/guide
and body contact inputs in the private room, inspect both body resources/LODs,
and prove the full attachment gate before installing. Analytical garment ring
guides must not be substituted for solver rod nodes or clinical skin radius.

Other spokes: retain the default-off branch until character calibration, worker
protocol and native attachment ownership are verified. Do not copy an independent
physics fork or infer bone names, units or backend capability.

## Current provisional source-motion diagnostic

Eight externally measured input episodes preserve solver history for960 fixed
substeps each, with2 seconds of imposed gait/side motion and2 of settling.
After correcting the ordinary suspension-gradient mass terms, all finite,
fixed-root, first-link and diagnostic hemisphere checks pass. Peak relative
pitch/yaw rate is15.610714 rad/s; maximum relative pitch1.038992 rad/yaw.423646;
late-settle peak rate.119973 rad/s. Station speed reaches3719.339 source units/s,
so these outputs must not be called accepted motion. Root-off continuous CSV
is byte-identical to reconstructed389 source (SHA256
`a9a1d0e4382121fbf7cf8418feb248363f885031341f773b30fb7b3c81625543`).
The actual Hermite suspension mass matches finite differencing within.002083
relative error across moving samples. Ordinary particle gradient contributions
remain `inverseMass * |gradient|^2`; only station1 uses generalized mobility.

The source diagnostic and shared independent free-response test are distinct.
Neither proves game mesh clearance, clinically plausible swing, campaign
transitions or performance. The caller recipe is `TANGENT-ROOT-ADOPTION.md`.

The final generated proof buffers are private data on D:, after C: ran out of
space during duplicate exports. Older regenerable replay buffers were removed;
their exact hash receipts, immutable input recipes, source and key previews
remain. `build/replay-retention-20261009.json` records the bounded cleanup.
`verify_root_source.py --diagnostics-root <owned-root> --output <new-child>`
permits a fresh external diagnostics directory without changing an install.
