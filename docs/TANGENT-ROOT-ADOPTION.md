# Provisional angular-root caller recipe

This API is default-off and numerically experimental. Neither the full moving
skin nor native/campaign attachment gate has passed. In particular the measured
eight-episode source diagnostic still has a transient angular rate above
15 rad/s, and a moving-rest clinical fixture has an unaccepted large lateral
swing.
These measurements must not be described as accepted physics or fit.

Include `malemod/surface/tangent_root.hpp`. Game adapters supply actual source
station arrays, bone/contact calibration, drive and original material/state
parameters. The helper has no skeleton, units, renderer, input or installation
dependency. Wolverine may use these numerical helpers with its existing native
reference functions while a separate full Session/bone/output migration is
tested. Do not copy generated Base kernel loops into an independent library.
The checked extractor transform is an integration reference, not a new fork.

1. Keep the original scalar reference path intact when opt-in is false. The
   Base extractor preserves its original function closure and dispatches to
   separate active-root functions because inserting even an unused precomputed
   `delta/distance` changed float factoring in the disabled `PDDistance` path.
   Require complete source output and continuous history replay to be exact.

2. Keep the welded root and original first-link length fixed. `pdInvMass[1]`
   remains zero: its angular response is represented by two coordinates, not
   by a fictitious isotropic point mass. Maintain total pitch/yaw in persistent
   state throughout the position solve; do not recover them repeatedly from
   `atan2` after each correction. Reconstruct station1 after every angular
   impulse with `Direction(state) * originalLinkLength + root`.

3. `Jacobian(state,length)` gives the pitch/yaw columns. The current extracted
   source rounds those columns into source float vectors before constructing
   the shared `Basis`. Use `PositionImpulse(state,basis,Iroot,impulse)` for
   angular position response. `Response(basis,Iroot,impulse)` supplies tangent
   velocity response, `EffectiveMass` supplies the scalar gradient mass, and
   `Mobility` supplies the full 3x3 matrix. Other stations keep source masses.

4. Replace every active-root scalar/direct-mass path, including distance,
   scalar contact, interpolated rod/body contact, static friction, contact
   velocity/friction impulses and proximal suspension reactions. For a rod
   interpolation `a*(1-t)+b*t`, scalar mobility is
   `w_a*(1-t)^2 + w_b*t^2`; send each weighted impulse through its station
   response. Include the root station's Hermite gradient in both analytic and
   finite-difference suspension mass/reaction paths. For ordinary stations retain
   `inverseMass * |gradient|^2`: their scalar `PDEffectiveMass` does not itself
   multiply a non-unit gradient magnitude. Body angular terms keep
   their existing source inertia and arms.

5. At root-adjacent vector bends, form `M = sum(c_i^2 * Mobility_i)` using
   coefficients `[1,-2,1]`. Root station mobility is the tensor; other source
   particle mobility is their scalar inverse mass times the identity. Solve
   `((1+gamma)*M + alpha*identity) * deltaLambda = -rhs` with shared `Solve`,
   then apply coefficient-weighted station impulses. The provisional extractor
   retains the source bend compliance/damping preparation; it does not claim
   an independently measured new material law.

6. Record the old relative spring angles and imposed rest/clinical angles
   before advancing the drive. Predict enabled angular motion with the
   existing external gait/side drive, leaving the source restoring, damping
   and soft-stop terms for the coupled implicit solve. `ModalInertia` takes
   original dynamic source shaft stations2..11 with source rest arms and
   masses, plus original root inertia. Do not guess lobe/body participation.
   `EquivalentLaw(Iroot,Imodal,{sourceK,sourceDamping})` defines the implicit
   spring conversion; preserve the source yaw stiffness weight and targets.
   `Spring` returns the updated multiplier and angular increment. Soft stop
   stiffness is `90*Imodal`, with unchanged pitch state onset and yaw onset.

7. Keep all24 original coupled position iterations and their original order.
   After each original ventral recovery, apply the implicit regular angular
   spring, then `SoftStop` for pitch and yaw, rebuilding station1. Reset their
   multipliers at each substep. These are soft constraints, not a new hard
   travel/rate clamp. Do not add a rod/pelvis repulsion at the welded root;
   the neutral pelvis capsule is deliberately overlapped by recruited skin.

8. Reconstruct the accepted station1 velocity **before** contact velocity
   impulses, using the accepted Jacobian and total angle differences over
   this substep. Contact velocity/friction response then uses the same tensor.
   Recover total angular rates with `Rates`. Subtract actual finite per-step
   imposed rest/clinical angle changes with `root_contact::Relative`; subtract
   the angles as well. Analytic command rate alone and total recovered rates
   stored as spring rates each integrate drive motion twice. Include clinical
   inactive transitions and reset previous imposed yaw with compliant state.

9. Enable only with accepted actual contact calibration. Do not substitute
   rest thigh endpoints or garment analytical ring guides for live skeleton
   measurements. The SDK-free helper rejects singular/invalid algebra rather
   than inventing an angular stop or clinical skin radius. Inspect default,
   extrema, intermediate/combined controls, every state, moving histories and
   materials on both body resources/LODs before any native promotion/install.

Witcher consumes the identical helper through the source worker, wire7 explicit
flag and a pinned clean Base. Its bindings/render/profile/worker receipts must
identify that pin; current installed431 identities remain the rollback. Other
spokes follow the same staged path with their measured calibration. All source,
offline, native, visual and installed evidence remain separately recorded.
