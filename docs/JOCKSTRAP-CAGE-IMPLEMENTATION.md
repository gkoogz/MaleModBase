## October 10 - optimized pre-shaped jockstrap installed

Installed normal runtime 9919da09202c9e339db79a4b38df95eeaa0118055dc121df3759279f8c829cb8
to C:/Games/X-Men Origins Wolverine/Binaries/d3d9.dll. Canonical c718b1b plus
named hashed overlays, Base pin 3ee59844520e9e4ca6ad949594df9e6c9b2a297c;
source identity 37e6b6afca3d61f4b49105fa0197e8bd4bba4f93731821abbaebb327b28672dd.
Exact rollback: C:/Games/X-Men Origins Wolverine/WGame/ModBackups/
Meridian-development-20261010-134102-530873/Restore.ps1 (prior ab1f8d8e).
Installer, installed hash, protected save/settings/audio hashes and rollback
ValidateOnly passed. No private test hooks in installed DLL.

Pre-shaped cage/local contact, CPU source caching and covered cadence replace
the expensive walker; covered support damping, coherent minimum Length and
measured Angle bounds repair visual failures; buffer ownership resets on reload.
147 static / 540 motion / 480 pulse numerical cases passed. Actual character
has one supported LOD and both body resources were covered. Required sampled
attachment gate recorded: 133 room images plus campaign evidence, all 3 states,
size combinations, extreme mapped angles, orbit and motion. This is sampled
visual acceptance, not continuous collision or every possible pose proof.

Paired fitting CPU 37.875 -> 2.110 ms (94.43% less, 18x). Final native campaign
Tank + Jockstrap averaged 32.124 ms / 31.13 FPS over 1,080 active frames at720p.
Landing, damage, running, death and checkpoint rebind/visibility observed.
Post-reload movement was not captured before bounded timeout. Room exit0,
campaign bounded exit124; no owned game remains. Room prior DLL restored.
The earlier host MEMORY_MANAGEMENT BSOD also occurred October8; cause remains
unknown and no machine stability fix is claimed. Short reruns completed.

See canonical docs/JOCKSTRAP-PLAYABLE-ACCEPTANCE.md and private evidence
D:/MaleModBuilds/jockstrap-acceptance-20261010/install-evidence-final.json.
No commit/push/release. Named overlays and unrelated dirty work retained;
Wolverine ahead68. Other spokes adopt pinned shared Base with native tests.

# Pre-shaped jockstrap pouch

The new shared pouch is implemented and adopted by Wolverine's source through
Base compatibility pin `c2bf9699646a607e48f272d88eb4d58bc6b2f49d`.
It replaces the walking/following/retry path in the regular Meridian adapter.
The normal DLL is built privately. Retail is unchanged; full attachment,
campaign and supported-LOD acceptance remains incomplete.

The self-contained comparison is
`D:/MaleModBuilds/jockstrap-cage-20261010/report/Pouch-comparison.html`.
It embeds 103 original native PNGs, including the optimized walker comparison,
the new pouch's front/side/rear/orbit views and movement samples, plus the complete 20-case control sweep. Image hashes
and source/build identities accompany it. Browser preview was blocked by the
local-file security policy; content, embedded assets and source were checked.

## Measured performance

| Measurement | Earlier path | New path | Meaning |
| --- | --- | --- | --- |
| Paired offline fit | Optimized walker 37.875 ms | Cage 2.110 ms | 94.43% less fit CPU, about 18.0 times faster |
| Earlier exact walker optimization | 139.876 ms | 36.044 ms | Previous separate paired replay, 74.23% less CPU with identical geometry and rejection receipts |
| Covered physics replay | 240 Hz, mean 7.820 ms | 120 Hz, mean 4.054 ms | 48.16% less CPU across the 36-case matrix; trajectories intentionally differ |
| Native preparation | Old retail log had 425.261 ms mean Draw | New complete preparation roughly 3 ms | Different runs and timing scopes; not a paired whole-game speedup |
| Native complete frame | Earlier cage builds later reached roughly 52-60 ms | Persistent CPU geometry plus covered cadence: 25.209 ms settled mean in final candidate 16 | About 39.7 FPS over 2,040 frames; separate runs, not a paired campaign benchmark |

The final fit replay uses 14 measured poses, three repetitions per pose and
three alternating baseline/candidate runs. The new output is deterministic
across all three runs. The three candidate means are 2.097, 2.203 and 2.029 ms; peaks are
2.559, 2.475 and 2.552 ms. These are measured replay times, not a worst-case guarantee.

The whole-frame gain also includes exact-input skin-support caching and a
persistent CPU copy of the authored native geometry. The previous staging
experiment still read the mapped graphics buffer every update and later native
runs degraded to roughly 52-60 ms per frame. The retained implementation reads
that buffer only on a new binding/rest reset, evaluates in CPU memory, then
publishes only the first 20 position/shading bytes of vertices changed by the
evaluator. It never publishes cached native UVs or weights. Its two CPU copies
cost roughly 3.3 MB at the measured vertex count. All 72 complete replay frames
match the original buffer output byte-for-byte, including both body resources.


## Fit and appearance

`pouch_cage.hpp` starts from one cap aligned to the measured sewn boundary.
Only 24 angular rays and 17 axial control rows receive local support
corrections: 408 scalar controls. A continuous splat, outward envelope and
bicubic interpolation bridge the contents into one pouch rather than copying
each anatomical lobe. The display still uses the existing 64 by 40 material
grid, seam aliases, trim, straps, UVs and bindings; the expensive contact work
does not scale with every display triangle.

The sewn vertices are unchanged exactly. The shorter five-percent axial seam
transition corrected the audited row-2 penetration in the earlier prototype. Pole
clearance accounts for the rounded endpoint interpolation. The outward axis comes from sewn-boundary winding, never from the moving
contents; a minimum cap depth prevents collapse when supports cross behind the
sewn plane. This fixes the observed stride-dependent inward flip. Nonfinite seam,
tip or support inputs are rejected before producing invalid output.

This is deliberately a reasonably conforming garment, not a detailed imprint
of the contents. Its size follows the existing anatomy controls. Maximum
anatomy settings still require a large pouch. The reviewed native views retain
one continuous silhouette; the earlier separate lobes are absent. Cotton
ribbing and the existing grey-room lighting remain visible, especially in side
views. Moving dark bands also appear on an unlit fixed-color diagnostic,
establishing later native attenuation rather than a missing cloth surface. This
does not certify the accuracy of every shadow receiver. The speculative local
shadow bypass was removed; original material lighting is retained. The diagnostic
run is `20261010-114316-2d12aa` (25 flat and 25 normal-field samples). No stock
texture was changed.

## Physics and frame work

The covered contents use 120 Hz integration with the same 24 contact iterations
and the same 100 ms catch-up capacity. Naked mode retains 240 Hz. The accumulator
retains fractional time across changes. This changes numerical trajectories;
it is not an exact physics replay optimization.

Fine skin deformation runs on alternate rendered frames only while wearing
the jockstrap. The complete evaluator still runs on those updates, including
both body resources and the attachment. Dirty controls and active clinical
sequences bypass the cadence. Skeletal animation, input, physics and cloth
draws continue every frame. This is half the rendered cadence, not a guaranteed
30 Hz skin update: at 39 FPS, fine deformation is approximately 19.5 Hz. Human
motion smoothness is still an acceptance item. The native frame timing measures
throughput; it does not establish evenly spaced fine-deformation updates.

The support cache invalidates on all 20 shape/orientation/pressure inputs;
position does not affect directional support extents. Newton iteration exits
only when its next double is bit-identical to the current double. One million
support queries and 80,000 cache comparisons matched the original arithmetic.
CPU staging matched 72 complete geometry frames byte-for-byte across three
states, minimum/default/maximum size and moving inputs.

## Verification and rejected approaches

- Shared tests pass for the exact seam, rigid covariance, finite-input rejection,
  monotone material rows, covered simulation budget and cadence bypasses. The
  committed compatibility pin was compiled and tested independently.
- Four existing Meridian C++ tests, the attempt-guard test and binding checks
  passed during the preceding exact-work optimization. Their implementations
  remain available for baseline comparison, although the new pouch no longer
  uses the walking/following path.
- The sampled contact audit reports zero outside proxy samples in 14 poses
  repeated three times. It excludes the first two sewn display rows and checks
  cross-section polygons. It is not a continuous cloth/body triangle certificate.
- The 36-case physics sweep covers three states, size/width 1/50/100 and frame
  intervals of 16.67/22.22/33.33/50 ms. All captured values are finite. Minimum
  recorded pair gap is 0.02469 source units for 120 Hz and 0.02360 for 240 Hz;
  maximum tether ratios are 1.02693 and 1.03047 respectively. Those metrics
  cover this replay, not every possible contact or movement.
- Final candidate 16 has 22 native views, 49 movement captures and the complete
  20-case size/state/extreme-angle matrix, with no reported fit rejection.
  The settled measurement follows these sweeps; its 17 windows of 120 frames
  range from 25.001 to 25.817 ms per frame (window means, not individual peaks).
  Screenshot capture and zero rejection logs alone are not full visual acceptance.
  The original PNGs were reviewed for silhouette, visibility and obvious clipping.
- The shared-log reader now permits concurrent native writes and the capture
  checker requires a completed PNG signature/IEND. Two earlier matrices stopped
  because of log sharing or a lost log line although the native PNG existed; the
  final 20-case matrix completed. Every receipt records whether its log line exists.
- The old oval loft was rejected for ribs, lobes and excess volume. Removing
  hidden skin detail was rejected: exposed positions changed by up to 2.989
  source units. Serial geometry scheduling was slower and was also rejected.
- Full Base verification currently stops at motion-header extraction drift;
  earlier checks also recorded motion/CMake provenance drift. The unrelated
  sandbox recipe checker still reports `sealed_d3d9.hpp` hash drift. No unrelated
  hashes were refreshed to conceal these failures.

Full attachment acceptance is **false**. The native room does not establish all
supported LODs, campaign transitions, continuous triangle contact, close-up
attachment shading in every combination, or human interactive feel. Retain the
accepted retail DLL and rollback until these remaining gates pass.

## Build identity and adoption

Normal DLL:
`D:/MaleModBuilds/jockstrap-cage-20261010/production-original-lighting/d3d9.dll`

SHA256: `2c45bbf6e84f042bb7939b99fa477d6a0d965896ac994260f2d0466832a4b61d`.
Source HEAD is `c718b1b15cbe16ca1cb309e57770cca9ccfd244d` plus the explicit
runtime overlays in `production-original-lighting/provenance.json`. No private factory or
unrelated dirty worker changes are included. Licensed stock DDS maps were
hash-checked and embedded without changing their assets.

Final sealed native derivative:
`D:/MaleModBuilds/jockstrap-cage-20261010/candidate16/d3d9.dll`, SHA256
`1e1e020778b47d115cb2c28d2a6f167edeac5d4dd376cde13c01aa64b2ad8c45`.
Its geometry-validation run is `20261010-114753-6238ec` under the existing owned room32.
The sealed derivative is test-only and is never a retail payload.
It inherits sealed source `3fe6ddc13b700d0925531d2435ea1849e9e39f32` /
Base `608fdb18eb4a7570afb1fd6f777e2d7eb67c86d3`, plus the exact named
current overlays and cage/budget headers recorded in its provenance. The normal
build uses canonical HEAD and the pinned compatibility Base. The native result
is evidence for this sealed derivative, not complete binary parity of the normal
DLL across all campaign behavior.

Retail remains SHA256
`ab1f8d8e028a1239474465aa5f98faedea45860e1547aff21ad4db8d4087a1a7`.
The final owned run exited 0. The room DLL was restored to SHA256
`74ec982f1add7c5f0e053df938549eca6cbb645bcd0b42cba60b5745aa56b83f`;
all diagnostic request files are removed, and no host input/focus was used.
No push, release or retail installation was performed. Base's compatibility
commit is local; main working changes and Wolverine changes remain uncommitted.

`prepare_pouch_production.py` reproduces the normal build from canonical HEAD,
the working Base lock, named source overlays and hash-checked licensed DDSs.
`prepare_pouch_loft.py --cage --cached-extents --staged-geometry
--covered-cadence --canonical-cage` prepares the sealed comparison against an
explicit earlier sealed build. Use fresh output directories.

Other spokes must consume the Base pin and provide their own measured sewn
boundary and current supports in their own source units. Preserve their body
boundaries and aliases, then repeat source, offline, native and visual gates.
This work does not modify or claim a working Witcher adapter.
