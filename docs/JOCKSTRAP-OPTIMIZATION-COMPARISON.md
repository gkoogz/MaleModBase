# Jockstrap optimization and alternative comparison — October 10, 2026

The exact-work optimizations are implemented and built. The experimental oval
loft is implemented for comparison and rejected visually. Neither runtime was
installed into retail. This continues JOCKSTRAP-REGRESSION-AUDIT-20261010.md;
that document's audit-only statements describe the earlier checkpoint.

## Results

Private self-contained report (24 embedded native images, front/oblique/side,
default/large/maximum, firm state, post-walk and naked attachment reference):
`D:/MaleModBuilds/jockstrap-audit-fixes-20261010/report/Jockstrap-comparison.html`.
Machine-readable evidence is alongside it. Generated captures stay outside Git.

| Candidate | Mean fresh rebuild/fit | Outcome |
|---|---:|---|
| Previous source solver | 139.876 ms | 9/14 poses accepted |
| Optimized walking solver | 36.044 ms | Same outputs and rejection receipts |
| New 16-section oval loft | 1.068 ms | Produces all surfaces; no equivalent contact certificate; visually rejected |

The optimization saves 74.23% / 103.83 ms per rebuild on this bank (3.88x
throughput). The loft saves 97.09% against its separately paired optimized
baseline of 36.649 ms. This is CPU fitting time, not whole-game FPS. Each test
used the same 14 captured 64x40 poses, x86 MSVC /O2, one warm-up per pose,
three evaluations per pose and three alternating trial pairs. Final replays ran
after the owned game closed. Every optimized trial matched output SHA-256
`31ab1b4782c70da90151ebf0311b4b57afd9f9d31e1c8028e829a62e0d80f653`.
The prior source baseline already includes improvements over installed retail.

Native preparation windows averaged 19.114 ms optimized and 1.635 ms loft;
worst calls were 164.307 and 9.644 ms. These are descriptive changing-control
runs, not controlled FPS measurements. Optimized logs contain 193 rejected
calls and 254 suppressed duplicate hooks across nine 120-call windows. The
loft's thirteen windows had neither; it also lacks the old contact certificate.

## Implementation

Base taut_contact.hpp caches identical refinement support routes, postpones
the unchanged seed clearance until retry, and uses an early-exit threshold
predicate for final certification. The random-triangle regression checks
predicate equivalence, empty support and nonfinite rejection. Numerical order
and accepted geometry are preserved on the recorded replay bank.

Wolverine meridian_attempt.h suppresses repeated failed solves only for equal
actual posed inputs in the same frame/control epoch. Changed poses may retry;
device reset and upload failures reset the guard. Preparation timing now
includes failures and duplicate input sampling. Earlier current-source follow
fairing removal, unused donor skipping, support bounds and futile fixed-seam
retry removal are retained.

The experimental Base pouch_loft.hpp builds 16 elliptical sections from current
support samples and interpolates the dense display surface, retaining the sewn
vertices. It avoids iterative contact projection. It is NOT selected in the
normal adapter: tools/iteration/prepare_pouch_loft.py creates an explicit private
experimental build. The regular-column revision still shows protruding lobes,
ribbed/disc-like forms in side views, and excessive length/volume at high sizes.
Front views alone would conceal these defects. Existing solver views retain a
broad hanging panel/wedge; arithmetic optimization does not improve that shape.

Recommendation: retain the exact-work improvements, reject the oval prototype.
A smooth pre-shaped low-resolution pouch cage driven by a few anatomical
landmarks, with local contact correction, is a better next design hypothesis
than independently enclosing every anatomical slice. It is not implemented or
validated as a third candidate. Keep dense rendering independent of collision
resolution. No reason was found to simulate thousands of independent contacts
just to produce the requested approximate conforming silhouette.

## Verification and adoption

Four x86 meridian tests, attempt-gate test, loft finite/seam/translation test,
meridian binding verification and relevant header hashes passed. Production
DLL compiled. Full Base verification stops at pre-existing motion/calibration
CMakeLists.txt provenance drift. These narrower results do not make it pass.
Sandbox recipe verification also stops at sealed_d3d9.hpp hash drift, confirmed
to exist in committed Wolverine HEAD before these changes. New comparison
tools have their own LF hashes in the recipe inventory; unrelated hashes were
not rewritten to conceal the existing discrepancy.
The prototype's tests do not certify continuous contact or good appearance.

Native runs used the owned grey room, no host input/focus, 1280x960 and matching
timed camera/control recipes. Angles and animation phases are approximate,
not registered pairs. Optimized run: 20261010-095734-dda88d; final loft run:
20261010-095959-f49006. Both exited zero. Native attachment references and brief
movement were reviewed; full size/control/mechanical-state coverage, extreme
angles, both resource/LOD coverage and campaign acceptance remain incomplete.
The full attachment gate is NOT passed. The report's browser preview was
blocked by file-URL policy; embedded images and mappings were checked from source.

Wolverine main source c718b1b15cbe16ca1cb309e57770cca9ccfd244d plus recorded
adapter/attempt overlays consumes local Base pin
bd7c4a77387b3f340c1ee2b3aa4a63300519d922 (codex/jockstrap-audit-perf).
That compatibility commit is based on its prior adapter pin and avoids unrelated
Base working changes. Managed worktree remains at
`C:/Users/Administrator/.codex/worktrees/jockstrap-audit-perf/MaleMod`.
It is local/unpushed; other spokes must adopt the shared revision explicitly
and rerun their own binding/native gates. No other spoke adoption is claimed.

Production build: `D:/MaleModBuilds/jockstrap-audit-fixes-20261010/production`;
DLL SHA-256 `9d23137b8cc279be1152942cc05a31491d6f4854c963762222d4233ff4432262`.
This normal build excludes private factory and unrelated dirty worker code.
Runtime overlay hashes are in its provenance.json. Private native candidate
identities, source snapshots and final paired trials remain in the same parent.
Retail DLL remains `ab1f8d8e028a1239474465aa5f98faedea45860e1547aff21ad4db8d4087a1a7`.
No install, release or push. Existing unrelated dirty work remains intact.
The reused private room DLL was restored byte-identically to prior-room.dll
after both owned runs exited. Wolverine main remains 68 commits ahead of the
fetched origin; Base main is 41 ahead of its existing tracking ref. Base was
not fetched in this task. Main working changes remain uncommitted; the isolated
compatibility commit above is local only.
