# Compact raised root ramp

The requested change is a small continuous radial flare from the retained pelvic
surface into the shaft. It must scale with the attachment, retain the surrounding
body shape, and preserve the separately repaired suspension materials and motion.

The existing stitch uses a Hermite annulus between the measured pelvic cut and
the complete shaft root. Its common derivative limit prevents adjacent narrow
columns from crossing, but can also leave a very flat transition. Removing that
limit would reintroduce folds. This change keeps the bound and supplies a shallow
forward shoulder inside the annulus instead.

Base revision `fcb98a4e6386aa70606c31f24a1b1046ef10eeae` changes the measured
aperture margin from 0.12 to 0.18 of the reference radial size; the existing 0.08
underside allowance remains. Both initial stitching and live reposing use
`RaisedRampPoint`. The forward offset is `height * 64*t^3*(1-t)^3`, where height
is the harmonic combination of 0.08 times the local radial size and 0.18 times
the available forward clearance. Thus it shrinks with the model and with limited
clearance instead of applying a fixed displacement. The added term has zero
value, first derivative and second derivative at both joined edges.

This raises only the joining annulus. Original body vertices, source-edge donors,
waist boundaries, shaft circumference, UV aliases, texture packages, suspension
assets and numerical worker inputs are retained. The small footprint expansion
recruits additional local pelvic triangles; it does not inflate the entire pelvis.

Eight shared C++ tests pass, including scale invariance, exact boundary positions,
boundary differential preservation, bounded forward offset, donor reconstruction
and stitched manifoldness. All32 previously captured representative poses replay
with unchanged original body coordinates, no reversed ramp faces, no nonmanifold
edges, and the original78 external boundary edges. These are source/offline
checks, not native visual acceptance.

Native candidate evidence is under
`D:/MaleModBuilds/modular-anatomy-20261015/root-ramp-20261019/candidate-01`.
The Witcher adapter consumes the pinned Base headers; only three of114 recorded
compiler inputs differ from the previous installed build. Coupled asset and
material hashes are unchanged. Its handoff records final native review,
installation identity and rollback separately.

Other spokes must adopt this Base revision through their dependency pin, retain
their measured body cuts and original boundary donors, then run their own native
attachment gate across controls, states, motion, body resources, LODs and campaign
scenes. No other game is modified by the Witcher adoption. This is a local
development runtime with archived compiler inputs, not a clean-checkout release.

The Witcher adoption `b977c39` was installed after252 native captured stages:
246 usable views reviewed, with six wall-occluded views replaced by a dedicated
nine-view distance run. All68 new captured-pose audits pass. Representative
minimum/default/maximum/intermediate sizes, angle extremes, three mechanical
states, Hang, motion, both body resources, supported LODs and one campaign scene
were covered. This is finite coverage, not every control combination or scene.
Existing extreme distal shape defects remain outside this local root change.

Installed DLL SHA256:
`fd95ca61331a983788208480b9ecd372b16d61d24751a3274eaaf64c240b0935`.
Installed startup/walking was observed;53 managed and31 protected files verify.
The Steam grey-room preflight passes. Rollback is in the Witcher adapter at
`local/native-backup-modular-20261019-114157`.
