# Compact authoritative mesh rebuild

Implemented: 35,000 final anatomy triangles; 32,000 supporting triangles; 17,528 final vertices. All runtime mappings and joined body/seam aliases are compacted together. The earlier `2ea2b7b` optimizations are retained.

Quality correction: preserve pressure-field support nodes and safety-sensitive returning strips. Removed unused smoothing rows without changing active stencils or their accumulation order; all 88 packed captures remain identical after this pruning.

The static/motion/pulse, sequence/hitch, collider/anchor and fluid budget checks pass. Geometry audit bounds meaningful reversals and separately reports tiny near-collinear normal changes. See README.md and the JSON reports for methods and limits.

Generated inputs, candidates, remaps and safety locks are offline only. The game uses the compact headers directly, with no high-resolution fallback or runtime decimator.

Built for review; prior automatic approval review blocked installation. The installed DLL has not been replaced.

Collar shading repair: preserve the joined mesh's reverse-to-render winding when rebaking anatomy faces. This fixes inward-facing collar normals without adding runtime work or triangles. 88 lighting captures pass packed-normal versus geometry agreement (>0.99 cosine), unchanged UV/skin weights/physics, and <0.000008-unit position drift. Up to six baseline normal sign differences persist in tiny folded collar neighborhoods and are recorded in lighting-audit.json. Built and installed after the user closed Wolverine; previous DLL backed up.

Upright support pass: tune the existing angular inertia multiplier 5 -> 1.5, orientation spring 10 -> 20000 and angular drag 11 -> 120. Fixed 240Hz stepping, 24 constraint iterations, collision queries, geometry and control settings stay unchanged. Two-constant spring changes alone were insufficient against packed contacts; the reduced angular impulse response is necessary. Six overturned-pose regressions recover without inverted settled samples (minimum upright agreement 0.994661), and 18 perched/launch/angle cases pass with peak speed 186.002. Built and installed with verified DLL SHA256 E6FB9F98FF814FC9C94C4494C992112641A0F4D6F554266DD1AD43344F69B882; previous DLL backed up. This is support tuning; the user's unconfirmed shading report remains a separate open issue.

Shaft/scrotum contact repair: movement fixture reproduced 37 triangle-pair crossings on 21 shaft faces in floppy mode. Rendered pure shaft/ventral radial extent measured ~3.9 vs collision mean radius ~2.9. Existing rod/body capsule scales now 1.20 + 0.20*min(j/4,1), and existing contact compliance 1e-7 instead of 1e-6. Same segment coverage, 24 iterations and contact routines. No geometry or triangles added. After identical settle/movement/settle input, all three modes show zero shaft/pouch-body edge-triangle crossings (shared root/neck and coplanar contact excluded). 18 perched/launch cases pass, peak speed 171.056. Six support inversion regressions pass; contact may tilt a support (minimum agreement 0.567062) but no settled sample crosses the suspension equator. Tests do not prove every live character pose. Built/installed verified SHA256 FED0695979D6DA9D0C7B21466FF9E1F36009E496687E89A552AA443737C1B71D, backup retained.
