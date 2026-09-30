# First extraction status

The source commit and original release tag are recorded in
`provenance/wolverine.json`. The source checkout was clean and matched
origin/main when imported. Wolverine and Witcher repositories and installed
games were not modified.

Imported: all tracked source/data/docs/tools except compiled linker/resource
artifacts (`.exp`, `.lib`, `.res`). Original AGENTS.md is archived under a
non-active name. Existing simulation/sequence code is retained byte-for-byte
as source reference; this extraction adds no new sequence behavior.

Exported: R14 coarse sculpt, support reference and reconstructed 35k final
reference topology with UVs, plus available geometry, morphology and physics
binding arrays. These are actual source data, not replacement sample meshes.
Source material maps are copied only when found locally and matching the
upstream manifest; their status appears in provenance.

Portable C++: math and the conservative area limiter. Full XPBD, seam/collar
evaluation, fluid compute and material rendering remain in the reference
runtime. No complete engine-independent runtime or working Witcher port is
claimed. Profiles expose known requirements rather than pretending that
engine or skeleton mappings are solved.

Unavailable from Git: live capture banks, full character skeleton/palettes,
some original authoring scripts, some build dependencies referenced by scripts,
release runtime DLL and payload. The include/build audit records missing
relative dependencies. Do not call the archived project reproducibly buildable
until these are recovered. Published verification reports are historical;
they are not test results for this new repository.
