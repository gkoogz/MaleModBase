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

Portable C++: math, the conservative area limiter, clinical timing, CPU fluid
simulation, mesh generation, deposition and audio cue scheduling. The existing
sequence is usable through a per-character Session and adapter callbacks; see
CLINICAL-SEQUENCE.md and provenance/clinical.json. Full anatomy XPBD, seam/collar
evaluation, alternate GPU particle compute and material rendering remain in the
reference runtime. No complete engine-independent anatomy runtime or Witcher port is
claimed. Profiles expose known requirements rather than pretending that
engine or skeleton mappings are solved.

The generic framework adds a metric skinned proxy, body preference controls,
semantic sockets, linear skinning, capsule projection and an instance-owned
fixed-step chain using extracted distance/bend kernels. Kernel parity is tested;
full authored solver parity and live engine integration are still pending.

The final unified collar metric is extracted as standard C++ and its coupled
energy/hard edge constraints are available in a per-character offline evaluator.
Source arithmetic, sparse/dense agreement, expansion and seam checks pass;
complete posed runtime parity remains pending. See PELVIC-COLLAR.md.

Unavailable from Git: live capture banks, full character skeleton/palettes,
some original authoring scripts, some build dependencies referenced by scripts,
release runtime DLL and payload. The include/build audit records missing
relative dependencies. Do not call the archived project reproducibly buildable
until these are recovered. Published verification reports are historical;
they are not test results for this new repository.
