# Validation of the initial extraction

Passed locally:

- All 516 imported source/data files match source SHA-256, including the staged
  Git object bytes. No imported file was omitted by nested ignore rules.
- Six material maps match the published release manifest.
- All 185 exported literal source arrays match their decoded originals.
- Three OBJ exports match manifest hashes and counts; every triangle references
  an existing vertex. The final reference is 17,528 vertices / 35,000 triangles.
- Six Python tests cover fitting, reflected winding with UV indices, invalid
  transforms, overwrite rejection and safe numerical table parsing.
- Standalone C++ limiter compiled with MSVC C++17, without Direct3D or a game
  SDK; 2,000 randomized corrections passed sampled continuous area bounds and
  prepared-evaluation parity.

`templates/github-actions-verify.yml` supplies Python verification and a CMake
C++ test on Linux. GitHub rejected creating an active workflow because the
current OAuth login lacks workflow scope, so the template is not active CI.
Local MSVC verification does not establish that Linux CI has passed. Full
source runtime, material appearance, animation, entire physics solver and
Witcher integration were not tested in this extraction.
