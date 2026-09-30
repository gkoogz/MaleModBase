# 2.0 Beta packaging

The initial DLL-only upload was incomplete and has been replaced with the established full release format. Runtime source and binary are unchanged by the packaging correction. All published 1.9 asset hashes were verified before reuse, and installed Natural/WGame/WStart packages and body textures match those assets exactly.

Included: current d3d9.dll, three WBX deltas, two PNG and two DDS state-specific whole-body skin textures, two menu blend textures, shared normal/specular DDS maps, 22 public default idle WAV clips, camera/fluid defaults, checksum manifest, installer, install/upgrade/uninstall/rollback launchers, current runtime source, build commands and regression tools. Excluded: complete game packages, private recordings, user settings/backups, raw logs, capture banks, unrelated previous releases and obsolete 1.9 runtime source.

Install.ps1 checks all payload/audio checksums and supported game-package input hashes before replacing files. Backups and installed hashes are recorded under a version-specific directory. Rollback verifies original backup hashes and the installed state before restoration. Custom fluid/shape/camera settings are preserved. Newly created camera defaults remain after rollback by design.

The full 1.9 ZIP measured 182.16 MiB (191 MB decimal). Approximate compressed contributors: game deltas 32.15 MiB; two skin DDS files 39.71 MiB; four PNG files 45.23 MiB; DLL 15.30 MiB; normal/specular DDS 9.39 MiB. The remainder includes public audio, source/data and fixtures. Each skin DDS occupies 85.33 MiB uncompressed and includes precomputed mipmaps. PNGs are runtime fallbacks, so carrying both has download cost but preserves existing loading behavior. Texture consolidation/resolution changes are not part of this runtime-preserving release correction.

Installer validation: clean stock-package installation and exact approved-patched-package upgrade, reconstructed output hashes, every installed payload hash, prior-file restoration, removal of newly created payloads, and existing fluid/camera settings preservation. Stock fixtures were recovered locally by reversing the released XOR deltas against hash-verified installed packages and then verified against all three original manifest hashes. No stock fixtures or full game packages are distributed.
Validation passed on 2026-09-29: clean-install and complete patched-package upgrade fixtures, exact rollback, custom fluid/camera settings preservation, and unsupported package rejection before writes. Runtime DLL SHA-256: FED0695979D6DA9D0C7B21466FF9E1F36009E496687E89A552AA443737C1B71D. No runtime code changed during packaging.


## Compact installation revision

The original 190.85 MB ZIP combined 152.88 MB of runtime payload and about 38 MB of development source/data/tools. The required install archive now excludes developer-only data and the two natural/erect PNG fallback copies (about 24.4 MB compressed). The existing native DDS-first load path uses unchanged DDS files; all resolutions/mipmaps and pixels remain intact. This is a packaging change, with no runtime rebuild or in-game cost. Menu PNG blend files remain required and included. The 34-file manifest still includes DLL, all required maps/textures, three WBX deltas, 22 public audio files and fluid/camera defaults.

Current source/authoring/regression files are delivered separately as an optional Source ZIP and remain on GitHub. The installer and its codec/regression are included in the install archive. Users extract the -Install.7z archive with 7-Zip and then use the same Install/Upgrade/Rollback launchers. No game installation or 1.9 download is needed beyond the supported stock game. The compact package passed the clean stock, approved-patched upgrade, rollback, settings-preservation and unsupported-input fixtures.

Measured compact install archive: 89,202,254 bytes (89.20 MB), down from 190,846,800 bytes: 53.26% smaller. All 34 manifest payload hashes verified after extraction with installed 7-Zip 21.04; complete archive integrity passed with 7-Zip 26.02. Installer fixtures passed against the compact staged file set. Runtime/installed DLL SHA-256 remains FED0695979D6DA9D0C7B21466FF9E1F36009E496687E89A552AA443737C1B71D.

