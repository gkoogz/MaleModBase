# Wolverine Anatomy Tool 2.0 Beta 1

This is the complete beta installation package: installer, game patches, textures, public default audio, runtime and installer validation tools. Development source/authoring tools are available through GitHub and the separate optional Source ZIP. The final anatomy render/fluid collision skin uses 35,000 triangles, with remapped deformation/collision references and reinforced existing support/contact parameters.

Close Wolverine, extract the entire -Install.7z archive with 7-Zip and run Install.cmd or Upgrade-Beta-2.0.cmd. The default location is C:\Games\X-Men Origins Wolverine. For another location, run `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Install.ps1 -GamePath "D:\Your Game Folder"`. Supported stock or exact approved Natural/WGame/WStart assets are accepted; unrelated package modifications are rejected before installation writes. No prior mod release is required.

Rollback-Beta-2.0.cmd or Uninstall.cmd restores pre-install files from WGame/ModBackups/WolverineAnatomyTool-v2.0.0-beta.1. Pass the same GamePath for a custom installation. Backups are never overwritten. Existing shape/fluid settings, camera poses and local TeachingAudio remain. Published default idle clips may be replaced with backups; private recordings are not bundled. Camera defaults/migrated poses remain after rollback.

F6 opens controls. J starts/cancels the teaching sequence. See releases/2.0.0-beta.1.md for changes, validation and known live-visual limitations. Offline CPU timings are not game FPS guarantees.

The install archive retains the original full-resolution DDS skin textures and their precomputed mipmaps, menu PNG blend textures, shared normal/specular maps, game deltas, DLL and public audio. Redundant natural/erect PNG fallback copies and developer-only source/fixtures are omitted from the install download. Texture pixels, DDS mipmaps, loading path, runtime DLL, mesh and physics are unchanged. Existing fallback PNGs in older installations are left alone. The optional Source ZIP is not needed to install or play.

Build with build.cmd using x86 MSVC and the June 2010 DirectX SDK. tools/Test-ReleaseBeta20.ps1 validates clean installation, prior-version upgrade, settings preservation and exact rollback against supplied local game-package fixtures. manifest.json identifies every runtime payload by SHA-256. Package verification and size accounting are in docs/RELEASE-2.0-BETA-PACKAGING.md.

This is an illustrative speculative-biology tool without clinical calibration. No full game packages, private study audio, personal settings, backups or raw session logs are included.