# Local repository workflow

Canonical checkout: C:/Users/Administrator/Documents/Codex/2026-09-28/hello-https-github-com-gkoogz-xmenoriginswolverinemalemod/work/installed-baseline-20260929
Remote: https://github.com/gkoogz/XMenOriginsWolverineMaleMod
Working branch: main, tracking origin/main. Authentication uses the signed-in GitHub CLI; no access tokens are stored in project files. Git author identity is the account's GitHub no-reply address.

The desktop shortcut opens this exact checkout. Source/build/install work belongs here. Other work folders are historical references, not separate active projects. Build output is build/d3d9.dll. Installed DLL: C:/Games/X-Men Origins Wolverine/Binaries/d3d9.dll.

From this checkout:

```powershell
git status --short --branch
git fetch origin --tags
git pull --ff-only
git add <specific-files>
git commit -m "Describe the change"
git push origin main
git log --oneline -10
git revert <commit>
```

Fetch and inspect before pulling. A dirty or diverged checkout requires preserving/reconciling changes; do not use reset --hard or force pushes as routine synchronization. Revert creates a new undo commit; rebuilding/installing the resulting source is a separate action. Reverting source alone does not change the game binary. A release is a tested/tagged snapshot and full downloadable package, not an automatic consequence of a source push.

The 2.0 beta already contains the currently installed runtime. Bringing main up to date must preserve the published beta rather than recreate it. The older upstream and current development histories are joined with a merge retaining the current development tree; old sources/assets remain accessible through historical commits/tags. This avoids force-pushing main and reintroducing obsolete runtime files.

Large reusable release payloads are obtained from the full GitHub release, verified against manifest.json, and kept locally in ignored payload/. Generated build inputs and current runtime geometry are tracked. Build outputs, release archives, captured game files, personal audio, settings and logs are not tracked. Use tools/Get-ReleasePayload.ps1 to obtain the manifest-matching payload when setting up a fresh checkout.

Future agents must follow AGENTS.md, keep docs/PROJECT_HANDOFF.md current and distinguish source, built DLL, installed DLL and published release. Before saying synchronized, confirm a clean working tree, equal local/upstream commit IDs and matching installed runtime hash. User settings are intentionally outside version control.
Compact releases: download the -Install.7z for users; source/authoring files are separate and optional. Get-ReleasePayload.ps1 supports the current 7z payload and legacy ZIP input and verifies all manifest assets before copying. The two optional state PNG fallbacks are no longer required manifest assets; existing local copies are preserved.

