param([switch]$Upgrade,[Parameter(Mandatory=$true)][string]$OriginalNatural,[Parameter(Mandatory=$true)][string]$OriginalWGame,[string]$ModifiedNatural,[Parameter(Mandatory=$true)][string]$OriginalWStart,[string]$ModifiedWGame,[string]$ModifiedWStart)
$ErrorActionPreference='Stop'
$repo=Split-Path $PSScriptRoot -Parent
$base=Join-Path ([IO.Path]::GetTempPath()) ('release-20-beta-test-'+[guid]::NewGuid().ToString('N'))
$game=Join-Path $base 'Game with spaces';$docs=Join-Path $base 'Documents'
foreach($d in @('Binaries','WGame/CookedPC','WGame/Config')){New-Item -ItemType Directory (Join-Path $game $d) -Force|Out-Null}
New-Item -ItemType Directory (Join-Path $docs 'Wolverine/WGame/Config') -Force|Out-Null
$source=if($Upgrade){$ModifiedNatural}else{$OriginalNatural}
Copy-Item $source (Join-Path $game 'WGame/CookedPC/CH_Wolverine_Natural_SF.xxx')
Copy-Item $(if($Upgrade){$ModifiedWGame}else{$OriginalWGame}) (Join-Path $game 'WGame/CookedPC/WGame.xxx')
Copy-Item $(if($Upgrade){$ModifiedWStart}else{$OriginalWStart}) (Join-Path $game 'WGame/CookedPC/WStart.xxx')
Set-Content (Join-Path $game 'Binaries/Wolverine.exe') 'fixture only'
Set-Content (Join-Path $game 'Binaries/d3d9.dll') 'prior proxy'
Set-Content (Join-Path $game 'Binaries/R14-skin-natural.png') 'prior texture'
New-Item -ItemType Directory (Join-Path $game 'Binaries/WolverineIdle') -Force | Out-Null
Set-Content (Join-Path $game 'Binaries/WolverineIdle/standard-01.wav') 'prior idle audio'
Set-Content (Join-Path $game 'Binaries/WolverineLive.ini') '[Shape] Overall=57'
Set-Content (Join-Path $game 'WGame/Config/DefaultCheckpoints.ini') 'mOutfit=CH_Wolverine_Tank'
Set-Content (Join-Path $docs 'Wolverine/WGame/Config/WCheckpoints.ini') 'mOutfit=CH_Wolverine_Tank'
Set-Content (Join-Path $game 'Binaries/R14-skin-natural.dds') 'prior texture cache'
Set-Content (Join-Path $game 'Binaries/SharedBody-normal.dds') 'prior normal map'
if($Upgrade){Copy-Item (Join-Path $repo 'payload/TankCameraPresets.txt') (Join-Path $game 'Binaries/WolverineLive.log')}
$before=@{};Get-ChildItem $base -Recurse -File|ForEach-Object {$before[$_.FullName]=(Get-FileHash $_.FullName).Hash}
# This fixture uses a text placeholder executable and never the live game.
# Mock only the process guard so fixture checks can run during gameplay.
if((Get-Content (Join-Path $game 'Binaries/Wolverine.exe') -Raw).Trim() -ne 'fixture only'){throw 'Not a fixture executable'}
function Get-Process {
 [CmdletBinding()]param([string]$Name)
 if($Name -ne 'Wolverine'){throw 'Unexpected fixture process query'}
}
& (Join-Path $repo 'Install.ps1') -GamePath $game -DocumentsPath $docs
$m=Get-Content (Join-Path $repo 'manifest.json') -Raw|ConvertFrom-Json
foreach($name in @('d3d9.dll','MenuTank-skin-blend33.png','MenuTank-skin-blend67.png','R14-skin-natural.dds','R14-skin-erect.dds','SharedBody-normal.dds','SharedBody-specular.dds')){if((Get-FileHash (Join-Path $game ('Binaries/'+$name))).Hash -ne $m.payload.$name){throw 'Payload mismatch'}}
if((Get-FileHash (Join-Path $game 'Binaries/TeachingFluid.ini')).Hash -ne $m.payload.'TeachingFluid.ini'){throw 'Default fluid config payload mismatch'}
foreach($clip in $m.idleClips.PSObject.Properties){if((Get-FileHash (Join-Path $game ('Binaries/WolverineIdle/'+$clip.Name))).Hash -ne $clip.Value){throw "Idle clip mismatch: $($clip.Name)"}}
if((Get-FileHash (Join-Path $game 'WGame/CookedPC/WGame.xxx')).Hash -ne $m.installedWGameSHA256){throw 'WGame mismatch'}
if((Get-FileHash (Join-Path $game 'WGame/CookedPC/CH_Wolverine_Natural_SF.xxx')).Hash -ne $m.installedPackageSHA256){throw 'Natural mismatch'}
if((Get-FileHash (Join-Path $game 'WGame/CookedPC/WStart.xxx')).Hash -ne $m.installedWStartSHA256){throw 'WStart mismatch'}
if(-not(Select-String -LiteralPath (Join-Path $game 'Binaries/TankCameraPresets.txt') -Pattern '^TANK CAMERA POSE ' -Quiet)){throw 'Camera presets missing'}
& (Join-Path $repo 'Install.ps1') -Mode Uninstall -GamePath $game -DocumentsPath $docs
foreach($p in $before.Keys){if((Get-FileHash $p).Hash -ne $before[$p]){throw "Restore mismatch $p"}}
if(Test-Path (Join-Path $game 'Binaries/R14-skin-erect.dds')){throw 'New DDS not removed'}
if(Test-Path (Join-Path $game 'Binaries/SharedBody-specular.dds')){throw 'New shared specular map not removed'}
if(Test-Path (Join-Path $game 'Binaries/R14-skin-erect.png')){throw 'New texture not removed'}
if(Test-Path (Join-Path $game 'Binaries/WolverineIdle/custom-01.wav')){throw 'New idle clip not removed'}
if(Test-Path (Join-Path $game 'Binaries/TeachingFluid.ini')){throw 'Default fluid config not removed on exact rollback'}
"PASS: upgrade=$Upgrade reconstruction, install, existing texture backup, settings and exact restoration. $base"
$configGame=Join-Path $base 'Game with custom fluid config'
foreach($d in @('Binaries','WGame/CookedPC','WGame/Config')){New-Item -ItemType Directory (Join-Path $configGame $d) -Force|Out-Null}
Set-Content (Join-Path $configGame 'Binaries/Wolverine.exe') 'fixture only'
Copy-Item $source (Join-Path $configGame 'WGame/CookedPC/CH_Wolverine_Natural_SF.xxx')
Copy-Item $(if($Upgrade){$ModifiedWGame}else{$OriginalWGame}) (Join-Path $configGame 'WGame/CookedPC/WGame.xxx')
Copy-Item $(if($Upgrade){$ModifiedWStart}else{$OriginalWStart}) (Join-Path $configGame 'WGame/CookedPC/WStart.xxx')
Set-Content (Join-Path $configGame 'Binaries/d3d9.dll') 'prior proxy'
Set-Content (Join-Path $configGame 'Binaries/WolverineLive.ini') '[Shape] Overall=57'
Set-Content (Join-Path $configGame 'Binaries/TeachingFluid.ini') '[Fluid]','Volume=74','Duration=2.25'
Set-Content (Join-Path $configGame 'Binaries/TankCameraPresets.txt') 'existing user camera settings'
$cameraBefore=(Get-FileHash (Join-Path $configGame 'Binaries/TankCameraPresets.txt')).Hash
$fluidBefore=Get-FileHash (Join-Path $configGame 'Binaries/TeachingFluid.ini') -Algorithm SHA256
& (Join-Path $repo 'Install.ps1') -GamePath $configGame -DocumentsPath $docs
if((Get-FileHash (Join-Path $configGame 'Binaries/TeachingFluid.ini') -Algorithm SHA256).Hash -ne $fluidBefore.Hash){throw 'Existing custom fluid config was overwritten'}
& (Join-Path $repo 'Install.ps1') -Mode Uninstall -GamePath $configGame -DocumentsPath $docs
if((Get-FileHash (Join-Path $configGame 'Binaries/TeachingFluid.ini') -Algorithm SHA256).Hash -ne $fluidBefore.Hash){throw 'Existing custom fluid config was not preserved on uninstall'}
if((Get-FileHash (Join-Path $configGame 'Binaries/TankCameraPresets.txt')).Hash -ne $cameraBefore){throw 'Existing camera settings changed'}
'PASS pre-existing fluid and camera settings preserved across install and rollback.'
# Reject an unsupported game package before creating a backup or replacing files.
$badGame=Join-Path $base 'Unsupported package game'
foreach($d in @('Binaries','WGame/CookedPC')){New-Item -ItemType Directory (Join-Path $badGame $d) -Force|Out-Null}
Set-Content (Join-Path $badGame 'Binaries/Wolverine.exe') 'fixture only'
Set-Content (Join-Path $badGame 'Binaries/d3d9.dll') 'do not replace'
Set-Content (Join-Path $badGame 'WGame/CookedPC/CH_Wolverine_Natural_SF.xxx') 'unsupported package'
$beforeHash=(Get-FileHash (Join-Path $badGame 'Binaries/d3d9.dll')).Hash
$rejected=$false
try{& (Join-Path $repo 'Install.ps1') -GamePath $badGame -DocumentsPath $docs}catch{if($_.Exception.Message -notlike 'Unsupported package:*'){throw};$rejected=$true}
if(-not $rejected){throw 'Unsupported package was accepted'}
if((Get-FileHash (Join-Path $badGame 'Binaries/d3d9.dll')).Hash -ne $beforeHash){throw 'Unsupported package changed DLL'}
if(Test-Path (Join-Path $badGame 'WGame/ModBackups')){throw 'Unsupported package created installation backup'}
'PASS unsupported package rejected before writes.'
