#requires -Version 5.1
[CmdletBinding()]
param([ValidateSet('Install','Uninstall')][string]$Mode='Install',[string]$GamePath='C:\Games\X-Men Origins Wolverine',[string]$DocumentsPath=[Environment]::GetFolderPath('MyDocuments'))
$ErrorActionPreference='Stop'
Set-StrictMode -Version 2
$GamePath=[IO.Path]::GetFullPath($GamePath.Trim('"')).TrimEnd('\')
$manifest=Get-Content (Join-Path $PSScriptRoot 'manifest.json') -Raw | ConvertFrom-Json
function Hash($p){(Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash}
function WriteFile($p,$bytes,$readOnly){
 New-Item -ItemType Directory -Path (Split-Path -Path $p -Parent) -Force | Out-Null
 if(Test-Path -LiteralPath $p){(Get-Item -LiteralPath $p).IsReadOnly=$false}
 [IO.File]::WriteAllBytes($p,$bytes)
 (Get-Item -LiteralPath $p).IsReadOnly=$readOnly
}
function Restore($records){
 foreach($r in $records){
  if($r.Existed){WriteFile $r.Target ([IO.File]::ReadAllBytes($r.Backup)) ([bool]$r.ReadOnly)}
  elseif(Test-Path -LiteralPath $r.Target){(Get-Item -LiteralPath $r.Target).IsReadOnly=$false;Remove-Item -LiteralPath $r.Target}
 }
}
if(-not(Test-Path -LiteralPath (Join-Path $GamePath 'Binaries\Wolverine.exe'))){throw 'Select the folder containing Binaries\Wolverine.exe.'}
if(Get-Process -Name Wolverine -ErrorAction SilentlyContinue){throw 'Close Wolverine before installing or restoring.'}
$backupRoot=Join-Path $GamePath 'WGame\ModBackups\WolverineAnatomyTool-v2.0.0-beta.1'
$statePath=Join-Path $backupRoot 'state.json'
if($Mode -eq 'Uninstall'){
 $state=Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
 if($state.GamePath -ne $GamePath -or $state.Version -ne '2.0.0-beta.1'){throw 'Backup identity mismatch.'}
 if($state.Status -eq 'Uninstalled'){Write-Output '2.0 Beta 1 is already restored.';return}
 foreach($r in $state.Files){
  if($r.Existed -and (Hash $r.Backup) -ne $r.OriginalHash){throw "Backup mismatch: $($r.Backup)"}
  if(-not(Test-Path -LiteralPath $r.Target) -or (Hash $r.Target) -ne $r.InstalledHash){throw "Installed file changed: $($r.Target)"}
 }
 Restore $state.Files
 $state.Status='Uninstalled';$state|ConvertTo-Json -Depth 8|Set-Content $statePath -Encoding UTF8
 Write-Output 'Previous files restored. Saved settings preserved.';return
}
if(Test-Path -LiteralPath $statePath){throw 'A 2.0 Beta 1 backup already exists. Restore it before another installation; retained backups are never overwritten.'}
foreach($p in $manifest.payload.PSObject.Properties){if((Hash (Join-Path $PSScriptRoot ('payload\'+$p.Name))) -ne $p.Value){throw "Payload checksum mismatch: $($p.Name)"}}
if(@($manifest.idleClips.PSObject.Properties).Count -ne 22){throw 'Idle clip manifest must list all 22 WAV files.'}
foreach($p in $manifest.idleClips.PSObject.Properties){if((Hash (Join-Path $PSScriptRoot ('payload\WolverineIdle\'+$p.Name))) -ne $p.Value){throw "Idle clip checksum mismatch: $($p.Name)"}}
if(-not('WolverinePatchCodec' -as [type])){Add-Type -Path (Join-Path $PSScriptRoot 'tools\PatchCodec.cs')}
$stage=Join-Path ([IO.Path]::GetTempPath()) ('WolverineRelease20Beta-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory $stage|Out-Null
$changes=@()
try{
 foreach($spec in @(
  @('WGame\CookedPC\CH_Wolverine_Natural_SF.xxx',$manifest.sourcePackageSHA256,$manifest.installedPackageSHA256,'Natural.wbx'),
  @('WGame\CookedPC\WGame.xxx',$manifest.sourceWGameSHA256,$manifest.installedWGameSHA256,'WGame.wbx'),
  @('WGame\CookedPC\WStart.xxx',$manifest.sourceWStartSHA256,$manifest.installedWStartSHA256,'WStart.wbx')
 )){
  $target=Join-Path $GamePath $spec[0];$current=Hash $target
  if($current -eq $spec[2]){continue}
  if($current -ne $spec[1]){throw "Unsupported package: $($spec[0]). No files changed."}
  $temp=Join-Path $stage ([IO.Path]::GetFileName($target))
  [WolverinePatchCodec]::Apply($target,(Join-Path $PSScriptRoot ('payload\'+$spec[3])),$temp)
  if((Hash $temp) -ne $spec[2]){throw 'Reconstructed package checksum mismatch.'}
  $changes+=@{Target=$target;Bytes=[IO.File]::ReadAllBytes($temp)}
 }
 foreach($name in @('d3d9.dll','MenuTank-skin-blend33.png','MenuTank-skin-blend67.png','R14-skin-natural.dds','R14-skin-erect.dds','SharedBody-normal.dds','SharedBody-specular.dds')){
  $changes+=@{Target=(Join-Path $GamePath ('Binaries\'+$name));Bytes=[IO.File]::ReadAllBytes((Join-Path $PSScriptRoot ('payload\'+$name)))}
 }
 foreach($p in $manifest.idleClips.PSObject.Properties){
  $changes+=@{Target=(Join-Path $GamePath ('Binaries\WolverineIdle\'+$p.Name));Bytes=[IO.File]::ReadAllBytes((Join-Path $PSScriptRoot ('payload\WolverineIdle\'+$p.Name)))}
 }
 $fluidConfig=Join-Path $GamePath 'Binaries\TeachingFluid.ini'
 if(-not(Test-Path -LiteralPath $fluidConfig)){
  $changes+=@{Target=$fluidConfig;Bytes=[IO.File]::ReadAllBytes((Join-Path $PSScriptRoot 'payload\TeachingFluid.ini'))}
 }
 $encoding=[Text.Encoding]::GetEncoding(28591)
 foreach($p in @((Join-Path $GamePath 'WGame\Config\DefaultCheckpoints.ini'),(Join-Path $DocumentsPath 'Wolverine\WGame\Config\WCheckpoints.ini'))){
  if(Test-Path -LiteralPath $p){
   $old=$encoding.GetString([IO.File]::ReadAllBytes($p));$new=[regex]::Replace($old,'(?m)^([ \t]*mOutfit[ \t]*=[ \t]*)CH_Wolverine_[A-Za-z0-9_]+([ \t]*\r?)$','${1}CH_Wolverine_Natural${2}')
   if($new -ne $old){$changes+=@{Target=$p;Bytes=$encoding.GetBytes($new)}}
  }
 }
 New-Item -ItemType Directory $backupRoot -Force|Out-Null
 $records=@();$i=0
 foreach($c in $changes){
  $exists=Test-Path -LiteralPath $c.Target;$backup=Join-Path $backupRoot ('original-'+$i);$i++
  $hash='';$ro=$false
  if($exists){Copy-Item -LiteralPath $c.Target -Destination $backup;$hash=Hash $backup;$ro=(Get-Item -LiteralPath $c.Target).IsReadOnly}
  $sha=[Security.Cryptography.SHA256]::Create();$installed=[BitConverter]::ToString($sha.ComputeHash($c.Bytes)).Replace('-','');$sha.Dispose()
  $records+=@{Target=$c.Target;Backup=$backup;Existed=$exists;OriginalHash=$hash;ReadOnly=$ro;InstalledHash=$installed}
 }
 $state=@{Version='2.0.0-beta.1';GamePath=$GamePath;Status='Pending';Files=$records}
 $state|ConvertTo-Json -Depth 8|Set-Content $statePath -Encoding UTF8
 try{
  for($i=0;$i -lt $changes.Count;$i++){WriteFile $changes[$i].Target $changes[$i].Bytes ([bool]$records[$i].ReadOnly);if((Hash $changes[$i].Target) -ne $records[$i].InstalledHash){throw 'Installation verification failed.'}}
  # Camera presets are retained user settings, separate from session diagnostics.
  $camera=Join-Path $GamePath 'Binaries\TankCameraPresets.txt'
  if(-not(Test-Path -LiteralPath $camera)){
   $poses=New-Object 'System.Collections.Generic.List[string]'
   $legacy=Join-Path $GamePath 'Binaries\WolverineLive.log'
   if(Test-Path -LiteralPath $legacy){foreach($line in [IO.File]::ReadLines($legacy)){if($line.StartsWith('TANK CAMERA POSE ') -and -not $poses.Contains($line)){$poses.Add($line)}}}
   if($poses.Count){[IO.File]::WriteAllLines($camera,$poses)}else{Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'payload\TankCameraPresets.txt') -Destination $camera}
  }
  $state.Status='Installed';$state|ConvertTo-Json -Depth 8|Set-Content $statePath -Encoding UTF8
 }catch{Restore $records;$state.Status='Uninstalled';$state|ConvertTo-Json -Depth 8|Set-Content $statePath -Encoding UTF8;throw}
 Write-Output '2.0 Beta 1 installed and verified. F6 opens the controls. Saved settings preserved.'
}finally{
 # Only known staged files are removed; never recursively remove a computed path.
 foreach($name in @('CH_Wolverine_Natural_SF.xxx','WGame.xxx','WStart.xxx')){$p=Join-Path $stage $name;if(Test-Path -LiteralPath $p){Remove-Item -LiteralPath $p}}
 Remove-Item -LiteralPath $stage
}
