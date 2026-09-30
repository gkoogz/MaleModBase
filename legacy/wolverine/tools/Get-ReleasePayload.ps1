#requires -Version 5.1
[CmdletBinding()]
param([string]$ArchivePath)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$m=Get-Content (Join-Path $root 'manifest.json') -Raw|ConvertFrom-Json
$name='WolverineAnatomyTool-v'+$m.version+'-Install.7z'
$temporary=$null
$expanded=$null
try {
 if(-not $ArchivePath){
  $temporary=Join-Path ([IO.Path]::GetTempPath()) ('wolverine-payload-'+[guid]::NewGuid().ToString('N'))
  New-Item -ItemType Directory $temporary|Out-Null
  & gh release download ('v'+$m.version) --repo gkoogz/XMenOriginsWolverineMaleMod --pattern $name --dir $temporary
  if($LASTEXITCODE){throw 'Release download failed. Authenticate using gh auth login.'}
  $ArchivePath=Join-Path $temporary $name
 }
 if([IO.Path]::GetExtension($ArchivePath) -eq '.7z'){
  $extractor=Get-Command 7z -ErrorAction SilentlyContinue
  if(-not $extractor){
   $sevenZip=Join-Path $env:ProgramFiles '7-Zip/7z.exe'
   if(Test-Path -LiteralPath $sevenZip){$extractor=Get-Command $sevenZip}
  }
  if(-not $extractor){throw 'Install 7-Zip to extract the compact complete release, or provide an extracted payload locally.'}
  $expanded=Join-Path ([IO.Path]::GetTempPath()) ('wolverine-payload-extract-'+[guid]::NewGuid().ToString('N'))
  New-Item -ItemType Directory $expanded|Out-Null
  $prefix='WolverineAnatomyTool-v'+$m.version+'/payload/'
  $specs=@()
  foreach($p in $m.payload.PSObject.Properties){$specs+=@{Name=$p.Name;Hash=$p.Value}}
  foreach($p in $m.idleClips.PSObject.Properties){$specs+=@{Name=('WolverineIdle/'+$p.Name);Hash=$p.Value}}
  $wanted=@($specs|ForEach-Object {$prefix+$_.Name})
  & $extractor.Source x ([IO.Path]::GetFullPath($ArchivePath)) ('-o'+$expanded) -y -bsp0 @wanted
  if($LASTEXITCODE){throw 'Release extraction failed'}
  foreach($s in $specs){
   $source=Join-Path $expanded ($prefix+$s.Name)
   if((Get-FileHash -LiteralPath $source).Hash -ne $s.Hash){throw ('Release checksum mismatch: '+$s.Name)}
   $destination=Join-Path $root ('payload/'+$s.Name)
   if((Test-Path -LiteralPath $destination) -and (Get-FileHash -LiteralPath $destination).Hash -ne $s.Hash){throw ('Different local payload already exists: '+$destination)}
  }
  foreach($s in $specs){
   $destination=Join-Path $root ('payload/'+$s.Name)
   if(Test-Path -LiteralPath $destination){continue}
   New-Item -ItemType Directory (Split-Path $destination -Parent) -Force|Out-Null
   Copy-Item -LiteralPath (Join-Path $expanded ($prefix+$s.Name)) -Destination $destination
  }
  Write-Output ('Verified '+$specs.Count+' local release payload files in '+(Join-Path $root 'payload'))
  return
 }
 Add-Type -AssemblyName System.IO.Compression.FileSystem
 $archive=[IO.Compression.ZipFile]::OpenRead([IO.Path]::GetFullPath($ArchivePath))
 try {
  $prefix='WolverineAnatomyTool-v'+$m.version+'/payload/'
  $specs=@()
  foreach($p in $m.payload.PSObject.Properties){$specs+=@{Name=$p.Name;Hash=$p.Value}}
  foreach($p in $m.idleClips.PSObject.Properties){$specs+=@{Name=('WolverineIdle/'+$p.Name);Hash=$p.Value}}
  $destination=[IO.Path]::GetFullPath((Join-Path $root 'payload'))
  # Validate every release payload before writing any local payload files.
  foreach($s in $specs){
   $path=[IO.Path]::GetFullPath((Join-Path $destination $s.Name))
   if(-not $path.StartsWith(($destination+'\'),[StringComparison]::OrdinalIgnoreCase)){throw 'Invalid payload destination'}
   $entry=$archive.GetEntry($prefix+$s.Name)
   if(-not $entry){throw ('Missing release payload: '+$s.Name)}
   $inputStream=$entry.Open();$sha=[Security.Cryptography.SHA256]::Create()
   try{$hash=[BitConverter]::ToString($sha.ComputeHash($inputStream)).Replace('-','')}finally{$sha.Dispose();$inputStream.Dispose()}
   if($hash -ne $s.Hash){throw ('Release payload checksum mismatch: '+$s.Name)}
   if((Test-Path -LiteralPath $path) -and (Get-FileHash -LiteralPath $path).Hash -ne $s.Hash){throw ('Different local payload already exists: '+$path)}
  }
  foreach($s in $specs){
   $path=Join-Path $destination $s.Name
   if(Test-Path -LiteralPath $path){continue}
   New-Item -ItemType Directory (Split-Path $path -Parent) -Force|Out-Null
   [IO.Compression.ZipFileExtensions]::ExtractToFile($archive.GetEntry($prefix+$s.Name),$path,$false)
   if((Get-FileHash -LiteralPath $path).Hash -ne $s.Hash){throw ('Extracted payload checksum mismatch: '+$s.Name)}
  }
  Write-Output ('Verified '+$specs.Count+' local release payload files in '+$destination)
 }finally{$archive.Dispose()}
}finally{
 if($expanded){
  $resolved=[IO.Path]::GetFullPath($expanded)
  $tempRoot=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')+'\'
  if(-not $resolved.StartsWith($tempRoot,[StringComparison]::OrdinalIgnoreCase) -or [IO.Path]::GetFileName($resolved) -notlike 'wolverine-payload-extract-*'){throw 'Unsafe temporary cleanup path'}
  Remove-Item -LiteralPath $resolved -Recurse
 }
 if($temporary){
  $file=Join-Path $temporary $name
  if(Test-Path -LiteralPath $file){Remove-Item -LiteralPath $file}
  Remove-Item -LiteralPath $temporary
 }
}