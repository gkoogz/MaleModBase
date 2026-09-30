@echo off
call "C:\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul
pushd "%~dp0"
rc /nologo /fo splat_bakes.res splat_bakes.rc
set splatAssetStatus=%errorlevel%
popd
exit /b %splatAssetStatus%
