@echo off
call "C:\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul
if not exist "%~dp0build" mkdir "%~dp0build"
pushd "%~dp0src\runtime"
cl.exe /nologo /LD /O2 /MT /EHsc /std:c++17 /I"..\..\third-party" /I"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Include" d3d9_proxy.cpp splat_bakes.res /link /DEF:d3d9_proxy.def /OUT:..\..\build\d3d9.dll /LIBPATH:"%WindowsSdkDir%Lib\%WindowsSDKVersion%um\x86" /LIBPATH:"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Lib\x86" d3d9.lib d3dx9.lib d3d11.lib d3dcompiler.lib user32.lib gdi32.lib ole32.lib winmm.lib
set buildResult=%ERRORLEVEL%
popd
exit /b %buildResult%
