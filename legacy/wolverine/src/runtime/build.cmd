@echo off
call "%~dp0build_splat_assets.cmd"
if errorlevel 1 exit /b 1
call "%~dp0..\..\tools\fluid\build_splat_shader.cmd"
if errorlevel 1 exit /b 1
call "C:\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul
cl.exe /nologo /LD /O2 /MT /EHsc /std:c++17 /I"%WindowsSdkDir%Include\%WindowsSDKVersion%um" /I"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Include" d3d9_proxy.cpp splat_bakes.res /link /DEF:d3d9_proxy.def /OUT:d3d9.dll /LIBPATH:"%WindowsSdkDir%Lib\%WindowsSDKVersion%um\x86" /LIBPATH:"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Lib\x86" d3d9.lib d3dx9.lib d3d11.lib d3dcompiler.lib user32.lib gdi32.lib ole32.lib winmm.lib
