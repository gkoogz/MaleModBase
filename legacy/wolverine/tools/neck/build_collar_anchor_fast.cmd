@echo off
call "C:\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul
cl.exe /nologo /DNO_CLINICAL_COLLAR /DNO_CLINICAL_NECK /O2 /MT /EHsc /std:c++17 /I"..\..\third-party" /I"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Include" collar_anchor_capture.cpp ..\..\src\runtime\splat_bakes.res /link /OUT:collar-anchor-fast.exe /LIBPATH:"%WindowsSdkDir%Lib\%WindowsSDKVersion%um\x86" /LIBPATH:"C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)\Lib\x86" d3d9.lib d3dx9.lib d3d11.lib d3dcompiler.lib user32.lib gdi32.lib ole32.lib winmm.lib
