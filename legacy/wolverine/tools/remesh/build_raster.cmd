@echo off
call "C:\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /O2 /MT /EHsc /std:c++17 /LD raster.cpp /link /OUT:raster.dll
