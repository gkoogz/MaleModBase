#pragma once
#include <cstdint>

namespace malemod::controls::menu {
// Engine-independent presentation; adapters retain native input and font APIs.
struct Color { unsigned char r,g,b; };
inline constexpr Color Background{22,24,27}, Text{235,233,228}, Muted{153,157,163};
inline constexpr Color Accent{185,157,112}, Track{65,69,74}, Selection{39,42,47};
inline constexpr const wchar_t* Font=L"Bahnschrift SemiCondensed";
inline constexpr const wchar_t* FallbackFont=L"Arial";
inline constexpr unsigned FontWeight=600;
inline constexpr float TextHeight=16.f, HeadingHeight=18.f;
inline constexpr const char* Title="ANATOMY";
inline constexpr const wchar_t* WideTitle=L"ANATOMY";
inline constexpr const char* Navigation="Arrows select / adjust   Shift x5";
inline constexpr const wchar_t* WideNavigation=L"Arrows select / adjust   Shift x5";
inline constexpr std::uint32_t ARGB(Color c){return 0xff000000u|(std::uint32_t(c.r)<<16)|(std::uint32_t(c.g)<<8)|c.b;}
}
