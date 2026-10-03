#pragma once
#include <array>
namespace malemod::controls {
// The canonical Wolverine presentation vocabulary. Index order follows the
// portable surface Controls contract; numerical mappings remain unchanged.
inline constexpr std::array<const char*,18> Labels={"Erection","Overall","Length","Width","Glans Size","Scrotum","Hang","Angle","Forward","Vertical","Shaft Stiff","Shaft Weight","Shaft Bounce","Shaft Velocity","Balls Stiff","Balls Weight","Balls Bounce","Balls Velocity"};
inline constexpr std::array<const wchar_t*,18> WideLabels={L"Erection",L"Overall",L"Length",L"Width",L"Glans Size",L"Scrotum",L"Hang",L"Angle",L"Forward",L"Vertical",L"Shaft Stiff",L"Shaft Weight",L"Shaft Bounce",L"Shaft Velocity",L"Balls Stiff",L"Balls Weight",L"Balls Bounce",L"Balls Velocity"};
inline constexpr std::array<const char*,3> States={"Erect","Semi","Full Floppy"};
inline constexpr std::array<const wchar_t*,3> WideStates={L"Erect",L"Semi",L"Full Floppy"};
inline constexpr std::array<const char*,4> ThrobModes={"Off","Gentle","Medium","Intense"};
inline constexpr std::array<const wchar_t*,4> WideThrobModes={L"Off",L"Gentle",L"Medium",L"Intense"};
inline constexpr std::array<const char*,2> Garments={"Naked","Jockstrap"};
inline constexpr std::array<const wchar_t*,2> WideGarments={L"Naked",L"Jockstrap"};
}
