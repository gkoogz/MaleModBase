#pragma once
#include <array>
namespace malemod::controls {
enum class Top : unsigned { Naked=0, TankTop=1 };
enum class Bottom : unsigned { Naked=0, Jockstrap=1 };
inline constexpr std::array<const char*,2> Tops={"Naked","Tank Top"};
inline constexpr std::array<const char*,2> Bottoms={"Naked","Jockstrap"};
struct Costume {
 Top top=Top::Naked;
 Bottom bottom=Bottom::Naked;
 // Missing keys use -1. Legacy Style only selects the bottom; invalid explicit
 // values fail closed to Naked instead of exposing an unsupported costume.
 static constexpr Costume Load(int topKey,int bottomKey,int legacyStyle) {
  return {topKey==1?Top::TankTop:Top::Naked,
          (bottomKey==-1?legacyStyle:bottomKey)==1?Bottom::Jockstrap:Bottom::Naked};
 }
};
}
