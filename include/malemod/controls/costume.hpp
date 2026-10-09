#pragma once
#include <array>
namespace malemod::controls {
enum class Top : unsigned { Naked=0, TankTop=1 };
enum class Bottom : unsigned { Naked=0, Jockstrap=1, Jeans=2, JeansOpen=3 };
inline constexpr std::array<const char*,2> Tops={"Naked","Tank Top"};
inline constexpr std::array<const char*,4> Bottoms={"Naked","Jockstrap","Jeans","Jeans (open)"};
struct Costume {
 Top top=Top::Naked;
 Bottom bottom=Bottom::Naked;
 // Missing keys use -1. Legacy Style only selects the bottom; invalid explicit
 // values fail closed to Naked instead of exposing an unsupported costume.
 static constexpr Costume Load(int topKey,int bottomKey,int legacyStyle) {
  const int bottom=bottomKey==-1?(legacyStyle==1?1:0):bottomKey;
  return {topKey==1?Top::TankTop:Top::Naked,
          bottom>=0&&bottom<int(Bottoms.size())?Bottom(bottom):Bottom::Naked};
 }
};
}
