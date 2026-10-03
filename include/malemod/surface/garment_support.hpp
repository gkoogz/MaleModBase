#pragma once
#include <malemod/math.hpp>
#include <stdexcept>
namespace malemod::surface {
inline constexpr float maximumGarmentGravityFraction=.15f;
// Magnitudes of the measured source's maximum shaft/lobe gravity. These are
// acceleration units in that model, not an inference about meters.
inline constexpr float maximumShaftSupportAcceleration=16.5f;
inline constexpr float maximumLobeSupportAcceleration=10.8f;
inline V3 BoundGarmentAcceleration(V3 acceleration,float sourceGravity){
 if(!std::isfinite(acceleration.x)||!std::isfinite(acceleration.y)||!std::isfinite(acceleration.z)||!std::isfinite(sourceGravity)||sourceGravity<0)
  throw std::invalid_argument("Invalid calibrated garment acceleration");
 const float length=Length(acceleration),limit=maximumGarmentGravityFraction*sourceGravity;
 return length>limit&&length>0?acceleration*(limit/length):acceleration;
}
}
