#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface::collar_arc {
inline double Smooth(double x) {
 if(!std::isfinite(x))throw std::invalid_argument("Nonfinite collar arc parameter");
 x=std::clamp(x,0.,1.);return std::clamp(x*x*x*(10+x*(-15+6*x)),0.,1.);
}
struct Release { double upper=0,lower=0; };
// Angles are signed downward. The authored neutral pose keeps the old solve
// exactly; each compressed arc releases continuously as pitch leaves neutral.
inline Release FromAngle(double degrees,double neutral=30,double interval=55) {
 if(!std::isfinite(degrees)||!std::isfinite(neutral)||!std::isfinite(interval)||interval<=0)
  throw std::invalid_argument("Invalid collar arc angle");
 return {Smooth((neutral-degrees)/interval),Smooth((degrees-neutral)/interval)};
}
// cosine and ownership are immutable material data, never posed coordinates.
// Ownership must exclude independently controlled neighbouring tissue.
inline double Weight(double cosine,double ownership,Release release) {
 if(!std::isfinite(cosine)||!std::isfinite(ownership)||ownership<0||ownership>1||
    !std::isfinite(release.upper)||!std::isfinite(release.lower)||
    release.upper<0||release.upper>1||release.lower<0||release.lower>1)
  throw std::invalid_argument("Invalid collar arc binding");
 return ownership*(release.upper*Smooth(cosine)+release.lower*Smooth(-cosine));
}
// Release the pelvis pull progressively away from its exact attachment edge.
// This changes neither the shaft target nor its requested cross-section.
inline double ShaftWeight(double original,double release,double seamDistance,double neutralRadius) {
 if(!std::isfinite(original)||original<0||original>1||!std::isfinite(release)||release<0||release>1||
    !std::isfinite(seamDistance)||seamDistance<0||!std::isfinite(neutralRadius)||neutralRadius<=0)
  throw std::invalid_argument("Invalid collar arc handoff");
 return original+(1-original)*release*Smooth(seamDistance/neutralRadius);
}
}
