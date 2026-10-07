#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace malemod::garments::cloth_contact {
// A bounded oracle returns its radius (and no physical triangle) on a miss.
// That result must lie strictly outside every downstream application guard.
inline double SearchRadius(double requested,double application){
 if(!std::isfinite(requested)||!std::isfinite(application)||requested<=0||application<0)
  throw std::invalid_argument("Invalid cloth contact search/application radius");
 const double radius=std::nextafter((std::max)(requested,application),std::numeric_limits<double>::infinity());
 if(!std::isfinite(radius))throw std::invalid_argument("Cloth contact search radius overflow");
 return radius;
}
}
