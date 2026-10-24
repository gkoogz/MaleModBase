#pragma once
#include <stdexcept>
namespace malemod::surface {
// The target-owned continuous surface represents a solid soft rod rather than
// the old thin centerline skin. Keep all user stiffness/bounce/weight mappings;
// calibrate its flexible-state bending compliance before the coupled solve.
inline float SolidRodComplianceScale(int state){
 if(state<0||state>2)throw std::invalid_argument("Invalid solid rod state");
 return state==2?.4f:1.f;
}
}
