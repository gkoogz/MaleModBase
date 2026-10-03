#pragma once
#include <malemod/math.hpp>
#include <stdexcept>

namespace malemod::collar {
// Character-local measured attachment frame. Axes are directions, root and
// geometry use the caller's explicitly calibrated common length units.
// neutralAxis belongs to the pelvic attachment, not a live shaft command.
struct RecruitmentFrame {V3 root,axis,lateral,up;};
inline RecruitmentFrame StableRecruitmentFrame(V3 root,V3 neutralAxis,V3 pelvicLateral){
 auto finite=[](V3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
 if(!finite(root)||!finite(neutralAxis)||!finite(pelvicLateral)||Length(neutralAxis)<1e-6f||Length(pelvicLateral)<1e-6f)
  throw std::invalid_argument("Invalid measured pelvic recruitment frame");
 V3 axis=Unit(neutralAxis),up=Cross(axis,pelvicLateral);
 if(Length(up)<1e-6f)throw std::invalid_argument("Degenerate pelvic recruitment axes");
 up=Unit(up);V3 lateral=Unit(Cross(up,axis));
 return {root,axis,lateral,up};
}
}
