#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace malemod::surface {
// Restore the authored crown as an isotropic material body after a shaft-length
// morph. The original first ring remains exact; its residual fades smoothly
// before the free crown. Vec needs x/y/z and ordinary vector arithmetic.
template<class Vec> void RestoreCrownMaterial(Vec* points,const Vec* material,
 unsigned count,unsigned first,unsigned rows,unsigned columns,unsigned crown,
 Vec axis,float blend){
 if(!points||!material||rows<2||columns<3||!crown||crown>=rows||
    first+std::uint64_t(rows)*columns>count||!std::isfinite(blend)||blend<0||blend>1)
  throw std::invalid_argument("Invalid crown material topology");
 if(blend==0)return;
 auto dot=[](Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;};
 auto length=[&](Vec v){return std::sqrt(dot(v,v));};
 auto unit=[&](Vec v){float n=length(v);if(!std::isfinite(n)||n<1e-6f)throw std::invalid_argument("Collapsed crown material frame");return v/n;};
 auto cross=[](Vec a,Vec b){return Vec{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};};
 auto ease=[](float t){t=std::clamp(t,0.f,1.f);return t*t*t*(10.f+t*(-15.f+6.f*t));};
 Vec origin{},referenceOrigin{},referenceTip{};
 for(unsigned j=0;j<columns;j++){
  origin=origin+points[first+j];referenceOrigin=referenceOrigin+material[first+j];
  referenceTip=referenceTip+material[first+(rows-1)*columns+j];
 }
 origin=origin/float(columns);referenceOrigin=referenceOrigin/float(columns);referenceTip=referenceTip/float(columns);
 axis=unit(axis);const auto oldAxis=unit(referenceTip-referenceOrigin);
 const auto oldSide=unit(Vec{0,1,0}-oldAxis*oldAxis.y),oldUp=unit(cross(oldAxis,oldSide));
 const auto side=unit(Vec{0,1,0}-axis*axis.y),up=unit(cross(axis,side));
 float measured=0,referenceRadius=0;
 for(unsigned j=0;j<columns;j++){
  auto a=points[first+j]-origin,b=material[first+j]-referenceOrigin;
  measured+=length(a-axis*dot(a,axis));referenceRadius+=length(b-oldAxis*dot(b,oldAxis));
 }
 if(measured<1e-6f||referenceRadius<1e-6f)throw std::invalid_argument("Collapsed crown material section");
 const float scale=measured/referenceRadius;
 std::vector<Vec> seam(columns);
 for(unsigned i=first;i<count;i++){
  const auto q=material[i]-referenceOrigin;
  const auto desired=origin+(axis*dot(q,oldAxis)+side*dot(q,oldSide)+up*dot(q,oldUp))*scale;
  const unsigned row=(i-first)/columns,column=(i-first)%columns;
  if(row==0){seam[column]=points[i]-desired;continue;}
  const float follow=1.f-ease(float(row)/float(crown));
  points[i]=points[i]*(1.f-blend)+(desired+seam[column]*follow)*blend;
 }
}
}
