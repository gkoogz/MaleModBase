#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace malemod::surface::normal_frame {
using Vector=std::array<float,3>;
using Matrix=std::array<float,9>;
inline float Dot(Vector a,Vector b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Vector Cross(Vector a,Vector b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Vector Unit(Vector a){float length=std::sqrt(Dot(a,a));if(!std::isfinite(length)||length<1e-8f)throw std::invalid_argument("Invalid source normal frame");for(auto& x:a)x/=length;return a;}
inline Matrix Rotation(Vector from,Vector to){
 from=Unit(from);to=Unit(to);auto axis=Cross(from,to);const float squared=Dot(axis,axis),cosine=(std::max)(-1.f,(std::min)(1.f,Dot(from,to)));
 Matrix out{1,0,0,0,1,0,0,0,1};
 if(squared<1e-12f){
  if(cosine>0)return out;
  axis=Unit(Cross(from,std::abs(from[2])<.9f?Vector{0,0,1}:Vector{1,0,0}));
  for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)out[a*3+b]=2*axis[a]*axis[b]-(a==b?1.f:0.f);return out;
 }
 const Matrix skew{0,-axis[2],axis[1],axis[2],0,-axis[0],-axis[1],axis[0],0};const float factor=(1-cosine)/squared;
 for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)out[a*3+b]+=skew[a*3+b]+(axis[a]*axis[b]-(a==b?squared:0.f))*factor;return out;
}
// Capture the complete observed source frame, including tangent-only twist.
// Tangent magnitude/normal contamination are discarded by Gram orthogonalizing.
// A missing or degenerate tangent retains the existing normal-only rotation.
inline Matrix FrameRotation(Vector restNormal,Vector restTangent,Vector liveNormal,Vector liveTangent){
 const auto originalRestNormal=restNormal,originalLiveNormal=liveNormal;
 restNormal=Unit(restNormal);liveNormal=Unit(liveNormal);
 const float restAlong=Dot(restTangent,restNormal),liveAlong=Dot(liveTangent,liveNormal);
 for(unsigned i=0;i<3;i++){restTangent[i]-=restNormal[i]*restAlong;liveTangent[i]-=liveNormal[i]*liveAlong;}
 const float restSquared=Dot(restTangent,restTangent),liveSquared=Dot(liveTangent,liveTangent);
 if(!std::isfinite(restSquared)||!std::isfinite(liveSquared)||restSquared<1e-12f||liveSquared<1e-12f)return Rotation(originalRestNormal,originalLiveNormal);
 restTangent=Unit(restTangent);liveTangent=Unit(liveTangent);
 if(restNormal==liveNormal&&restTangent==liveTangent)return {1,0,0,0,1,0,0,0,1};
 const auto restBitangent=Cross(restNormal,restTangent),liveBitangent=Cross(liveNormal,liveTangent);
 Matrix result{};
 for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)result[a*3+b]=liveTangent[a]*restTangent[b]+liveBitangent[a]*restBitangent[b]+liveNormal[a]*restNormal[b];
 return result;
}
// Apply the source normal's rotation to a target rest-frame vector. This keeps
// the target's seam/UV frame and rotates its normal and tangent together.
inline Vector Transfer(Vector from,Vector to,Vector value,Vector tangentHint){
 from=Unit(from);to=Unit(to);auto axis=Cross(from,to);float squared=Dot(axis,axis),cosine=(std::max)(-1.f,(std::min)(1.f,Dot(from,to)));
 if(squared<1e-12f){
  if(cosine>0)return value;
  const float along=Dot(tangentHint,from);for(unsigned i=0;i<3;i++)axis[i]=tangentHint[i]-from[i]*along;
  if(Dot(axis,axis)<1e-12f)axis=Cross(from,std::abs(from[2])<.9f?Vector{0,0,1}:Vector{1,0,0});
  axis=Unit(axis);float projection=Dot(axis,value);for(unsigned i=0;i<3;i++)value[i]=2*axis[i]*projection-value[i];return value;
 }
 auto first=Cross(axis,value),second=Cross(axis,first);const float factor=(1-cosine)/squared;
 for(unsigned i=0;i<3;i++)value[i]+=first[i]+second[i]*factor;return value;
}
}
