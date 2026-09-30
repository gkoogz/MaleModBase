#pragma once
#include "math.hpp"
#include <algorithm>
#include <cstdint>
namespace malemod {
struct Capsule {V3 a{},b{};float radius=0;std::uint64_t receiverId=0;};
struct Projection {V3 position{},normal{};std::uint64_t receiverId=0;};
// Cached character-local hitboxes supplied by an adapter; no native queries.
inline bool ProjectCapsule(V3 point,float particleRadius,const Capsule& body,Projection& out){
 if(!std::isfinite(body.radius)||!std::isfinite(particleRadius)||body.radius<0||particleRadius<0)return false;
 V3 axis=body.b-body.a;float length2=Dot(axis,axis);float t=length2>1e-12f?std::clamp(Dot(point-body.a,axis)/length2,0.f,1.f):0;
 V3 center=body.a+axis*t,delta=point-center;float distance=Length(delta),radius=body.radius+particleRadius;
 if(!std::isfinite(distance)||distance>=radius)return false;
 V3 n=distance>1e-7f?delta/distance:Unit(Cross(axis,Length(axis)>1e-6f&&std::abs(Unit(axis).z)<.8f?V3{0,0,1}:V3{0,1,0}));
 out={center+n*radius,n,body.receiverId};return true;
}
}
