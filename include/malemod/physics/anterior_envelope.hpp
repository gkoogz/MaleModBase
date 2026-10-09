#pragma once
#include <algorithm>
#include <cmath>

namespace malemod::physics {
// The connected enclosing tissue has an anterior surface between its two
// contents. Independent convex contacts alone leave that material bridge out.
// Coordinates are an adapter-supplied anatomical frame: X anterior, Y lateral.
struct AnteriorContact { bool active=false; float fraction=0,gap=0,nx=1,ny=0,nz=0; };
template<class V> struct EnvelopeDeltas {V point{},first{},second{};};
template<class V>
EnvelopeDeltas<V> DistributeEnvelopeCorrection(const AnteriorContact& c,float amount,float pointInverseMass,float firstInverseMass,float secondInverseMass){
 const float a=1-c.fraction,b=c.fraction;
 const float mass=pointInverseMass+a*a*firstInverseMass+b*b*secondInverseMass;
 if(!c.active||mass<1e-8f||amount<=0)return {};
 const float impulse=amount/mass;
 auto delta=[&](float factor)->V{return {c.nx*factor,c.ny*factor,c.nz*factor};};
 return {delta(impulse*pointInverseMass),delta(-impulse*a*firstInverseMass),delta(-impulse*b*secondInverseMass)};
}
template<class V>
EnvelopeDeltas<V> CancelEnvelopeInwardVelocity(const AnteriorContact& c,V point,V first,V second,float pointInverseMass,float firstInverseMass,float secondInverseMass){
 float a=1-c.fraction,b=c.fraction;
 const float inward=(point.x-a*first.x-b*second.x)*c.nx+(point.y-a*first.y-b*second.y)*c.ny+(point.z-a*first.z-b*second.z)*c.nz;
 return DistributeEnvelopeCorrection<V>(c,(std::max)(0.f,-inward),pointInverseMass,firstInverseMass,secondInverseMass);
}
template<class V>
AnteriorContact AnteriorEnvelope(V p,V a,V b,V radii,float padding){
 AnteriorContact c;
 const float span=b.y-a.y;
 c.fraction=std::abs(span)>1e-6f?std::clamp((p.y-a.y)/span,0.f,1.f):.5f;
 const float x=a.x+(b.x-a.x)*c.fraction;
 const float y=a.y+span*c.fraction,z=a.z+(b.z-a.z)*c.fraction;
 const float rx=radii.x+padding,ry=radii.y+padding,rz=radii.z+padding;
 if(!(rx>0&&ry>0&&rz>0))return c;
 const float dy=(p.y-y)/ry,dz=(p.z-z)/rz,q=dy*dy+dz*dz;
 if(q>=1.f)return c; // Free below, above, and outside the enclosing tissue.
 const float root=std::sqrt((std::max)(1e-8f,1.f-q));
 const float surface=x+rx*root;
 if(p.x>=surface)return c;
 float gy=rx*dy/(ry*root),gz=rx*dz/(rz*root);
 // Include the moving bridge's slope, rather than an axis-aligned wall.
 if(c.fraction>0&&c.fraction<1&&std::abs(span)>1e-6f)
  gy=-(b.x-a.x)/span-gz*(b.z-a.z)/span;
 const float inverse=1/std::sqrt(1+gy*gy+gz*gz);
 c.active=true;c.nx=inverse;c.ny=gy*inverse;c.nz=gz*inverse;
 c.gap=(p.x-surface)*inverse;
 return c;
}
}
