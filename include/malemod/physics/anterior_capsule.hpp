#pragma once
#include <algorithm>
#include <cmath>
namespace malemod::physics {
// A one-sided capsule envelope. The adapter supplies its anatomical anterior
// frame and measured support extents; no skeleton or source units live here.
template<class V> struct AnteriorCapsuleContact {bool active=false;float gap=0,station=0;V normal{};};
template<class V> AnteriorCapsuleContact<V> AnteriorCapsule(V center,V a,V b,V extents,float radius){
 AnteriorCapsuleContact<V> result;
 const float ry=extents.y+radius,rz=extents.z+radius,rx=extents.x+radius;
 if(!(rx>0&&ry>0&&rz>0))return result;
 // Closest station in the transverse ellipse, independent of which side
 // the contents occupied last frame. A rear-side contact cannot flip this.
 float ey=b.y-a.y,ez=b.z-a.z;
 float denominator=ey*ey/(ry*ry)+ez*ez/(rz*rz);
 float t=denominator>1e-8f?((center.y-a.y)*ey/(ry*ry)+(center.z-a.z)*ez/(rz*rz))/denominator:0;
 t=(std::max)(0.f,(std::min)(1.f,t));V q{a.x+(b.x-a.x)*t,a.y+ey*t,a.z+ez*t};
 float dy=(center.y-q.y)/ry,dz=(center.z-q.z)/rz,u=1-dy*dy-dz*dz;
 if(u<=0)return result;
 float root=std::sqrt(u),surface=q.x+rx*root;
 if(center.x>=surface)return result;
 V n{1.f,rx*dy/(ry*(std::max)(root,.05f)),rx*dz/(rz*(std::max)(root,.05f))};
 float length=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);n={n.x/length,n.y/length,n.z/length};
 result.active=true;result.normal=n;result.gap=(center.x-surface)*n.x;result.station=t;return result;
}
// Position recovery is independent of frame rate and excludes artificial
// impulse. Tangential velocity remains free, including lateral/vertical swing.
template<class V> V AnteriorRecovery(const AnteriorCapsuleContact<V>& contact,float maximumDistance){
 float distance=contact.active?(std::min)(-contact.gap,(std::max)(0.f,maximumDistance)):0;
 return {contact.normal.x*distance,contact.normal.y*distance,contact.normal.z*distance};
}
template<class V> V RemoveAnteriorInwardVelocity(const AnteriorCapsuleContact<V>& contact,V velocity,V frameVelocity){
 if(!contact.active)return velocity;
 float rate=(velocity.x-frameVelocity.x)*contact.normal.x+(velocity.y-frameVelocity.y)*contact.normal.y+(velocity.z-frameVelocity.z)*contact.normal.z;
 rate=(std::min)(0.f,rate);return {velocity.x-contact.normal.x*rate,velocity.y-contact.normal.y*rate,velocity.z-contact.normal.z*rate};
}
}
