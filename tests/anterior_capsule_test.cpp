#include <malemod/physics/anterior_capsule.hpp>
#include <cstdio>
#include <stdexcept>
struct V {float x,y,z;};
static void Require(bool ok){if(!ok)throw std::runtime_error("Anterior capsule regression");}
int main(){try{
 using namespace malemod::physics;
 for(float size:{.25f,1.f,2.5f}){
  V extents{6*size,5*size,8*size},a{3,0,70},b{5.4f,0,86};
  auto c=AnteriorCapsule(V{-12,0,70},a,b,extents,6.4f);
  Require(c.active&&c.gap<0&&c.normal.x>0);
  auto shift=AnteriorRecovery(c,.1f);Require(shift.x>0&&std::sqrt(shift.x*shift.x+shift.y*shift.y+shift.z*shift.z)<=.10001f);
  V velocity{-3,4,5},frame{2,0,0};auto v=RemoveAnteriorInwardVelocity(c,velocity,frame);
  Require(std::abs(v.x-2)<1e-5f&&v.y==velocity.y&&v.z==velocity.z);
  auto outward=RemoveAnteriorInwardVelocity(c,V{9,4,5},frame);Require(outward.x==9&&outward.y==4&&outward.z==5);
  Require(!AnteriorCapsule(V{100,0,70},a,b,extents,6.4f).active);
  Require(!AnteriorCapsule(V{-12,100,70},a,b,extents,6.4f).active);
  // Crouch and stand rotate a measured capsule in the adapter. The shared
  // constraint must retain its anterior side through the whole sweep.
  for(unsigned i=0;i<=120;i++){
   float angle=1.2f*i/120;V knee{a.x+36*std::sin(angle),a.y,a.z-36*std::cos(angle)};
   V center{a.x-15,a.y,a.z-3};auto q=AnteriorCapsule(center,a,knee,extents,7.2f);
   Require(q.active&&q.normal.x>0&&q.station>=0&&q.station<=1);
   auto step=AnteriorRecovery(q,.1f);Require(std::isfinite(step.x)&&step.x>0);
  }
 }
 std::puts("PASS: size-aware anterior contacts, crouch sweep, bounded recovery and tangential velocity");return 0;
 }catch(const std::exception& e){std::puts(e.what());return 1;}}
