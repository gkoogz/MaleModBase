#include <malemod/garments/meridian_continuity.hpp>
#include <malemod/physics/anterior_envelope.hpp>
#include <cassert>
#include <cstdio>
#include <cstring>
using namespace malemod::garments::meridian;
struct Point {float x,y,z;};
int main(){
 Point a{0,-3,0},b{0,3,0},r{3,2,4};
 auto contact=malemod::physics::AnteriorEnvelope(Point{-1,0,0},a,b,r,1.f);
 assert(contact.active&&contact.nx>0&&contact.gap<0&&std::abs(contact.fraction-.5f)<1e-6f);
 auto impulse=malemod::physics::CancelEnvelopeInwardVelocity(contact,Point{-4,7,2},Point{0,0,0},Point{0,0,0},1.f,.5f,.25f);
 assert(std::abs(impulse.point.x+impulse.first.x/.5f+impulse.second.x/.25f)<1e-6f);
 assert(impulse.point.y==0&&impulse.point.z==0);
 assert(std::abs(-4+impulse.point.x-.5f*impulse.first.x-.5f*impulse.second.x)<1e-6f);
 auto free=malemod::physics::CancelEnvelopeInwardVelocity(contact,Point{4,7,2},Point{},Point{},1.f,.5f,.25f);
 assert(free.point.x==0&&free.first.x==0&&free.second.x==0);
 assert(!malemod::physics::AnteriorEnvelope(Point{5,0,0},a,b,r,1.f).active);
 assert(!malemod::physics::AnteriorEnvelope(Point{0,0,-6},a,b,r,1.f).active);
 assert(!malemod::physics::AnteriorEnvelope(Point{0,7,0},a,b,r,1.f).active);
 // A crossing through the inter-lobe gap encounters the anterior material
 // surface before reaching the individually disjoint interior volumes.
 unsigned blocked=0;
 for(float x=5;x>=-5;x-=.05f){auto c=malemod::physics::AnteriorEnvelope(Point{x,0,0},a,b,r,1.f);if(c.active){assert(c.nx>.999f);++blocked;}}
 assert(blocked>150);
 std::vector<Vec> raw={{-2,-2,0},{2,-2,0},{2,2,0},{-2,2,0},{0,0,3}};
 auto solved=raw;solved[4]={.5f,0,3.5f};std::vector<Vec> anchors=raw;
 SurfaceContinuity history;history.Remember(solved,raw,anchors,4,5);
 auto rigid=[](Vec p){return Vec{10-p[1],20+p[0],30+p[2]};};
 auto current=raw;auto movedAnchors=anchors;for(auto& p:current)p=rigid(p);for(auto& p:movedAnchors)p=rigid(p);
 assert(history.Transport(current,movedAnchors,4));
 for(unsigned i=0;i<4;i++){Vec d=Sub(current[i],rigid(solved[i]));assert(Dot(d,d)<1e-9f);}
 {Vec d=Sub(current.back(),rigid(raw.back()));assert(Dot(d,d)<1e-9f);}
 // Boundary motion is exact, including nonrigid deformation; no world-space
 // freeze and no accumulating one-frame lag across repeated rejected wraps.
 auto deformed=raw;deformed[0][2]=.4f;anchors=deformed;
 assert(history.Transport(deformed,anchors,4));assert(std::abs(deformed[0][2]-.4f)<1e-5f);
 auto repeated=raw;repeated[0][2]=.4f;assert(history.Transport(repeated,anchors,4));
 for(unsigned i=0;i<5;i++){auto d=Sub(repeated[i],deformed[i]);assert(Dot(d,d)<1e-10f);}
 history.Reset();assert(!history.Transport(current,movedAnchors,4));
 // Hints must preserve the uncached result, including when obstacles move,
 // planes reorder, topology changes, or an old separating plane penetrates.
 std::vector<unsigned> certificates;unsigned clearCases=0,blockedCases=0;
 std::vector<Face> faces={{0,1,3},{1,2,3},{2,0,3}};
 for(unsigned frame=0;frame<300;frame++){
  float shift=4.5f*std::sin(frame*.17f);
  Hull box={{{1,0,0},1+shift},{{-1,0,0},1-shift},{{0,1,0},1},{{0,-1,0},1},{{0,0,1},1},{{0,0,-1},1}};
  std::rotate(box.begin(),box.begin()+frame%box.size(),box.end());
  std::vector<Hull> hulls={box};if(frame%5==0)hulls.push_back(box);
  auto topology=faces;if(frame%7==0)topology.pop_back();
  std::vector<Vec> reference={{-2,-2,0},{2,-2,0},{0,2,0},{shift*.5f,0,.7f},{0,0,4}};
  auto cached=reference;
  bool expected=RepairTransportedSurface(reference,3,5,topology.data(),unsigned(topology.size()),hulls);
  bool actual=RepairTransportedSurface(cached,3,5,topology.data(),unsigned(topology.size()),hulls,4,&certificates);
  assert(expected==actual);assert(std::memcmp(reference.data(),cached.data(),reference.size()*sizeof(Vec))==0);
  if(actual)++clearCases;else ++blockedCases;
 }
 assert(clearCases&&blockedCases);
 // Failed contact repair used to publish a partially displaced spike and
 // even move the current tip pole. Both failures must leave the mesh exact.
 {std::vector<Vec> p={{2,0,0},{2,1,0},{2,0,1},{.8f,.2f,.2f},{.8f,.3f,.2f},{.8f,.2f,.3f},{0,0,4}};
  auto before=p;Face f{3,4,5};Hull box={{{1,0,0},1},{{-1,0,0},1},{{0,1,0},1},{{0,-1,0},1},{{0,0,1},1},{{0,0,-1},1}};
  assert(!RepairTransportedSurface(p,3,7,&f,1,{box},1));assert(p==before);
  f={3,4,6};assert(!RepairTransportedSurface(p,3,7,&f,1,{box},4));assert(p==before);
 }
 std::puts("anterior envelope and posed garment continuity passed");
}
