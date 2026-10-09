#include <malemod/surface/runtime.hpp>
#include <cmath>
#include <cstdio>
using namespace malemod::surface;
int main(){
 Session session;
 Frame f;f.rootContacts=true;
 // Explicit source-calibrated numerical fixture; this is not a game capture.
 f.thighEndpoints=std::array<Point,4>{{{2,-7.8f,79},{1,-8.2f,43},{2,7.8f,79},{1,8.2f,43}}};
 f.collision=CollisionCalibration{{7.2f,7.2f},{{{3,0,70},{5.4f,0,86}}},6.4f};
 f.clinical.active=true;f.clinical.lateralWobbleDegrees=8;
 f.clinical.lateralGain={1,1,1,1};
 float maximumYaw=0,maximumError=0;
 for(unsigned i=0;i<80;i++){
  Controls controls;controls.values[7]=i<40?1.f:100.f;session.SetControls(controls);
  f.clinical.active=i<65;f.clinical.time=6.8+i/60.;session.Step(f);auto o=session.Read();
  auto p=o.shaftGuide[0],q=o.shaftGuide[1];
  const float x=q.x-p.x,y=q.y-p.y,z=q.z-p.z,length=std::sqrt(x*x+y*y+z*z);
  float error=std::sqrt(std::pow(x/length-o.rootDirection.x,2)+std::pow(y/length-o.rootDirection.y,2)+std::pow(z/length-o.rootDirection.z,2));
  if(!std::isfinite(error)||error>2e-5f)return 1;
  maximumError=std::max(maximumError,error);maximumYaw=std::max(maximumYaw,std::abs(o.rootDirection.y));
 }
 if(maximumYaw<.03f)return 2;
 std::printf("PASS enabled root joint with moving rest pitch, nonzero clinical yaw and inactive transition: guide/direction error %.9g, lateral excursion %.9g\n",maximumError,maximumYaw);
}
