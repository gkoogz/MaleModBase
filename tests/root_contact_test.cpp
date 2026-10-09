#include <malemod/surface/root_contact.hpp>
#include <cstdio>
#include <limits>
using namespace malemod::surface::root_contact;
int main(){
 // Full angular/velocity round trip, including the rest-command wrap. The
 // accepted link is an output of contact constraints, not a virtual ray.
 const Point root={9,0,84.3};const double length=3.78;
 for(double pitch:{-3.2,-1.4,0.,1.7,3.2})for(double yaw:{-.6,0.,.6}){
  State s{pitch,yaw,.3,-.2};auto d=Direction(s);Point link{},v{};
  const double sp=std::sin(pitch),cp=std::cos(pitch),sy=std::sin(yaw),cy=std::cos(yaw);
  Point dp={-sp*cy,0,-cp*cy},dy={-cp*sy,cy,sp*sy};
  for(unsigned k=0;k<3;k++){link[k]=root[k]+d[k]*length;v[k]=length*(dp[k]*s.pitchVelocity+dy[k]*s.yawVelocity)+d[k]*7;}
  auto actual=FromJoint(root,link,v,pitch);
  if(std::abs(actual.pitch-pitch)>1e-12||std::abs(actual.yaw-yaw)>1e-12||std::abs(actual.pitchVelocity-.3)>1e-12||std::abs(actual.yawVelocity+.2)>1e-12)return 1;
 }
 // Effective point inverse mass follows J M^-1 J^T: tangent Jacobian length.
 if(PointInverseMass(4,2)!=8||PointInverseMass(2,2)!=2)return 2;
 bool rejected=false;try{FromJoint(root,root,{},0);}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 3;
 rejected=false;try{PointInverseMass(0,2);}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 4;
 rejected=false;try{FromJoint(root,{9,0,85},{},std::numeric_limits<double>::quiet_NaN());}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 5;
 std::puts("PASS contact-driven joint round trips, radial-velocity removal, angle continuity, inertia scaling and malformed-state rejection");
}
