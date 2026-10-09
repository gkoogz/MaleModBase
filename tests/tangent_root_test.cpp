#include <malemod/surface/tangent_root.hpp>
#include <cstdio>
using namespace malemod::surface;
namespace t=tangent_root;
static bool close(double a,double b,double e=1e-10){return std::abs(a-b)<=e*(1+std::abs(a)+std::abs(b));}
int main(){
 for(double pitch:{-2.5,0.,2.5})for(double yaw:{-.9,0.,.9}){
  root_contact::State s{pitch,yaw,.4,-.3};auto j=t::Jacobian(s,3.7);auto d=root_contact::Direction(s);const double eps=1e-6;
  for(unsigned a=0;a<2;a++){auto next=s;if(a==0)next.pitch+=eps;else next.yaw+=eps;auto n=root_contact::Direction(next);auto column=a?j.yaw:j.pitch;for(unsigned k=0;k<3;k++)if(!close(3.7*(n[k]-d[k])/eps,column[k],3e-6))return 1;}
  auto radial=t::Response(j,2,d);if(t::Dot(radial,radial)>1e-22)return 2;
  t::Point force{.3,-.4,.8},other{-.2,.1,.6};auto response=t::Response(j,2,force),responseOther=t::Response(j,2,other);if(t::Dot(force,response)<0||!close(t::Dot(force,responseOther),t::Dot(other,response)))return 3;
  const double dqPitch=t::Dot(j.pitch,force)/2,dqYaw=t::Dot(j.yaw,force)/2;
  if(!close(t::Dot(force,t::Velocity(j,dqPitch,dqYaw)),2*(dqPitch*dqPitch+dqYaw*dqYaw)))return 4;
  auto applied=t::PositionImpulse(s,j,2,force);auto position=t::Scale(root_contact::Direction(applied),3.7);if(!close(t::Dot(position,position),3.7*3.7))return 5;
  auto rates=t::Rates(s,j,t::Velocity(j,s.pitchVelocity,s.yawVelocity));if(!close(rates.pitchVelocity,s.pitchVelocity)||!close(rates.yawVelocity,s.yawVelocity))return 6;
  for(double growth:{-.7,0.,.7}){
   t::Point rootVelocity{.1,-.2,.3};auto driven=t::DrivenVelocity(j,s.pitchVelocity,s.yawVelocity,d,growth,rootVelocity);
   auto shifted=[&](double time){auto q=s;q.pitch+=s.pitchVelocity*time;q.yaw+=s.yawVelocity*time;auto point=t::Scale(root_contact::Direction(q),3.7+growth*time);for(unsigned axis=0;axis<3;axis++)point[axis]+=rootVelocity[axis]*time;return point;};
   auto plus=shifted(eps),minus=shifted(-eps);for(unsigned axis=0;axis<3;axis++)if(!close(driven[axis],(plus[axis]-minus[axis])/(2*eps),1e-8))return 16;
   auto local=driven;for(unsigned axis=0;axis<3;axis++)local[axis]-=rootVelocity[axis];auto recovered=t::Rates(s,j,local);
   if(!close(recovered.pitchVelocity,s.pitchVelocity)||!close(recovered.yawVelocity,s.yawVelocity))return 17;
   if(!close(t::Dot(local,d),growth))return 18;
  }
  if(t::DrivenVelocity(j,s.pitchVelocity,s.yawVelocity,d,0)!=t::Velocity(j,s.pitchVelocity,s.yawVelocity))return 19;
  auto matrix=t::Mobility(j,2);for(unsigned a=0;a<3;a++)matrix[a][a]+=.6;auto solution=t::Solve(matrix,force);for(unsigned a=0;a<3;a++)if(!close(t::Dot(matrix[a],solution),force[a]))return 7;
 }
 std::vector<t::Station> stations;for(unsigned i=2;i<12;i++)stations.push_back({1.25,i*2.3});const double rootMass=2.2,total=t::ModalInertia(rootMass,stations);
 double exact=rootMass;for(unsigned i=2;i<12;i++)exact+=1.25*(i*2.3)*(i*2.3);if(!close(total,exact))return 8;
 auto law=t::EquivalentLaw(rootMass,total,{22,2*std::sqrt(22*rootMass)*.72});if(!close(law.stiffness/total,22/rootMass)||!close(law.damping/total,2*std::sqrt(22*rootMass)*.72/rootMass))return 9;
 // With the nominal modal inertia, both converted and original free-response
 // recurrences must match. This tests dynamics, rather than only coefficient ratios.
 double a=.5,av=.2,b=a,bv=av;const double dt=1./240;
 for(unsigned step=0;step<960;step++){
  double predicted=a+av*dt,other=b+bv*dt;auto ca=t::Spring(predicted,a,.11,0,1/rootMass,{22,2*std::sqrt(22*rootMass)*.72},dt);auto cb=t::Spring(other,b,.11,0,1/total,law,dt);
  predicted+=ca.angle;other+=cb.angle;av=(predicted-a)/dt;bv=(other-b)/dt;a=predicted;b=other;if(!close(a,b,1e-9)||!close(av,bv,1e-8))return 10;
 }
 if(std::abs(a-.11)>.002||std::abs(av)>.002)return 11;
 for(double sign:{-1.,1.}){auto c=t::SoftStop(sign*.8,.6,0,1/rootMass,90*total,dt);if(sign*c.angle>=0||c.multiplier>=0)return 12;auto release=t::SoftStop(sign*.4,.6,c.multiplier,1/rootMass,90*total,dt);if(release.multiplier!=0)return 13;}
 bool rejected=false;try{t::Solve({},{});}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 14;
 rejected=false;try{t::Jacobian({},0);}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 15;
 rejected=false;try{t::DrivenVelocity(t::Jacobian({},1),0,0,{2,0,0},0);}catch(const std::invalid_argument&){rejected=true;}if(!rejected)return 20;
 std::puts("PASS tangent Jacobians, radial nullspace, virtual work, fixed link, prescribed growth/translation and angular recovery, modal free-response parity, implicit stops and SPD solve; native acceptance untested");
}
