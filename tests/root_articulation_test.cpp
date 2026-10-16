#include <malemod/surface/root_articulation.hpp>
#include <iostream>
using namespace malemod::surface::root_articulation;
using namespace malemod::surface::root_motion;
void Check(bool b){if(!b)throw std::runtime_error("Root articulation regression");}
int main(){
 std::vector<Point> points;std::vector<Face> faces;std::vector<bool> fixed,moving;
 for(unsigned ring=0;ring<13;ring++)for(unsigned j=0;j<32;j++){
  double a=j*6.283185307179586/32;points.push_back({double(ring),2*std::cos(a),2*std::sin(a)});fixed.push_back(ring==0);moving.push_back(ring>=4);
  if(ring){unsigned a0=(ring-1)*32+j,b0=(ring-1)*32+(j+1)%32,a1=ring*32+j,b1=ring*32+(j+1)%32;faces.push_back({a0,b0,a1});faces.push_back({b0,b1,a1});}
 }
 Junction junction(points,faces,fixed,moving);
 for(double angle:{-.52,0.,.96}){
  malemod::surface::root_articulation::Rotation rotate({0,0,0},{1,0,0},{0,0,1},angle);auto out=junction.Apply(rotate);
  Check(rotate.Apply({0,0,0})==Point({0,0,0}));
  for(unsigned i=0;i<points.size();i++){
   Finite(out[i]);if(fixed[i])Check(out[i]==points[i]);if(moving[i])Check(out[i]==rotate.Apply(points[i]));
   if(angle==0){auto error=Add(out[i],Scale(points[i],-1));Check(Dot(error,error)<1e-16);}
  }
  // A proximal handle and a distal one share exactly the same angle, which
  // specifically detects the rejected delayed or second-half bend behavior.
  auto proximal=Scale(Add(out[4*32],out[4*32+16]),.5),distal=Scale(Add(out[12*32],out[12*32+16]),.5);
  Check(std::abs(std::atan2(proximal[2],proximal[0])-angle)<1e-12);
  auto delta=Add(distal,Scale(proximal,-1));Check(std::abs(std::atan2(delta[2],delta[0])-angle)<1e-12);
  auto diameter=Add(out[4*32],Scale(out[4*32+16],-1));Check(std::abs(Dot(diameter,diameter)-16)<1e-12);
 }
 bool rejected=false;try{auto bad=moving;bad[0]=true;Junction conflict(points,faces,fixed,bad);}catch(const std::invalid_argument&){rejected=true;}Check(rejected);
 // Passing below the original height must never reclassify a moving shaft
 // handle as junction material. Live pose values use the same ownership.
 auto live=points;for(unsigned i=0;i<live.size();i++)if(moving[i])live[i][2]-=10;
 malemod::surface::root_articulation::Rotation turn({0,0,0},{1,0,0},{0,0,1},.6);
 auto liveOut=junction.Apply(live,turn);for(unsigned i=0;i<live.size();i++){if(moving[i])Check(liveOut[i]==turn.Apply(live[i]));if(fixed[i])Check(liveOut[i]==live[i]);}
 for(unsigned frame=0;frame<12;frame++){
  auto pose=points;for(unsigned i=0;i<pose.size();i++)pose[i][2]+=.2*std::sin(i*.13+frame*.7);
  junction.UpdateReference(pose);Junction fresh(pose,faces,fixed,moving);
  auto cached=junction.Apply(turn),exact=fresh.Apply(turn);
  for(unsigned i=0;i<pose.size();i++)for(unsigned k=0;k<3;k++)Check(std::abs(cached[i][k]-exact[i][k])<1e-11);
 }
 junction.UpdateReference(points);
 std::vector<Point> path;unsigned calls=0;
 Check(ContinueEndpoint(points,path,[&](double t){return junction.Apply(malemod::surface::root_articulation::Rotation({0,0,0},{1,0,0},{0,0,1},.6*t));},[&](auto&){calls++;return true;}));
 Check(calls==8);auto direct=junction.Apply(turn);for(unsigned i=0;i<path.size();i++){auto d=Add(path[i],Scale(direct[i],-1));Check(Dot(d,d)<1e-20);}
 auto unchanged=path;Check(!ContinueEndpoint(points,path,[&](double){return direct;},[](auto&){return false;}));Check(path==unchanged);
 std::cout<<"Root pivot, proximal/distal rigid angle, girth, fixed boundary and identity passed\n";
}
