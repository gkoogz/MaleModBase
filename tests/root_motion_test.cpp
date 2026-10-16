#include <malemod/surface/root_motion.hpp>
#include <iostream>
using namespace malemod::surface::root_motion;
void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
bool Near(Point a,Point b,double tolerance=1e-11){return std::sqrt(Dot(Add(a,Scale(b,-1)),Add(a,Scale(b,-1))))<tolerance;}
int main(){try{
 Rotation quarter({1,0,0},{0,0,1},{0,1,0});Point root={3,-2,5},p={7,1,6},zero={};
 const auto q=Add(p,Scale(root,-1));const auto moving=Add(quarter.Apply(q,1),Scale(q,-1));
 for(unsigned k=0;k<=100;k++){
  const double t=k/100.;auto d=quarter.Blend(p,root,zero,moving,t);auto actual=Add(q,d);
  Check(Near(actual,quarter.Apply(q,t)),"Rigid section lost its material rotation");
  Check(std::abs(Dot(actual,actual)-Dot(q,q))<1e-10,"Rigid section collapsed");
 }
 Point fixed={.2,-.7,.1},elastic={2,.5,-1};
 Check(quarter.Blend(p,root,fixed,elastic,0)==fixed,"Fixed seam endpoint changed");
 Check(quarter.Blend(p,root,fixed,elastic,1)==elastic,"Moving endpoint changed");
 Rotation same({1,0,0},{1,0,0},{0,0,1});
 Check(Near(same.Blend(p,root,fixed,elastic,.37),Add(Scale(fixed,.63),Scale(elastic,.37))),"Zero rotation changed morphology interpolation");
 Rotation half({1,0,0},{-1,0,0},{0,1,0});Point halfMoving=Add(half.Apply(q,1),Scale(q,-1));
 Check(std::abs(Dot(Add(q,half.Blend(p,root,zero,halfMoving,.5)),Add(q,half.Blend(p,root,zero,halfMoving,.5)))-Dot(q,q))<1e-10,"Half turn collapsed");
 // A rigid change of coordinate frame must commute with the shared law.
 Rotation coordinate({0,1,0},{0,0,1},{1,0,0});auto rotate=[&](Point a){return coordinate.Apply(a,1);};
 Rotation transported(rotate({1,0,0}),rotate({0,0,1}),rotate({0,1,0}));
 Check(Near(transported.Blend(rotate(p),rotate(root),rotate(fixed),rotate(elastic),.43),rotate(quarter.Blend(p,root,fixed,elastic,.43))),"Coordinate covariance failed");
 bool rejected=false;try{Rotation bad({0,0,0},{1,0,0},{0,1,0});}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Degenerate direction accepted");
 rejected=false;try{quarter.Blend(p,root,fixed,elastic,1.1);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Out of range blend accepted");
 std::cout<<"PASS root motion: endpoints, 101 rigid sections, half turn, elastic interpolation and covariance\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
