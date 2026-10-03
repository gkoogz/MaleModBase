#include <malemod/surface/presentation.hpp>
#include <iostream>
using namespace malemod::surface;
static void Check(bool value){if(!value)throw std::runtime_error("Presentation invariant failed");}
int main(){try{
 PresentationFrames a{},b{};b[12].origin={3,-2,1};b[12].basis={{{0,1,0},{-1,0,0},{0,0,1}}};
 const PresentationPoint p{1,0,0},q{0,2,1};auto end=[&](PresentationPoint x){return Add(b[12].origin,Rotate(b[12].basis,x));};
 for(unsigned i=0;i<=100;i++){
  PresentationPlan plan(a,b,i/100.);auto x=plan.Point(p,end(p),{12,1}),y=plan.Point(q,end(q),{12,1});
  Check(std::abs(Dot(Sub(x,y),Sub(x,y))-Dot(Sub(p,q),Sub(p,q)))<1e-12);
  auto direction=plan.Direction(p,Rotate(b[12].basis,p),{12,1});Check(std::abs(Dot(direction,direction)-1)<1e-12);
  const auto fixed=plan.Point({0,0,0},{0,0,0},{0,0});Check(fixed==PresentationPoint{0,0,0});
 }
 Check(PresentationPlan(a,b,0).Point(p,end(p),{12,1})==p);Check(PresentationPlan(a,b,1).Point(p,end(p),{12,1})==end(p));
 auto bad=b;bad[13].basis[2]={0,0,-1};bool rejected=false;try{PresentationPlan plan(a,bad,.5);}catch(const std::invalid_argument&){rejected=true;}Check(rejected);
 std::cout<<"PASS: material-frame interpolation preserves rigid distances through 90-degree motion, exact endpoints and fixed boundaries; reflected frames rejected\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
