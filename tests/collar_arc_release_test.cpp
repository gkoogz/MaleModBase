#include <malemod/surface/collar_arc_release.hpp>
#include <iostream>
#include <stdexcept>
using namespace malemod::surface::collar_arc;
void Check(bool b){if(!b)throw std::runtime_error("Collar arc regression");}
int main(){
 auto n=FromAngle(30);Check(n.upper==0&&n.lower==0);
 Check(FromAngle(-25).upper==1&&FromAngle(85).lower==1);
 double previous=1;
 for(double d=-80;d<=120;d+=.01){auto r=FromAngle(d);Check(r.upper<=previous+1e-14);previous=r.upper;
  for(double c=-1;c<=1;c+=.1){double w=Weight(c,.8,r);Check(w>=0&&w<=.8);Check(Weight(c,0,r)==0);}
 }
 Check(Weight(-1,1,FromAngle(-25))==0&&Weight(1,1,FromAngle(85))==0);
 Check(Weight(1,1,FromAngle(-25))==1);
 Check(FromAngle(30-1e-3).upper<1e-10&&1-FromAngle(-25+1e-3).upper<1e-10);
 Check(ShaftWeight(0,1,0,3)==0&&ShaftWeight(.25,1,3,3)==1);
 Check(ShaftWeight(.25,0,2,3)==.25);
 bool bad=false;try{Weight(1,2,n);}catch(const std::invalid_argument&){bad=true;}Check(bad);
 std::cout<<"Arc release bounds, neutral identity, ownership exclusion, opposite arc preservation, C2 endpoints and exact seam passed\n";
}
