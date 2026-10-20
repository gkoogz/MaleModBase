#include "../include/malemod/surface/angle_conditioned_collar.hpp"
#include <iostream>
#include <stdexcept>
using namespace malemod::surface::angle_collar;
static void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(){
 const Point forward{1,0,0},up{0,0,1},side{0,1,0},root{0,0,0};
 auto response=[&](double degrees){const double a=degrees*3.14159265358979323846/180.;return Response({std::cos(a),0,std::sin(a)},forward,up);};
 for(double a=-89;a<=0;a+=.1)Check(response(a)==0,"Downward pose changed");
 double previous=0;
 for(double a=0;a<=89;a+=.01){const double v=response(a);Check(v>=previous-1e-14&&v>=0&&v<=1,"Nonmonotone/unbounded angle response");previous=v;}
 Check(response(25)==1&&response(60)==1,"Upward endpoint");
 Check(SliderResponse(25)==1&&SliderResponse(50)==0&&SliderResponse(100)==0,"Angle slider mapping");
 Check(SliderResponse(35)>0&&SliderResponse(35)<SliderResponse(30),"Moderate slider ordering");
 Check(GrowthResponse(2,3)==0&&GrowthResponse(3,3)==0&&GrowthResponse(6,3)==1,"Neutral/small anatomy response");
 RestWeights weights({{0,0,0},{3,0,0},{6,0,0}},{{0,0,0},{0,1,0},{0,-1,0}});
 Check(weights.Weight(0,6)==0&&weights.Weight(1,6)==.5&&weights.Weight(2,6)==1,"Rest-space handoff endpoints");
 Check(response(.001)<1e-10&&1-response(24.999)<1e-10,"Angle endpoint derivative discontinuity");
 for(int i=0;i<10000;i++){double a=i*6.283185307179586/10000.;double w=DorsalWeight({0,std::cos(a),std::sin(a)},root,side,up);Check(w>=0&&w<=1,"Sector roundoff escaped unit interval");if(std::sin(a)<=0)Check(w==0,"Ventral anatomy changed");}
 Point original{3,4,5},supported{7,-8,12};auto p=Reconcile(original,supported,1,1,up);
 Check(p[0]==original[0]&&p[1]==original[1]&&p[2]==12,"Corrective narrowed or advanced anatomy");
 Check(Reconcile(original,supported,0,1,up)==original,"Ventral exact preservation");
 Check(Reconcile(original,supported,1,0,up)==original,"Inactive exact preservation");
 Check(Reconcile(original,{9,9,1},1,1,up)==original,"Corrective depressed collar");
 bool invalid=false;try{Response({0,0,0},forward,up);}catch(const std::invalid_argument&){invalid=true;}Check(invalid,"Invalid guide accepted");
 std::cout<<"Angle response: continuity, boundedness, no downward correction, horizontal and ventral preservation passed\n";
}
