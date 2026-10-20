#include <malemod/surface/scrotal_compliance.hpp>
#include <iostream>
using namespace malemod::surface::scrotal;
static void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
static float Length(Vector x){return std::sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);}
int main(){try{
 static_assert(MechanicalPartition::shaftNodes==12 && MechanicalPartition::lobes==2);
 Check(SkinOwnership(1,1)==0 && SkinOwnership(0,0)==0,"Shaft/body must not be skin-owned");
 Check(SkinOwnership(1,0)==1,"Scrotal core skin lost ownership");
 for(int a=0;a<=100;a++)for(int b=0;b<=100;b++){
  float w=SkinOwnership(a/100.f,b/100.f);Check(w>=0&&w<=1,"Unbounded material mask");
  if(b>=50)Check(w==0,"Shaft-owned donor received soft skin motion");
 }
 float low=0,high=0;
 for(float stiffness:{1.f,50.f,100.f})for(float bounce:{1.f,50.f,100.f})for(float radius:{.5f,5.f,20.f}){
  Envelope skin;float peak=0;
  for(int i=0;i<720;i++){
   float t=i/240.f;Vector center{radius*.6f*std::sin(8*t),radius*.3f*std::cos(6*t),0};
   skin.Step(1.f/240,center,radius,stiffness,bounce);peak=std::max(peak,Length(skin.Displacement()));
   Check(Length(skin.Displacement())<=radius*.14001f,"Unbounded skin displacement");
  }
  Check(peak>radius*.005f,"Skin has no secondary response");
  Vector last{radius*.6f*std::sin(8*719/240.f),radius*.3f*std::cos(6*719/240.f),0};
  for(int i=0;i<1200;i++)skin.Step(1.f/240,last,radius,stiffness,bounce);
  Check(Length(skin.Displacement())<radius*1e-5f,"Skin failed to settle");
  skin.Reset();Check(Length(skin.Displacement())==0,"Reset retained motion");
 }
 for(float stiffness:{1.f,100.f}){
  Envelope skin;for(int i=0;i<240;i++)skin.Step(1.f/240,{i*.002f,0,0},5,stiffness,50);
  (stiffness<50?low:high)=Length(skin.Displacement());
 }
 Check(low>high,"Stiffness control does not govern skin compliance");
 Check(Response(1,50).angularStiffness<Response(100,50).angularStiffness,"Angular stiffness mapping");
 std::cout<<"PASS: continuous rod partition, protected shaft ownership, 27 skin scenarios, bounds, decay, reset, stiffness response\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
