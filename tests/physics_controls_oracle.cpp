// SDK-free harness around exact statements from the immutable reference.
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
using std::min; using std::max;
struct V3 {float x,y,z;};
static V3 operator+(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 operator-(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static float Length(V3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
static float Smoother01(float t){t=max(0.f,min(1.f,t));return t*t*t*(t*(t*6.f-15.f)+10.f);}
static float shaftMode,physUI[8],physValues[8],constraintRestLength;
static V3 constraintBallRest[2]{{0,0,3},{0,0,3}};
static V3 CPRestSlack(int){return {};}
static V3 RestBallAnchor(int){return {};}
static V3 CPRadii(int){return {4,5,6};}
struct PDSuspensionData {float rest,hard,compliance,shearCompliance,ratio;};
#include "data/wolverine-physics-controls.inc"
int main(){
 std::cout<<std::setprecision(9);
 while(std::cin>>shaftMode>>constraintRestLength){
  for(float& value:physUI)if(!(std::cin>>value))return 2;
  for(float& value:physValues)if(!(std::cin>>value))return 2;
  auto values=SourceMaterialProfile();
  for(unsigned i=0;i<values.size();i++)std::cout<<(i?" ":"")<<values[i];
  std::cout<<"\n";
 }
}
