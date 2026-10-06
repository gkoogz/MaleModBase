#include <malemod/surface/root_transition.hpp>
#include <malemod/surface/lighting.hpp>
#include <iostream>
using namespace malemod::surface;
static void Check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
static double Error(PrecisePoint a,PrecisePoint b){double e=0;for(unsigned k=0;k<3;k++)e=std::max(e,std::abs(a[k]-b[k]));return e;}
int main(){try{
 std::vector<PrecisePoint> opening={{0,-1,0},{0,0,1},{0,1,0},{0,0,-1}};
 std::vector<PrecisePoint> rest=opening;rest.push_back({-100,0,0});rest.insert(rest.end(),opening.begin(),opening.end());rest.push_back({100,0,0});
 RootTransition ramp(rest,5,opening);GraftFrame f{{0,0,0},{1,0,0},{0,0,1},7,25,1};
 std::vector<PrecisePoint> delta(rest.size(),{3,4,5});auto out=ramp.Evaluate(delta,f,2.9);
 for(unsigned i=0;i<4;i++)Check(out[i]==out[i+5],"Body/module sewn displacement differs");
 Check(out.back()==delta.back(),"Distant anatomy motion was overwritten");Check(out[4]==PrecisePoint{},"Distant pelvis was displaced");
 auto neutral=f;neutral.radius=2.9;auto base=ramp.Evaluate(delta,neutral,2.9);for(unsigned i=0;i<5;i++)Check(base[i]==PrecisePoint{},"Neutral pelvis altered");
 auto scaledRest=rest,scaledOpening=opening,scaledDelta=delta;for(auto* v:{&scaledRest,&scaledOpening,&scaledDelta})for(auto& p:*v)for(auto& x:p)x*=2;
 RootTransition scaled(scaledRest,5,scaledOpening);auto twice=f;twice.radius*=2;twice.length*=2;twice.sourceLengthScale=2;auto doubled=scaled.Evaluate(scaledDelta,twice,5.8);for(unsigned i=0;i<out.size();i++){auto expected=out[i];for(auto& x:expected)x*=2;Check(Error(doubled[i],expected)<1e-12,"Calibrated unit scaling changes ramp");}
 auto rotate=[](PrecisePoint p){return PrecisePoint{-p[1],p[0],p[2]};};auto rotatedRest=rest,rotatedOpening=opening,rotatedDelta=delta;for(auto* v:{&rotatedRest,&rotatedOpening,&rotatedDelta})for(auto& p:*v)p=rotate(p);
 RootTransition rotated(rotatedRest,5,rotatedOpening,rotate({1,0,0}));auto turn=f;turn.axis=rotate(f.axis);turn.up=rotate(f.up);auto spun=rotated.Evaluate(rotatedDelta,turn,2.9);for(unsigned i=0;i<out.size();i++)Check(Error(spun[i],rotate(out[i]))<1e-12,"Rotated character frame changes ramp");
 std::vector<LightingFrame> light={{{1,0,0},{0,1,0},1},{{0,1,0},{1,0,0},-1},{{0,0,-1},{1,0,0},-1}};
 WeldEdgeLighting(light,{{2,0,1,.25}});auto expected=LightingUnit({.75,.25,0},{1,0,0});Check(Error(light[2].normal,expected)<1e-12,"Lighting seam ignores original edge donors");Check(light[2].sign==-1&&std::abs(LightingDot(light[2].normal,light[2].tangent))<1e-12,"UV handedness or tangent orthogonality lost");
 bool rejected=false;try{ramp.Evaluate(delta,f,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid neutral radius accepted");
 std::cout<<"PASS measured body/module seam, neutral/exterior, distal motion, unit/rotation covariance and donor lighting\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
