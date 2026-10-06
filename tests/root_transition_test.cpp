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
 Check(out.back()==delta.back(),"Distant anatomy motion was overwritten");Check(out[4]==delta[4],"Distant body morphology was overwritten");
 std::vector<PrecisePoint> shoulderRest={{2,0,9},{-10,0,9},{2,0,100},{0,-1,0}};
 RootTransition shoulderRamp(shoulderRest,3,opening);
 const auto shoulder=shoulderRamp.BodyDisplacement(0,f,2.9);
 Check(shoulder[0]>0&&shoulder[2]<f.radius-2.9,"Upper annulus lacks bounded forward recruitment");
 Check(shoulderRamp.BodyDisplacement(1,f,2.9)==PrecisePoint{},"Shoulder moves posterior pelvic tissue");
 Check(shoulderRamp.BodyDisplacement(2,f,2.9)==PrecisePoint{},"Shoulder reaches distant abdomen");
 // Sample the actual field across radial contact onset and the outer support.
 // Adjacent rings must participate progressively with no hard contact kink;
 // recruitment must wrap the sides as well as the upper sector.
 std::vector<PrecisePoint> rings;for(unsigned i=0;i<=800;i++)rings.push_back({2,0,i*.05});
 rings.push_back({2,9,0});RootTransition annulus(rings,unsigned(rings.size()-1),opening);
 Check(annulus.BodyDisplacement(unsigned(rings.size()-1),f,2.9)[0]>0,"Lateral pelvic annulus excluded from anterior ramp");
 double previousSlope=0;bool haveSlope=false;
 for(unsigned i=1;i<=800;i++){
  auto a=annulus.BodyDisplacement(i-1,f,2.9),b=annulus.BodyDisplacement(i,f,2.9);
  // The origin has no radial direction and is inside the removed opening.
  if(i<60)continue;
  const double slope=(b[2]-a[2])/.05;
  Check(1+slope>.05,"Concentric tissue rings collapse or reverse their radial order");
  if(haveSlope)Check(std::abs(slope-previousSlope)<.045,"Radial contact produces an abrupt slope discontinuity");
  previousSlope=slope;haveSlope=true;
 }
 auto neutral=f;neutral.radius=2.9;auto base=ramp.Evaluate(delta,neutral,2.9);for(unsigned i=0;i<4;i++)Check(base[i]==PrecisePoint{},"Neutral opening altered");
 auto scaledRest=rest,scaledOpening=opening,scaledDelta=delta;for(auto* v:{&scaledRest,&scaledOpening,&scaledDelta})for(auto& p:*v)for(auto& x:p)x*=2;
 RootTransition scaled(scaledRest,5,scaledOpening);auto twice=f;twice.radius*=2;twice.length*=2;twice.sourceLengthScale=2;auto doubled=scaled.Evaluate(scaledDelta,twice,5.8);for(unsigned i=0;i<out.size();i++){auto expected=out[i];for(auto& x:expected)x*=2;Check(Error(doubled[i],expected)<1e-12,"Calibrated unit scaling changes ramp");}
 auto rotate=[](PrecisePoint p){return PrecisePoint{-p[1],p[0],p[2]};};auto rotatedRest=rest,rotatedOpening=opening,rotatedDelta=delta;for(auto* v:{&rotatedRest,&rotatedOpening,&rotatedDelta})for(auto& p:*v)p=rotate(p);
 RootTransition rotated(rotatedRest,5,rotatedOpening,rotate({1,0,0}));auto turn=f;turn.axis=rotate(f.axis);turn.up=rotate(f.up);auto spun=rotated.Evaluate(rotatedDelta,turn,2.9);for(unsigned i=0;i<out.size();i++)Check(Error(spun[i],rotate(out[i]))<1e-12,"Rotated character frame changes ramp");
 std::vector<LightingFrame> light={{{1,0,0},{0,1,0},1},{{0,1,0},{1,0,0},-1},{{0,0,-1},{1,0,0},-1}};
 WeldEdgeLighting(light,{{2,0,1,.25}});auto expected=LightingUnit({.75,.25,0},{1,0,0});Check(Error(light[2].normal,expected)<1e-12,"Lighting seam ignores original edge donors");Check(light[2].sign==-1&&std::abs(LightingDot(light[2].normal,light[2].tangent))<1e-12,"UV handedness or tangent orthogonality lost");
 bool rejected=false;try{ramp.Evaluate(delta,f,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid neutral radius accepted");
 std::cout<<"PASS measured body/module seam, neutral/exterior, distal motion, unit/rotation covariance and donor lighting\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
