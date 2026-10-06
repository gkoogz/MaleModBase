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
  Check(b[0]>=0&&b[0]<=a[0]+1e-12,"Forward ramp oscillates or pulls surrounding pelvis backward");
  const double slope=(b[2]-a[2])/.05;
  Check(1+slope>.05,"Concentric tissue rings collapse or reverse their radial order");
  if(haveSlope)Check(std::abs(slope-previousSlope)<.045,"Radial contact produces an abrupt slope discontinuity");
  previousSlope=slope;haveSlope=true;
 }
 auto tilted=f;tilted.axis={0,0,1};tilted.up={-1,0,0};
 for(unsigned i=0;i<rings.size();i++)Check(annulus.BodyDisplacement(i,tilted,2.9)==annulus.BodyDisplacement(i,f,2.9),"Shaft rest angle rotates the pelvic ramp");
 auto neutral=f;neutral.radius=2.9;auto base=ramp.Evaluate(delta,neutral,2.9);for(unsigned i=0;i<4;i++)Check(base[i]==PrecisePoint{},"Neutral opening altered");
 // A noncircular measured opening is not inflated by its nominal radius.
 // Exterior pelvis stays exactly at rest, including at maximum dilation.
 std::vector<PrecisePoint> oval={{0,0,5},{0,3,0},{0,0,-4},{0,-3,0}};
 std::vector<PrecisePoint> pelvic={{0,0,5},{0,0,12},{0,0,30},{-20,0,5},{0,10,0},{0,0,20},{0,-3,0}};
 RootTransition bellRamp(pelvic,6,oval);auto biggest=f;biggest.radius=7.7;
 Check(bellRamp.BodyDisplacement(2,biggest,2.9)==PrecisePoint{},"Maximum bell reshapes exterior abdomen");
 Check(bellRamp.BodyDisplacement(3,biggest,2.9)==PrecisePoint{},"Maximum bell reshapes posterior pelvis");
 // The widened annulus now approaches this formerly exterior sample. Its
 // tail must still be negligible, rather than raising an abdominal platform.
 Check(Error(bellRamp.BodyDisplacement(5,biggest,2.9),{})<.005*2.9,"Broad bell raises exterior shoulder");
 auto smaller=biggest;smaller.radius=2.9;for(unsigned i=0;i<pelvic.size();i++)Check(bellRamp.BodyDisplacement(i,smaller,2.9)==PrecisePoint{},"Neutral noncircular pelvis altered");
 // Check the analytical outer boundary in a circular section: position and
 // both derivatives return to the original body, rather than a raised lip.
 const double radius0=1,dilation=biggest.radius*1.12-radius0,outer=radius0+dilation+.81*radius0+2.18*dilation;
 std::vector<PrecisePoint> lip={{0,0,outer-.001},{0,0,outer},{0,0,outer+.001},{0,1,0}};
 RootTransition lipRamp(lip,3,opening);auto near=lipRamp.BodyDisplacement(0,biggest,2.9);
 Check(Error(near,{0,0,0})<1e-9&&lipRamp.BodyDisplacement(1,biggest,2.9)==PrecisePoint{}&&lipRamp.BodyDisplacement(2,biggest,2.9)==PrecisePoint{},"Bell outer edge leaves a shelf");
 // A steep cylindrical fit must not compress adjacent native edge samples
 // into a nearly collapsed ring. Preserve half their radial spacing locally.
 std::vector<PrecisePoint> closeRings={{0,0,1.01},{0,0,1.02},{0,1,0}};
 RootTransition spacing(closeRings,2,opening);
 auto innerA=spacing.BodyDisplacement(0,biggest,2.9),innerB=spacing.BodyDisplacement(1,biggest,2.9);
 Check(.01+innerB[2]-innerA[2]>.0049,"Bell collapses adjacent inner rings");
 // Broaden both lateral sides independently of the upper bell. The field is
 // symmetric and must still return to the untouched body with a smooth tail.
 const double sideOuter=outer;
 std::vector<PrecisePoint> sideSamples={{0,1,0},{0,-1,0},{0,0,1},{0,sideOuter-.001,0},{0,sideOuter,0},{0,sideOuter+.001,0},{0,-sideOuter,0}};
 RootTransition sideBell(sideSamples,6,opening);
 auto positive=sideBell.BodyDisplacement(0,biggest,2.9),negative=sideBell.BodyDisplacement(1,biggest,2.9),upper=sideBell.BodyDisplacement(2,biggest,2.9);
 Check(positive[1]>upper[2]+.3,"Bell mouth was not widened laterally");
 Check(Error(positive,{negative[0],-negative[1],negative[2]})<1e-12,"Lateral bell is asymmetric");
 Check(Error(sideBell.BodyDisplacement(3,biggest,2.9),{})<1e-9,"Lateral bell has a raised outer lip");
 Check(Error(sideBell.BodyDisplacement(4,biggest,2.9),{})<1e-12&&Error(sideBell.BodyDisplacement(6,biggest,2.9),{})<1e-12,"Lateral flare moves its analytical outer boundary");
 Check(sideBell.BodyDisplacement(5,biggest,2.9)==PrecisePoint{},"Lateral flare reshapes exterior pelvis");
 const double lowerOuter=outer;
 std::vector<PrecisePoint> lowerSamples={{0,0,-1},{0,0,1},{0,0,-lowerOuter+.001},{0,0,-lowerOuter-.001},{-20,0,-1},{0,1,0}};
 RootTransition lowerBell(lowerSamples,5,opening);
 auto lowerRoot=lowerBell.BodyDisplacement(0,biggest,2.9);
 Check(-lowerRoot[2]>lowerBell.BodyDisplacement(1,biggest,2.9)[2]+1,"Underside root does not thicken at large size");
 Check(lowerRoot[0]>0&&lowerRoot[0]<.7*lowerBell.BodyDisplacement(1,biggest,2.9)[0],"Underside root does not return deeper into the pelvis");
 Check(Error(lowerBell.BodyDisplacement(2,biggest,2.9),{})<1e-9,"Deeper underside creates an exterior lip");
 Check(lowerBell.BodyDisplacement(3,biggest,2.9)==PrecisePoint{}&&lowerBell.BodyDisplacement(4,biggest,2.9)==PrecisePoint{},"Deeper underside moves exterior or posterior tissue");
 // Additional thickness ends near the opening, not at a new broader body
 // boundary. Equal-radius samples outside it retain the original radial law.
 std::vector<PrecisePoint> localSamples={{0,8,0},{0,0,8},{0,0,-8},{0,1,0}};
 RootTransition localRamp(localSamples,3,opening);
 auto farSide=localRamp.BodyDisplacement(0,biggest,2.9),farUp=localRamp.BodyDisplacement(1,biggest,2.9),farDown=localRamp.BodyDisplacement(2,biggest,2.9);
 Check(Error(farSide,{farUp[0],farUp[2],farUp[1]})<1e-12&&Error(farDown,{farUp[0],farUp[1],-farUp[2]})<1e-12,"Local thickness changes the surrounding pelvic/leg annulus");
 auto scaledRest=rest,scaledOpening=opening,scaledDelta=delta;for(auto* v:{&scaledRest,&scaledOpening,&scaledDelta})for(auto& p:*v)for(auto& x:p)x*=2;
 RootTransition scaled(scaledRest,5,scaledOpening);auto twice=f;twice.radius*=2;twice.length*=2;twice.sourceLengthScale=2;auto doubled=scaled.Evaluate(scaledDelta,twice,5.8);for(unsigned i=0;i<out.size();i++){auto expected=out[i];for(auto& x:expected)x*=2;Check(Error(doubled[i],expected)<1e-12,"Calibrated unit scaling changes ramp");}
 auto rotate=[](PrecisePoint p){return PrecisePoint{-p[1],p[0],p[2]};};auto rotatedRest=rest,rotatedOpening=opening,rotatedDelta=delta;for(auto* v:{&rotatedRest,&rotatedOpening,&rotatedDelta})for(auto& p:*v)p=rotate(p);
 RootTransition rotated(rotatedRest,5,rotatedOpening,rotate({1,0,0}));auto turn=f;turn.axis=rotate(f.axis);turn.up=rotate(f.up);auto spun=rotated.Evaluate(rotatedDelta,turn,2.9);for(unsigned i=0;i<out.size();i++)Check(Error(spun[i],rotate(out[i]))<1e-12,"Rotated character frame changes ramp");
 std::vector<LightingFrame> light={{{1,0,0},{0,1,0},1},{{0,1,0},{1,0,0},-1},{{0,0,-1},{1,0,0},-1}};
 WeldEdgeLighting(light,{{2,0,1,.25}});auto expected=LightingUnit({.75,.25,0},{1,0,0});Check(Error(light[2].normal,expected)<1e-12,"Lighting seam ignores original edge donors");Check(light[2].sign==-1&&std::abs(LightingDot(light[2].normal,light[2].tangent))<1e-12,"UV handedness or tangent orthogonality lost");
 const LightingFrame baked{{1,0,0},{0,1,0},-1},geometric{{0,0,1},{0,1,0},1};
 auto aligned=AlignGeometricNormal(baked,geometric,1);Check(aligned.normal==geometric.normal&&aligned.sign==baked.sign&&std::abs(LightingDot(aligned.normal,aligned.tangent))<1e-12,"Geometric normal alignment loses the surface frame or UV handedness");
 Check(AlignGeometricNormal(baked,geometric,0).normal==baked.normal,"Exterior cooked normal changed");
 std::vector<LightingFrame> joined={baked,geometric,{{0,1,0},{1,0,0},-1}};
 const auto exterior=joined[2];FairLightingNormals(joined,{0,1,2},{{1},{0},{}},{1,1,0});
 Check(Error(joined[0].normal,joined[1].normal)<.01,"Attachment normal crease persists across connected groups");
 Check(joined[2].normal==exterior.normal&&joined[2].tangent==exterior.tangent&&joined[2].sign==exterior.sign,"Normal fairing changes the exterior");
 for(auto f:joined)Check(std::abs(LightingDot(f.normal,f.tangent))<1e-12,"Normal fairing breaks tangent orthogonality");
 Check(joined[0].sign==-1&&joined[1].sign==1,"Normal fairing merges mirrored UV handedness");
 bool rejected=false;try{ramp.Evaluate(delta,f,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid neutral radius accepted");
 std::cout<<"PASS measured body/module seam, neutral/exterior, distal motion, unit/rotation covariance and donor lighting\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
