// Diagnostic source replay with externally supplied measured input cases.
// No captured user data is checked in; successful execution is not acceptance.
// Private continuous numerical replay using the exact extracted source kernel.
// The owned Session initializes its immutable seed; it is idle during direct
// process-isolated solver calls. No SDK, native input or game process is opened.
#include "../src/surface/runtime.cpp"
#include <fstream>
#include <cstdio>
namespace s=malemod::surface;
namespace k=malemod::surface::source;
struct Case{std::string name;int state;float glans,hang,controls[7],physics[8];k::V3 thighs[4];};

#ifdef MALEMOD_TEST_TANGENT_ROOT_GRADIENT
static bool RootGradientCheck(double& maxError){
 const auto normal=k::Unit(k::V3{.4f,.3f,.7f});
 for(int side=0;side<2;side++)for(bool material:{false,true}){
  k::V3 gradient[k::shaftNodeCount],finiteGradient[k::shaftNodeCount];const auto arm=k::cpBasis[side][2]*(k::CPRadii(side).z*.23f);
  const double mass=k::PDSuspensionMassAngular(side,arm,normal,gradient,material);
  double expected=k::PDEffectiveMassAngular(k::pdBody0+side,arm,normal);
  for(int i=1;i<k::shaftNodeCount;i++)expected+=i==1?k::PDEffectiveMassAngular(i,{},gradient[i]):k::pdInvMass[i]*k::Dot(gradient[i],gradient[i]);
  if(std::abs(mass-expected)>2e-6*(1+std::abs(expected)))return false;
  const double finite=k::PDSuspensionMassFiniteDifferenceAngular(side,arm,normal,finiteGradient,material);
  const double error=std::abs(mass-finite)/(1+std::abs(finite));maxError=(std::max)(maxError,error);
  if(error>.02)return false;
 }
 return true;
}
#endif
int main(int argc,char** argv){
 if(argc!=4)return 2;std::ifstream input(argv[1]);unsigned count=0;input>>count;if(!input||!count||count>100)return 3;
 std::vector<Case> cases(count);for(auto& c:cases){input>>c.name>>c.state>>c.glans>>c.hang;for(auto& x:c.controls)input>>x;for(auto& x:c.physics)input>>x;for(auto& p:c.thighs)input>>p.x>>p.y>>p.z;}if(!input)return 4;
 const bool enabled=std::atoi(argv[3])!=0;s::Session session;
 FILE* report=nullptr;fopen_s(&report,argv[2],"wb");if(!report)return 5;
 fprintf(report,"case,step,pitch,yaw,pitchVelocity,yawVelocity,softPitchLimit,driveDegrees,firstX,firstY,firstZ,tipX,tipY,tipZ,maxStationSpeed,rootError,firstLengthError\n");
 cases.push_back(cases.front());cases.back().name+="-after-extremes";
 double gradientError=0;
 k::PDInput previous{};bool havePrevious=false;unsigned invalid=0,reversed=0;float maxPitch=0,maxYaw=0,maxRate=0,maxSpeed=0,maxSettledRate=0;
 for(const auto& c:cases){
  s::Controls controls;controls.values[0]=float(c.state);controls.values[4]=c.glans;controls.values[6]=c.hang;const unsigned shape[7]={1,2,3,5,7,8,9};
  for(unsigned i=0;i<7;i++)controls.values[shape[i]]=c.controls[i];for(unsigned i=0;i<8;i++)controls.values[10+i]=c.physics[i];session.SetControls(controls);
  s::Frame f;f.seconds=0;f.rootContacts=enabled;f.thighEndpoints=std::array<s::Point,4>{};for(unsigned i=0;i<4;i++)(*f.thighEndpoints)[i]={c.thighs[i].x,c.thighs[i].y,c.thighs[i].z};f.collision=s::CollisionCalibration{{7.2f,7.2f},{{{3,0,70},{5.4f,0,86}}},6.4f};session.Step(f);
  auto target=k::PDReadInput(0,0);if(!havePrevious)previous=target;
  for(unsigned step=0;step<960;step++){
   k::PDSetInput(previous,target,k::min(1.f,float(step)/48.f));const float gait=step<480?sinf(step*.04f)*.3f:0,side=step<480?cosf(step*.035f)*.3f:0;k::StepConstraintSolver(1.f/240.f,gait,side);
   bool finite=true;float speed=0;for(int i=0;i<k::pdCount;i++){auto p=k::pdPosition[i],v=k::pdVelocity[i];finite&=std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);if(i<k::shaftNodeCount)speed=k::max(speed,k::Length(v));}
   #ifdef MALEMOD_TEST_TANGENT_ROOT_GRADIENT
   if(enabled&&step%16==0&&!RootGradientCheck(gradientError))invalid++;
#endif
   const auto spring=k::shaftSpring;finite&=std::isfinite(spring.pitch)&&std::isfinite(spring.yaw)&&std::isfinite(spring.pitchVelocity)&&std::isfinite(spring.yawVelocity);const float rootError=k::Length(k::shaftNodes[0]-k::ShaftRoot()),lengthError=fabsf(k::Length(k::shaftNodes[1]-k::shaftNodes[0])-k::constraintRestLength/11.f);if(!finite||rootError>1e-5f||lengthError>2e-5f)invalid++;
   if(fabsf(spring.pitch)>=1.5707963268f||fabsf(spring.yaw)>=1.5707963268f)reversed++;
   float rate=k::max(fabsf(spring.pitchVelocity),fabsf(spring.yawVelocity));maxPitch=k::max(maxPitch,fabsf(spring.pitch));maxYaw=k::max(maxYaw,fabsf(spring.yaw));maxRate=k::max(maxRate,rate);maxSpeed=k::max(maxSpeed,speed);if(step>=720)maxSettledRate=k::max(maxSettledRate,rate);
   if(step%16==0||step==959){auto first=k::shaftNodes[1],tip=k::shaftNodes[11];fprintf(report,"%s,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",c.name.c_str(),step,spring.pitch,spring.yaw,spring.pitchVelocity,spring.yawVelocity,k::ModeValue(.42f,.62f,.92f),k::rootDriveAngle,first.x,first.y,first.z,tip.x,tip.y,tip.z,speed,rootError,lengthError);}
  }
  previous=target;havePrevious=true;printf("EPISODE %s pitch=%.9g yaw=%.9g pitchRate=%.9g yawRate=%.9g\n",c.name.c_str(),k::shaftSpring.pitch,k::shaftSpring.yaw,k::shaftSpring.pitchVelocity,k::shaftSpring.yawVelocity);
 }
 fclose(report);printf("GRADIENT maximumRelativeMassError=%.9g\n",gradientError);printf("RESULT enabled=%d invalid=%u reversed=%u maxPitch=%.9g maxYaw=%.9g maxRate=%.9g maxSpeed=%.9g maxSettledRate=%.9g nativeGate=UNTESTED\n",enabled,invalid,reversed,maxPitch,maxYaw,maxRate,maxSpeed,maxSettledRate);return invalid?6:reversed?7:0;
}
