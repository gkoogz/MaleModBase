#pragma once
#include "fluid_types.h"
#include "clear_strand.h"
#include "thread_mesh.h"
#include "viscous_thread.h"
namespace teaching {
struct Fluid {
 volumeFluid::Settings settings;std::string error;
 explicit Fluid(bool =true){}
 bool ready=false,surfaceDeposits=false;double accumulator=0,clock=0;V3 origin{},lastTip{},lastDir{1,0,0};
 unsigned emitted[6]{},clearEmitted[8]{},steps=0;double emittedVolume=0;float viscosity=2,cohesion=1.8f,spread=1;
 volumeFluid::F4 collision[volumeFluid::collisionCount]{};std::vector<volumeFluid::FluidImpact> impacts;double flowCDF[513]{};float flowValues[513]{};double flowArea=1;
 unsigned variationSeed=0x6d2b79f5u;float pulseVolume[4]{},pulseDuration[4]{},angleGain[4]={1,1,1,1};int pulseChannel[4]{},pulseChannelCount=4;
 float lateralGain[4]{};
 float forceGain[4]={1,1,1,1};
 // A small alternating excursion with a softer return swing. The live shaft
 // solver consumes this target; fluid continues to follow the actual mesh lip.
 float MainLateralYaw(double time) const {
  float yaw=0;
  for(int e=0;e<4;e++){
   float age=(float)(time-EventTime(e+2));
   float attack=PulseEnvelopeAt(time,EventTime(e+2));
   float rebound=age>.20f?.28f*Ease((age-.20f)/.18f)*(1-Ease((age-.38f)/.40f)):0.f;
   yaw+=lateralGain[e]*(attack-rebound);
  }
  return settings.lateralWobbleDegrees*.01745329252f*yaw;
 }
 static const int clearCount=8;
 ClearStrand clear[clearCount];double clearStart[clearCount]{};bool clearScheduled[clearCount]{};bool passiveMode=false;
 ViscousThread streams[4];LiquidMesh mesh;
 std::vector<volumeFluid::Particle> surface;double delivered[4]{};
 void SetVariationSeed(unsigned seed){variationSeed=seed?seed:0x6d2b79f5u;}
 static unsigned RandomNext(unsigned& state){state^=state<<13;state^=state>>17;state^=state<<5;return state;}
 static float RandomSigned(unsigned& state){return (RandomNext(state)*(2.f/4294967295.f))-1.f;}
 void Clear(){ready=false;surfaceDeposits=false;accumulator=clock=0;steps=0;emittedVolume=0;passiveMode=false;memset(emitted,0,sizeof(emitted));memset(clearEmitted,0,sizeof(clearEmitted));memset(delivered,0,sizeof(delivered));memset(clearStart,0,sizeof(clearStart));memset(clearScheduled,0,sizeof(clearScheduled));surface.clear();impacts.clear();mesh.Clear();for(auto& s:clear)s.Reset();for(auto& s:streams)s.Reset();}
 bool Prepare(){settings=volumeFluid::config;
  if(!std::isfinite(settings.volume)||settings.volume<=0||settings.feed<.15f||settings.feed>10||settings.nozzle<.15f||settings.viscosity<0||settings.meshSides<8||settings.meshSides>20||settings.pulseVolumeVariation<0||settings.pulseVolumeVariation>.75f||settings.pulseDurationVariation<0||settings.pulseDurationVariation>.9f||settings.angleVariation<0||settings.angleVariation>1.f||settings.pulseTaper<0||settings.pulseTaper>1.f){error="Invalid stream configuration.";return false;}
  unsigned random=variationSeed;
  if(!std::isfinite(settings.lateralWobbleDegrees)||settings.lateralWobbleDegrees<0||settings.lateralWobbleDegrees>8){error="LateralWobbleDegrees must be 0..8.";return false;}
  if(!std::isfinite(settings.pulseForceVariation)||settings.pulseForceVariation<0||settings.pulseForceVariation>.3f){error="PulseForceVariation must be 0..0.3.";return false;}
  for(int e=0;e<4;e++){
   pulseVolume[e]=settings.volume*(1.f+settings.pulseVolumeVariation*RandomSigned(random));
   pulseDuration[e]=max(.15f,min(10.f,settings.feed*(1.f+settings.pulseDurationVariation*RandomSigned(random))));
   angleGain[e]=max(.05f,1.f+settings.angleVariation*RandomSigned(random));
  }
  // Separate random stream preserves all existing volume/duration/angle samples.
  unsigned lateralRandom=variationSeed^0xa511e9b3u;if(!lateralRandom)lateralRandom=1;
  float side=RandomSigned(lateralRandom)<0?-1.f:1.f;
  float largest=*std::max_element(pulseVolume,pulseVolume+4);
  for(int e=0;e<4;e++){
   lateralGain[e]=side*(.80f+.20f*RandomSigned(lateralRandom))*sqrtf(pulseVolume[e]/largest);
   side=-side;
  }
  unsigned forceRandom=variationSeed^0x63d83595u;if(!forceRandom)forceRandom=1;
  for(int e=0;e<4;e++){
   // Rejection sampling gives six equiprobable buckets without modulo bias.
   unsigned roll;do{roll=RandomNext(forceRandom);}while(roll>4294967292u);
   unsigned bucket=(roll-1u)%6u;
   forceGain[e]=1.f+settings.pulseForceVariation*(bucket==0?-1.5f:bucket==1?1.f:.25f*RandomSigned(forceRandom));
  }
  // When variance can cross the 1.5-second interval, retain one long pump
  // so the merged feed visibly carries through its neighbor.
  if(settings.pulseDurationVariation>0&&settings.feed*(1.f+.8f*settings.pulseDurationVariation)>1.5f){
   int longest=0;for(int e=1;e<4;e++)if(pulseDuration[e]>pulseDuration[longest])longest=e;
   if(pulseDuration[longest]<=1.5f)pulseDuration[longest]=min(10.f,settings.feed*(1.f+.8f*settings.pulseDurationVariation));
  }
  pulseChannelCount=0;double groupEnd=-1;
  for(int e=0;e<4;e++){
   double start=EventTime(e+2);
   if(e==0||start>groupEnd+1e-6){pulseChannel[e]=pulseChannelCount++;groupEnd=start+pulseDuration[e];}
   else{pulseChannel[e]=pulseChannelCount-1;groupEnd=max(groupEnd,start+pulseDuration[e]);}
  }
  BuildFlowProfile();
  float largestVolume=*std::max_element(pulseVolume,pulseVolume+4);
  float radius=min(settings.nozzle,powf(largestVolume/18.f,1.f/3));double peakSpeed=0;
  double mainEnd=0;for(int e=0;e<4;e++)mainEnd=max(mainEnd,(double)EventTime(e+2)+pulseDuration[e]);
  for(double t=0;t<=mainEnd;t+=1.0/120.0){
   double rate=0;for(int e=0;e<4;e++){double x=(t-EventTime(e+2))/pulseDuration[e];if(x>0&&x<1)rate+=forceGain[e]*pulseVolume[e]/pulseDuration[e]*Rate((float)x);}
   peakSpeed=max(peakSpeed,rate/(3.14159265*radius*radius));
  }
  if(peakSpeed>250){error="Variable pulse flow exceeds 250 game units/s. Increase nozzle radius or duration.";return false;}
  sequenceEnd=(float)max(max(20.0,mainEnd+settings.lifetime+1.0),4.5+settings.dropDuration+settings.dropHold+settings.lifetime+1.0);return true;
 }
 bool Begin(V3 tip){Clear();if(!Prepare())return false;origin=lastTip=tip;ready=true;error.clear();
  clearScheduled[0]=clearScheduled[1]=true;clearStart[0]=EventTime(0);clearStart[1]=EventTime(1);
  return true;
 }
 bool BeginPassive(V3 tip){if(!Begin(tip))return false;passiveMode=true;memset(clearScheduled,0,sizeof(clearScheduled));return true;}
 bool TriggerPassiveClear(){
  if(!ready||!passiveMode)return false;
  const double lifetime=settings.dropDuration+settings.dropHold+settings.lifetime;
  for(int i=0;i<clearCount;i++)if(!clearScheduled[i]||clock-clearStart[i]>lifetime){
   clear[i].Reset();clearStart[i]=clock;clearScheduled[i]=true;return true;
  }
  return false;
 }
 void BuildFlowProfile(){
  flowCDF[0]=0;for(int i=0;i<=512;i++){float x=i/512.f;float texture=settings.flowVariation*(1-settings.pulseTaper);flowValues[i]=FlowRate(x,settings.pulseTaper)*(1+texture*sinf(x*6.2831853f*3));if(i)flowCDF[i]=flowCDF[i-1]+(flowValues[i-1]+flowValues[i])*.5/512;}flowArea=flowCDF[512];for(double& f:flowCDF)f/=flowArea;
 }
 static float EventTime(int e){return e==0?2.5f:e==1?4.5f:7.f+1.5f*(e-2);}
 // Cumulative smooth-ramp flow, normalized to exactly one requested event volume.
 static double FlowIntegral(double x){x=x<0?0:x>1?1:x;const double r=.12,area=1-r;if(x<r){double u=x/r;return r*(u*u*u-.5*u*u*u*u)/area;}if(x>1-r)return 1-FlowIntegral(1-x);return (x-r*.5)/area;}
 static float FlowRate(float x,float taper=0){if(x<=0||x>=1)return 0;taper=Clamp(taper,0,1);float edge=.12f-.07f*taper;float u=min(1.f,min(x,1-x)/edge);float ramp=u*u*(3-2*u)/(1-edge);return ramp*expf(-2.6f*taper*x);}
 double Cumulative(double x) const {x=x<0?0:x>1?1:x;double q=x*512;int i=min(511,(int)q);return flowCDF[i]+(flowCDF[i+1]-flowCDF[i])*(q-i);}
 float Rate(float x) const {float q=Clamp(x,0,1)*512;int i=min(511,(int)q);return float((flowValues[i]+(flowValues[i+1]-flowValues[i])*(q-i))/flowArea);}

 bool Tick(float dt,V3 tip,V3 dir,V3 inherited){
  double next=clock+dt;
  for(int e=0;e<(passiveMode?clearCount:2);e++)if(clearScheduled[e]){
   double age=next-clearStart[e];if(age<=0)continue;
   double fraction=FlowIntegral(age/settings.dropDuration);
   size_t firstImpact=impacts.size();clear[e].Step(dt,(float)age,tip,dir,settings,collision,(float)fraction,settings.dropVolume,origin,&impacts);
   for(size_t k=firstImpact;k<impacts.size();k++)impacts[k].sourceId=variationSeed^(0xa531u+e);
   emittedVolume+=settings.dropVolume*(fraction-FlowIntegral((clock-clearStart[e])/settings.dropDuration));clearEmitted[e]=fraction>=1?1:0;
   if(age>settings.dropDuration+settings.dropHold+settings.lifetime)clearScheduled[e]=false;
  }
  if(passiveMode){clock=next;++steps;return true;}
  double targets[4]{},amount[4]{};float speeds[4]{};bool feed[4]{};
  float largestVolume=*std::max_element(pulseVolume,pulseVolume+4);float radius=min(settings.nozzle,powf(largestVolume/18.f,1.f/3));
  float peak=0;
  for(int e=0;e<4;e++){float x=(float)((clock+next)*.5-EventTime(e+2))/pulseDuration[e];if(x>0&&x<1)peak+=forceGain[e]*pulseVolume[e]/pulseDuration[e]*Rate(x)/(3.14159265f*radius*radius);}
  unsigned stride=peak>settings.threadSpacing*60?1:2;bool terminal=false;
  for(int e=0;e<4;e++){
   double fraction=Cumulative((next-EventTime(e+2))/pulseDuration[e]);targets[e]=pulseVolume[e]*fraction;
   int channel=pulseChannel[e];
   amount[channel]+=max(0.,targets[e]-delivered[e]);
   float mid=(float)((clock+next)*.5-EventTime(e+2))/pulseDuration[e];
   if(mid>0&&mid<1){float rate=max(.001f,Rate(mid));float axialExponent=max(0.f,1.f-1.5f*settings.pulseTaper);float axialFlow=forceGain[e]*pulseVolume[e]/pulseDuration[e]*powf(rate,axialExponent);speeds[channel]+=axialFlow/(3.14159265f*radius*radius);}
   feed[channel]=feed[channel]||(fraction>0&&fraction<1);
   if(fraction>=1&&targets[e]>delivered[e]+1e-9)terminal=true;
  }
  bool birth=(steps%stride)==stride-1||terminal;
  for(int channelIndex=0;channelIndex<4;channelIndex++){
   int e=(channelIndex+steps)%4;
   auto& stream=streams[e];size_t firstImpact=impacts.size();stream.Step(dt,(float)next,settings,collision,origin,volumeFluid::collisionCount,&impacts);
   for(size_t k=firstImpact;k<impacts.size();k++)impacts[k].sourceId=variationSeed^(0x917fu+e);
   if(birth&&amount[e]>1e-9)stream.Feed((float)amount[e],tip,dir,inherited+dir*min(settings.speedLimit,speeds[e]),(float)next,dt*stride,settings);
   stream.feeding=feed[e];
  }
  if(birth)for(int e=0;e<4;e++){if(targets[e]>delivered[e]+1e-9)++emitted[e+2];emittedVolume+=targets[e]-delivered[e];delivered[e]=targets[e];}
  clock=next;++steps;return true;
 }
 void BuildMesh(){
  mesh.Clear();surface.clear();for(const auto& stream:streams){
   bool contactOwned=false;if(surfaceDeposits)for(const auto& n:stream.nodes)if(n.reportedContact>1e-6f){contactOwned=true;break;}
   if(!contactOwned){stream.Append(mesh,settings.meshSides,surface);continue;}
   // Change display ownership only; preserve the solver's mass and momentum.
   auto display=stream;for(auto& n:display.nodes)if(n.reportedContact>1e-6f)n.lump=0;
   for(size_t i=0;i<display.links.size();i++)if(display.nodes[i].reportedContact>1e-6f||display.nodes[i+1].reportedContact>1e-6f){display.links[i].broken=true;display.links[i].volume=0;}
   display.Append(mesh,settings.meshSides,surface);
  }mesh.opaqueIndices=(unsigned)mesh.indices.size();
  for(const auto& strand:clear)if(strand.live&&strand.volume>1e-7f){
   std::vector<V3> p(strand.p,strand.p+strand.count);std::vector<float> r;float integral=0;
   for(int i=0;i<strand.count;i++){float t=float(i)/(strand.count-1);r.push_back(.7f+.9f*powf(t,6));if(i)integral+=Length(p[i]-p[i-1])*(r[i]*r[i]+r[i-1]*r[i-1])*.5f;}
   float scale=sqrtf(strand.volume/(3.14159265f*max(.0001f,integral)));for(float& a:r)a*=scale;
   mesh.Tube(p,r,settings.meshSides,strand.volume);for(size_t i=0;i<p.size();i++)surface.push_back({p[i],3,{},r[i]});
  }
 }
 void Advance(float dt,V3 tip,V3 dir,V3 inherited){
  impacts.clear();if(!ready||!std::isfinite(dt)||dt<=0)return;
  // A brief render hitch must not tear down the teaching sequence. Keep the
  // solver bounded and let the timeline resume smoothly on the next frame.
  dt=min(dt,maxFrameAdvance);
  if(Length(tip-lastTip)>100){Clear();return;}
  // The caller supplies actor-root translation. Deriving velocity from the
  // nozzle itself also includes skeletal rotation, which made already-airborne
  // liquid turn with Wolverine. Direction still follows the live outlet, while
  // emitted material inherits only whole-character movement.
  V3 velocity=inherited;float speed=Length(velocity);if(speed>settings.speedLimit)velocity=velocity*(settings.speedLimit/speed);
  accumulator+=dt;const double fixed=1.0/120.0;int n=(int)((accumulator+1e-8)/fixed);
  for(int k=0;k<n;k++){float t=(k+1.f)/max(1.f,float(n));if(!Tick((float)fixed,lastTip+(tip-lastTip)*t,Unit(lastDir+(dir-lastDir)*t),velocity))return;}
  accumulator-=n*fixed;lastTip=tip;lastDir=dir;if(n)BuildMesh();
 }
 int Live()const {int n=0;for(const auto& s:streams)if(s.live)n+=(int)s.nodes.size();for(const auto& s:clear)if(s.live)n+=25;return ready?n:0;}
};

struct PassiveThrobGate {
 bool armed=true;
 void Reset(){armed=true;}
 bool Update(float pulse){
  if(!std::isfinite(pulse))return false;
  if(pulse<=.25f){armed=true;return false;}
  if(armed&&pulse>=.5f){armed=false;return true;}
  return false;
 }
};
}
