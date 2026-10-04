#include <malemod/surface/runtime.hpp>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <atomic>
#include "surface-kernel.inc"
#include <malemod/surface/collar_field.hpp>
#include <malemod/surface/garment_support.hpp>

namespace malemod::surface {
namespace {
namespace kernel=source;
#ifdef MALEMOD_SURFACE_PROCESS_ISOLATED
std::atomic<bool> processSessionOwned{false};
#endif
constexpr float lowShape[7]={.85f,1.0f,.95f,1.0f,-80.f,-2.f,-3.f};
constexpr float neutralPhys[8]={78.f,86.f,12.f,62.f,28.f,94.f,18.f,72.f};
constexpr float lowPhys[8]={-100,0,0,10,0,0,0,10};
constexpr float highPhys[8]={400,100,100,200,100,100,100,200};
void Map(const Controls& controls){
 const auto& v=controls.values;
 for(float value:v)if(!std::isfinite(value))throw std::invalid_argument("Non-finite control");
 if(v[0]!=std::floor(v[0])||v[0]<0||v[0]>2)throw std::invalid_argument("State must be 0, 1 or 2");
 for(unsigned i=1;i<18;i++)if(v[i]<(i==2||i==4?0.f:1.f)||v[i]>100.f)throw std::invalid_argument("Control outside source range");
 kernel::physicsState=int(v[0]);
 const unsigned shapeIndices[7]={1,2,3,5,7,8,9};
 for(unsigned i=0;i<7;i++){
  kernel::sliderUI[i]=v[shapeIndices[i]];
  kernel::sliderValues[i]=i==1?kernel::MapLength100(kernel::sliderUI[i]):kernel::MapControl100(kernel::sliderUI[i],lowShape[i],kernel::neutralShape[i],kernel::sliderSpecs[i].hi);
 }
 kernel::glansUI=v[4];kernel::hangUI=kernel::effectiveHangUI=v[6];
 for(unsigned i=0;i<8;i++){
  kernel::physUI[i]=v[i+10];kernel::physValues[i]=kernel::MapControl100(v[i+10],lowPhys[i],neutralPhys[i],highPhys[i]);
 }
}
Surface Decode(const unsigned char* packed,unsigned count,unsigned first){
 Surface out;out.positions.resize(count);out.normals.resize(count);out.tangents.resize(count);out.uv.resize(count);out.sourceVertexIDs.resize(count);
 for(unsigned i=0;i<count;i++){
  const auto* p=packed+i*32;std::memcpy(&out.positions[i],p,12);
  out.normals[i]={p[16]/255.f*2-1,p[17]/255.f*2-1,p[18]/255.f*2-1};
  out.tangents[i]={p[12]/255.f*2-1,p[13]/255.f*2-1,p[14]/255.f*2-1};
  std::uint16_t uv[2];std::memcpy(uv,p+28,4);kernel::D3DXFloat16To32Array(out.uv[i].data(),uv,2);
  out.sourceVertexIDs[i]=first+i;
 }
 return out;
}
}
struct Session::Impl {
 std::mutex submitMutex,mutex;
 std::condition_variable cv;
 std::function<void()> job;
 bool stop=false;
 std::thread worker;
 std::vector<unsigned char> seed;
 std::vector<Point> collarQueries;
 Controls controls;
 explicit Impl(const Controls& controls):worker([this]{Loop();}){
  try{Call([&]{Initialize(controls);});}catch(...){Stop();throw;}
 }
 ~Impl(){Stop();}
 void Stop(){ {std::lock_guard<std::mutex> guard(mutex);stop=true;}cv.notify_one();if(worker.joinable())worker.join(); }
 void Loop(){
  for(;;){std::function<void()> next;{
   std::unique_lock<std::mutex> guard(mutex);cv.wait(guard,[&]{return stop||bool(job);});
   if(stop)return;next=std::move(job);
  }next();}
 }
 void Call(std::function<void()> fn){
  std::lock_guard<std::mutex> submission(submitMutex);
  auto task=std::make_shared<std::packaged_task<void()>>(std::move(fn));auto done=task->get_future();
  {std::lock_guard<std::mutex> guard(mutex);job=[task]{(*task)();};}cv.notify_one();done.get();
 }
 void Initialize(const Controls& controls){
  this->controls=controls;
  Map(controls);seed.resize(50915*32);
  for(unsigned i=0;i<kernel::collarNormalTriangleCount*3;i++)std::memcpy(seed.data()+kernel::collarNormalTriangleIndices[i]*32,kernel::collarNormalTriangleBasePositions+i*3,12);
  for(unsigned i=0;i<kernel::pelvisControlCount;i++)std::memcpy(seed.data()+kernel::pelvisControlIndices[i]*32,kernel::pelvisControlBasePositions+i*3,12);
  for(unsigned i=0;i<kernel::graftCount;i++)std::memcpy(seed.data()+(47050+i)*32,kernel::morph_base+i*3,12);
  kernel::ResetCompliantDynamics();kernel::ResetPelvicAttachmentBody(seed.data());kernel::EvaluateAnatomy(seed.data(),47050);
 }
 void Advance(const Frame& frame){
  auto validateSupport=[](Point p,float limit){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z>double(limit)*limit+1e-5)throw std::invalid_argument("Garment support exceeds source acceleration budget");};
  validateSupport(frame.garment.shaftAcceleration,maximumShaftSupportAcceleration);for(auto p:frame.garment.lobeAcceleration)validateSupport(p,maximumLobeSupportAcceleration);
  kernel::surfaceGarmentEnabled=frame.garment.enabled;
  kernel::surfaceGarmentContactReaction=frame.garment.contactReaction;
  auto force=frame.garment.shaftAcceleration;kernel::surfaceGarmentShaft={force.x,force.y,force.z};
  for(unsigned i=0;i<2;i++){force=frame.garment.lobeAcceleration[i];kernel::surfaceGarmentLobes[i]={force.x,force.y,force.z};}
  // Commit cursor and pending impulses only after every component validates.
  // A rejected source submission must remain retryable without losing momentum.
  auto nextCursor=kernel::surfaceGarmentCursor;auto delta=nextCursor.Consume(frame.garment);
  std::array<kernel::V3,12> nextRod;std::array<kernel::V3,2> nextLobes,nextAngular;
  std::copy_n(kernel::surfaceGarmentPendingRod,12,nextRod.begin());std::copy_n(kernel::surfaceGarmentPendingLobes,2,nextLobes.begin());std::copy_n(kernel::surfaceGarmentPendingAngular,2,nextAngular.begin());
  bool nextReady=kernel::surfaceGarmentPendingReady;
  if(delta.reset){nextRod={};nextLobes={};nextAngular={};nextReady=false;}
  auto append=[&](auto& pending,ImpulsePoint p){for(double x:{p.x,p.y,p.z})if(!std::isfinite(x)||std::abs(x)>1e6)throw std::invalid_argument("Unstable pending garment impulse");if(p.x||p.y||p.z){auto next=pending+kernel::V3{float(p.x),float(p.y),float(p.z)};for(float x:{next.x,next.y,next.z})if(!std::isfinite(x)||std::abs(x)>1e6)throw std::invalid_argument("Unstable accumulated garment impulse");pending=next;nextReady=true;}};
  for(unsigned i=2;i<12;i++)append(nextRod[i],delta.rod[i]);
  for(unsigned i=0;i<2;i++){append(nextLobes[i],delta.lobes[i]);append(nextAngular[i],delta.angular[i]);}
  kernel::surfaceGarmentCursor=nextCursor;std::copy(nextRod.begin(),nextRod.end(),kernel::surfaceGarmentPendingRod);std::copy(nextLobes.begin(),nextLobes.end(),kernel::surfaceGarmentPendingLobes);std::copy(nextAngular.begin(),nextAngular.end(),kernel::surfaceGarmentPendingAngular);kernel::surfaceGarmentPendingReady=nextReady;
  collarQueries=frame.collarQueries;
  Map(controls);
  const auto& c=frame.clinical;
  kernel::teachingTimeline.active=c.active;kernel::teachingTimeline.time=c.time;
  kernel::throbMode=int(c.throbMode);kernel::teachingFluid.settings.lateralWobbleDegrees=c.lateralWobbleDegrees;
  std::copy(c.lateralGain.begin(),c.lateralGain.end(),kernel::teachingFluid.lateralGain);
  std::copy(c.angleGain.begin(),c.angleGain.end(),kernel::teachingFluid.angleGain);
  kernel::throbSizePulse=kernel::ThrobEnvelope(c.sizeTime,.20f,1.05f,false);
  kernel::throbTwitchPulse=kernel::ThrobEnvelope(c.twitchTime,1.15f,4.6f,true);
  kernel::throbAngleSizePulse=kernel::ThrobEnvelope(c.twitchTime,1.15f,1.05f,true);
  if(c.active||c.throbMode)kernel::ApplyControlMapping();
  if(!std::isfinite(frame.seconds)||frame.seconds<0||!std::isfinite(frame.pitchForce)||!std::isfinite(frame.yawForce))throw std::invalid_argument("Invalid frame");
  if(frame.thighEndpoints){
   for(auto p:*frame.thighEndpoints)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::invalid_argument("Invalid thigh endpoint");
  }
  auto finite=[](Point p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
  if(frame.collision){
   if(!frame.thighEndpoints)throw std::invalid_argument("Collision calibration requires measured thigh endpoints");
   const auto& c=*frame.collision;
   for(float r:c.thighRadii)if(!std::isfinite(r)||r<=0)throw std::invalid_argument("Invalid measured thigh radius");
   if(!std::isfinite(c.pelvisRadius)||c.pelvisRadius<=0)throw std::invalid_argument("Invalid measured pelvis radius");
   for(unsigned i=0;i<2;i++)if(!finite(c.pelvisEndpoints[i]))throw std::invalid_argument("Invalid measured pelvis endpoint");
  }
#ifdef MALEMOD_SURFACE_PROCESS_ISOLATED
  for(double& value:kernel::surfaceGeometryMilliseconds)value=0;
#endif
  kernel::collisionCapsuleOverride=bool(frame.thighEndpoints);
  kernel::surfaceCollisionEnabled=bool(frame.collision);
  if(frame.collision){
   const auto& c=*frame.collision;kernel::surfaceTargetPelvisRadius=c.pelvisRadius;
   for(unsigned i=0;i<2;i++){
    kernel::surfaceTargetThighRadii[i]=c.thighRadii[i];
    auto p=c.pelvisEndpoints[i];kernel::surfaceTargetPelvis[i]={p.x,p.y,p.z};
   }
  }
  if(frame.thighEndpoints){
   const auto& p=*frame.thighEndpoints;
   kernel::overrideLeftA={p[0].x,p[0].y,p[0].z};kernel::overrideLeftB={p[1].x,p[1].y,p[1].z};
   kernel::overrideRightA={p[2].x,p[2].y,p[2].z};kernel::overrideRightB={p[3].x,p[3].y,p[3].z};
  }
  kernel::UpdateConstraintSolver(frame.seconds,frame.pitchForce,frame.yawForce);
  kernel::ResetPelvicAttachmentBody(seed.data());
  kernel::EvaluateAnatomy(seed.data(),47050);
 }
 Output Capture(){
  if(!kernel::UnifiedCollar::solved.allFinite())throw std::runtime_error("Non-finite collar solve");
  Output out;out.anatomy=Decode(kernel::nrPacked,kernel::nrCount,0);
  for(unsigned i=0;i<2;i++)out.body[i]=Decode(kernel::sharedBodyOutput[i],kernel::sharedBodyCount[i],kernel::sharedBodyFirst[i]);
  out.anatomyIndices.assign(kernel::nrIndices,kernel::nrIndices+kernel::nrIndexCount);
  out.proximalRadius=kernel::logicalShaftBodyRadius;out.restLength=kernel::constraintRestLength;
  auto point=[](kernel::V3 p){return Point{p.x,p.y,p.z};};
  for(unsigned i=0;i<12;i++){
   out.shaftGuide[i]=point(kernel::shaftNodes[i]);
   kernel::V3 center{},tangent{};kernel::SampleRestShaftFrame(i/11.f,center,tangent);
   out.restGuide[i]=point(center);
  }
  for(unsigned i=0;i<2;i++){
   out.lobeCenters[i]=point(kernel::CPCenter(i));
   out.lobeAnchors[i]=point(kernel::BallAnchor(i));out.lobeRadii[i]=point(kernel::CPRadii(i));
   for(unsigned j=0;j<3;j++)out.lobeAxes[i][j]=point(kernel::cpBasis[i][j]);
  }
  out.rootDirection=point(kernel::LiveRootDirection());
  auto ring=[](unsigned id){kernel::V3 sum{};for(unsigned i=0;i<kernel::r14SegmentCount;i++)sum=sum+kernel::r14Positions[kernel::r14NewStart+id*kernel::r14SegmentCount+i];return sum/float(kernel::r14SegmentCount);};
  auto lip=ring(kernel::r14RingCount-6),back=ring(kernel::r14CrownRing+(kernel::r14RingCount-kernel::r14CrownRing)/2);auto direction=kernel::Unit(lip-back);
  out.nozzlePosition=point(lip+direction*.06f);out.nozzleDirection=point(direction);
  for(unsigned i=0;i<10;i++)out.bendMultipliers[i]=kernel::RapheTubeBendMultiplier((i+1)/11.f);
  const auto* metric=kernel::UnifiedCollar::surfaceMetricFrame;
  out.collarMetric={{metric[0],metric[1],metric[2]},{metric[3],metric[4],metric[5]},
   {metric[6],metric[7],metric[8]},metric[9],metric[10],kernel::UnifiedCollar::surfaceMetricGeneration};
  if(!out.collarMetric.generation)throw std::runtime_error("Source collar metric not built");
  // Exact active source target expressions on the target character samples.
  // The cached metric masks and the moving guide are separate, as in Wolverine.
  const auto pelvic=kernel::UnifiedCollar::StablePelvicRecruitmentFrame();
  kernel::V3 root{pelvic.root.x,pelvic.root.y,pelvic.root.z},axis{pelvic.axis.x,pelvic.axis.y,pelvic.axis.z};
  const kernel::V3 up{pelvic.up.x,pelvic.up.y,pelvic.up.z};
  float length=0;auto previous=root;
  for(unsigned k=1;k<=100;k++){kernel::V3 p,t;kernel::SampleShaftChain(k*.01f,p,t);length+=kernel::Length(p-previous);previous=p;}
  length=std::max(.01f,length);const float radius=kernel::logicalShaftBodyRadius,growth=kernel::Smoother01((radius-2.9f)/4.72f);
  for(auto query:collarQueries){
   kernel::V3 p{query.x,query.y,query.z},q=p-root;float s=kernel::Dot(q,axis),y=q.y,z=kernel::Dot(q,up),radial=sqrtf(y*y+z*z);
   auto cv=[](Point p){return malemod::V3{p.x,p.y,p.z};};
   const double mask=collar::EvaluateMetricRow({p.x,p.y,p.z},cv(out.collarMetric.root),cv(out.collarMetric.axis),cv(out.collarMetric.up),out.collarMetric.radius,out.collarMetric.length,1).mask;
   if(radial<1e-8f||mask==0){out.collarDisplacements.push_back({0,0,0});continue;}
   auto delta=kernel::UnifiedCollar::RadialTargetDelta(p,root,axis,up,radius,length,mask);
   out.collarDisplacements.push_back(point(delta));
  }
  for(auto p:out.anatomy.positions)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::runtime_error("Non-finite surface");
  return out;
 }
};
Session::Session(const Controls& controls){
#ifdef MALEMOD_SURFACE_PROCESS_ISOLATED
 bool expected=false;
 if(!processSessionOwned.compare_exchange_strong(expected,true))throw std::logic_error("Parallel source session requires one character per process");
 // Even a failed initialization can change process-global source caches.
 // Replace the worker process to reset the character lifetime.
 impl_=std::make_unique<Impl>(controls);
#else
 impl_=std::make_unique<Impl>(controls);
#endif
}
Session::~Session(){
 impl_.reset();
#ifdef MALEMOD_SURFACE_PROCESS_ISOLATED
 // Source globals/caches cannot be reinitialized by constructing a second
 // character. A reset replaces the owned worker process, not this session.
#endif
}
void Session::SetControls(const Controls& controls){impl_->Call([&]{Map(controls);impl_->controls=controls;});}
void Session::Step(const Frame& frame){impl_->Call([&]{impl_->Advance(frame);});}
Output Session::Read(){Output output;impl_->Call([&]{output=impl_->Capture();});return output;}
Diagnostics Session::ReadDiagnostics(){
 Diagnostics result;
#ifdef MALEMOD_SURFACE_PROCESS_ISOLATED
 impl_->Call([&]{std::copy(std::begin(kernel::surfaceGeometryMilliseconds),std::end(kernel::surfaceGeometryMilliseconds),result.geometryMilliseconds.begin());});
#endif
 return result;
}
}
