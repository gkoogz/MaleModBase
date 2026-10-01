#include <malemod/surface/runtime.hpp>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include "surface-kernel.inc"

namespace malemod::surface {
namespace {
namespace kernel=source;
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
  Map(controls);seed.resize(50915*32);
  for(unsigned i=0;i<kernel::collarNormalTriangleCount*3;i++)std::memcpy(seed.data()+kernel::collarNormalTriangleIndices[i]*32,kernel::collarNormalTriangleBasePositions+i*3,12);
  for(unsigned i=0;i<kernel::pelvisControlCount;i++)std::memcpy(seed.data()+kernel::pelvisControlIndices[i]*32,kernel::pelvisControlBasePositions+i*3,12);
  for(unsigned i=0;i<kernel::graftCount;i++)std::memcpy(seed.data()+(47050+i)*32,kernel::morph_base+i*3,12);
  kernel::ResetCompliantDynamics();kernel::ResetPelvicAttachmentBody(seed.data());kernel::EvaluateAnatomy(seed.data(),47050);
 }
 void Advance(const Frame& frame){
  if(!std::isfinite(frame.seconds)||frame.seconds<0||!std::isfinite(frame.pitchForce)||!std::isfinite(frame.yawForce))throw std::invalid_argument("Invalid frame");
  if(frame.thighEndpoints){
   for(auto p:*frame.thighEndpoints)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::invalid_argument("Invalid thigh endpoint");
  }
  kernel::collisionCapsuleOverride=bool(frame.thighEndpoints);
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
  for(auto p:out.anatomy.positions)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::runtime_error("Non-finite surface");
  return out;
 }
};
Session::Session(const Controls& controls):impl_(std::make_unique<Impl>(controls)){}
Session::~Session()=default;
void Session::SetControls(const Controls& controls){impl_->Call([&]{Map(controls);});}
void Session::Step(const Frame& frame){impl_->Call([&]{impl_->Advance(frame);});}
Output Session::Read(){Output output;impl_->Call([&]{output=impl_->Capture();});return output;}
}
