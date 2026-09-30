#pragma once
#include "xpbd_kernels.hpp"
#include <functional>
#include <stdexcept>

namespace malemod::physics {
struct ChainSettings {
 unsigned nodes=12,iterations=24;float length=.2f,radius=.02f,mass=1.f;
 float distanceCompliance=1e-8f,bendCompliance=1e-6f,bounce01=.3f,drag=2.f;
 float fixedStep=1.f/120.f,maxFrameAdvance=.1f;V3 gravity{0,0,-9.81f};
};
struct RootPose {V3 position{},direction{0,-1,0};};
struct Contact {V3 position{},normal{};};
class Chain {
 State state_;ChainSettings config_;RootPose previous_{};bool ready_=false;double accumulator_=0;
 std::vector<float> lengths_;std::vector<V3> bends_;std::vector<PDBendData> prepared_;
 static bool Finite(V3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
 void Step(float dt,RootPose root){
  auto& s=state_;s.oldPosition=s.position;float segment=config_.length/(config_.nodes-1);
  for(unsigned i=2;i<config_.nodes;i++){s.velocity[i]=(s.velocity[i]+config_.gravity*dt)*std::exp(-config_.drag*dt);s.position[i]=s.position[i]+s.velocity[i]*dt;}
  s.position[0]=root.position;s.position[1]=root.position+root.direction*segment;
  std::fill(lengths_.begin(),lengths_.end(),0.f);std::fill(bends_.begin(),bends_.end(),V3{});
  for(unsigned i=1;i+1<config_.nodes;i++)prepared_[i]=PrepareBend(s,int(i),config_.bendCompliance,dt,config_.bounce01);
  for(unsigned iteration=0;iteration<config_.iterations;iteration++){
   for(unsigned i=0;i+1<config_.nodes;i++)SolveDistance(s,int(i),int(i+1),segment,config_.distanceCompliance,lengths_[i],dt);
   for(unsigned i=1;i+1<config_.nodes;i++)SolveBendPrepared(s,int(i),prepared_[i],bends_[i]);
   if(project)for(unsigned i=2;i<config_.nodes;i++){
    Contact hit;if(project(i,s.oldPosition[i],s.position[i],config_.radius,hit)&&Finite(hit.position)&&Finite(hit.normal)&&Length(hit.normal)>1e-6f)s.position[i]=hit.position;
   }
  }
  for(unsigned i=0;i<config_.nodes;i++)s.velocity[i]=(s.position[i]-s.oldPosition[i])/dt;
 }
public:
 // Adapter projection must include the particle radius and surface motion.
 // This generic chain does not reproduce Wolverine's coupled ellipsoid solver.
 std::function<bool(unsigned,V3,V3,float,Contact&)> project;
 void Begin(const ChainSettings& c,RootPose root){
  for(float value:{c.length,c.radius,c.mass,c.distanceCompliance,c.bendCompliance,c.bounce01,c.drag,c.fixedStep,c.maxFrameAdvance})if(!std::isfinite(value))throw std::invalid_argument("Nonfinite chain settings");
  if(c.nodes<3||c.nodes>64||!c.iterations||c.iterations>64||c.length<=0||c.radius<=0||c.mass<=0||c.distanceCompliance<0||c.bendCompliance<0||c.bounce01<0||c.bounce01>1||c.drag<0||c.fixedStep<=0||c.fixedStep>.1f||c.maxFrameAdvance<c.fixedStep||c.maxFrameAdvance>.1f||c.maxFrameAdvance/c.fixedStep>128||!Finite(c.gravity)||!Finite(root.position)||!Finite(root.direction)||Length(root.direction)<1e-6f)throw std::invalid_argument("Invalid chain configuration");
  root.direction=Unit(root.direction);config_=c;previous_=root;accumulator_=0;ready_=true;
  state_.position.resize(c.nodes);state_.oldPosition.resize(c.nodes);state_.velocity.assign(c.nodes,{});state_.invMass.assign(c.nodes,1/c.mass);state_.invMass[0]=state_.invMass[1]=0;
  for(unsigned i=0;i<c.nodes;i++)state_.position[i]=root.position+root.direction*(c.length*i/(c.nodes-1));
  state_.oldPosition=state_.position;lengths_.resize(c.nodes);bends_.resize(c.nodes);prepared_.resize(c.nodes);
 }
 unsigned Advance(float dt,RootPose root){
  if(!ready_)throw std::logic_error("Begin the chain before advancing");
  if(!std::isfinite(dt)||dt<0||!Finite(root.position)||!Finite(root.direction)||Length(root.direction)<1e-6f)throw std::invalid_argument("Invalid chain frame");
  root.direction=Unit(root.direction);accumulator_+=min(dt,config_.maxFrameAdvance);
  unsigned steps=unsigned((accumulator_+1e-9)/config_.fixedStep);
  for(unsigned i=0;i<steps;i++){float t=(i+1.f)/steps;RootPose sample{previous_.position+(root.position-previous_.position)*t,Unit(previous_.direction+(root.direction-previous_.direction)*t)};Step(config_.fixedStep,sample);}
  accumulator_-=steps*config_.fixedStep;previous_=root;return steps;
 }
 const State& Get()const{return state_;}
};
}
