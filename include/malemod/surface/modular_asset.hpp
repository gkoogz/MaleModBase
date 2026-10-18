#pragma once
#include "modular_mount.hpp"
#include "skin_relief.hpp"
#include <optional>
#include <fstream>
#include <filesystem>

namespace malemod::surface::modular {
// Geometry and semantic bindings share one immutable, versioned artifact.
// Native skin palettes, materials and buffer layouts remain adapter-owned.
struct Asset {
 std::vector<Vec> rest,normals;
 std::vector<std::array<unsigned,3>> faces;
 std::vector<Binding> bindings;
 std::vector<Loft> lofts;
 State reference;Dimensions dimensions;
 std::optional<MountData> mount;
 std::optional<SkinRelief> skin;
 explicit Asset(const std::filesystem::path& path){
  if(std::filesystem::file_size(path)>128*1024*1024)throw std::invalid_argument("Oversize modular asset");
  std::ifstream in(path,std::ios::binary);
  auto bytes=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::invalid_argument("Truncated modular asset");};
  auto u=[&]{unsigned x;bytes(&x,4);return x;};auto d=[&]{double x;bytes(&x,8);if(!std::isfinite(x))throw std::invalid_argument("Nonfinite modular asset");return x;};
  auto vec=[&]{return Vec{d(),d(),d()};};
  auto binding=[&]{Binding b;for(auto& v:b.influences){v.frame=u();v.weight=d();v.local=vec();}return b;};
  char magic[8];bytes(magic,8);if(std::string(magic,8)!="MMMOD002")throw std::invalid_argument("Modular topology/binding version mismatch");
  auto nv=u(),nf=u(),nl=u();if(!nv||nv>500000||!nf||nf>1000000||nl>nv)throw std::invalid_argument("Invalid modular dimensions");
  dimensions.radius=d();dimensions.length=d();dimensions.anchor=vec();for(auto& r:dimensions.lobes)r=vec();
  for(auto& f:reference.frames){f.origin=vec();for(auto& a:f.basis)a=vec();Quaternion(f.basis);}
  for(unsigned i=0;i<nv;i++){rest.push_back(vec());normals.push_back(vec());bindings.push_back(binding());}
  for(unsigned i=0;i<nf;i++){std::array<unsigned,3> f{u(),u(),u()};for(auto x:f)if(x>=nv)throw std::invalid_argument("Modular face outside topology");faces.push_back(f);}
  for(unsigned i=0;i<nl;i++){
   Loft l;l.vertex=u();l.start=u();l.end=u();l.t=d();l.first=vec();l.last=vec();l.restValue=vec();l.firstOwner=binding();l.lastOwner=binding();lofts.push_back(l);
  }
  auto hasMount=u();if(hasMount>1)throw std::invalid_argument("Invalid modular mount presence");
  if(hasMount){
   MountData m;m.origin=vec();for(auto& a:m.basis)a=vec();m.step=d();m.station=d();m.length=d();m.side=u();auto nr=u();
   if(m.side<3||m.side>1025||nr<8||nr>nv)throw std::invalid_argument("Invalid modular chart size");
   for(unsigned i=0;i<m.side*m.side;i++)m.height.push_back(d());
   for(unsigned i=0;i<nv;i++)m.bodyWeight.push_back(d());for(unsigned i=0;i<nv;i++)m.shaftWeight.push_back(d());
   for(unsigned i=0;i<nr;i++)m.outer.push_back(u());for(unsigned i=0;i<nr;i++)m.inner.push_back(u());for(unsigned i=0;i<nr;i++)m.theta.push_back(d());
   mount=std::move(m);
  }
  if(in.peek()!=EOF)throw std::invalid_argument("Trailing modular asset data");
  std::vector<bool> active(nv);bool hasSkin=false;
  for(unsigned i=0;i<nv;i++){
   bool suspended=true;for(auto owner:bindings[i].influences)if(owner.weight>0&&owner.frame<13)suspended=false;
   active[i]=suspended;hasSkin|=suspended;
  }
  for(const auto& l:lofts)if(ShaftBinding(l.lastOwner)&&!ShaftBinding(l.firstOwner)){
   bool suspended=false;for(auto owner:l.firstOwner.influences)suspended|=owner.frame>=13&&owner.weight>0;
   if(suspended)active[l.vertex]=true;
  }
  if(hasSkin){
   auto center=Scale(Add(reference.frames[13].origin,reference.frames[14].origin),.5);
   auto side=Unit(Sub(reference.frames[14].origin,reference.frames[13].origin));
   auto forward=reference.frames[1].basis[0];forward=Unit(Sub(forward,Scale(side,Dot(forward,side))));
   const double radius=.5*(dimensions.lobes[0][0]+dimensions.lobes[1][0]);
   skin.emplace(rest,faces,active,center,PresentationBasis{forward,side,Unit(Cross(forward,side))},radius);
  }
 }
 SkinRelief::Result Relieve(std::vector<Vec>& points,const State& state,std::vector<Vec>* normalDelta=nullptr)const{
  if(normalDelta)normalDelta->assign(points.size(),{});
  return skin?skin->Apply(points,.5*(state.scales[13][0]+state.scales[14][0]),normalDelta):SkinRelief::Result{};
 }
 Plan MakePlan()const{return Plan(rest,bindings,lofts,reference);}
 std::vector<Vec> Evaluate(const Plan& plan,const State& state)const{
  if(!mount){auto points=plan.Evaluate(state);Relieve(points,state);return points;}
  MountPlan recruitment(*mount,rest);
  auto points=plan.Evaluate(state,[&](auto& p){recruitment.Apply(p,reference,state);});
  recruitment.Finish(points,lofts,reference,state);Relieve(points,state);return points;
 }
};
}
