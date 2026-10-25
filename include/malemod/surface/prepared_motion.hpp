#pragma once
#include "presentation.hpp"
#include "live_controls.hpp"
#include <malemod/task_arena.hpp>
#include <deque>
#include <optional>
#include <string>
namespace malemod::surface {
// Slots 1..12 are material rod stations, 13/14 independent suspended bodies,
// 15 is common skin motion,16 is sewn attachment motion. Zero weight is fixed body.
using PreparedMotionWeights=std::array<double,17>;
using PreparedMotionFrames=std::array<PresentationFrame,17>;
struct PreparedMotionCorrection {unsigned projections=0;double maximum=0;};
inline void ValidatePreparedMotionAreas(const std::vector<PresentationPoint>& points,
 const std::vector<std::array<unsigned,3>>& faces,const std::vector<PresentationPoint>& reference,
 TaskArena* arena=nullptr){
 if(faces.size()!=reference.size())throw std::invalid_argument("Prepared motion area topology differs");
 auto validate=[&](unsigned i){auto f=faces[i];auto area=Cross(Sub(points[f[1]],points[f[0]]),Sub(points[f[2]],points[f[0]]));
  const double square=Dot(area,area),rest=Dot(reference[i],reference[i]);
  if(!std::isfinite(square)||(rest>1e-18&&square<rest*1e-10))throw std::runtime_error("Prepared motion collapsed a triangle "+std::to_string(i));
 };
 if(arena)arena->For(unsigned(faces.size()),validate);else for(unsigned i=0;i<faces.size();i++)validate(i);
}
// A retained mesh can contain very thin triangles. Constrain their signed
// area during incremental transport instead of hiding an inversion in shading.
// Corrections are bounded in model units; a larger deformation is a failed
// candidate, never a request to freeze or average the independently moving bodies.
struct PreparedMotionAdjacency {
 std::vector<unsigned> rows,faces;
 PreparedMotionAdjacency(unsigned count,const std::vector<std::array<unsigned,3>>& triangles):rows(count+1){
  for(auto f:triangles)for(auto v:f){if(v>=count)throw std::invalid_argument("Motion adjacency leaves vertices");rows[v+1]++;}
  for(unsigned i=1;i<rows.size();i++)rows[i]+=rows[i-1];faces.resize(rows.back());auto cursor=rows;
  for(unsigned i=0;i<triangles.size();i++)for(auto v:triangles[i])faces[cursor[v]++]=i;
 }
};
inline PreparedMotionCorrection ConstrainPreparedMotion(
 const std::vector<PresentationPoint>& before,std::vector<PresentationPoint>& after,
 const std::vector<std::array<unsigned,3>>& faces,const std::vector<PreparedMotionWeights>& weights,double budget,
 const std::vector<PresentationPoint>* referenceNormals=nullptr,const PreparedMotionAdjacency* adjacency=nullptr){
 const auto proposed=after;PreparedMotionCorrection result;
 std::vector<PresentationPoint> normals;if(!referenceNormals){normals.reserve(faces.size());for(auto f:faces)normals.push_back(Cross(Sub(before[f[1]],before[f[0]]),Sub(before[f[2]],before[f[0]])));referenceNormals=&normals;}
 auto invalid=[&](unsigned i){auto f=faces[i];auto n=referenceNormals->at(i);double rest=Dot(n,n);if(rest<=1e-18)return false;auto area=Cross(Sub(after[f[1]],after[f[0]]),Sub(after[f[2]],after[f[0]]));return Dot(n,area)<=0||Dot(area,area)<rest*1e-10;};
 std::deque<unsigned> pending;std::vector<unsigned char> queued(faces.size());
 for(unsigned i=0;i<faces.size();i++){auto f=faces[i];if(after[f[0]]==before[f[0]]&&after[f[1]]==before[f[1]]&&after[f[2]]==before[f[2]])continue;if(invalid(i)){pending.push_back(i);queued[i]=1;}}
 if(pending.empty())return result;
 std::optional<PreparedMotionAdjacency> local;if(!adjacency){local.emplace(unsigned(before.size()),faces);adjacency=&*local;}
 while(!pending.empty()){
  auto fi=pending.front();pending.pop_front();queued[fi]=0;if(!invalid(fi))continue;
  auto f=faces[fi];auto normal=referenceNormals->at(fi);double rest=Dot(normal,normal);
  auto area=Cross(Sub(after[f[1]],after[f[0]]),Sub(after[f[2]],after[f[0]]));double current=Dot(normal,area);
  std::array<PresentationPoint,3> gradient{};double denominator=0;
  for(unsigned k=0;k<3;k++){bool free=false;for(unsigned j=1;j<17;j++)free|=weights[f[k]][j]>0;if(free){gradient[k]=Cross(Sub(after[f[(k+1)%3]],after[f[(k+2)%3]]),normal);denominator+=Dot(gradient[k],gradient[k]);}}
  if(denominator<=1e-30)throw std::runtime_error("Prepared motion cannot preserve fixed triangle");
  const double amount=(rest*1e-4-current)*1.05/denominator;
  for(unsigned k=0;k<3;k++){after[f[k]]=Add(after[f[k]],Scale(gradient[k],amount));auto d=Sub(after[f[k]],proposed[f[k]]);result.maximum=(std::max)(result.maximum,std::sqrt(Dot(d,d)));if(result.maximum>budget)throw std::runtime_error("Prepared motion exceeded its local orientation correction budget");}
  if(++result.projections>200000)throw std::runtime_error("Prepared motion orientation constraints did not converge");
  // A correction can only affect incident triangles. Preserve deterministic
  // original face order without repeatedly scanning the complete body mesh.
  for(auto v:f)for(unsigned at=adjacency->rows[v];at<adjacency->rows[v+1];at++){auto i=adjacency->faces[at];if(!queued[i]){pending.push_back(i);queued[i]=1;}}
 }
 return result;
}
inline void AddMotionWeights(PreparedMotionWeights& to,const PreparedMotionWeights& from,double w){
 for(unsigned i=1;i<to.size();i++)to[i]+=from[i]*w;
}
class PreparedMotionPlan {
 struct Transform {PresentationBasis matrix{};PresentationPoint translation{};};
 std::array<Transform,17> transforms_{};
public:
 PreparedMotionPlan(const PreparedMotionFrames& from,const PreparedMotionFrames& to,
                    const std::array<PresentationPoint,17>& scale){
  for(unsigned i=1;i<17;i++){
   auto& t=transforms_[i];
   for(unsigned k=0;k<3;k++){PresentationPoint axis{};axis[k]=1;auto q=Unrotate(from[i].basis,axis);for(unsigned j=0;j<3;j++)q[j]*=scale[i][j];t.matrix[k]=Rotate(to[i].basis,q);}
   t.translation=Sub(to[i].origin,Rotate(t.matrix,from[i].origin));
  }
 }
 PresentationPoint Point(PresentationPoint p,const PreparedMotionWeights& weights)const{
  auto out=p;for(unsigned i=1;i<17;i++)if(weights[i]>0){const auto& t=transforms_[i];out=Add(out,Scale(Sub(Add(Rotate(t.matrix,p),t.translation),p),weights[i]));}return out;
 }
 PresentationBasis Differential(const PreparedMotionWeights& weights)const{
  PresentationBasis j{{{1,0,0},{0,1,0},{0,0,1}}};
  for(unsigned i=1;i<17;i++)if(weights[i]>0)for(unsigned k=0;k<3;k++){auto v=transforms_[i].matrix[k];v[k]-=1;j[k]=Add(j[k],Scale(v,weights[i]));}
  return j;
 }
 PresentationPoint Normal(PresentationPoint normal,const PreparedMotionWeights& weights)const{
  const auto j=Differential(weights);
  if(Dot(j[0],Cross(j[1],j[2]))<=1e-8)throw std::runtime_error("Prepared motion collapsed a material frame");
  return Add(Add(Scale(Cross(j[1],j[2]),normal[0]),Scale(Cross(j[2],j[0]),normal[1])),Scale(Cross(j[0],j[1]),normal[2]));
 }
 std::vector<PresentationPoint> Evaluate(const std::vector<PresentationPoint>& points,
   const std::vector<PreparedMotionWeights>& weights,TaskArena* arena=nullptr)const{
  if(points.size()!=weights.size())throw std::invalid_argument("Prepared motion topology differs");
  std::vector<PresentationPoint> out(points.size());auto pose=[&](unsigned i){out[i]=Point(points[i],weights[i]);};
  if(arena)arena->For(unsigned(out.size()),pose);else for(unsigned i=0;i<out.size();i++)pose(i);return out;
 }
};
// Preserve the sewn connection's endpoint directions as its owners rotate.
// Merely translating endpoint displacements leaves the old tangent frame in
// place and can flatten/fold a short connector during a modest body turn.
inline void TransportPreparedMotionLofts(const PreparedMotionPlan& motion,
 const std::vector<PreparedMotionWeights>& weights,const std::vector<LiveControlLoft>& lofts,
 const std::vector<LiveControlTangents>& tangents,const std::vector<PresentationPoint>& before,
 std::vector<PresentationPoint>& after,unsigned first,unsigned end,TaskArena* arena=nullptr){
 if(lofts.size()!=tangents.size()||first>end||end>lofts.size())throw std::invalid_argument("Prepared motion loft layout differs");
 auto hermite=[](auto a,auto b,auto u,auto v,double t){double t2=t*t,t3=t2*t;return Add(Add(Scale(a,2*t3-3*t2+1),Scale(b,-2*t3+3*t2)),Add(Scale(u,t3-2*t2+t),Scale(v,t3-t2)));};
 auto evaluate=[&](unsigned i){auto l=lofts[i];auto a=motion.Differential(weights[l.start]),b=motion.Differential(weights[l.end]);auto tangent=tangents[i];
  auto residual=Sub(before[l.vertex],hermite(before[l.start],before[l.end],tangent.first,tangent.last,l.t));
  const double t=ControlSmooth(l.t);auto offset=Add(Scale(Rotate(a,residual),1-t),Scale(Rotate(b,residual),t));
  // Endpoint motion changes the available span independently of either
  // owner's local scale. Handles follow span length; curvature relief follows
  // its square under compression. Keeping the old relief amplitude in a
  // shorter interval produces loops even when all local normals face outward.
  auto chord=Sub(before[l.end],before[l.start]),current=Sub(after[l.end],after[l.start]);
  auto ca=Rotate(a,chord),cb=Rotate(b,chord);
  double reference=.5*(std::sqrt(Dot(ca,ca))+std::sqrt(Dot(cb,cb)));
  if(reference<=1e-12)throw std::runtime_error("Prepared connection span collapsed");
  const double ratio=std::sqrt(Dot(current,current))/reference,relief=(std::min)(1.,ratio);
  after[l.vertex]=Add(hermite(after[l.start],after[l.end],Scale(Rotate(a,tangent.first),ratio),Scale(Rotate(b,tangent.last),ratio),l.t),Scale(offset,relief*relief));
 };
 if(arena)arena->For(end-first,[&](unsigned i){evaluate(first+i);});else for(unsigned i=first;i<end;i++)evaluate(i);
}
}
