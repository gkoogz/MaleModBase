#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
// A failed optimization must not invalidate an otherwise drawable garment.
// Retain material coordinates in a frame carried by the sewn body boundary;
// transport them with current boundary/rig controls, never with world history.
class SurfaceContinuity {
 struct Frame {Vec center,x,y,z;};
 Frame frame{};
 std::vector<Vec> surface,controls,boundary;
 mutable std::vector<float> weights,weightSums;
 static Frame MakeFrame(const std::vector<Vec>& p,unsigned columns){
  Frame f{};f.center=AttachmentCentroid(p,columns);Vec area{};
  for(unsigned i=0;i<columns;i++)area=Add(area,Cross(Sub(p[i],f.center),Sub(p[(i+1)%columns],f.center)));
  f.z=Unit(area);Vec edge=Sub(p[columns/2],p[0]);f.x=Unit(Sub(edge,Mul(f.z,Dot(edge,f.z))));f.y=Cross(f.z,f.x);return f;
 }
 static Vec Local(Vec p,const Frame& f){p=Sub(p,f.center);return {Dot(p,f.x),Dot(p,f.y),Dot(p,f.z)};}
 static Vec Global(Vec p,const Frame& f){return Add(f.center,Add(Mul(f.x,p[0]),Add(Mul(f.y,p[1]),Mul(f.z,p[2]))));}
public:
 void Reset(){surface.clear();controls.clear();boundary.clear();weights.clear();weightSums.clear();}
 bool Ready()const{return !surface.empty();}
 void Remember(const std::vector<Vec>& solved,const std::vector<Vec>& raw,const std::vector<Vec>& anchors,unsigned columns,unsigned count){
  weights.clear();weightSums.clear();
  frame=MakeFrame(raw,columns);surface.resize(count);boundary.resize(columns);controls.resize(anchors.size());
  for(unsigned i=0;i<count;i++)surface[i]=Local(solved[i],frame);
  for(unsigned i=0;i<columns;i++)boundary[i]=Local(raw[i],frame);
  for(unsigned i=0;i<anchors.size();i++)controls[i]=Local(anchors[i],frame);
 }
 bool Transport(std::vector<Vec>& current,const std::vector<Vec>& anchors,unsigned columns)const{
  if(surface.empty()||anchors.size()!=controls.size()||surface.size()>current.size()||columns!=boundary.size())return false;
  // Material-space distances are fixed until Remember/Reset. Build only when
  // fallback actually needs them, retaining the original float sum order.
  if(weightSums.empty()){
   weights.resize(surface.size()*controls.size());weightSums.resize(surface.size());
   for(unsigned i=columns;i<surface.size();i++)for(unsigned k=0;k<controls.size();k++){
    Vec d=Sub(surface[i],controls[k]);float squared=Dot(d,d)+.25f,weight=1/(squared*squared);
    weights[i*controls.size()+k]=weight;weightSums[i]+=weight;
   }
  }
  Frame now=MakeFrame(current,columns);std::vector<Vec> delta(anchors.size());
  for(unsigned i=0;i<anchors.size();i++)delta[i]=Sub(Local(anchors[i],now),controls[i]);
  auto raw=current;
  for(unsigned i=0;i<surface.size();i++){
   Vec correction{};
   if(i<columns)correction=Sub(Local(raw[i],now),boundary[i]);else{
    for(unsigned k=0;k<controls.size();k++)correction=Add(correction,Mul(delta[k],weights[i*controls.size()+k]));
    correction=Mul(correction,1/weightSums[i]);
   }
   current[i]=Global(Add(surface[i],correction),now);
  }return true;
 }
};
// Unrestricted current-pose contact repair for the transported surface. This
// doesn't rebuild longitude coordinates, so it can bridge a temporary chart
// singularity. Return the actual triangle certificate status separately.
inline bool RepairTransportedSurface(std::vector<Vec>& p,unsigned columns,unsigned count,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,unsigned budget=4,std::vector<unsigned>* certificates=nullptr){
 std::vector<Vec> corrections(count);std::vector<unsigned> hits(count);
 // A cached index is only a hint. Re-evaluate its plane against all three
 // CURRENT vertices every pass. Changed pose, plane order or hull count cannot
 // turn a stale hint into a false clearance result. Failed hints use the exact
 // original plane scan and correction accumulation order.
 if(certificates&&certificates->size()!=faceCount*hulls.size())certificates->assign(faceCount*hulls.size(),0);
 for(unsigned pass=0;pass<budget;pass++){
  std::fill(corrections.begin(),corrections.end(),Vec{});std::fill(hits.begin(),hits.end(),0);bool clear=true;
  for(unsigned f=0;f<faceCount;f++)for(const auto& hull:hulls){
   auto ids=faces[f];const Plane* best=nullptr;float score=-std::numeric_limits<float>::infinity();
   const unsigned key=f*unsigned(hulls.size())+unsigned(&hull-hulls.data());
   if(certificates&&(*certificates)[key]<hull.size()){
    const auto& plane=hull[(*certificates)[key]];
    if((std::min)({Signed(plane,p[ids[0]]),Signed(plane,p[ids[1]]),Signed(plane,p[ids[2]])})>=0)continue;
   }
   for(const auto& plane:hull){float separation=(std::min)({Signed(plane,p[ids[0]]),Signed(plane,p[ids[1]]),Signed(plane,p[ids[2]])});if(separation>score){score=separation;best=&plane;}if(score>=0)break;}
   if(certificates&&best)(*certificates)[key]=unsigned(best-hull.data());
   if(score>=0)continue;clear=false;
   if(!best)continue;
   for(unsigned id:ids)if(id>=columns){float gap=Signed(*best,p[id]);if(gap<.04f){corrections[id]=Add(corrections[id],Mul(best->normal,(std::min)(.5f,.04f-gap)));hits[id]++;}}
  }
  if(clear)return true;
  for(unsigned i=columns;i<count;i++)if(hits[i])p[i]=Add(p[i],Mul(corrections[i],1.f/hits[i]));
 }return false;
}
}
