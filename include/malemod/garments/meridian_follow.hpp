#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
// Kinematic skinning of a successful wrap. Frames come from the already posed
// anatomy, not another simulation. No integration, lag or accumulated offsets.
struct FollowFrame {Vec center,x,y,z;};
inline Vec FollowGlobal(Vec p,const FollowFrame& f){return Add(f.center,Add(Mul(f.x,p[0]),Add(Mul(f.y,p[1]),Mul(f.z,p[2]))));}
struct FollowDualFrame {Vec center,row[3];};
inline FollowDualFrame FollowDual(const FollowFrame& f){
 float determinant=Dot(f.x,Cross(f.y,f.z));
 if(!std::isfinite(determinant)||std::abs(determinant)<1e-8f)throw std::runtime_error("Degenerate follow frame");
 return {f.center,{Mul(Cross(f.y,f.z),1/determinant),Mul(Cross(f.z,f.x),1/determinant),Mul(Cross(f.x,f.y),1/determinant)}};
}
inline Vec FollowCoordinates(Vec p,const FollowDualFrame& f){p=Sub(p,f.center);return {Dot(p,f.row[0]),Dot(p,f.row[1]),Dot(p,f.row[2])};}
inline Vec FollowLocal(Vec p,const FollowFrame& f){return FollowCoordinates(p,FollowDual(f));}
inline FollowFrame FollowBoundary(const std::vector<Vec>& p,unsigned columns){
 FollowFrame f{};f.center=AttachmentCentroid(p,columns);Vec area{};
 for(unsigned i=0;i<columns;i++)area=Add(area,Cross(Sub(p[i],f.center),Sub(p[(i+1)%columns],f.center)));
 f.z=Unit(area);Vec edge=Sub(p[columns/2],p[0]);f.x=Unit(Sub(edge,Mul(f.z,Dot(edge,f.z))));f.y=Cross(f.z,f.x);return f;
}
class SurfaceFollower {
 struct Influence {unsigned frame;float weight;Vec local;};
 struct Binding {Vec body;float bodyWeight;std::array<Influence,4> rig;};
 std::vector<Binding> bindings;
 std::vector<Vec> seamOffsets;
 std::vector<Vec> frameScales;
 unsigned columns_=0;
public:
 void Reset(){bindings.clear();seamOffsets.clear();frameScales.clear();columns_=0;}
 bool Ready()const{return !bindings.empty();}
 void Remember(const std::vector<Vec>& solved,const std::vector<Vec>& raw,const std::vector<FollowFrame>& rig,unsigned columns,unsigned count){
  if(rig.empty()||count>solved.size()||count>raw.size()||columns<3||count<columns*2+1||(count-1)%columns)throw std::runtime_error("Invalid follow binding");
  auto body=FollowDual(FollowBoundary(raw,columns));std::vector<FollowDualFrame> dual;dual.reserve(rig.size());for(const auto& f:rig)dual.push_back(FollowDual(f));
  std::vector<Binding> next(count);std::vector<Vec> offsets(columns);
  for(unsigned i=0;i<columns;i++)offsets[i]=Sub(FollowCoordinates(solved[i],body),FollowCoordinates(raw[i],body));
  for(unsigned i=columns;i+1<count;i++){
   auto& b=next[i];b.body=FollowCoordinates(solved[i],body);
   float t=float(i/columns)/float((count-1)/columns);b.bodyWeight=(1-t)*(1-t)*(1-t);
   std::array<Influence,4> choices{};
   for(unsigned k=0;k<rig.size();k++){
    Vec local=FollowCoordinates(solved[i],dual[k]);float distance=Dot(local,local)+.25f,weight=1/(distance*distance*distance);
    for(unsigned j=0;j<4;j++)if(weight>choices[j].weight){for(unsigned n=3;n>j;n--)choices[n]=choices[n-1];choices[j]={k,weight,local};break;}
   }
   float sum=0;for(const auto& choice:choices)sum+=choice.weight;if(!std::isfinite(sum)||sum<=0)throw std::runtime_error("Invalid follow weights");
   for(unsigned k=0;k<4;k++){b.rig[k]=choices[k];b.rig[k].weight*= (1-b.bodyWeight)/sum;}
  }
  frameScales.clear();for(const auto& f:rig)frameScales.push_back({Dot(f.x,f.x),Dot(f.y,f.y),Dot(f.z,f.z)});
  bindings.swap(next);seamOffsets.swap(offsets);columns_=columns;
 }
 bool Move(std::vector<Vec>& current,const std::vector<FollowFrame>& rig,unsigned columns)const{
  if(!Ready()||columns!=columns_||current.size()<bindings.size()||rig.size()!=frameScales.size())return false;
  // Gross scale changes need a new wrap. Small pose-dependent changes in the
  // measured radii follow directly; callers separately invalidate UI changes.
  for(unsigned k=0;k<rig.size();k++){Vec scale{Dot(rig[k].x,rig[k].x),Dot(rig[k].y,rig[k].y),Dot(rig[k].z,rig[k].z)};for(unsigned j=0;j<3;j++)if(!std::isfinite(scale[j])||scale[j]<frameScales[k][j]*.5f||scale[j]>frameScales[k][j]*2.f)return false;}
  auto body=FollowBoundary(current,columns);
  for(unsigned i=0;i<columns;i++)current[i]=Add(current[i],Sub(FollowGlobal(seamOffsets[i],body),body.center));
  // Leave the current terminal pole exact. The adapter computes it from the
  // current dome support before calling this method.
  for(unsigned i=columns;i+1<bindings.size();i++){
   const auto& b=bindings[i];Vec p=Mul(FollowGlobal(b.body,body),b.bodyWeight);
   for(const auto& influence:b.rig)if(influence.weight>0){if(influence.frame>=rig.size())throw std::runtime_error("Follow rig changed");p=Add(p,Mul(FollowGlobal(influence.local,rig[influence.frame]),influence.weight));}
   for(float v:p)if(!std::isfinite(v))throw std::runtime_error("Nonfinite follow point");current[i]=p;
  }return true;
 }
};
// A plane index is a hint only: all three current vertices must clear a current
// support plane. Scan another plane on a cache miss, without running the walk.
inline bool CertifyFollowedSurface(const std::vector<Vec>& p,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,std::vector<unsigned>& certificates){
 if(certificates.size()!=faceCount*hulls.size())certificates.assign(faceCount*hulls.size(),0);
 for(unsigned f=0;f<faceCount;f++){
  const auto ids=faces[f];for(unsigned id:ids)if(id>=p.size())throw std::runtime_error("Invalid followed face");
  Vec area=Cross(Sub(p[ids[1]],p[ids[0]]),Sub(p[ids[2]],p[ids[0]]));if(!std::isfinite(Dot(area,area))||Dot(area,area)<1e-14f)return false;
  for(unsigned h=0;h<hulls.size();h++){
  const auto& hull=hulls[h];auto& cached=certificates[f*unsigned(hulls.size())+h];
  auto clear=[&](unsigned k){const auto& plane=hull[k];return Signed(plane,p[ids[0]])>=-1e-5f&&Signed(plane,p[ids[1]])>=-1e-5f&&Signed(plane,p[ids[2]])>=-1e-5f;};
  if(cached<hull.size()&&clear(cached))continue;
  unsigned k=0;while(k<hull.size()&&!clear(k))++k;if(k==hull.size())return false;cached=k;
 }}return true;
}
// Small current-frame contact corrections can keep a skinned wrap clear between
// rebuilds. This is a bounded projection, not cloth dynamics or path walking.
// Fixed seam/pole, original bindings and topology remain unchanged on failure.
inline bool RefitFollowedSurface(std::vector<Vec>& p,unsigned columns,unsigned count,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,std::vector<unsigned>& certificates,float limit=.12f){
 if(count>p.size()||columns>=count||!std::isfinite(limit)||limit<=0)throw std::runtime_error("Invalid follow contact budget");
 if(CertifyFollowedSurface(p,faces,faceCount,hulls,certificates))return true;
 auto work=p;std::vector<Vec> corrections(count);std::vector<unsigned> hits(count);
 for(unsigned pass=0;pass<2;pass++){
  std::fill(corrections.begin(),corrections.end(),Vec{});std::fill(hits.begin(),hits.end(),0);
  for(unsigned f=0;f<faceCount;f++)for(unsigned h=0;h<hulls.size();h++){
   const auto ids=faces[f];const auto& hull=hulls[h];auto& cached=certificates[f*unsigned(hulls.size())+h];
   auto score=[&](unsigned k){const auto& plane=hull[k];return (std::min)({Signed(plane,work[ids[0]]),Signed(plane,work[ids[1]]),Signed(plane,work[ids[2]])});};
   float best=cached<hull.size()?score(cached):-std::numeric_limits<float>::infinity();if(best>=-1e-5f)continue;
   for(unsigned k=0;k<hull.size();k++){float separation=score(k);if(separation>best){best=separation;cached=k;}if(best>=-1e-5f)break;}
   if(best>=-1e-5f)continue;if(best< -limit||cached>=hull.size())return false;
   const auto& plane=hull[cached];
   for(unsigned id:ids){float gap=Signed(plane,work[id]);if(gap>=0)continue;if(id<columns||id==count-1)return false;
    corrections[id]=Add(corrections[id],Mul(plane.normal,.04f-gap));hits[id]++;
   }
  }
  for(unsigned i=columns;i+1<count;i++)if(hits[i]){work[i]=Add(work[i],Mul(corrections[i],1.f/hits[i]));Vec displacement=Sub(work[i],p[i]);if(Dot(displacement,displacement)>limit*limit)return false;}
  for(unsigned f=0;f<faceCount;f++){auto ids=faces[f];Vec before=Cross(Sub(p[ids[1]],p[ids[0]]),Sub(p[ids[2]],p[ids[0]])),after=Cross(Sub(work[ids[1]],work[ids[0]]),Sub(work[ids[2]],work[ids[0]]));if(Dot(after,after)<1e-14f||Dot(before,after)<=0)return false;}
  if(CertifyFollowedSurface(work,faces,faceCount,hulls,certificates)){p.swap(work);return true;}
 }return false;
}
}
