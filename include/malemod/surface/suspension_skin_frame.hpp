#pragma once
#include "presentation.hpp"
#include <queue>

namespace malemod::surface {
// The outside skin has a suspension material frame, independent of the rigid
// contents' axial spin. Use measured attachment points, not a guessed shaft
// station. Recompute from the immutable reference each time; no cadence or
// accumulated orientation state is introduced. The caller supplies the SAME
// centers/anchors for full-surface and compact-mechanical publication.
inline PresentationFrame SuspensionSkinFrame(const PresentationFrame& reference,
 PresentationPoint restAnchor,PresentationPoint liveCenter,PresentationPoint liveAnchor){
 Quaternion(reference.basis); // Validate a proper reference material frame.
 for(const auto p:{reference.origin,restAnchor,liveCenter,liveAnchor})
  for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite suspension skin frame");
 const auto rest=Unit(Sub(restAnchor,reference.origin)),live=Unit(Sub(liveAnchor,liveCenter));
 auto cross=Cross(rest,live);const double sine=std::sqrt(Dot(cross,cross)),cosine=std::clamp(Dot(rest,live),-1.,1.);
 PresentationFrame out{liveCenter,reference.basis};
 if(sine<1e-14&&cosine>0)return out;
 PresentationPoint axis{};
 if(sine>1e-14)axis=Scale(cross,1/sine);
 else{
  // At exactly opposite cords there is no unique minimum-twist rotation.
  // Resolve it from the authored side frame, so rigidly transforming the
  // model cannot change the choice. Normal hanging motion stays away from
  // this singular configuration; no stateless map can remove its ambiguity.
  double best=-1;
  for(auto basis:reference.basis){auto side=Sub(basis,Scale(rest,Dot(basis,rest)));double length=Dot(side,side);if(length>best){best=length;axis=side;}}
  axis=Unit(axis);
 }
 const double angle=std::atan2(sine,cosine),c=std::cos(angle),s=std::sin(angle);
 for(unsigned i=0;i<3;i++){
  const auto v=reference.basis[i];
  out.basis[i]=Add(Add(Scale(v,c),Scale(Cross(axis,v),s)),Scale(axis,Dot(axis,v)*(1-c)));
 }
 return out;
}
// The enclosing skin is one connected bag. Independent lobe transforms shear
// its shared upper rim and raphe, even when each lobe's frame is well behaved.
// This common transport belongs ONLY to the sewn upper rim. Applying it to
// both complete lobes discards their independent simulated movement.
inline std::array<PresentationFrame,2> SuspensionEnvelopeFrames(
 const std::array<PresentationFrame,2>& reference,
 const std::array<PresentationPoint,2>& restAnchors,
 const std::array<PresentationPoint,2>& liveCenters,
 const std::array<PresentationPoint,2>& liveAnchors){
 auto mean=[](const auto& a){return Scale(Add(a[0],a[1]),.5);};
 PresentationFrame rest{Scale(Add(reference[0].origin,reference[1].origin),.5),{{{1,0,0},{0,1,0},{0,0,1}}}};
 const auto common=SuspensionSkinFrame(rest,mean(restAnchors),mean(liveCenters),mean(liveAnchors));
 auto result=reference;
 for(unsigned i=0;i<2;i++){
  result[i].origin=Add(common.origin,Rotate(common.basis,Sub(reference[i].origin,rest.origin)));
  for(unsigned k=0;k<3;k++)result[i].basis[k]=Rotate(common.basis,reference[i].basis[k]);
 }
 return result;
}
// The sewn rim follows the measured suspension attachments. Following the
// contents' average center instead makes the attachment itself swing into the
// supporting surface; independent distal motion must release below this rim.
inline std::array<PresentationFrame,2> SuspensionAttachmentFrames(
 const std::array<PresentationFrame,2>& reference,
 const std::array<PresentationPoint,2>& restAnchors,
 const std::array<PresentationPoint,2>& liveAnchors){
 const auto rest=Scale(Add(restAnchors[0],restAnchors[1]),.5),live=Scale(Add(liveAnchors[0],liveAnchors[1]),.5);
 const auto oldSpan=Sub(restAnchors[1],restAnchors[0]),newSpan=Sub(liveAnchors[1],liveAnchors[0]);
 auto transport=SuspensionSkinFrame({rest,{{{1,0,0},{0,1,0},{0,0,1}}}},Add(rest,oldSpan),live,Add(live,newSpan));
 auto result=reference;for(auto& f:result){f.origin=Add(live,Rotate(transport.basis,Sub(f.origin,rest)));for(auto& axis:f.basis)axis=Rotate(transport.basis,axis);}
 return result;
}
// Geodesic release from the actual sewn rim, not a guessed world-height band.
// Zero value and gradient at the rim preserve its common differential; tissue
// farther down each lobe follows that lobe's own constrained mechanical frame.
inline std::vector<double> SuspensionRimFreedom(const std::vector<PresentationPoint>& points,
 const std::vector<std::array<unsigned,3>>& faces,const std::vector<unsigned>& rim,double span){
 if(rim.empty()||!std::isfinite(span)||span<=0)throw std::invalid_argument("Missing suspension rim release");
 std::vector<std::vector<unsigned>> adjacent(points.size());
 for(auto f:faces)for(unsigned k=0;k<3;k++){auto a=f[k],b=f[(k+1)%3];adjacent.at(a).push_back(b);adjacent.at(b).push_back(a);}
 std::vector<double> distance(points.size(),std::numeric_limits<double>::infinity());
 using Item=std::pair<double,unsigned>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
 for(auto id:rim){distance.at(id)=0;queue.push({0,id});}
 while(!queue.empty()){auto [d,id]=queue.top();queue.pop();if(d!=distance[id]||d>=span)continue;
  for(auto next:adjacent[id]){auto v=Sub(points[next],points[id]);const double candidate=d+std::sqrt(Dot(v,v));if(candidate<distance[next]){distance[next]=candidate;queue.push({candidate,next});}}
 }
 for(auto& d:distance){const double t=std::clamp(d/span,0.,1.);d=t*t*t*(10+t*(-15+6*t));}
 return distance;
}
}
