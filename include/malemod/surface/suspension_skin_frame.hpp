#pragma once
#include "presentation.hpp"

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
// Transport that envelope by the common suspension frame, preserving its
// authored relative shape while following the contents' mean displacement.
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
}
