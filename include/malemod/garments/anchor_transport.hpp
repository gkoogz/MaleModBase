#pragma once
namespace malemod::garments::anchor_transport {
// Carry the fitted clearance with the local skin normal, in character-local
// coordinates. Retaining a pelvis-fixed offset can pull a bending waist band
// through the skin. Tangential twist remains free rather than inventing bones.
inline Point Offset(Point residual,Point referenceNormal,Point currentNormal,double reserve){
 auto from=Unit(referenceNormal),to=Unit(currentNormal);
 double cosine=std::clamp(Dot(from,to),-1.,1.);
 auto axis=Cross(from,to);
 if(cosine>-.999999){
  residual=Add(Add(residual,Cross(axis,residual)),Mul(Cross(axis,Cross(axis,residual)),1/(1+cosine)));
 }else{
  // Stable half-turn for an exactly reversed normal.
  Point basis=std::abs(from[0])<.8?Point{1,0,0}:Point{0,1,0};
  axis=Unit(Cross(from,basis));residual=Sub(Mul(axis,2*Dot(axis,residual)),residual);
 }
 return Add(residual,Mul(to,reserve));
}
}
