#pragma once
#include <malemod/garments/jockstrap.hpp>
#include <malemod/surface/runtime.hpp>
#include <malemod/surface/garment_support.hpp>
namespace malemod::garments {
struct MechanicalBinding {std::array<double,14> weights{};};
struct SourceMasses {double rod=0,lobe=0;double Total()const{return 10*rod+2*lobe;}};
// Exact current source control law: shaft weight neutral86, suspended weight
// neutral94; both map0..100 then mass=.75+mapped*.0125. This is a dimensionless
// mechanical mass, not a claim about kilograms or anatomical tissue density.
inline SourceMasses CurrentSourceMasses(const surface::Controls& controls){
 auto mapped=[](float ui,float neutral){if(!std::isfinite(ui)||ui<1||ui>100)throw std::invalid_argument("Source garment mass control");return ui<=50?neutral*((ui-1)/49):neutral+(100-neutral)*((ui-50)/50);};
 return {double(.75f+mapped(controls.values[11],86)*.0125f),double(.75f+mapped(controls.values[15],94)*.0125f)};
}
// Impulse first moments transform as pseudovectors. This supports measured
// affine unit/frame conversions, including reflected source conventions.
template<class Direction> inline Point TransformMoment(Point moment,Direction toSource){
 const auto a=toSource({1,0,0}),b=toSource({0,1,0}),c=toSource({0,0,1});
 return Add(Add(Mul(Cross(b,c),moment[0]),Mul(Cross(c,a),moment[1])),Mul(Cross(a,b),moment[2]));
}
template<class Binding,class Direction>
inline surface::Frame::GarmentSupport AggregateContactReactions(const Output& cloth,
 Binding binding,Direction toSource,Point worldOriginInSource,
 const std::array<Point,2>& lobeCenters,const std::array<Point,2>& lobeRadii,
 const SourceMasses& mass){
 surface::Frame::GarmentSupport out;
 if(cloth.style==Style::Naked||cloth.reactions.empty()||cloth.reaction.activeSeconds<=0)return out;
 if(!std::isfinite(mass.rod)||mass.rod<=0||!std::isfinite(mass.lobe)||mass.lobe<=0||!std::isfinite(cloth.reaction.activeSeconds))throw std::invalid_argument("Physical garment mass/duration");
 std::array<Point,14> impulses{},moments{};
 for(const auto& reaction:cloth.reactions){
  auto impulse=toSource(reaction.impulse),moment=TransformMoment(reaction.moment,toSource);
  if(!Finite(impulse)||!Finite(moment))throw std::invalid_argument("Nonfinite calibrated garment impulse");
  double donorSum=0;for(auto donor:reaction.lineage.donors){if(!std::isfinite(donor.weight)||donor.weight<0)throw std::invalid_argument("Invalid physical reaction lineage");donorSum+=donor.weight;}
  if(donorSum<=0)throw std::invalid_argument("Empty physical reaction lineage");
  for(auto donor:reaction.lineage.donors)if(donor.weight>0&&donor.surface==Surface::Anatomy){auto map=binding(donor);double sum=0;for(double weight:map.weights){if(!std::isfinite(weight)||weight<0)throw std::invalid_argument("Invalid measured mechanical garment binding");sum+=weight;}if(sum>1+1e-6)throw std::invalid_argument("Mechanical reaction binding amplifies load");for(unsigned i=0;i<14;i++){double weight=map.weights[i]*donor.weight/donorSum;impulses[i]=Add(impulses[i],Mul(impulse,weight));moments[i]=Add(moments[i],Mul(moment,weight));}}
 }
 auto checked=[](Point p){if(!Finite(p)||Length(p)>1e12)throw std::invalid_argument("Unstable physical garment impulse");return surface::ImpulsePoint{p[0],p[1],p[2]};};
 out.enabled=true;out.contactReaction=true;
 // Source rod0/1 are prescribed pelvis attachments; their external reaction
 // cannot move them. Remaining local impulses act on their actual free nodes.
 for(unsigned i=0;i<12;i++)out.rodImpulseTotals[i]=checked(impulses[i]);
 for(unsigned i=0;i<2;i++){
  out.lobeImpulseTotals[i]=checked(impulses[12+i]);
  double radiusSquared=Dot(lobeRadii[i],lobeRadii[i]);if(!std::isfinite(radiusSquared)||radiusSquared<=0)throw std::invalid_argument("Measured garment lobe inertia");
  // Stored contact moments are about world zero. The affine point map of
  // that zero, not the pelvis origin, identifies their origin in source space.
  auto localMoment=Sub(moments[12+i],Cross(Sub(lobeCenters[i],worldOriginInSource),impulses[12+i]));
  out.lobeAngularImpulseTotals[i]=checked(localMoment);
 }
 return out;
}
}
