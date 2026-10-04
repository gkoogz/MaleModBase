#pragma once
namespace malemod::garments {
// Barycentric distribution preserves both linear impulse and first moment.
// No proximity/gravity heuristic enters this collector. A call represents an
// actual unilateral contact projection applied to moving fabric particles.
class ReactionCollector {
 std::vector<ContactReaction> vertices_;
 ReactionTelemetry totals_;
public:
 explicit ReactionCollector(const std::vector<Sample>& samples):vertices_(samples.size()){
  for(unsigned i=0;i<samples.size();i++)vertices_[i].lineage=samples[i].lineage;
 }
 void Add(unsigned triangle,Point bary,Point clothImpulse,Point tissuePoint,
          double separation,const std::vector<std::array<std::uint32_t,3>>& faces,
          Point particleImpulse,Point particleMoment){
  if(triangle>=faces.size()||!Finite(bary)||!Finite(clothImpulse)||!Finite(tissuePoint)||!Finite(particleImpulse)||!Finite(particleMoment)||!std::isfinite(separation))throw std::invalid_argument("Invalid physical garment reaction");
  double sum=0;for(auto& w:bary){if(w<-1e-7)throw std::invalid_argument("Reaction barycentric outside physical triangle");w=(std::max)(0.,w);sum+=w;}if(sum<1e-14)throw std::invalid_argument("Reaction barycentric sum");
  bary=Mul(bary,1/sum);auto anatomy=Mul(clothImpulse,-1),moment=Cross(tissuePoint,anatomy);
  for(unsigned k=0;k<3;k++){auto& record=vertices_.at(faces[triangle][k]);record.impulse=garments::Add(record.impulse,Mul(anatomy,bary[k]));record.moment=garments::Add(record.moment,Mul(moment,bary[k]));record.separation=separation;}
  totals_.contacts++;totals_.clothImpulse=garments::Add(totals_.clothImpulse,clothImpulse);totals_.anatomyImpulse=garments::Add(totals_.anatomyImpulse,anatomy);totals_.clothMoment=Sub(totals_.clothMoment,moment);totals_.anatomyMoment=garments::Add(totals_.anatomyMoment,moment);
  totals_.particleImpulse=garments::Add(totals_.particleImpulse,particleImpulse);totals_.particleMoment=garments::Add(totals_.particleMoment,particleMoment);
  totals_.supportImpulse=garments::Add(totals_.supportImpulse,Sub(clothImpulse,particleImpulse));totals_.supportMoment=garments::Add(totals_.supportMoment,Sub(Mul(moment,-1),particleMoment));
 }
 void Add(unsigned triangle,Point bary,Point clothImpulse,Point tissuePoint,
          double separation,const std::vector<std::array<std::uint32_t,3>>& faces){
  Add(triangle,bary,clothImpulse,tissuePoint,separation,faces,clothImpulse,Cross(tissuePoint,clothImpulse));
 }
 void Publish(Output& output,double seconds,double mass)const{
  output.reactions.clear();output.reaction=totals_;output.reaction.activeSeconds=seconds;output.reaction.clothMass=mass;
  if(seconds>0)for(const auto& record:vertices_)if(Length(record.impulse)>1e-20||Length(record.moment)>1e-20)output.reactions.push_back(record);
  output.reaction.records=unsigned(output.reactions.size());
 }
};
}
