#pragma once
#include "jockstrap.hpp"
// SDK-free contact directions compatible with the current linear weighted
// stitch rows. Full expanded support and measured nodal impulse/moment are
// retained. This helper is not yet enabled in the production cloth runtime;
// nonlinear rendered-seam and source-motion gates remain independent.
namespace malemod::garments::sewn_contact {
struct Entry {unsigned node;Point gradient;};
struct Projection {std::vector<Entry> entries;double inverseMass=0;Point impulse{},moment{};};
class Projector {
 std::vector<int> index_;std::vector<unsigned> nodes_;std::vector<double> squareRootMass_,mass_;std::vector<std::vector<double>> basis_,columns_;
public:
 template<class Seams> Projector(const std::vector<double>& mass,const Seams& seams):index_(mass.size(),-1),mass_(mass){
  for(double value:mass)if(!std::isfinite(value)||value<0)throw std::invalid_argument("Invalid sewn contact inverse mass");
  for(const auto& seam:seams){if(seam.count>seam.nodes.size()||seam.count>seam.weights.size())throw std::invalid_argument("Sewn contact support capacity");for(unsigned k=0;k<seam.count;k++){if(!std::isfinite(seam.weights[k]))throw std::invalid_argument("Nonfinite stitch weight");if(seam.weights[k]&&mass.at(seam.nodes[k])>0&&index_.at(seam.nodes[k])<0){index_[seam.nodes[k]]=int(nodes_.size());nodes_.push_back(seam.nodes[k]);squareRootMass_.push_back(std::sqrt(mass[seam.nodes[k]]));}}}
  for(const auto& seam:seams){std::vector<double> row(nodes_.size());for(unsigned k=0;k<seam.count;k++){int slot=index_.at(seam.nodes[k]);if(slot>=0)row[slot]+=seam.weights[k]*squareRootMass_[slot];}double original=0;for(double value:row)original+=value*value;if(!std::isfinite(original))throw std::invalid_argument("Unstable weighted stitch row");if(original<=1e-30)continue;for(auto& value:row)value/=std::sqrt(original);
   // Reorthogonalized rank revelation removes exact redundant stitches, not
   // material constraints. Publication still verifies every original stitch.
   for(unsigned pass=0;pass<2;pass++)for(const auto& q:basis_){double coefficient=0;for(unsigned k=0;k<row.size();k++)coefficient+=row[k]*q[k];for(unsigned k=0;k<row.size();k++)row[k]-=coefficient*q[k];}
   double squared=0;for(double value:row)squared+=value*value;if(squared<1e-26)continue;for(auto& value:row)value/=std::sqrt(squared);basis_.push_back(std::move(row));
  }
  // Cache every column of the orthogonal projector. Application combines the
  // exact raw support columns and retains all affected sewn particles; no
  // magnitude threshold, locality approximation or bounded-support truncation.
  columns_.assign(nodes_.size(),std::vector<double>(nodes_.size()));
  for(unsigned column=0;column<nodes_.size();column++)for(unsigned row=0;row<nodes_.size();row++){double value=row==column?1.:0.;for(const auto& q:basis_)value-=q[row]*q[column];columns_[column][row]=value*squareRootMass_[column]/squareRootMass_[row];if(!std::isfinite(columns_[column][row]))throw std::invalid_argument("Unstable weighted sewn projector");}
 }
 Projection Filter(const render_contact::Gradient& raw,const std::vector<Point>& positions,const std::vector<double>& mass)const{
  if(mass.size()!=mass_.size()||positions.size()!=mass.size()||raw.count>raw.nodes.size())throw std::invalid_argument("Sewn contact state mismatch");
  Projection result;bool sewn=false;for(unsigned k=0;k<raw.count;k++){if(mass.at(raw.nodes[k])!=mass_[raw.nodes[k]])throw std::invalid_argument("Sewn contact mass changed without rebuilding operator");if(!Finite(raw.values[k])||!Finite(positions.at(raw.nodes[k])))throw std::invalid_argument("Nonfinite sewn contact");if(index_.at(raw.nodes[k])>=0&&mass[raw.nodes[k]]>0)sewn=true;}
  if(!sewn){for(unsigned k=0;k<raw.count;k++)if(mass[raw.nodes[k]]>0)result.entries.push_back({raw.nodes[k],raw.values[k]});}
  else{std::vector<Point> projected(nodes_.size());for(unsigned k=0;k<raw.count;k++){unsigned node=raw.nodes[k];int slot=index_[node];if(slot>=0){for(unsigned row=0;row<nodes_.size();row++)projected[row]=Add(projected[row],Mul(raw.values[k],columns_[slot][row]));}else if(mass[node]>0)result.entries.push_back({node,raw.values[k]});}
   for(unsigned k=0;k<projected.size();k++)if(Length(projected[k])>0)result.entries.push_back({nodes_[k],projected[k]});
  }
  for(auto entry:result.entries){if(mass.at(entry.node)!=mass_[entry.node])throw std::invalid_argument("Sewn contact mass changed without rebuilding operator");if(!Finite(positions.at(entry.node))||!Finite(entry.gradient))throw std::invalid_argument("Unstable sewn contact projection");result.inverseMass+=mass[entry.node]*Dot(entry.gradient,entry.gradient);result.impulse=Add(result.impulse,entry.gradient);result.moment=Add(result.moment,Cross(positions[entry.node],entry.gradient));}if(!std::isfinite(result.inverseMass)||!Finite(result.impulse)||!Finite(result.moment))throw std::invalid_argument("Unstable sewn contact load");return result;
 }
};
}
