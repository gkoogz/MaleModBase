#pragma once
#include "presentation.hpp"
#include <vector>
#include <stdexcept>

namespace malemod::surface {
// Harmonic continuation of a closed boundary field. A boundary ripple of
// wavelength lambda decays as exp(-2*pi*distance/lambda), so pore-scale normal
// variation cannot become a long stripe on the adjoining surface. Broad shape
// directions travel farther. Positions and exact boundary normals are untouched.
class PeriodicNormalField {
 using Vec=PresentationPoint;
 static constexpr unsigned count=128,modes=count/2;
 std::vector<Vec> original_;std::vector<double> phase_;
 std::array<Vec,modes+1> cosine_{},sine_{};
 std::array<Vec,count> base_{};
 static constexpr double tau=6.2831853071795864769;
 std::array<Vec,count> Grid(double distance)const{
  std::array<Vec,count> result{};
  for(unsigned j=0;j<count;j++){
   result[j]=cosine_[0];
   for(unsigned k=1;k<=modes;k++){double a=tau*k*j/count,w=std::exp(-tau*k*distance);result[j]=Add(result[j],Scale(Add(Scale(cosine_[k],std::cos(a)),Scale(sine_[k],std::sin(a))),w));}
  }return result;
 }
 Vec Sample(const std::array<Vec,count>& grid,double phase)const{
  double u=phase*count;unsigned j=unsigned(std::floor(u))%count;double t=u-std::floor(u);return Add(Scale(grid[j],1-t),Scale(grid[(j+1)%count],t));
 }
public:
 PeriodicNormalField(std::vector<Vec> values,const std::vector<double>& coordinates,double period):original_(std::move(values)){
  if(original_.size()!=coordinates.size()||coordinates.size()<3||!std::isfinite(period)||period<=coordinates.back()-coordinates.front())throw std::invalid_argument("Invalid normal field coordinates");
  for(unsigned i=0;i<coordinates.size();i++){if(!std::isfinite(coordinates[i])||(i&&coordinates[i]<=coordinates[i-1]))throw std::invalid_argument("Unordered normal field coordinates");for(double x:original_[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite boundary normal");}
  for(auto s:coordinates)phase_.push_back((s-coordinates.front())/period);
  std::array<Vec,count> uniform{};
  for(unsigned j=0;j<count;j++){
   double u=double(j)/count;auto hi=std::upper_bound(phase_.begin(),phase_.end(),u);unsigned next=unsigned(hi-phase_.begin())%phase_.size(),previous=(next+unsigned(phase_.size())-1)%phase_.size();
   double a=phase_[previous],b=phase_[next];if(b<=a)b+=1;if(u<a)u+=1;double t=(u-a)/(b-a);uniform[j]=Add(Scale(original_[previous],1-t),Scale(original_[next],t));
  }
  for(unsigned k=0;k<=modes;k++)for(unsigned j=0;j<count;j++){
   double a=tau*k*j/count,w=(k==0||k==modes?1.:2.)/count;cosine_[k]=Add(cosine_[k],Scale(uniform[j],w*std::cos(a)));sine_[k]=Add(sine_[k],Scale(uniform[j],w*std::sin(a)));
  }base_=Grid(0);
 }
 std::vector<Vec> At(double distanceOverCircumference)const{
  if(!std::isfinite(distanceOverCircumference)||distanceOverCircumference<0)throw std::invalid_argument("Invalid normal continuation distance");
  if(distanceOverCircumference==0)return original_;
  auto grid=Grid(distanceOverCircumference);std::vector<Vec> result;result.reserve(original_.size());double fine=std::exp(-tau*modes*distanceOverCircumference);
  for(unsigned j=0;j<original_.size();j++)result.push_back(Unit(Add(Sample(grid,phase_[j]),Scale(Sub(original_[j],Sample(base_,phase_[j])),fine))));
  return result;
 }
};
struct StripDifferential {unsigned vertex;PresentationPoint tangent,meridian;};
// Reconstruct the material directions from the current surface. Transporting
// only its normal loses rotation within the tangent plane and makes a normal
// map change shading across a moving weld. Coordinates are immutable material
// arclengths; cut-edge subdivisions are not equally spaced samples.
inline std::vector<StripDifferential> StripDifferentials(
 const std::vector<PresentationPoint>& points,const std::vector<std::vector<unsigned>>& rings,
 const std::vector<double>& coordinates,double period){
 if(rings.size()<2||coordinates.size()<3||!std::isfinite(period)||period<=coordinates.back()-coordinates.front())throw std::invalid_argument("Invalid strip lighting coordinates");
 const unsigned count=unsigned(coordinates.size());
 for(unsigned j=0;j<count;j++)if(!std::isfinite(coordinates[j])||(j&&coordinates[j]<=coordinates[j-1]))throw std::invalid_argument("Unordered strip lighting coordinates");
 for(const auto& ring:rings)if(ring.size()!=count)throw std::invalid_argument("Strip lighting dimensions differ");
 std::vector<StripDifferential> result;result.reserve(rings.size()*count);
 for(unsigned row=0;row<rings.size();row++)for(unsigned j=0;j<count;j++){
  unsigned previous=(j+count-1)%count,next=(j+1)%count;
  double a=coordinates[j]-coordinates[previous],b=coordinates[next]-coordinates[j];if(a<=0)a+=period;if(b<=0)b+=period;
  auto p=points.at(rings[row][j]);
  auto left=Scale(Sub(p,points.at(rings[row][previous])),1/a),right=Scale(Sub(points.at(rings[row][next]),p),1/b);
  auto tangent=Scale(Add(Scale(left,b),Scale(right,a)),1/(a+b));
  auto meridian=Sub(points.at(rings[(std::min)(row+1,unsigned(rings.size()-1))][j]),points.at(rings[row?row-1:row][j]));
  result.push_back({rings[row][j],tangent,meridian});
 }
 return result;
}
}
