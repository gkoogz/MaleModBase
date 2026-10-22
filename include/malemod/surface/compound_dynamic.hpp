#pragma once
#include "modular_dynamic.hpp"
#include "compound_contact_io.hpp"
#include <sstream>

namespace malemod::surface {
// Generic bridge between a dynamically recruited support and a compound graft.
// No engine palette, shader, draw-layout or input concepts belong here.
class CompoundDynamicPlan {
public:
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 struct CornerDonor {unsigned parent=0;Vec weights{};};
 CompoundContactReference reference;
 std::vector<unsigned> sourceVertices;
 std::vector<UV> assetChart,bodyChart;
 std::vector<std::array<CornerDonor,3>> sourceCorners;
 Vec sourceTranslation{},referenceAxis{};
 double angularLimit=0;
 std::unique_ptr<CompoundContactPlan> contact;
 explicit CompoundDynamicPlan(const std::vector<unsigned char>& bytes,const modular::Asset& asset,const modular::DynamicMount& mount){
  bodyFaces_=&mount.faces;
  std::istringstream in(std::string(bytes.begin(),bytes.end()),std::ios::binary);
  auto read=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::runtime_error("Truncated compound dynamic binding");};
  char magic[8];read(magic,8);if(std::string(magic,8)!="MMCDYN01")throw std::invalid_argument("Compound dynamic binding version differs");
  std::array<unsigned,3> n;read(n.data(),12);
  if(n[0]!=asset.rest.size()||n[1]!=mount.body.size()||!n[2]||n[2]>asset.rest.size())throw std::invalid_argument("Compound dynamic binding counts differ");
  read(sourceTranslation.data(),24);read(referenceAxis.data(),24);read(&angularLimit,8);
  if(!std::isfinite(angularLimit)||angularLimit<=0||angularLimit>=3.141592653589793)throw std::invalid_argument("Invalid compound angular domain");
  assetChart.resize(n[0]);bodyChart.resize(n[1]);sourceVertices.resize(n[2]);read(assetChart.data(),n[0]*16);read(bodyChart.data(),n[1]*16);read(sourceVertices.data(),n[2]*4);
  reference=ReadCompoundContact(in);if(reference.source.size()!=n[2])throw std::invalid_argument("Compound retained source count differs");
  sourceCorners.resize(reference.sourceFaces.size());
  for(auto& face:sourceCorners)for(auto& c:face){read(&c.parent,4);read(c.weights.data(),24);if(c.parent>=asset.faces.size())throw std::invalid_argument("Compound material parent leaves asset");double sum=0;for(auto w:c.weights){if(!std::isfinite(w)||w< -1e-8||w>1+1e-8)throw std::invalid_argument("Invalid compound material donor");sum+=w;}if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Unnormalized compound material donor");}
  if(in.peek()!=EOF)throw std::invalid_argument("Trailing compound dynamic data");
  for(auto id:sourceVertices)if(id>=asset.rest.size())throw std::invalid_argument("Compound source leaves asset");
  for(const auto* field:{&assetChart,&bodyChart})for(auto p:*field)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite compound dynamic chart");
  for(auto p:{sourceTranslation,referenceAxis})for(auto x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite compound dynamic vector");
  contact=std::make_unique<CompoundContactPlan>(reference);
 }
 std::vector<Vec> Source(const std::vector<Vec>& posed,const modular::State& state)const{
  const double scale=.5*(state.scales[13][0]+state.scales[14][0]);auto shift=Scale(sourceTranslation,scale);
  std::vector<Vec> out;out.reserve(sourceVertices.size());for(auto id:sourceVertices)out.push_back(Add(posed.at(id),shift));return out;
 }
 struct PreparedSupport {std::vector<UV> chart;std::vector<std::array<unsigned,3>> faces;std::vector<bool> eligible,protect;};
 PreparedSupport PrepareSupport(const modular::DynamicResult& support,const modular::Asset& asset,const modular::State& state)const{
  const auto& mesh=support.surface;std::vector<UV> chart(mesh.points.size());std::vector<std::array<unsigned,3>> faces;
  // The authored body footprint is the verified capacity of the neutral
  // pelvic surface, not a cylindrical shaft cross-section. Shrink it with a
  // smaller shaft, but do not multiply it beyond that capacity: doing so walks
  // across the folded medial thigh/back-facing chart. At larger sizes the
  // already-recruited root ramp carries the extra width; its interpolation
  // below moves the contact naturally from body onto ramp without moving body.
  double scale=(std::min)(1.,state.scales[1][1]/asset.reference.scales[1][1]);
  for(auto face:mesh.faces)faces.push_back({face[0].vertex,face[1].vertex,face[2].vertex});
  // Body chart parents are supplied by the mount rather than a parallel copy.
  return BuildChart(support,asset,std::move(chart),std::move(faces),scale);
 }
 CompoundContactCache Build(const modular::DynamicResult& support,const modular::Asset& asset,const modular::State& state,const std::vector<Vec>& posed)const{
  auto input=PrepareSupport(support,asset,state);
  return contact->Build(Source(posed,state),support.surface.points,input.faces,input.chart,input.eligible,input.protect);
 }
private:
 const std::vector<std::array<unsigned,3>>* bodyFaces_=nullptr;
 PreparedSupport BuildChart(const modular::DynamicResult& support,const modular::Asset& asset,std::vector<UV> chart,std::vector<std::array<unsigned,3>> faces,double scale)const{
  if(!bodyFaces_)throw std::logic_error("Compound body faces are not bound");
  const auto& mesh=support.surface;std::vector<bool> eligible,protect(mesh.points.size());
  for(auto face:mesh.faces)for(auto c:face)if(c.domain<2){UV uv{};const auto f=c.domain==0?bodyFaces_->at(c.parent):asset.faces[c.parent];
   constexpr double tau=6.2831853071795864769;const double branch=(c.domain==0?bodyChart.at(f[0]):assetChart.at(f[0]))[1];
   for(unsigned j=0;j<3;j++){auto q=c.domain==0?bodyChart.at(f[j]):assetChart.at(f[j]);if(c.domain==0)q[0]/=scale;q[1]+=tau*std::round((branch-q[1])/tau);for(unsigned d=0;d<2;d++)uv[d]+=q[d]*c.donor[j];}
   uv[1]=std::remainder(uv[1],tau);
   chart[c.vertex]=uv;if(c.domain==0)for(auto w:c.donor)if(w>1-1e-12)protect[c.vertex]=true;
  }
  const auto& rings=mesh.rings;
  for(unsigned row=1;row+1<rings.size();row++)for(unsigned j=0;j<rings[row].size();j++){
   double t=.5*(1-std::cos(3.14159265358979323846*row/(rings.size()-1)));auto a=chart[rings.front()[j]],b=chart[rings.back()[j]];constexpr double tau=6.2831853071795864769;b[1]+=tau*std::round((a[1]-b[1])/tau);
   for(unsigned d=0;d<2;d++)chart[rings[row][j]][d]=a[d]*(1-t)+b[d]*t;
  }
  double low=1e300,high=-1e300;for(auto p:reference.footprint){low=(std::min)(low,p[0]);high=(std::max)(high,p[0]);}
  for(unsigned i=0;i<faces.size();i++){
   auto f=faces[i];bool valid=true;for(auto id:f)valid&=std::abs(chart[id][1])<angularLimit;
   auto normal=Cross(Sub(mesh.points[f[1]],mesh.points[f[0]]),Sub(mesh.points[f[2]],mesh.points[f[0]]));
   if(mesh.faces[i][0].domain==0)valid&=Dot(normal,referenceAxis)>0;
   valid&=(std::min)({chart[f[0]][0],chart[f[1]][0],chart[f[2]][0]})<high+1;
   valid&=(std::max)({chart[f[0]][0],chart[f[1]][0],chart[f[2]][0]})>low-1;eligible.push_back(valid);
  }
  return {std::move(chart),std::move(faces),std::move(eligible),std::move(protect)};
 }
};
}
