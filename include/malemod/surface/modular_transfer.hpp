#pragma once
#include "modular_asset.hpp"
#include "raphe_profile.hpp"

namespace malemod::surface::modular {
// The complete rod owns axial material coordinates. A legacy surface may
// carry a second length/collar correction, which must not pull one retained
// vertex backwards through its neighboring rod section. Transfer only the
// cross-sectional residual; length and bending still come from every frame.
inline Vec CrossSectionResidual(const Binding& binding,const State& state,Vec residual,double axialShapeWeight=0){
 Vec axis{};for(const auto& influence:binding.influences)if(influence.weight>0)
  axis=Add(axis,Scale(state.frames[influence.frame].basis[0],influence.weight));
 axis=Unit(axis);return Sub(residual,Scale(axis,Dot(residual,axis)*(1-axialShapeWeight)));
}
// Preserve measured surface deformation on retained anatomy. Generated shaft
// and connecting skin remain owned by the modular mechanical construction.
class SurfaceTransfer {
 struct Row {unsigned target,kind;double axialShapeWeight=0;std::vector<std::pair<unsigned,double>> donors;};
 std::vector<Row> rows_;unsigned sourceCount_=0,targetCount_=0;
 public:
 std::string geometryHash;
 explicit SurfaceTransfer(const std::filesystem::path& path){
  if(std::filesystem::file_size(path)>32*1024*1024)throw std::invalid_argument("Oversize modular surface transfer");std::ifstream in(path,std::ios::binary);
  auto bytes=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::invalid_argument("Truncated modular transfer");};auto u=[&]{unsigned v;bytes(&v,4);return v;};auto d=[&]{double v;bytes(&v,8);if(!std::isfinite(v))throw std::invalid_argument("Nonfinite transfer donor");return v;};
  char magic[8];bytes(magic,8);if(std::string(magic,8)!="MMXFER02")throw std::invalid_argument("Modular transfer version differs");geometryHash.resize(64);bytes(geometryHash.data(),64);
  sourceCount_=u();targetCount_=u();auto n=u();if(!sourceCount_||sourceCount_>500000||!targetCount_||targetCount_>500000||n>targetCount_)throw std::invalid_argument("Invalid surface transfer counts");std::vector<bool> seen(targetCount_);
  for(unsigned i=0;i<n;i++){Row row;row.target=u();row.kind=u();auto count=u();row.axialShapeWeight=d();if(row.target>=targetCount_||seen[row.target]||row.kind<1||row.kind>2||!count||count>256||row.axialShapeWeight<0||row.axialShapeWeight>1)throw std::invalid_argument("Invalid surface transfer row");seen[row.target]=true;double sum=0;for(unsigned j=0;j<count;j++){auto id=u();double w=d();if(id>=sourceCount_||w<0)throw std::invalid_argument("Invalid surface transfer donor");sum+=w;row.donors.emplace_back(id,w);}if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Unnormalized surface transfer");rows_.push_back(std::move(row));}
  if(in.peek()!=EOF)throw std::invalid_argument("Trailing surface transfer bytes");
 }
 std::shared_ptr<const std::vector<double>> CrownWeights()const{auto w=std::make_shared<std::vector<double>>(targetCount_);for(auto& r:rows_)if(r.kind==1)(*w)[r.target]=r.axialShapeWeight;return w;}
 // The adapter-owned mesh can follow compact mechanics directly. Retained
 // suspended skin starts from its fully authored material, then follows the
 // two independently simulated bodies through its original semantic weights.
 void ApplyPrepared(const Asset& asset,State& state,const State& material,
                    const std::vector<Vec>& points)const{
  if(points.size()!=targetCount_||asset.bindings.size()!=targetCount_)
   throw std::invalid_argument("Prepared transfer topology differs");
  auto offsets=std::make_shared<std::vector<Vec>>(targetCount_);
  auto normals=std::make_shared<std::vector<Vec>>(targetCount_);
  for(const auto& row:rows_)if(row.kind==2){
   const auto& binding=asset.bindings[row.target];Vec desired{},normal{};
   const auto restNormal=material.surfaceNormals?material.surfaceNormals->at(row.target):Vec{};
   for(const auto& owner:binding.influences)if(owner.weight>0){
    if(owner.frame<13)throw std::invalid_argument("Prepared suspended skin has non-lobe ownership");
    const auto& a=material.frames[owner.frame];const auto& b=state.frames[owner.frame];
    Vec scale{};for(unsigned k=0;k<3;k++)scale[k]=state.scales[owner.frame][k]/material.scales[owner.frame][k];
    desired=Add(desired,Scale(Add(b.origin,Rotate(b.basis,Components(Unrotate(a.basis,Sub(points[row.target],a.origin)),scale))),owner.weight));
    if(Dot(restNormal,restNormal)>1e-12){
     auto local=Unrotate(a.basis,restNormal);for(unsigned k=0;k<3;k++)local[k]/=scale[k];
     normal=Add(normal,Scale(Rotate(b.basis,local),owner.weight));
    }
   }
   (*offsets)[row.target]=Sub(desired,Point(binding,state));
   if(Dot(normal,normal)>1e-12)(*normals)[row.target]=Unit(normal);
  }
  state.offsets=offsets;state.surfaceNormals=normals;
 }
 void Apply(const Asset& asset,const Plan& plan,State& state,const Output& source,const std::function<Vec(Vec)>& shaftTransform,bool transferShaft=true,bool restMorphology=false)const{
  if(asset.rest.size()!=targetCount_||source.anatomy.positions.size()!=sourceCount_)throw std::invalid_argument("Surface transfer topology differs");
  auto offsets=std::make_shared<std::vector<Vec>>(targetCount_),normals=std::make_shared<std::vector<Vec>>(targetCount_);auto sourceFrames=MaterialFrames(source);auto shaftShift=Sub(state.frames[1].origin,sourceFrames[1].origin);
  if(source.anatomy.normals.size()!=sourceCount_)throw std::invalid_argument("Surface transfer normal count differs");
  // Axial suppression belongs only to a live legacy deformation. Applying it
  // to the immutable rest morphology drops the authored longitudinal shape on
  // one side of the crown blend, making an artificial lip/step before physics.
  // The cached material then owns all subsequent bending and length response.
  for(const auto& row:rows_){if(row.kind==1&&!transferShaft)continue;Vec desired{},normal{};for(auto donor:row.donors){desired=Add(desired,Scale(Precise(source.anatomy.positions[donor.first]),donor.second));normal=Add(normal,Scale(Precise(source.anatomy.normals[donor.first]),donor.second));}if(row.kind==1){desired=Add(shaftTransform(desired),shaftShift);normal=Sub(shaftTransform(normal),shaftTransform({0,0,0}));}auto residual=Sub(desired,Point(asset.bindings[row.target],state));(*offsets)[row.target]=row.kind==1&&!restMorphology?CrossSectionResidual(asset.bindings[row.target],state,residual,row.axialShapeWeight):residual;(*normals)[row.target]=Unit(normal);}
  state.offsets=offsets;state.surfaceNormals=normals;
  // The removed axial correction also changes the surface differential.
  // Rebuild the transferred shaft normals from its actual connected surface.
  // Suspended tissue retains its measured source field and independent motion.
  if(!transferShaft)return; // Native geometry supplies current shaft differentials.
  auto posed=plan.Evaluate(state);
  if(restMorphology&&state.sectionRadius>0){
   const auto before=posed;
   const auto correction=RapheProfileCorrection(posed,asset.bindings,state.frames,*CrownWeights(),state.sectionRadius);
   for(unsigned i=0;i<targetCount_;i++)(*offsets)[i]=Add((*offsets)[i],correction[i]);
   posed=plan.Evaluate(state);
   const double fraction=RapheCorrectionFraction(before,posed,asset.faces);
   if(fraction<1){
    for(unsigned i=0;i<targetCount_;i++)(*offsets)[i]=Add((*offsets)[i],Scale(correction[i],fraction-1));
    posed=plan.Evaluate(state);
   }
  }
  std::vector<Vec> geometric(targetCount_);
  for(auto face:asset.faces){auto n=Cross(Sub(posed[face[1]],posed[face[0]]),Sub(posed[face[2]],posed[face[0]]));for(auto i:face)geometric[i]=Add(geometric[i],n);}
  for(const auto& row:rows_)if(row.kind==1&&Dot(geometric[row.target],geometric[row.target])>1e-20)(*normals)[row.target]=Unit(geometric[row.target]);
 }
};
}
