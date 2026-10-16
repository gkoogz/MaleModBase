#pragma once
#include "modular_asset.hpp"

namespace malemod::surface::modular {
// Preserve measured surface deformation on retained anatomy. Generated shaft
// and connecting skin remain owned by the modular mechanical construction.
class SurfaceTransfer {
 struct Row {unsigned target,kind;std::vector<std::pair<unsigned,double>> donors;};
 std::vector<Row> rows_;unsigned sourceCount_=0,targetCount_=0;
 public:
 std::string geometryHash;
 explicit SurfaceTransfer(const std::filesystem::path& path){
  if(std::filesystem::file_size(path)>32*1024*1024)throw std::invalid_argument("Oversize modular surface transfer");std::ifstream in(path,std::ios::binary);
  auto bytes=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::invalid_argument("Truncated modular transfer");};auto u=[&]{unsigned v;bytes(&v,4);return v;};auto d=[&]{double v;bytes(&v,8);if(!std::isfinite(v))throw std::invalid_argument("Nonfinite transfer donor");return v;};
  char magic[8];bytes(magic,8);if(std::string(magic,8)!="MMXFER01")throw std::invalid_argument("Modular transfer version differs");geometryHash.resize(64);bytes(geometryHash.data(),64);
  sourceCount_=u();targetCount_=u();auto n=u();if(!sourceCount_||sourceCount_>500000||!targetCount_||targetCount_>500000||n>targetCount_)throw std::invalid_argument("Invalid surface transfer counts");std::vector<bool> seen(targetCount_);
  for(unsigned i=0;i<n;i++){Row row;row.target=u();row.kind=u();auto count=u();if(row.target>=targetCount_||seen[row.target]||row.kind<1||row.kind>2||!count||count>256)throw std::invalid_argument("Invalid surface transfer row");seen[row.target]=true;double sum=0;for(unsigned j=0;j<count;j++){auto id=u();double w=d();if(id>=sourceCount_||w<0)throw std::invalid_argument("Invalid surface transfer donor");sum+=w;row.donors.emplace_back(id,w);}if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Unnormalized surface transfer");rows_.push_back(std::move(row));}
  if(in.peek()!=EOF)throw std::invalid_argument("Trailing surface transfer bytes");
 }
 void Apply(const Asset& asset,State& state,const Output& source,const std::function<Vec(Vec)>& shaftTransform)const{
  if(asset.rest.size()!=targetCount_||source.anatomy.positions.size()!=sourceCount_)throw std::invalid_argument("Surface transfer topology differs");
  auto offsets=std::make_shared<std::vector<Vec>>(targetCount_),normals=std::make_shared<std::vector<Vec>>(targetCount_);auto sourceFrames=MaterialFrames(source);auto shaftShift=Sub(state.frames[1].origin,sourceFrames[1].origin);
  if(source.anatomy.normals.size()!=sourceCount_)throw std::invalid_argument("Surface transfer normal count differs");
  for(const auto& row:rows_){Vec desired{},normal{};for(auto donor:row.donors){desired=Add(desired,Scale(Precise(source.anatomy.positions[donor.first]),donor.second));normal=Add(normal,Scale(Precise(source.anatomy.normals[donor.first]),donor.second));}if(row.kind==1){desired=Add(shaftTransform(desired),shaftShift);normal=Sub(shaftTransform(normal),shaftTransform({0,0,0}));}(*offsets)[row.target]=Sub(desired,Point(asset.bindings[row.target],state));(*normals)[row.target]=Unit(normal);}
  state.offsets=std::move(offsets);state.surfaceNormals=std::move(normals);
 }
};
}
