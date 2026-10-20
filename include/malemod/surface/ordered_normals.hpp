#pragma once
#include "presentation.hpp"
#include <malemod/task_arena.hpp>
#include <vector>
#include <stdexcept>
#include <string>

namespace malemod::surface {
// Immutable triangle adjacency. Each vertex gathers faces in original order,
// so parallel evaluation preserves the serial accumulation exactly.
class OrderedNormals {
 std::vector<std::array<unsigned,3>> faces_;
 std::vector<unsigned> rows_,incidents_;
 unsigned count_=0;
public:
 OrderedNormals(unsigned count,std::vector<std::array<unsigned,3>> faces)
  :faces_(std::move(faces)),rows_(count+1),count_(count){
  for(auto face:faces_)for(auto i:face){
   if(i>=count)throw std::invalid_argument("Normal triangle leaves its vertex buffer");
   ++rows_[i+1];
  }
  for(unsigned i=1;i<rows_.size();i++)rows_[i]+=rows_[i-1];
  incidents_.resize(rows_.back());auto cursor=rows_;
  for(unsigned f=0;f<faces_.size();f++)for(auto i:faces_[f])incidents_[cursor[i]++]=f;
 }
 std::pair<std::vector<PresentationPoint>,std::vector<PresentationPoint>> Pair(
  const std::vector<PresentationPoint>& a,const std::vector<PresentationPoint>& b,
  TaskArena* arena=nullptr,bool guardMorph=false,bool normalize=true)const{
  if(a.size()!=count_||b.size()!=count_)throw std::invalid_argument("Normal points differ from prepared topology");
  auto each=[&](unsigned count,auto&& f){if(arena)arena->For(count,f);else for(unsigned i=0;i<count;i++)f(i);};
  std::vector<PresentationPoint> af(faces_.size()),bf(faces_.size());
  each(unsigned(faces_.size()),[&](unsigned i){
   auto f=faces_[i];
   af[i]=Cross(Sub(a[f[1]],a[f[0]]),Sub(a[f[2]],a[f[0]]));
   bf[i]=Cross(Sub(b[f[1]],b[f[0]]),Sub(b[f[2]],b[f[0]]));
   if(guardMorph&&Dot(af[i],af[i])>1e-18){
    if(Dot(bf[i],bf[i])<Dot(af[i],af[i])*1e-10)throw std::runtime_error("Live controls collapsed a prepared triangle");
    if(Dot(af[i],bf[i])<=0)throw std::runtime_error("Live controls folded a prepared triangle "+std::to_string(i));
   }
  });
  std::pair<std::vector<PresentationPoint>,std::vector<PresentationPoint>> result;
  result.first.resize(count_);result.second.resize(count_);
  each(count_,[&](unsigned i){
   auto& x=result.first[i];auto& y=result.second[i];
   for(unsigned at=rows_[i];at<rows_[i+1];at++){x=Add(x,af[incidents_[at]]);y=Add(y,bf[incidents_[at]]);}
   if(normalize){
    auto unit=[](auto n){const double length=Dot(n,n);return length>1e-24?Scale(n,1/std::sqrt(length)):PresentationPoint{1,0,0};};
    x=unit(x);y=unit(y);
   }
  });
  return result;
 }
};
}
