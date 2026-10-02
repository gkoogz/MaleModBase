#pragma once
#include "graft_runtime.hpp"
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
// Explicit authoring-to-native calibration. Positions, vectors and lengths are
// separate operations; a normal or capsule radius must never receive a root
// translation. Bone names and measured matrices live in the adapter artifact.
struct CoordinateCalibration {
 std::array<PrecisePoint,3> basis;
 PrecisePoint sourceRoot,targetRoot;
 double targetUnitsPerSourceUnit;
 void Validate()const{
  auto dot=[](auto a,auto b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
  if(!std::isfinite(targetUnitsPerSourceUnit)||targetUnitsPerSourceUnit<=0)throw std::invalid_argument("Invalid measured character scale");
  for(auto p:{sourceRoot,targetRoot,basis[0],basis[1],basis[2]})for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Non-finite character calibration");
  for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)if(std::abs(dot(basis[i],basis[j])-(i==j?1.:0.))>1e-8)throw std::invalid_argument("Character basis must be orthonormal");
  const auto& a=basis[0];const auto& b=basis[1];const auto& c=basis[2];
  double determinant=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]);
  if(std::abs(determinant-1)>1e-8)throw std::invalid_argument("Character calibration must preserve handedness");
 }
 PrecisePoint VectorToTarget(PrecisePoint p)const{
  PrecisePoint out{};for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)out[i]+=basis[i][j]*p[j];return out;
 }
 PrecisePoint VectorToSource(PrecisePoint p)const{
  PrecisePoint out{};for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)out[i]+=basis[j][i]*p[j];return out;
 }
 PrecisePoint PointToTarget(PrecisePoint p)const{
  for(unsigned i=0;i<3;i++)p[i]-=sourceRoot[i];auto out=VectorToTarget(p);
  for(unsigned i=0;i<3;i++)out[i]=out[i]*targetUnitsPerSourceUnit+targetRoot[i];return out;
 }
 PrecisePoint PointToSource(PrecisePoint p)const{
  for(unsigned i=0;i<3;i++)p[i]=(p[i]-targetRoot[i])/targetUnitsPerSourceUnit;auto out=VectorToSource(p);
  for(unsigned i=0;i<3;i++)out[i]+=sourceRoot[i];return out;
 }
 double LengthToTarget(double length)const{return length*targetUnitsPerSourceUnit;}
 double LengthToSource(double length)const{return length/targetUnitsPerSourceUnit;}
};
// Exact sparse lineage, usable for source vertex interpolation and body-field
// donors. Do not use a nearest rendered vertex as a replacement for its rows.
struct DeltaBinding {
 std::uint32_t sourceVertexCount=0;
 std::vector<std::uint32_t> offsets,donors;
 std::vector<double> weights;
 void Validate()const{
  if(!sourceVertexCount||offsets.empty()||offsets.front()!=0||offsets.back()!=donors.size()||weights.size()!=donors.size())throw std::invalid_argument("Invalid lineage dimensions");
  for(unsigned row=0;row+1<offsets.size();row++){
   if(offsets[row]>=offsets[row+1]||offsets[row+1]>donors.size())throw std::invalid_argument("Invalid lineage row");
   double sum=0;for(unsigned k=offsets[row];k<offsets[row+1];k++){
    if(donors[k]>=sourceVertexCount||!std::isfinite(weights[k])||weights[k]<0)throw std::invalid_argument("Invalid lineage donor/weight");sum+=weights[k];
   }
   if(std::abs(sum-1)>1e-8)throw std::invalid_argument("Lineage weights must preserve constant fields");
  }
 }
 std::vector<PrecisePoint> Apply(const std::vector<PrecisePoint>& current,const std::vector<PrecisePoint>& neutral)const{
  Validate();if(current.size()!=sourceVertexCount||neutral.size()!=sourceVertexCount)throw std::invalid_argument("Lineage source topology differs");
  for(const auto* points:{&current,&neutral})for(auto p:*points)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Non-finite lineage source");
  std::vector<PrecisePoint> out(offsets.size()-1);
  for(unsigned row=0;row<out.size();row++)for(unsigned k=offsets[row];k<offsets[row+1];k++)for(unsigned axis=0;axis<3;axis++)out[row][axis]+=(current[donors[k]][axis]-neutral[donors[k]][axis])*weights[k];return out;
 }
};
}
