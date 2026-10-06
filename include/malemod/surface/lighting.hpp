#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace malemod::surface {
// Target topology lighting, independent of graphics packing and game shaders.
// Normal groups preserve authored hard edges while welding UV aliases. Tangents
// remain per render vertex, retaining each UV island's handedness.
using LightingPoint=std::array<double,3>;
struct LightingFrame {LightingPoint normal,tangent;double sign=1;};
struct LightingEdgeConstraint {std::uint32_t slave,a,b;double weight;};
inline LightingPoint LightingSub(LightingPoint a,LightingPoint b){for(unsigned i=0;i<3;i++)a[i]-=b[i];return a;}
inline LightingPoint LightingCross(LightingPoint a,LightingPoint b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline double LightingDot(LightingPoint a,LightingPoint b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline LightingPoint LightingUnit(LightingPoint p,LightingPoint fallback){double s=LightingDot(p,p);if(s<1e-24){p=fallback;s=LightingDot(p,p);}if(!std::isfinite(s)||s<1e-24)throw std::invalid_argument("Degenerate lighting direction");for(auto& x:p)x/=std::sqrt(s);return p;}
// A deformed attachment must not inherit a baked hard normal from the old
// disconnected surface. Blend toward its actual geometric normal while
// retaining each UV chart's tangent direction and handedness.
inline LightingFrame AlignGeometricNormal(LightingFrame cooked,LightingFrame geometric,double weight){
 if(!std::isfinite(weight)||weight<0||weight>1)throw std::invalid_argument("Invalid geometric normal blend");
 if(weight==0)return cooked;
 LightingPoint n{};for(unsigned a=0;a<3;a++)n[a]=(1-weight)*cooked.normal[a]+weight*geometric.normal[a];
 cooked.normal=LightingUnit(n,geometric.normal);const double dot=LightingDot(cooked.tangent,cooked.normal);
 for(unsigned a=0;a<3;a++)cooked.tangent[a]-=dot*cooked.normal[a];
 auto fallback=geometric.tangent;const double fallbackDot=LightingDot(fallback,cooked.normal);
 for(unsigned a=0;a<3;a++)fallback[a]-=fallbackDot*cooked.normal[a];
 if(LightingDot(fallback,fallback)<1e-24)fallback=LightingCross(cooked.normal,std::abs(cooked.normal[2])<.9?LightingPoint{0,0,1}:LightingPoint{0,1,0});
 cooked.tangent=LightingUnit(cooked.tangent,fallback);return cooked;
}
// Smooth the sampled normal field only within a measured attachment band.
// The positional surface is checked separately; this cannot repair a gap or
// fold. UV charts keep their own tangent and sign, including mirrored islands.
inline void FairLightingNormals(std::vector<LightingFrame>& frames,const std::vector<std::uint32_t>& groups,
 const std::vector<std::vector<unsigned>>& neighbors,const std::vector<double>& weights,unsigned iterations=8){
 const auto n=frames.size();if(groups.size()!=n||neighbors.size()!=n||weights.size()!=n)throw std::invalid_argument("Normal fairing dimensions differ");
 std::vector<LightingPoint> normal(n);std::vector<unsigned> count(n);std::vector<double> blend(n);
 for(unsigned i=0;i<n;i++){if(groups[i]>=n||!std::isfinite(weights[i])||weights[i]<0||weights[i]>1)throw std::invalid_argument("Invalid normal fairing binding");for(unsigned a=0;a<3;a++)normal[groups[i]][a]+=frames[i].normal[a];count[groups[i]]++;blend[groups[i]]=std::max(blend[groups[i]],weights[i]);}
 for(unsigned i=0;i<n;i++)if(count[i])normal[i]=LightingUnit(normal[i],frames[i].normal);
 for(unsigned step=0;step<iterations;step++){auto next=normal;
  for(unsigned i=0;i<n;i++)if(blend[i]>0&&!neighbors[i].empty()){
   LightingPoint mean{};for(auto j:neighbors[i]){if(j>=n)throw std::invalid_argument("Normal neighbor outside topology");for(unsigned a=0;a<3;a++)mean[a]+=normal[j][a]/neighbors[i].size();}
   const double t=.5*blend[i];for(unsigned a=0;a<3;a++)mean[a]=(1-t)*normal[i][a]+t*mean[a];next[i]=LightingUnit(mean,normal[i]);
  }
  normal.swap(next);
 }
 for(unsigned i=0;i<n;i++)if(weights[i]>0){auto geometric=frames[i];geometric.normal=normal[groups[i]];frames[i]=AlignGeometricNormal(frames[i],geometric,1);}
}
// Interpolated positional seams require the same donor normal field. Keep each
// UV island's own tangent direction and handedness; do not average UV charts.
inline void WeldEdgeLighting(std::vector<LightingFrame>& frames,const std::vector<LightingEdgeConstraint>& edges){
 const auto before=frames;
 for(auto e:edges){
  if(e.slave>=frames.size()||e.a>=frames.size()||e.b>=frames.size()||!std::isfinite(e.weight)||e.weight<0||e.weight>1)throw std::invalid_argument("Invalid lighting seam donor");
  auto& f=frames[e.slave];LightingPoint n{};for(unsigned a=0;a<3;a++)n[a]=(1-e.weight)*before[e.a].normal[a]+e.weight*before[e.b].normal[a];
  f.normal=LightingUnit(n,before[e.slave].normal);auto t=before[e.slave].tangent;const auto dot=LightingDot(t,f.normal);for(unsigned a=0;a<3;a++)t[a]-=dot*f.normal[a];
  auto fallback=LightingCross(f.normal,std::abs(f.normal[2])<.9?LightingPoint{0,0,1}:LightingPoint{0,1,0});f.tangent=LightingUnit(t,fallback);
 }
}
inline std::vector<LightingFrame> RebuildLighting(const std::vector<LightingPoint>& positions,
 const std::vector<std::array<double,2>>& uv,const std::vector<std::array<std::uint32_t,3>>& faces,
 const std::vector<std::uint32_t>& normalGroups,const std::vector<LightingFrame>& fallback){
 const auto n=positions.size();if(!n||uv.size()!=n||normalGroups.size()!=n||fallback.size()!=n)throw std::invalid_argument("Lighting topology dimensions differ");
 std::vector<LightingPoint> normals(n),tangents(n),bitangents(n);
 for(unsigned i=0;i<n;i++){if(normalGroups[i]>=n)throw std::invalid_argument("Lighting alias outside topology");for(auto x:positions[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite lighting position");for(auto x:uv[i])if(!std::isfinite(x))throw std::invalid_argument("Nonfinite lighting UV");}
 for(auto face:faces){for(auto i:face)if(i>=n)throw std::invalid_argument("Lighting triangle outside topology");
  auto e1=LightingSub(positions[face[1]],positions[face[0]]),e2=LightingSub(positions[face[2]],positions[face[0]]),normal=LightingCross(e1,e2);
  const double u1=uv[face[1]][0]-uv[face[0]][0],v1=uv[face[1]][1]-uv[face[0]][1],u2=uv[face[2]][0]-uv[face[0]][0],v2=uv[face[2]][1]-uv[face[0]][1],det=u1*v2-v1*u2;
  LightingPoint tangent{},bitangent{};if(std::abs(det)>1e-20)for(unsigned a=0;a<3;a++){tangent[a]=(e1[a]*v2-e2[a]*v1)/det;bitangent[a]=(e2[a]*u1-e1[a]*u2)/det;}
  for(auto i:face)for(unsigned a=0;a<3;a++){normals[normalGroups[i]][a]+=normal[a];tangents[i][a]+=tangent[a];bitangents[i][a]+=bitangent[a];}
 }
 std::vector<LightingFrame> out(n);
 for(unsigned i=0;i<n;i++){auto normal=LightingUnit(normals[normalGroups[i]],fallback[i].normal);auto tangent=tangents[i];double dot=LightingDot(normal,tangent);for(unsigned a=0;a<3;a++)tangent[a]-=normal[a]*dot;
  auto prior=fallback[i].tangent;dot=LightingDot(normal,prior);for(unsigned a=0;a<3;a++)prior[a]-=normal[a]*dot;
  if(LightingDot(prior,prior)<1e-24)prior=LightingCross(normal,std::abs(normal[2])<.9?LightingPoint{0,0,1}:LightingPoint{0,1,0});
  tangent=LightingUnit(tangent,prior);double sign=LightingDot(LightingCross(normal,tangent),bitangents[i]);out[i]={normal,tangent,std::abs(sign)<1e-20?fallback[i].sign:(sign<0?-1.:1.)};
 }
 return out;
}
}
