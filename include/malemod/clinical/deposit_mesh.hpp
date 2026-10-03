#pragma once
#include "fluid_splat_model.h"
namespace malemod::clinical {
struct DepositVertex {V3 p,n;float alpha=0;volumeFluid::LiquidPhase phase=volumeFluid::LiquidPhase::opaque;};
struct DepositMesh {std::vector<DepositVertex> vertices;std::vector<unsigned> indices;};
// Portable scene presentation of the source's baked deposition field. Engine
// adapters supply projection anchors and lit transparent materials. Unknown
// anchors never become a plane crossing an unrelated surface.
inline DepositMesh BuildDepositMesh(const volumeFluid::SplatModel& model,std::uint32_t now){
 using namespace volumeFluid;DepositMesh out;
 for(const auto& mark:model.marks){
  float age=std::uint32_t(now-mark.touched)*.001f,fade=age<15?1:std::max(0.f,(20-age)/5);
  V3 origin,normal,x,y;if(!model.Frame(mark,origin,normal,x,y)||!fade)continue;
  std::array<int,splatGrid*splatGrid> mapping{};mapping.fill(-1);
  auto vertex=[&](int id){if(mapping[id]>=0)return mapping[id];const auto& s=mark.samples[id];V3 p,n;
   if(s.state!=1||!model.Resolve(s.anchor,p,n))return -1;
   const float density=SplatModel::Field(mark,SplatModel::Coordinate(id%splatGrid,mark.span),SplatModel::Coordinate(id/splatGrid,mark.span));
   const float alpha=std::clamp((density-.00015f)/.001f,0.f,1.f)*fade;
   int index=int(out.vertices.size());out.vertices.push_back({p+n*.027f,n,alpha,mark.phase});mapping[id]=index;return index;
  };
  for(int iy=mark.y0;iy<mark.y1;iy++)for(int ix=mark.x0;ix<mark.x1;ix++){
   int ids[4]={iy*splatGrid+ix,iy*splatGrid+ix+1,(iy+1)*splatGrid+ix,(iy+1)*splatGrid+ix+1};
   float field=0;for(int id:ids)field=std::max(field,mark.samples[id].field);if(field<=.0001f)continue;
   int v[4];for(int i=0;i<4;i++)v[i]=vertex(ids[i]);
   const int triangles[6]={0,1,2,1,3,2};
   for(unsigned i=0;i<6;i+=3){int a=v[triangles[i]],b=v[triangles[i+1]],c=v[triangles[i+2]];
    if(a<0||b<0||c<0)continue;
    if(Length(out.vertices[a].p-out.vertices[b].p)>=mark.span/(splatGrid-1)*3||Length(out.vertices[a].p-out.vertices[c].p)>=mark.span/(splatGrid-1)*3)continue;
    out.indices.insert(out.indices.end(),{unsigned(a),unsigned(b),unsigned(c)});
   }
  }
 }
 return out;
}
}
