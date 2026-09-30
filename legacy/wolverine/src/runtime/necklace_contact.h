#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include "necklace_contact_data.h"

struct NcPoint { float x,y,z; };
struct NcRig { float matrix[128][12]{}; bool valid[128]{}; };
struct NcContactStats { int contacts=0; float penetrationBefore=0,remaining=0; bool limited=false; };
static NcPoint NcTransform(const float* m,NcPoint p){return {m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3],m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7],m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11]};}
static bool NcInverse(const float* m,float* r){
  float d=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]);
  if(!std::isfinite(d)||fabsf(d)<.0001f)return false;
  r[0]=(m[5]*m[10]-m[6]*m[9])/d;r[1]=(m[2]*m[9]-m[1]*m[10])/d;r[2]=(m[1]*m[6]-m[2]*m[5])/d;
  r[4]=(m[6]*m[8]-m[4]*m[10])/d;r[5]=(m[0]*m[10]-m[2]*m[8])/d;r[6]=(m[2]*m[4]-m[0]*m[6])/d;
  r[8]=(m[4]*m[9]-m[5]*m[8])/d;r[9]=(m[1]*m[8]-m[0]*m[9])/d;r[10]=(m[0]*m[5]-m[1]*m[4])/d;
  for(int i=0;i<3;i++)r[i*4+3]=-r[i*4]*m[3]-r[i*4+1]*m[7]-r[i*4+2]*m[11];
  return true;
}
static NcPoint NcSkin(const NcVertex& v,const NcRig& rig){
  NcPoint out{};
  for(int i=0;i<4;i++)if(v.weight[i]){
    NcPoint p=NcTransform(rig.matrix[v.bone[i]],{v.p[0],v.p[1],v.p[2]});float w=v.weight[i]/255.f;
    out.x+=w*p.x;out.y+=w*p.y;out.z+=w*p.z;
  }
  return out;
}
class NcContactSolver {
  static constexpr int gridSize=32;
  std::vector<int> bins[gridSize*gridSize];
  std::vector<NcPoint> chest;
  std::vector<int> order;
  float lowY=0,lowZ=0,scaleY=1,scaleZ=1;
  struct Constraint { float weight[4]{},need=0; };
  std::vector<Constraint> constraints;
  bool cacheValid=false;
  NcRig cachedRig;
  float cachedShift[4]{};
  NcContactStats cachedStats;
  int cell(float x){return (std::max)(0,(std::min)(gridSize-1,(int)floorf(x)));}
public:
  NcContactSolver(){
    chest.resize(sizeof(ncChestVertices)/sizeof(ncChestVertices[0]));
    constraints.resize(sizeof(ncTagVertices)/sizeof(ncTagVertices[0]));
    for(unsigned i=0;i<constraints.size();i++){
      order.push_back((int)i);
      for(int k=0;k<4;k++)if(ncTagVertices[i].bone[k]>=103&&ncTagVertices[i].bone[k]<=106)
        constraints[i].weight[ncTagVertices[i].bone[k]-103]+=ncTagVertices[i].weight[k]/255.f;
    }
    std::stable_sort(order.begin(),order.end(),[&](int a,int b){
      return *std::max_element(constraints[a].weight,constraints[a].weight+4)>*std::max_element(constraints[b].weight,constraints[b].weight+4);
    });
  }
  bool surface(float y,float z,float& front){
    float cy=(y-lowY)*scaleY,cz=(z-lowZ)*scaleZ;
    if(cy<0||cz<0||cy>=gridSize||cz>=gridSize)return false;
    bool hit=false;front=-1e20f;
    for(int ti:bins[cell(cz)*gridSize+cell(cy)]){
      const auto& t=ncChestTriangles[ti];NcPoint a=chest[t[0]],b=chest[t[1]],c=chest[t[2]];
      float d=(b.y-a.y)*(c.z-a.z)-(b.z-a.z)*(c.y-a.y);if(fabsf(d)<1e-7f)continue;
      float u=((y-a.y)*(c.z-a.z)-(z-a.z)*(c.y-a.y))/d;
      float v=((b.y-a.y)*(z-a.z)-(b.z-a.z)*(y-a.y))/d;
      if(u>=-1e-5f&&v>=-1e-5f&&u+v<=1.00001f){front=(std::max)(front,a.x+u*(b.x-a.x)+v*(c.x-a.x));hit=true;}
    }
    return hit;
  }
  bool solve(const NcRig& rig,float shift[4],NcContactStats& stats){
    if(cacheValid&&memcmp(&rig,&cachedRig,sizeof(rig))==0){memcpy(shift,cachedShift,sizeof(cachedShift));stats=cachedStats;return true;}
    stats={};for(int i=0;i<4;i++){shift[i]=0;if(!rig.valid[103+i])return false;}
    for(unsigned char b:ncRequiredBones)if(!rig.valid[b])return false;
    float inverse[12];if(!NcInverse(rig.matrix[6],inverse))return false;
    lowY=lowZ=1e20f;float highY=-1e20f,highZ=-1e20f;
    for(unsigned i=0;i<chest.size();i++){
      NcPoint p=NcTransform(inverse,NcSkin(ncChestVertices[i],rig));
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
      chest[i]=p;lowY=(std::min)(lowY,p.y);lowZ=(std::min)(lowZ,p.z);highY=(std::max)(highY,p.y);highZ=(std::max)(highZ,p.z);
    }
    lowY-=.01f;lowZ-=.01f;highY+=.01f;highZ+=.01f;
    scaleY=gridSize/(highY-lowY);scaleZ=gridSize/(highZ-lowZ);
    for(auto& bin:bins)bin.clear();
    for(unsigned i=0;i<sizeof(ncChestTriangles)/sizeof(ncChestTriangles[0]);i++){
      const auto& t=ncChestTriangles[i];NcPoint a=chest[t[0]],b=chest[t[1]],c=chest[t[2]];
      int ymin=cell(((std::min)({a.y,b.y,c.y})-lowY)*scaleY),ymax=cell(((std::max)({a.y,b.y,c.y})-lowY)*scaleY);
      int zmin=cell(((std::min)({a.z,b.z,c.z})-lowZ)*scaleZ),zmax=cell(((std::max)({a.z,b.z,c.z})-lowZ)*scaleZ);
      for(int z=zmin;z<=zmax;z++)for(int y=ymin;y<=ymax;y++)bins[z*gridSize+y].push_back((int)i);
    }
    for(unsigned i=0;i<constraints.size();i++){
      auto& c=constraints[i];c.need=0;float w=c.weight[0]+c.weight[1]+c.weight[2]+c.weight[3];
      if(w<=.25f||ncTagVertices[i].p[0]<=3.f)continue;
      NcPoint p=NcTransform(inverse,NcSkin(ncTagVertices[i],rig));float front;
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
      if(surface(p.y,p.z,front)){
        c.need=(std::max)(0.f,front+.18f-p.x);
        if(c.need>0){stats.contacts++;stats.penetrationBefore=(std::max)(stats.penetrationBefore,c.need-.18f);}
      }
    }
    for(int i:order){
      auto& c=constraints[i];float current=0,denom=0;
      for(int k=0;k<4;k++){current+=c.weight[k]*shift[k];denom+=c.weight[k]*c.weight[k];}
      if(c.need>current&&denom>1e-8f)for(int k=0;k<4;k++)shift[k]+=c.weight[k]*(c.need-current)/denom;
    }
    // An extreme or unsupported pose must not produce another huge displacement.
    for(int k=0;k<4;k++)if(shift[k]>10.f){shift[k]=10.f;stats.limited=true;}
    for(const auto& c:constraints){float actual=0;for(int k=0;k<4;k++)actual+=c.weight[k]*shift[k];stats.remaining=(std::max)(stats.remaining,c.need-actual);}
    cachedRig=rig;memcpy(cachedShift,shift,sizeof(cachedShift));cachedStats=stats;cacheValid=true;
    return true;
  }
};
