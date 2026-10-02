#pragma once
#include "xpbd_kernels.hpp"
namespace malemod::physics {
struct Q4 {float x,y,z,w;};
inline int ClusterIndex(int i,int firstA,int countA,int firstB){
 if(i<countA)return firstA+i;
 return firstB+i-countA;
}
inline V3 ClusterCenter(State& state,int firstA,int countA,int firstB,int countB){
 V3 center={0,0,0};
 for(int i=0;i<countA+countB;i++)center=center+state.position[ClusterIndex(i,firstA,countA,firstB)];
 return center/(countA+countB);
}
inline V3 RotateCluster(Q4 q,V3 p){
 V3 axis={q.x,q.y,q.z};
 V3 cross=Cross(axis,p)*2.f;
 return p+cross*q.w+Cross(axis,cross);
}
// A morph changes the material pivot without changing the engine rig bind.
// Keep its skin map R*(vertex-material)+current exactly, including rotation.
inline V3 VirtualBindTranslation(V3 bind,V3 material,V3 current,Q4 rotation){
 V3 offset=material-bind;
 return current-material+offset-RotateCluster(rotation,offset);
}
// Horn's proper-rotation fit, with a positive spectral shift. The off-axis
// supports must span 3D. The caller supplies the previous accepted rotation as
// the warm start; no independently inferred tangent or missing roll axis.
inline Q4 FitCluster(State& state,const std::vector<V3>& rest,V3 restCenter,
 int firstA,int countA,int firstB,int countB,Q4 seed){
 V3 center=ClusterCenter(state,firstA,countA,firstB,countB);
 float xx=0,xy=0,xz=0,yx=0,yy=0,yz=0,zx=0,zy=0,zz=0;
 for(int i=0;i<countA+countB;i++){
  int id=ClusterIndex(i,firstA,countA,firstB);
  V3 a=rest[id]-restCenter,b=state.position[id]-center;
  xx=xx+a.x*b.x;xy=xy+a.x*b.y;xz=xz+a.x*b.z;
  yx=yx+a.y*b.x;yy=yy+a.y*b.y;yz=yz+a.y*b.z;
  zx=zx+a.z*b.x;zy=zy+a.z*b.y;zz=zz+a.z*b.z;
 }
 float shift=2.f*sqrtf(xx*xx+xy*xy+xz*xz+yx*yx+yy*yy+yz*yz+zx*zx+zy*zy+zz*zz);
 float x=seed.x,y=seed.y,z=seed.z,w=seed.w;
 for(int iteration=0;iteration<24;iteration++){
  float nx=(xx-yy-zz+shift)*x+(xy+yx)*y+(xz+zx)*z+(yz-zy)*w;
  float ny=(xy+yx)*x+(-xx+yy-zz+shift)*y+(yz+zy)*z+(zx-xz)*w;
  float nz=(xz+zx)*x+(yz+zy)*y+(-xx-yy+zz+shift)*z+(xy-yx)*w;
  float nw=(yz-zy)*x+(zx-xz)*y+(xy-yx)*z+(xx+yy+zz+shift)*w;
  float length=sqrtf(nx*nx+ny*ny+nz*nz+nw*nw);
  if(length<1e-20f)return seed;
  x=nx/length;y=ny/length;z=nz/length;w=nw/length;
 }
 return {x,y,z,w};
}
inline void ProjectCluster(State& state,const std::vector<V3>& rest,V3 restCenter,
 int firstA,int countA,int firstB,int countB,Q4 rotation){
 V3 center=ClusterCenter(state,firstA,countA,firstB,countB);
 for(int i=0;i<countA+countB;i++){
  int id=ClusterIndex(i,firstA,countA,firstB);
  state.position[id]=center+RotateCluster(rotation,rest[id]-restCenter);
 }
}
}
