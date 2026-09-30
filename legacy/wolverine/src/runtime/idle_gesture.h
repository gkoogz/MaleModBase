#pragma once
#include <cmath>
#include <cstring>
#include "idle_gesture_data.h"
struct IGPose { bool active=false; float q[2][4]{}; };
static float IGClamp(float x){return x<0?0:x>1?1:x;}
static float IGEase(float x){x=IGClamp(x);return x*x*x*(10+x*(-15+6*x));}
static void IGMultiply(const float*a,const float*b,float*out){
 float c[12];for(int r=0;r<3;r++){for(int k=0;k<3;k++)c[r*4+k]=a[r*4]*b[k]+a[r*4+1]*b[4+k]+a[r*4+2]*b[8+k];c[r*4+3]=a[r*4]*b[3]+a[r*4+1]*b[7]+a[r*4+2]*b[11]+a[r*4+3];}memcpy(out,c,sizeof(c));
}
static bool IGInverse(const float*m,float*out){
 float d=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]);
 if(!std::isfinite(d)||fabsf(d)<1e-7f)return false;
 out[0]=(m[5]*m[10]-m[6]*m[9])/d;out[1]=(m[2]*m[9]-m[1]*m[10])/d;out[2]=(m[1]*m[6]-m[2]*m[5])/d;
 out[4]=(m[6]*m[8]-m[4]*m[10])/d;out[5]=(m[0]*m[10]-m[2]*m[8])/d;out[6]=(m[2]*m[4]-m[0]*m[6])/d;
 out[8]=(m[4]*m[9]-m[5]*m[8])/d;out[9]=(m[1]*m[8]-m[0]*m[9])/d;out[10]=(m[0]*m[5]-m[1]*m[4])/d;
 for(int r=0;r<3;r++)out[r*4+3]=-out[r*4]*m[3]-out[r*4+1]*m[7]-out[r*4+2]*m[11];return true;
}
static IGPose IGEvaluate(int clip,float elapsed,float duration,bool enabled=true){
 IGPose pose;
 if(!enabled||clip<0||clip>=19||!std::isfinite(duration)||duration<=0||!std::isfinite(elapsed)||elapsed<=0||elapsed>=duration)return pose;
 float envelope=IGEase(elapsed/.28f)*IGEase((duration-elapsed)/.35f);
 float gain=(.42f+.04f*(clip%3))*envelope;
 float frame=IGClamp(elapsed/duration)*(igFrameCount-1);unsigned a=(unsigned)frame,b=a+1<igFrameCount?a+1:a;float u=frame-a;
 for(unsigned bone=0;bone<2;bone++){
  const float*qa=igRotationKeys+(((clip%4)*igFrameCount+a)*2+bone)*4;
  const float*qb=igRotationKeys+(((clip%4)*igFrameCount+b)*2+bone)*4;
  float q[4],length=0;for(int k=0;k<4;k++){q[k]=qa[k]*(1-u)+qb[k]*u;length+=q[k]*q[k];}
  length=sqrtf(length);if(length<1e-7f)return IGPose{};
  for(int k=0;k<4;k++)q[k]/=length;if(q[3]<0)for(float&v:q)v=-v;
  float w=q[3]<-1?-1:q[3]>1?1:q[3];float theta=acosf(w),s=sinf(theta);
  float factor=fabsf(s)>1e-6f?sinf(theta*gain)/s:gain;
  for(int k=0;k<3;k++)pose.q[bone][k]=q[k]*factor;pose.q[bone][3]=cosf(theta*gain);
 }
 pose.active=envelope>1e-6f;return pose;
}
static void IGLocalRotation(const float*q,const float*p,float*m){
 float x=q[0],y=q[1],z=q[2],w=q[3];
 m[0]=1-2*(y*y+z*z);m[1]=2*(x*y-z*w);m[2]=2*(x*z+y*w);
 m[4]=2*(x*y+z*w);m[5]=1-2*(x*x+z*z);m[6]=2*(y*z-x*w);
 m[8]=2*(x*z-y*w);m[9]=2*(y*z+x*w);m[10]=1-2*(x*x+y*y);
 for(int r=0;r<3;r++)m[r*4+3]=p[r]-m[r*4]*p[0]-m[r*4+1]*p[1]-m[r*4+2]*p[2];
}
static bool IGApply(float*matrices,const unsigned char*palette,unsigned count,const IGPose&pose){
 if(!pose.active)return false;int spine=-1,neck=-1;
 for(unsigned i=0;i<count;i++){if(palette[i]==6)spine=i;if(palette[i]==61)neck=i;}
 if(spine<0||neck<0)return false;
 float parent[2][12],inverse[12],local[12],temp[12],change[2][12],head[12];
 IGMultiply(matrices+spine*12,igParentBind,parent[0]);IGMultiply(matrices+neck*12,igParentBind+12,parent[1]);
 for(int i=0;i<2;i++){
  if(!IGInverse(parent[i],inverse))return false;
  IGLocalRotation(pose.q[i],igPivots+i*3,local);IGMultiply(parent[i],local,temp);IGMultiply(temp,inverse,change[i]);
 }
 IGMultiply(change[0],change[1],head);
 for(unsigned i=0;i<count;i++){
  unsigned bone=palette[i];if(bone==61)IGMultiply(change[0],matrices+i*12,matrices+i*12);
  else if(bone>=62&&bone<=102)IGMultiply(head,matrices+i*12,matrices+i*12);
 }
 return true;
}
static const unsigned char* IGPalette(unsigned start,unsigned triangles,unsigned&count){
 const unsigned char*palettes[]={igPalette1,igPalette2,igPalette4,igPalette5};
 const unsigned sizes[]={sizeof(igPalette1),sizeof(igPalette2),sizeof(igPalette4),sizeof(igPalette5)};
 for(unsigned i=0;i<4;i++)if(start==igDrawSignatures[i][0]&&triangles==igDrawSignatures[i][1]){count=sizes[i];return palettes[i];}
 count=0;return nullptr;
}
