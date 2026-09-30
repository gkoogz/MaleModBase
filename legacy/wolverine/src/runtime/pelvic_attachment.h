#pragma once
#include "pelvic_attachment_data.h"
// Authored, render-only junction shapes. Lobe vertices and a two-ring halo
// have no entries in this field, so the existing dynamics remain authoritative.
static V3 paBodyBefore[paBodyNormalCount],paBodyStep[paBodyNormalCount],paBodyNormals[paBodyNormalCount];
static V3 paR14Step[r14Count],paNewNormals[r14Count],paNewTangents[r14Count];
static V3 paBodyGroupNormals[paBodyGroupCount];
static unsigned char paSavedBasis[paBodyNormalCount][8];
static bool paBasisSaved=false;
static float paAppliedFraction=0.f;
static V3 PARead(const float* a,unsigned i){return {a[3*i],a[3*i+1],a[3*i+2]};}
static void ResetPelvicAttachmentBody(unsigned char* buffer){
  for(unsigned k=0;k<paBodyNormalCount;k++){
    unsigned char* v=buffer+paBodyNormalIDs[k]*graftStride;
    if(!paBasisSaved)memcpy(paSavedBasis[k],v+12,8);
    memcpy(v+12,paSavedBasis[k],8);
    V3 p=PARead(paBodyNormalBase,k);memcpy(v,&p,12);
  }
  paBasisSaved=true;
}
static void ApplyPelvicAttachment(unsigned char* buffer){PerfScope perf(6);
  paAppliedFraction=0.f;
  const float knots[3]={2.890559f,5.161067f,6.045556f};
  float radius=logicalShaftBodyRadius,blend=0.f,extra=1.f;unsigned a=0,b=1;
  if(radius<=knots[0])extra=max(.1f,radius/knots[0]);
  else if(radius<knots[1])blend=(radius-knots[0])/(knots[1]-knots[0]);
  else {a=1;b=2;blend=min(1.f,(radius-knots[1])/(knots[2]-knots[1]));if(radius>knots[2])extra=min(1.6f,radius/knots[2]);}
  V3 ref=Unit(PARead(paReferenceAxis,a)*(1.f-blend)+PARead(paReferenceAxis,b)*blend);
  V3 live=Unit(shaftNodes[5]-shaftNodes[1]);
  // The authored raised-shaft extension occupies the folding shaft's space.
  // Fade with the existing smoothly transitioning mode, not contact impulses.
  float fade=Smoother01((live.x+.15f)/.5f)*ModeValue(1.f,1.f,0.f);
  if(fade<=0.f)return;
  memset(paR14Step,0,sizeof(paR14Step));memset(paBodyStep,0,sizeof(paBodyStep));
  for(unsigned k=0;k<paBodyNormalCount;k++)memcpy(&paBodyBefore[k],buffer+paBodyNormalIDs[k]*graftStride,12);
  static unsigned bodyBinding[paBodyCount];static bool bindingsReady=false;
  if(!bindingsReady){for(unsigned k=0;k<paBodyCount;k++)bodyBinding[k]=unsigned(std::lower_bound(paBodyNormalIDs,paBodyNormalIDs+paBodyNormalCount,paBodyIDs[k])-paBodyNormalIDs);bindingsReady=true;}
  GeometryRotation rotation(ref,live);
  for(unsigned k=0;k<paBodyCount;k++){
    V3 delta=(PARead(paBodyDelta,a*paBodyCount+k)*(1.f-blend)+PARead(paBodyDelta,b*paBodyCount+k)*blend)*(extra*fade);
    unsigned n=bodyBinding[k];
    paBodyStep[n]=delta;
  }
  for(unsigned k=0;k<paR14Count;k++){
    V3 delta=(PARead(paR14Delta,a*paR14Count+k)*(1.f-blend)+PARead(paR14Delta,b*paR14Count+k)*blend)*extra;
    V3 turned=rotation.Apply(delta);
    paR14Step[paR14IDs[k]]=(delta*(1.f-paRotateWeight[k])+turned*paRotateWeight[k])*fade;
  }
  // Preserve exact body/skin welds, including at animated root angles.
  for(unsigned k=0;k<paSeamCount;k++)paR14Step[paSeamR14[k]]=paBodyStep[paSeamBody[k]];
  float fraction=1.f;
  for(unsigned k=0;k<paR14FaceCount*3;k+=3){unsigned x=paR14NormalFaces[k],y=paR14NormalFaces[k+1],z=paR14NormalFaces[k+2];
    fraction=min(fraction,SurfaceCorrectionLimit(r14Positions[x],r14Positions[y],r14Positions[z],paR14Step[x],paR14Step[y],paR14Step[z],.10f,1e-10f));}
  for(unsigned k=0;k<paBodyFaceCount;k++){unsigned x=paBodyNormalFaces[k*3],y=paBodyNormalFaces[k*3+1],z=paBodyNormalFaces[k*3+2];
    fraction=min(fraction,SurfaceCorrectionLimit(paBodyBefore[x],paBodyBefore[y],paBodyBefore[z],paBodyStep[x],paBodyStep[y],paBodyStep[z],.10f,1e-10f));}
  paAppliedFraction=fraction*fade;
  for(unsigned k=0;k<paR14Count;k++){unsigned i=paR14IDs[k];r14Positions[i]=r14Positions[i]+paR14Step[i]*fraction;}
  for(unsigned k=0;k<paBodyNormalCount;k++){paBodyBefore[k]=paBodyBefore[k]+paBodyStep[k]*fraction;memcpy(buffer+paBodyNormalIDs[k]*graftStride,&paBodyBefore[k],12);}
  memset(paNewNormals,0,sizeof(paNewNormals));memset(paNewTangents,0,sizeof(paNewTangents));memset(paBodyNormals,0,sizeof(paBodyNormals));
  for(unsigned k=0;k<paR14FaceCount*3;k+=3){unsigned x=paR14NormalFaces[k],y=paR14NormalFaces[k+1],z=paR14NormalFaces[k+2];V3 e1=r14Positions[y]-r14Positions[x],e2=r14Positions[z]-r14Positions[x],n=Cross(e2,e1);
    paNewNormals[x]=paNewNormals[x]+n;paNewNormals[y]=paNewNormals[y]+n;paNewNormals[z]=paNewNormals[z]+n;
    float u1=r14UV[y*2]-r14UV[x*2],v1=r14UV[y*2+1]-r14UV[x*2+1],u2=r14UV[z*2]-r14UV[x*2],v2=r14UV[z*2+1]-r14UV[x*2+1],det=u1*v2-v1*u2;
    if(fabsf(det)>1e-10f){V3 t=(e1*v2-e2*v1)/det;paNewTangents[x]=paNewTangents[x]+t;paNewTangents[y]=paNewTangents[y]+t;paNewTangents[z]=paNewTangents[z]+t;}}
  for(unsigned k=0;k<paBodyFaceCount;k++){unsigned x=paBodyNormalFaces[k*3],y=paBodyNormalFaces[k*3+1],z=paBodyNormalFaces[k*3+2];V3 n=Cross(paBodyBefore[z]-paBodyBefore[x],paBodyBefore[y]-paBodyBefore[x]);paBodyNormals[x]=paBodyNormals[x]+n;paBodyNormals[y]=paBodyNormals[y]+n;paBodyNormals[z]=paBodyNormals[z]+n;}
  memset(paBodyGroupNormals,0,sizeof(paBodyGroupNormals));
  for(unsigned k=0;k<paBodyNormalCount;k++)paBodyGroupNormals[paBodyNormalGroup[k]]=paBodyGroupNormals[paBodyNormalGroup[k]]+paBodyNormals[k];
  for(unsigned k=0;k<paSeamCount;k++){unsigned group=paBodyNormalGroup[paSeamBody[k]];paBodyGroupNormals[group]=paBodyGroupNormals[group]+paNewNormals[paSeamR14[k]];}
  for(unsigned k=0;k<paBodyNormalCount;k++)paBodyNormals[k]=paBodyGroupNormals[paBodyNormalGroup[k]];
  for(unsigned k=0;k<paSeamCount;k++)paNewNormals[paSeamR14[k]]=paBodyNormals[paSeamBody[k]];
  for(unsigned k=0;k<paNormalCount;k++){unsigned i=paR14NormalIDs[k];V3 n=Unit(paNewNormals[i]),t=Unit(paNewTangents[i]-n*Dot(n,paNewTangents[i]));unsigned char* v=r14Packed+i*32;memcpy(v,&r14Positions[i],12);v[12]=PackSigned(t.x);v[13]=PackSigned(t.y);v[14]=PackSigned(t.z);v[16]=PackSigned(n.x);v[17]=PackSigned(n.y);v[18]=PackSigned(n.z);}
  for(unsigned k=0;k<paBodyNormalCount;k++){
    if(!paBodyNormalWrite[k])continue;
    unsigned char* v=buffer+paBodyNormalIDs[k]*graftStride;V3 n=Unit(paBodyNormals[k]);if(Length(n)<.5f)continue;
    V3 t{(v[12]/255.f)*2.f-1.f,(v[13]/255.f)*2.f-1.f,(v[14]/255.f)*2.f-1.f};t=Unit(t-n*Dot(t,n));
    if(Length(t)<.5f)t=Unit(Cross(fabsf(n.z)<.9f?V3{0,0,1}:V3{0,1,0},n));
    v[12]=PackSigned(t.x);v[13]=PackSigned(t.y);v[14]=PackSigned(t.z);v[16]=PackSigned(n.x);v[17]=PackSigned(n.y);v[18]=PackSigned(n.z);
  }
  r14UploadPending=true;
}
