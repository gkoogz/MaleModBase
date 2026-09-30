#pragma once
#include "geometry_binding_data.h"
// The existing cage and solver remain authoritative. This layer transports
// the authored surface in local triangle frames after the existing solve.
static V3 rsBefore[r14Count],rsStep[r14Count],rsPositions[rsCount],rsNormals[rsCount],rsTangents[rsCount];
static V3 rsBodyStep[paBodyNormalCount];
static float rsAppliedFraction=0.f,rsFineFraction=0.f;
static float RSSafeFraction(V3 a,V3 b,V3 c,V3 da,V3 db,V3 dc,float floor){
  V3 before=Cross(b-a,c-a),after=Cross((b+db)-(a+da),(c+dc)-(a+da));
  if(Dot(before,after)>=floor*Dot(before,before)&&Dot(after,after)>1e-12f)return 1.f;
  return SurfaceCorrectionLimit(a,b,c,da,db,dc,floor,1e-12f);
}
static V3 RSFrameOffset(const V3* points,const unsigned* tri,const float* coeff){
  V3 a=points[tri[1]]-points[tri[0]],b=points[tri[2]]-points[tri[0]],normal=Cross(a,b);
  float length=Length(normal);if(length<1e-10f)return {};
  return a*coeff[0]+b*coeff[1]+normal*(coeff[2]/sqrtf(length));
}
struct RSPreparedFrame {
  V3 a,b,normal;float length,rootLength;
  void Prepare(const V3* points,const unsigned short* tri){
    a=points[tri[1]]-points[tri[0]];b=points[tri[2]]-points[tri[0]];normal=Cross(a,b);
    length=Length(normal);rootLength=sqrtf(length);
  }
  V3 Offset(const float* coeff) const {
    if(length<1e-10f)return {};
    return a*coeff[0]+b*coeff[1]+normal*(coeff[2]/rootLength);
  }
};
#include "pouch_surface.h"
#include "raphe_tube_surface.h"
static void ApplyRoundedShape(unsigned char* body){PerfScope perf(7);
  memcpy(rsBefore,r14Positions,sizeof(rsBefore));memset(rsStep,0,sizeof(rsStep));memset(rsBodyStep,0,sizeof(rsBodyStep));
  V3 live=Unit(shaftNodes[5]-shaftNodes[1]);float attachment=Smoother01((live.x+.15f)/.5f)*Smoother01((live.z+.05f)/.50f)*ModeValue(1.f,1.f,0.f);
  for(unsigned k=0;k<rsShapeCount;k++){
    float fade=rsShapeLobeFade[k]+(1.f-rsShapeLobeFade[k])*attachment;
    rsStep[rsShapeIDs[k]]=RSFrameOffset(rsBefore,rsShapeFrame+3*k,rsShapeCoeff+3*k)*fade;
  }
  if(physicsState!=2){
    static V3 contour[r14Count];memset(contour,0,sizeof(contour));
    for(unsigned k=0;k<rapheSupportCoreCount;k++){
      unsigned i=rapheSupportCore[k];float t=AxialMaterialFlex(i),blend=1.f-Smoother01((t-.42f)/.12f);if(blend<=0)continue;
      V3 center{},tangent{};SampleShaftChain(t,center,tangent);V3 ventral=Unit(Cross(Unit(V3{0,1,0}-tangent*tangent.y),tangent));
      float depth=Dot(rsBefore[i]+rsStep[i]-center,ventral);contour[i]=ventral*(max(0.f,axialCoreRadius-depth)*blend);
    }
    for(unsigned row=0;row<rapheSupportFreeCount;row++)for(unsigned k=0;k<rapheSupportCoreCount;k++)contour[rapheSupportFree[row]]=contour[rapheSupportFree[row]]+contour[rapheSupportCore[k]]*rapheSupportWeights[row*rapheSupportCoreCount+k];
    for(unsigned i=0;i<r14Count;i++)rsStep[i]=rsStep[i]+contour[i];
  }
  for(unsigned k=0;k<paBodyNormalCount;k++)memcpy(&paBodyBefore[k],body+paBodyNormalIDs[k]*32,12);
  float scale=max(.10f,min(2.f,logicalShaftBodyRadius/5.161067f));
  static unsigned bodyBinding[rsBodyCount];static bool bindingsReady=false;
  if(!bindingsReady){for(unsigned k=0;k<rsBodyCount;k++)bodyBinding[k]=unsigned(std::lower_bound(paBodyNormalIDs,paBodyNormalIDs+paBodyNormalCount,rsBodyIDs[k])-paBodyNormalIDs);bindingsReady=true;}
  for(unsigned k=0;k<rsBodyCount;k++){
    unsigned j=bodyBinding[k];
    rsBodyStep[j]=PARead(rsBodyDelta,k)*(scale*attachment);
  }
  for(unsigned k=0;k<paSeamCount;k++)rsStep[paSeamR14[k]]=rsBodyStep[paSeamBody[k]];
  // Restrict only locally strained triangles; a short crease edge must not
  // cancel the complete rounded-lobe field.
  for(int pass=0;pass<8;pass++){bool changed=false;for(unsigned k:geometryRoundedFaces){unsigned a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    float local=RSSafeFraction(rsBefore[a],rsBefore[b],rsBefore[c],rsStep[a],rsStep[b],rsStep[c],.025f);
    if(local<.9999f){rsStep[a]=rsStep[a]*local;rsStep[b]=rsStep[b]*local;rsStep[c]=rsStep[c]*local;changed=true;}}if(!changed)break;}
  for(unsigned k=0;k<paSeamCount;k++)rsStep[paSeamR14[k]]=rsBodyStep[paSeamBody[k]];
  float fraction=1.f;
  for(unsigned k:geometryRoundedFaces){unsigned a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    fraction=min(fraction,RSSafeFraction(rsBefore[a],rsBefore[b],rsBefore[c],rsStep[a],rsStep[b],rsStep[c],.01f));}
  for(unsigned k=0;k<paBodyFaceCount;k++){unsigned a=paBodyNormalFaces[3*k],b=paBodyNormalFaces[3*k+1],c=paBodyNormalFaces[3*k+2];
    fraction=min(fraction,RSSafeFraction(paBodyBefore[a],paBodyBefore[b],paBodyBefore[c],rsBodyStep[a],rsBodyStep[b],rsBodyStep[c],.01f));}
  rsAppliedFraction=fraction;
  for(unsigned k=0;k<r14Count;k++){r14Positions[k]=rsBefore[k]+rsStep[k]*fraction;rsPositions[k]=r14Positions[k];}
  for(unsigned k=0;k<paBodyNormalCount;k++){paBodyBefore[k]=paBodyBefore[k]+rsBodyStep[k]*fraction;memcpy(body+paBodyNormalIDs[k]*32,&paBodyBefore[k],12);}
  // Adaptive refinement is confined to the edited patch, retaining INDEX16.
  static V3 fineBase[rsFineCount],fineStep[rsFineCount];
  static RSPreparedFrame fineFrames[geometryFineFrameCount];
  for(unsigned k=0;k<geometryFineFrameCount;k++)fineFrames[k].Prepare(r14Positions,geometryFineFrames+3*k);
  GeometryFor(rsFineCount,[&](unsigned k){
    const unsigned* tri=rsFineSource+3*k;const float* w=rsFineBary+3*k;
    V3 base=r14Positions[tri[0]]*w[0]+r14Positions[tri[1]]*w[1]+r14Positions[tri[2]]*w[2];
    float fade=rsFineLobeFade[k]+(1.f-rsFineLobeFade[k])*attachment;
    fineBase[k]=base;fineStep[k]=fineFrames[geometryFineFrameIDs[k]].Offset(rsFineCoeff+3*k)*(fade*fraction);
    rsPositions[r14Count+k]=base;
  });
  for(int pass=0;pass<6;pass++){bool changed=false;for(unsigned k:geometryFineFaces){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];
    V3 da=a<r14Count?V3{}:fineStep[a-r14Count],db=b<r14Count?V3{}:fineStep[b-r14Count],dc=c<r14Count?V3{}:fineStep[c-r14Count];
    float local=RSSafeFraction(rsPositions[a],rsPositions[b],rsPositions[c],da,db,dc,.02f);
    if(local<.9999f){if(a>=r14Count)fineStep[a-r14Count]=da*local;if(b>=r14Count)fineStep[b-r14Count]=db*local;if(c>=r14Count)fineStep[c-r14Count]=dc*local;changed=true;}}if(!changed)break;}
  float fineFraction=1.f;
  for(unsigned k:geometryFineFaces){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];
    V3 da=a<r14Count?V3{}:fineStep[a-r14Count],db=b<r14Count?V3{}:fineStep[b-r14Count],dc=c<r14Count?V3{}:fineStep[c-r14Count];
    fineFraction=min(fineFraction,RSSafeFraction(rsPositions[a],rsPositions[b],rsPositions[c],da,db,dc,.01f));}
  rsFineFraction=fineFraction;
  for(unsigned k=0;k<rsFineCount;k++)rsPositions[r14Count+k]=fineBase[k]+fineStep[k]*fineFraction;
  ApplyPouchSurface();
  ApplyRapheTubeSkin(body);
  memcpy(rsPacked,r14Packed,sizeof(r14Packed));
  static bool attributesReady=false;
  if(!attributesReady)for(unsigned k=0;k<rsFineCount;k++){
    const unsigned* tri=rsFineSource+3*k;const float* w=rsFineBary+3*k;unsigned char* dst=rsPacked+(r14Count+k)*32;
    unsigned strongest=w[0]>w[1]?(w[0]>w[2]?0:2):(w[1]>w[2]?1:2);memcpy(dst,r14Packed+tri[strongest]*32,32);
    // Merge the three existing skinning palettes, retaining the four largest.
    float boneWeight[256]{};for(int j=0;j<3;j++)for(int b=0;b<4;b++)boneWeight[r14Bones[tri[j]*4+b]]+=w[j]*r14Weights[tri[j]*4+b];
    unsigned bones[4]{};float values[4]{};float total=0;
    for(int b=0;b<4;b++){unsigned best=0;for(unsigned j=1;j<256;j++)if(boneWeight[j]>boneWeight[best])best=j;bones[b]=best;values[b]=boneWeight[best];boneWeight[best]=0;total+=values[b];}
    unsigned used=0;for(int b=0;b<4;b++){dst[20+b]=(unsigned char)bones[b];unsigned value=b==3?255-used:(unsigned)max(0.f,min(float(255-used),floorf(values[b]*255.f/max(1e-6f,total)+.5f)));dst[24+b]=(unsigned char)value;used+=value;}
    float uv[2]{};for(int j=0;j<3;j++){uv[0]+=w[j]*r14UV[tri[j]*2];uv[1]+=w[j]*r14UV[tri[j]*2+1];}D3DXFloat32To16Array(reinterpret_cast<D3DXFLOAT16*>(dst+28),uv,2);
  }
  attributesReady=true;
  // UV topology is immutable; only the geometric edges change with the pose.
  // Decode the exact packed half floats once to retain the original tangent basis.
  static float uvDifferences[rsIndexCount/3][5];static bool uvReady=false;
  if(!uvReady){for(unsigned k=0;k<rsIndexCount;k+=3){
    float uv[3][2];for(unsigned j=0;j<3;j++)D3DXFloat16To32Array(uv[j],reinterpret_cast<const D3DXFLOAT16*>(rsPacked+rsIndices[k+j]*32+28),2);
    float* d=uvDifferences[k/3];d[0]=uv[1][0]-uv[0][0];d[1]=uv[1][1]-uv[0][1];d[2]=uv[2][0]-uv[0][0];d[3]=uv[2][1]-uv[0][1];d[4]=d[0]*d[3]-d[1]*d[2];
  }uvReady=true;}
  memset(rsNormals,0,sizeof(rsNormals));memset(rsTangents,0,sizeof(rsTangents));
  for(unsigned k=0;k<rsIndexCount;k+=3){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];V3 e=rsPositions[b]-rsPositions[a],g=rsPositions[c]-rsPositions[a],normal=Cross(g,e);
    rsNormals[a]=rsNormals[a]+normal;rsNormals[b]=rsNormals[b]+normal;rsNormals[c]=rsNormals[c]+normal;
    const float* uv=uvDifferences[k/3];float u1=uv[0],v1=uv[1],u2=uv[2],v2=uv[3],det=uv[4];
    if(fabsf(det)>1e-10f){V3 t=(e*v2-g*v1)/det;rsTangents[a]=rsTangents[a]+t;rsTangents[b]=rsTangents[b]+t;rsTangents[c]=rsTangents[c]+t;}}
  memset(paBodyNormals,0,sizeof(paBodyNormals));memset(paBodyGroupNormals,0,sizeof(paBodyGroupNormals));
  for(unsigned k=0;k<paBodyFaceCount;k++){unsigned a=paBodyNormalFaces[k*3],b=paBodyNormalFaces[k*3+1],c=paBodyNormalFaces[k*3+2];V3 normal=Cross(paBodyBefore[c]-paBodyBefore[a],paBodyBefore[b]-paBodyBefore[a]);paBodyNormals[a]=paBodyNormals[a]+normal;paBodyNormals[b]=paBodyNormals[b]+normal;paBodyNormals[c]=paBodyNormals[c]+normal;}
  for(unsigned k=0;k<paBodyNormalCount;k++)paBodyGroupNormals[paBodyNormalGroup[k]]=paBodyGroupNormals[paBodyNormalGroup[k]]+paBodyNormals[k];
  for(unsigned k=0;k<paSeamCount;k++)paBodyGroupNormals[paBodyNormalGroup[paSeamBody[k]]]=paBodyGroupNormals[paBodyNormalGroup[paSeamBody[k]]]+rsNormals[paSeamR14[k]];
  for(unsigned k=0;k<paSeamCount;k++)rsNormals[paSeamR14[k]]=paBodyGroupNormals[paBodyNormalGroup[paSeamBody[k]]];
  static bool normalWrite[rsCount]{},normalWriteReady=false;
  if(!normalWriteReady){for(unsigned k=0;k<cpNormalCount;k++)normalWrite[cpNormalIDs[k]]=true;normalWriteReady=true;}
  for(unsigned i=0;i<rsCount;i++){if(!normalWrite[i]&&!rapheNormalNeeded[i])continue;V3 normal=Unit(rsNormals[i]),tangent=rsTangents[i]-normal*Dot(normal,rsTangents[i]);
    if(Length(tangent)<1e-6f)tangent=Cross(fabsf(normal.z)<.9f?V3{0,0,1}:V3{0,1,0},normal);tangent=Unit(tangent);unsigned char* dst=rsPacked+i*32;
    dst[12]=PackSigned(tangent.x);dst[13]=PackSigned(tangent.y);dst[14]=PackSigned(tangent.z);dst[16]=PackSigned(normal.x);dst[17]=PackSigned(normal.y);dst[18]=PackSigned(normal.z);}
  for(unsigned k=0;k<rsCount;k++)memcpy(rsPacked+k*32,&rsPositions[k],12);
  memcpy(r14Packed,rsPacked,sizeof(r14Packed));
  for(unsigned k=0;k<paBodyNormalCount;k++)if(paBodyNormalWrite[k]){
    V3 normal=Unit(paBodyGroupNormals[paBodyNormalGroup[k]]);if(Length(normal)<.5f)continue;unsigned char* dst=body+paBodyNormalIDs[k]*32;
    V3 tangent{dst[12]/255.f*2.f-1.f,dst[13]/255.f*2.f-1.f,dst[14]/255.f*2.f-1.f};tangent=Unit(tangent-normal*Dot(normal,tangent));
    dst[12]=PackSigned(tangent.x);dst[13]=PackSigned(tangent.y);dst[14]=PackSigned(tangent.z);dst[16]=PackSigned(normal.x);dst[17]=PackSigned(normal.y);dst[18]=PackSigned(normal.z);}
  r14UploadPending=true;
}
