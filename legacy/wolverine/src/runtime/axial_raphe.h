#pragma once
// Invisible shaft-owned support. Only the existing skin vertices are moved;
// this constraint has no render geometry, material, seam or draw call.
static const unsigned axialSamples=65;
static V3 axialCenters[axialSamples];
static float axialRadii[axialSamples],axialMoved[r14Count];
static float axialReferenceT,axialCoreRadius;
static void BuildAxialRapheCore(){
  V3 rest{r14Base[355*3],r14Base[355*3+1],r14Base[355*3+2]};
  for(UINT k=r14Offsets[355];k<r14Offsets[356];k++){
    UINT j=r14Sources[k];
    rest=rest+(firmLobeRestSkin[j]-V3{r14Reference[j*3],r14Reference[j*3+1],r14Reference[j*3+2]})*r14Weight[k];
  }
  axialReferenceT=ClosestRestShaftFlex(rest);
  V3 ref{},refAxis{};SampleRestShaftFrame(axialReferenceT,ref,refAxis);
  V3 lateral=Unit(V3{0,1,0}-refAxis*refAxis.y);
  float crestDepth=Dot(rest-ref,Unit(Cross(lateral,refAxis)));
  crestDepth-=.30f*logicalShaftBodyRadius;
  axialCoreRadius=(std::max)(.12f,logicalShaftBodyRadius*.46f);
  V3 root{},axis{};SampleShaftChain(0.f,root,axis);
  for(unsigned k=0;k<axialSamples;k++){
    float t=-.20f+(axialReferenceT+.25f)*float(k)/float(axialSamples-1);
    V3 center{},tangent{};
    if(t<0.f){center=root+axis*(t*constraintRestLength);tangent=axis;}
    else SampleShaftChain(t,center,tangent);
    V3 side=Unit(V3{0,1,0}-tangent*tangent.y);
    axialCenters[k]=center+Unit(Cross(side,tangent))*(crestDepth-axialCoreRadius);
    axialRadii[k]=axialCoreRadius*Smoother01((t+.20f)/.09f)*(1.f-Smoother01((t-(axialReferenceT-.07f))/.12f));
  }
}
static void PreserveBuriedAxialSupport(){
  memset(axialMoved,0,sizeof(axialMoved));
  if(!constraintSolverReady||!shaftRestFrameReady)return;
  BuildAxialRapheCore();
  static V3 displacement[r14Count];memset(displacement,0,sizeof(displacement));
  for(UINT i=0;i<r14NewStart;i++){
    if(rapheSupportMask[i]<=0.f)continue;
    V3 original=r14Positions[i],p=original;
    for(unsigned pass=0;pass<4;pass++){
      float largest=0.f;V3 correction{};
      for(unsigned k=0;k+1<axialSamples;k++){
        V3 segment=axialCenters[k+1]-axialCenters[k];
        float u=(std::max)(0.f,(std::min)(1.f,Dot(p-axialCenters[k],segment)/(std::max)(1.e-8f,Dot(segment,segment))));
        V3 radial=p-(axialCenters[k]+segment*u);float distance=Length(radial);
        float penetration=axialRadii[k]*(1.f-u)+axialRadii[k+1]*u-distance;
        if(penetration>largest&&distance>1.e-6f){largest=penetration;V3 tangent=Unit(segment);V3 side=Unit(V3{0,1,0}-tangent*tangent.y);V3 direction=Unit(Cross(side,tangent));float depth=Dot(radial,direction);float radius=axialRadii[k]*(1.f-u)+axialRadii[k+1]*u;float height=sqrtf(max(0.f,radius*radius-Dot(radial,radial)+depth*depth));correction=direction*(height-depth);}
      }
      if(largest<1.e-5f)break;
      p=p+correction;
    }
    displacement[i]=p-original;
  }
  for(UINT row=0;row<rapheSupportFreeCount;row++){
    V3 shift{};
    for(UINT k=0;k<rapheSupportCoreCount;k++)shift=shift+displacement[rapheSupportCore[k]]*rapheSupportWeights[row*rapheSupportCoreCount+k];
    displacement[rapheSupportFree[row]]=shift;
  }
  for(UINT i=0;i<r14NewStart;i++){
    r14Positions[i]=r14Positions[i]+displacement[i];axialMoved[i]=Length(displacement[i]);undersideBlendMoved[i]+=axialMoved[i];
  }
}






static V3 ComputeAxialMaterialRest(UINT i){
  V3 rest{r14Base[i*3],r14Base[i*3+1],r14Base[i*3+2]};
  for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++){
    UINT j=r14Sources[k];
    rest=rest+(firmLobeRestSkin[j]-V3{r14Reference[j*3],r14Reference[j*3+1],r14Reference[j*3+2]})*r14Weight[k];
  }
  return rest;
}
static V3 AxialMaterialRest(UINT i){
  static unsigned revision[r14Count]{};static V3 points[r14Count];
  if(revision[i]!=geometryRestRevision+1){points[i]=ComputeAxialMaterialRest(i);revision[i]=geometryRestRevision+1;}
  return points[i];
}
static float AxialMaterialFlex(UINT i){
  static unsigned revision[r14Count]{};static float flex[r14Count];
  if(revision[i]!=geometryRestRevision+1){flex[i]=ClosestRestShaftFlex(AxialMaterialRest(i));revision[i]=geometryRestRevision+1;}
  return flex[i];
}
static void PreserveAxialRaphe(){
  PreserveBuriedAxialSupport();
  // Floppy uses the existing bent-shaft constraint. The straight underside
  // continuation applies to the extended Erect and Semi states.
  if(physicsState==2)return;
  if(!constraintSolverReady||!shaftRestFrameReady)return;
  // Continue the full underside contour through the proximal attachment.
  // Its old material profile tapered inward near the pouch even at rest.
  V3 reference=AxialMaterialRest(37),rc{},rt{};
  axialReferenceT=ClosestRestShaftFlex(reference);
  SampleRestShaftFrame(axialReferenceT,rc,rt);
  V3 rv=Unit(Cross(Unit(V3{0,1,0}-rt*rt.y),rt));
  axialCoreRadius=Dot(reference-rc,rv);
  float proximalT=AxialMaterialFlex(356);
  float nextT=AxialMaterialFlex(67);
  static V3 displacement[r14Count];memset(displacement,0,sizeof(displacement));
  for(UINT k=0;k<rapheSupportCoreCount;k++){
    UINT i=rapheSupportCore[k];V3 rest=AxialMaterialRest(i);
    float t=AxialMaterialFlex(i),blend=1.f-Smoother01((t-.42f)/.12f);
    if(blend<=0.f)continue;
    V3 center{},tangent{};SampleShaftChain(t,center,tangent);
    V3 ventral=Unit(Cross(Unit(V3{0,1,0}-tangent*tangent.y),tangent));
    float depth=Dot(r14Positions[i]-center,ventral);
    float orderedT=t;
    if(r14Flex[i]>=r14Flex[356]&&r14Flex[i]<=r14Flex[67]){
      float u=(r14Flex[i]-r14Flex[356])/(r14Flex[67]-r14Flex[356]);
      orderedT=max(t,proximalT+.70f*(nextT-proximalT)*u);
    }
    V3 orderedCenter{},orderedTangent{};SampleShaftChain(orderedT,orderedCenter,orderedTangent);
    displacement[i]=ventral*(max(0.f,axialCoreRadius-depth)*blend)+(orderedCenter-center)*blend;
  }
  for(UINT row=0;row<rapheSupportFreeCount;row++){
    V3 shift{};
    for(UINT k=0;k<rapheSupportCoreCount;k++)shift=shift+displacement[rapheSupportCore[k]]*rapheSupportWeights[row*rapheSupportCoreCount+k];
    displacement[rapheSupportFree[row]]=shift;
  }
  for(UINT i=0;i<r14NewStart;i++){
    r14Positions[i]=r14Positions[i]+displacement[i];axialMoved[i]=Length(displacement[i]);undersideBlendMoved[i]+=axialMoved[i];
  }
}
