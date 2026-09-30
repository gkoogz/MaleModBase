#pragma once
// Shape authoring is separate from pose deformation. Only effective geometry
// controls invalidate the prepared rest cage; pulses use those same controls.
// Live contacts, suspension, refinement and safety projections still run every frame.
struct PreparedShapeKey {
  float values[7],hang;int state;UINT first;
  bool operator==(const PreparedShapeKey& b) const {
    return memcmp(values,b.values,sizeof(values))==0&&hang==b.hang&&state==b.state&&first==b.first;
  }
};
static bool preparedShapeReady=false;
static unsigned preparedShapeBuilds=0,preparedShapeHits=0;
static void BuildPreparedShape(unsigned char* controlled,UINT graftFirstVertex,V3& tipSum,int& tipCount){
  restFrameUsesPreviousLength=false;
  float collarGrowth=PelvisCollarGrowth();
  // Recruit extra pelvis only beyond the neutral diameter; retain its exact
  // original shape below that threshold and ease into the large-size ramp.
  preparedPelvicRampBlend=Smoother01((collarGrowth-.15f)/1.0f);
  preparedPelvicSeamLift=.46f+(.75f+.10f*preparedPelvicRampBlend)*collarGrowth;
  preparedPelvicLateralGrowth=(.14f+.41f*preparedPelvicRampBlend)*collarGrowth;
  for(UINT i=0;i<pelvisControlCount;i++){
    float value[3]={pelvisControlBasePositions[i*3],pelvisControlBasePositions[i*3+1],pelvisControlBasePositions[i*3+2]};
    ApplyPelvisCollar(value,pelvisControlDistances[i],collarGrowth,false);
    unsigned char* bodyVertex=controlled+(pelvisControlIndices[i]-pelvisControlFirstVertex)*graftStride;
    memcpy(bodyVertex,value,12);
  }
  tipSum={};tipCount=0;
  for(UINT i=0;i<graftCount;i++){
    float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]),ball=min(1.f,phys_scrotum_weight[i]);
    // Lobe cores scale independently, but the broad upper neck must open with
    // the shaft it hangs from.  The former early pouch takeover stranded the
    // mixed rows at neutral width during large shaft dilation and stretched
    // their triangles into a narrow fan.  This later C2 handoff preserves the
    // independent sack while recruiting its throat into the supporting tube.
    float pouchOwner=Smoother01((ball-.18f)/.60f)*Smoother01((ball-shaft+.18f)/.70f);
    float value[3];for(UINT axis=0;axis<3;axis++){
      UINT q=i*3+axis;
      float shaftValue=SampleOverallWidthVertex(q,sliderValues[0],sliderValues[2]);
      float pouchValue=SampleOverallWidthVertex(q,sliderValues[0],neutralShape[2]);
      value[axis]=shaftValue*(1.f-pouchOwner)+pouchValue*pouchOwner;
      for(int s=1;s<7;s++){
        if(s==2||s==4)continue;const auto& spec=sliderSpecs[s];float v=sliderValues[s],delta=0.f;
        if(v<spec.def)delta=(spec.low[q]-morph_base[q])*(spec.def-v)/(spec.def-spec.lo);
        else if(v>spec.def)delta=(spec.high[q]-morph_base[q])*(v-spec.def)/(spec.hi-spec.def);
        // Scrotum scale is pouch-only.  It cannot alter any shaft-owned ring.
        value[axis]+=delta*(s==3?pouchOwner:1.f);
      }
    }
    ApplyPelvisCollar(value,graftCollarDistances[i],collarGrowth,true);
    ApplyShaftPoseAngle(value,i);
    FlareAttachment(value,i);
    graftDeformedPositions[i]=V3{value[0],value[1],value[2]};
  }
  FairUnifiedCollar(controlled,graftFirstVertex,collarGrowth);
  FairRetopologyBands();
  BuildShaftRestFrame();
  RegularizeSharedRootProfile();
  // Refit after the profile correction so capture and physics use the
  // corrected common centerline rather than the donor's scrotal-biased frame.
  // A second radial projection would over-constrain the irregular donor
  // tessellation and needlessly worsen its least-regular triangles.
  BuildShaftRestFrame();
  SculptConvergentVentralRaphe();
  // Fit the pelvic cuff in the rest shape, once before skin is transported
  // by the live chain. Refitting its radius from swinging skin made the
  // body attachment repeatedly expand and contract during otherwise smooth motion.
  FinishPelvicRamp(controlled,graftFirstVertex);
  // Cache the corrected authored cross-sections and transport them through
  // physics as a single shaft.  The scrotum remains a separate hanging system.
  ConstructLogicalShaftSurface(false);
  // Author the seam with the rest surface as well. Letting a compressed
  // moving pouch change its global relief limit also pulsed the fixed pelvis.
  // The requested ridge follows the shaft axis through the upper pouch.
  // Do not trace an external seam around the scrotal/perineal centerline.
  for(UINT i=0;i<graftCount;i++)graftDeformedPositions[i].z+=HangOffset()*suspensionWeight[i];
  for(UINT i=0;i<graftCount;i++){
    V3 value=graftDeformedPositions[i];
    if(phys_flex_coordinate[i]>.98f&&phys_shaft_weight[i]>.5f){tipSum=tipSum+value;tipCount++;}
  }
  FitEggRestShapes();
  // Preserve the established lobe supports, then broaden only their shared
  // skin connection. Cache the wider shaft-side blend for live transport.
  BroadenScrotalNeck();
  memcpy(firmLobeRestSkin,graftDeformedPositions,sizeof(firmLobeRestSkin));
  CaptureLogicalShaftSurface();
  ++geometryRestRevision;
}
static void PrepareShape(unsigned char* controlled,UINT graftFirstVertex,V3& tipSum,int& tipCount){PerfScope perf(4);
  static PreparedShapeKey key{};
  static V3 cage[graftCount],centers[shaftRestSampleCount],ballRest[2],radii[2],savedTip;
  static float radius,length,inputLength;static int savedCount;
  // Only cache body positions actually owned by rest preparation. Bone palettes,
  // original lighting bytes and unrelated game vertices stay with the game buffer.
  constexpr UINT memberCount=sizeof(collarFairMembers)/sizeof(collarFairMembers[0]);
  static UINT bodyIDs[pelvisControlCount+memberCount],bodyCount=0;
  static V3 bodyPositions[pelvisControlCount+memberCount];
  PreparedShapeKey next{};memcpy(next.values,sliderValues,sizeof(next.values));
  next.hang=effectiveHangUI;next.state=physicsState;next.first=graftFirstVertex;
  // Extreme folded/short rest frames can use the prior length as a fallback.
  // Preserve that dependency instead of treating those shapes as pure morphs.
  if(!preparedShapeReady||!(next==key)||!shaftRestFrameReady||!eggRestReady||(restFrameUsesPreviousLength&&inputLength!=constraintRestLength)){
    inputLength=constraintRestLength;
    BuildPreparedShape(controlled,graftFirstVertex,tipSum,tipCount);
    bodyCount=0;
    for(UINT i=0;i<pelvisControlCount;i++)bodyIDs[bodyCount++]=pelvisControlIndices[i];
    for(UINT i=0;i<memberCount;i++)if(collarFairMembers[i]<graftFirstVertex)bodyIDs[bodyCount++]=collarFairMembers[i];
    std::sort(bodyIDs,bodyIDs+bodyCount);bodyCount=UINT(std::unique(bodyIDs,bodyIDs+bodyCount)-bodyIDs);
    for(UINT i=0;i<bodyCount;i++)memcpy(&bodyPositions[i],controlled+(bodyIDs[i]-pelvisControlFirstVertex)*graftStride,12);
    memcpy(cage,graftDeformedPositions,sizeof(cage));memcpy(centers,shaftRestCenters,sizeof(centers));
    memcpy(ballRest,constraintBallRest,sizeof(ballRest));memcpy(radii,eggRadii,sizeof(radii));
    radius=logicalShaftBodyRadius;length=constraintRestLength;savedTip=tipSum;savedCount=tipCount;
    key=next;preparedShapeReady=true;++preparedShapeBuilds;
  }else{
    // The integrator temporarily interpolates rest inputs, and the end of
    // ApplyShape blends length. Restore the exact authoring outputs each time.
    memcpy(graftDeformedPositions,cage,sizeof(cage));memcpy(shaftRestCenters,centers,sizeof(centers));
    memcpy(constraintBallRest,ballRest,sizeof(ballRest));memcpy(eggRadii,radii,sizeof(radii));
    logicalShaftBodyRadius=radius;constraintRestLength=length;tipSum=savedTip;tipCount=savedCount;
    for(UINT i=0;i<bodyCount;i++)memcpy(controlled+(bodyIDs[i]-pelvisControlFirstVertex)*graftStride,&bodyPositions[i],12);
    ++preparedShapeHits;
  }
}
