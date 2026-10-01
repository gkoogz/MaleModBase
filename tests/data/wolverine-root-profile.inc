// Exact source functions; see tools/extract_root_profile.py.
static float Smooth01(float value){value=max(0.f,min(1.f,value));return value*value*(3.f-2.f*value);}

static float Smoother01(float value){value=max(0.f,min(1.f,value));return value*value*value*(value*(value*6.f-15.f)+10.f);}

static void SampleRestShaftFrame(float t,V3& center,V3& tangent){
  t=max(0.f,min(1.f,t));float u=t*(shaftRestSampleCount-1);int i=min(shaftRestSampleCount-2,(int)u);float q=u-i;
  center=shaftRestCenters[i]*(1.f-q)+shaftRestCenters[i+1]*q;
  V3 before=shaftRestCenters[max(0,i-1)],after=shaftRestCenters[min(shaftRestSampleCount-1,i+2)];
  tangent=Unit(after-before);
}

static float LogicalShaftOwner(UINT i,float t){
  float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]);
  float ball=min(1.f,phys_scrotum_weight[i]);
  // A vertex is either shaft surface or pouch surface.  Every mixed neck row
  // remains with the independently scaled scrotal system, so the final shaft
  // transport cannot pull that broad hourglass into a narrow diagonal bridge.
  if(shaft<.08f||suspensionWeight[i]>=1.f||t<0.f||t>.86f)return 0.f;
  // Spread radial ownership over the proximal shaft; the old .018 takeover
  // inflated one narrow row into a shelf at large widths. Quintic easing
  // matches value, slope and curvature at both ends of the shared root.
  float rootFollow=preparedPelvicRootFollow[i];
  return rootFollow*(1.f-Smoother01((t-.80f)/.06f))*(1.f-suspensionWeight[i]);
}

static void RegularizeSharedRootProfile(){
  // One angular profile owns the collar and the first free shaft.  Sample the
  // uncontaminated shaft downstream, then carry that same shape back to the
  // pelvis with a slight root flare.  This replaces the old logical stair-step
  // between attachment, scrotal junction and shaft systems.
  const int sectors=24;float radiusSum[sectors]{},weightSum[sectors]{},collarSum[sectors]{},collarWeight[sectors]{};float globalSum=0.f,globalWeight=0.f;
  for(UINT i=0;i<graftCount;i++){
    float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]),ball=min(1.f,phys_scrotum_weight[i]),t=graftRestFlex[i];
    bool stable=t>=.52f&&t<=.72f,collar=t>=.025f&&t<=.16f;
    if(shaft<.62f||ball>.05f||(!stable&&!collar))continue;
    V3 center{},tangent{};SampleRestShaftFrame(t,center,tangent);V3 offset=graftDeformedPositions[i]-center;
    V3 radial=offset-tangent*Dot(offset,tangent);float radius=Length(radial);if(radius<1e-4f)continue;
    V3 lateral={0.f,1.f,0.f};lateral=Unit(lateral-tangent*Dot(lateral,tangent));V3 vertical=Unit(Cross(tangent,lateral));
    float angle=atan2f(Dot(radial,vertical),Dot(radial,lateral));if(angle<0.f)angle+=6.283185307f;
    int sector=min(sectors-1,(int)(angle*(sectors/6.283185307f)));float weight=shaft*shaft*(1.f-ball)*(1.f-ball);
    if(stable){radiusSum[sector]+=radius*weight;weightSum[sector]+=weight;globalSum+=radius*weight;globalWeight+=weight;}
    if(collar){collarSum[sector]+=radius*weight;collarWeight[sector]+=weight;}
  }
  float fallback=globalWeight>1e-5f?globalSum/globalWeight:logicalShaftBodyRadius;
  float sectorRadius[sectors]{};
  for(int sector=0;sector<sectors;sector++)sectorRadius[sector]=weightSum[sector]>1e-5f?radiusSum[sector]/weightSum[sector]:fallback;
  // Circularly fill sparse sectors from their nearest populated neighbors.
  for(int sector=0;sector<sectors;sector++)if(weightSum[sector]<=1e-5f){
    float sum=0.f,weights=0.f;
    for(int distance=1;distance<sectors/2;distance++){
      int left=(sector-distance+sectors)%sectors,right=(sector+distance)%sectors;
      if(weightSum[left]>1e-5f){sum+=sectorRadius[left]/distance;weights+=1.f/distance;}
      if(weightSum[right]>1e-5f){sum+=sectorRadius[right]/distance;weights+=1.f/distance;}
      if(weights>0.f&&distance>=3)break;
    }
    if(weights>0.f)sectorRadius[sector]=sum/weights;
  }
  float growth=Smoother01(PelvisCollarGrowth()/1.5f),rootBoost=.012f+.030f*growth;
  float correctionStrength=.42f+.54f*growth;
  for(UINT i=0;i<graftCount;i++){
    float t=graftRestFlex[i];if(t>.60f)continue;
    float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]),ball=min(1.f,phys_scrotum_weight[i]);
    // The pouch hangs from the solved tube; it never supplies the tube shape.
    // Keep the complete mixed-weight neck on the pouch side of the ownership
    // boundary.  Projecting even its lightest ball-weighted rows back onto the
    // shaft pinched the hourglass and stretched the bridge triangles.
    if(shaft<.08f||ball>=.50f)continue;
    float owner=LogicalShaftOwner(i,t);if(owner<=.0001f)continue;
    V3 center{},tangent{};SampleRestShaftFrame(t,center,tangent);V3 point=graftDeformedPositions[i],offset=point-center;
    V3 radial=offset-tangent*Dot(offset,tangent);float radius=Length(radial);if(radius<1e-4f)continue;
    V3 lateral={0.f,1.f,0.f};lateral=Unit(lateral-tangent*Dot(lateral,tangent));V3 vertical=Unit(Cross(tangent,lateral));
    float angle=atan2f(Dot(radial,vertical),Dot(radial,lateral));if(angle<0.f)angle+=6.283185307f;
    float u=angle*(sectors/6.283185307f),base=floorf(u);int a=((int)base)%sectors,b=(a+1)%sectors;float q=u-base;
    q=Smooth01(q);float reference=sectorRadius[a]*(1.f-q)+sectorRadius[b]*q;
    // A monotone, sector-aware envelope: broadest at the collar, easing to the
    // regular shaft without a local minimum or a compensating downstream lump.
    float collarA=collarWeight[a]>1e-5f?collarSum[a]/collarWeight[a]:reference;
    float collarB=collarWeight[b]>1e-5f?collarSum[b]/collarWeight[b]:reference;
    float collarReference=collarA*(1.f-q)+collarB*q;
    float rootReference=max(reference*(1.f+rootBoost),collarReference*.985f);
    float rootFade=1.f-Smoother01(max(0.f,(t-.055f)/.505f));
    float desired=reference+(rootReference-reference)*rootFade;
    float change=max(-reference*.12f,min(reference*.12f,desired-radius));
    float seamFollow=Smoother01(t/.14f);
    float blend=owner*seamFollow*correctionStrength*(1.f-Smoother01(max(0.f,(t-.50f)/.10f)));
    graftDeformedPositions[i]=point+radial*(change*blend/radius);
  }
}
