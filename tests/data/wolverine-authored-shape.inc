// Original source oracle; extraction recipe is in tools/extract_authored_shape.py.
static float Smoother01(float value){value=max(0.f,min(1.f,value));return value*value*value*(value*(value*6.f-15.f)+10.f);}

static V3 ShaftRoot(){return {9.0f,0.f,84.3f};}

static float OverallShapeScale(){return sliderValues[0]/sliderSpecs[0].def;}

static float ShaftWidthScale(){return OverallShapeScale()*(sliderValues[2]/sliderSpecs[2].def);}

static float BallShapeScale(){return OverallShapeScale()*(sliderValues[3]/sliderSpecs[3].def);}

static float PelvisCollarGrowth(){
  // Overall and width both contribute to the actual proximal diameter.  Keep
  // the stock/default body bit-identical, then recruit progressively more of
  // the surrounding pelvis only as that diameter grows.
  float diameter=(sliderValues[0]/sliderSpecs[0].def)*(sliderValues[2]/sliderSpecs[2].def);
  return max(0.f,min(1.5f,diameter-1.f));
}

static void ApplyPelvisCollar(float value[3],float distance,float growth,bool graftSide){
  // Grow the actual support radius with diameter. At normal size a compact
  // fillet is still present; wider shapes progressively recruit the additional
  // pelvis loops generated for this revision.
  float radius=(graftSide?5.0f:6.0f)+(graftSide?3.0f:5.0f)*growth;
  float influence=1.f-Smoother01(distance/radius);
  if(influence<=.0001f)return;
  // Raise the surrounding skin into an annular ramp. Both sides receive the
  // same seam lift; the welded fairing below solves their common tangent.
  float seamLift=preparedPelvicSeamLift;
  float radialScale=1.f+.14f*growth*influence;
  value[0]+=seamLift*influence;
  value[1]*=1.f+preparedPelvicLateralGrowth*influence;
  value[2]=84.3f+(value[2]-84.3f)*radialScale;

  // Maximum-size playtesting exposed a shallow dorsal waist: the accepted
  // shaft crown is wider than the last supported collar row.  Fill only that
  // upper collar sector.  The field starts farther out on the pelvis than on
  // the graft, so the abdomen eases into the weld while the shaft body keeps
  // its existing radius law.  The lower/inter-leg arc receives no supplement.
  float maximum=Smoother01(growth/1.5f);
  // The previous field was too narrow and too weak after unified fairing.  At
  // O100/W100 its final supported row still fell below both the abdomen and
  // shaft crown, leaving the exact saddle seen in the user's L100 floppy
  // capture.  Start the dorsal blend below the weld and recruit a wider body
  // arc, but retain the zero-valued lower sector through `upper`.
  float upper=Smoother01((value[2]-80.80f)/8.20f);
  float fillRadius=(graftSide?6.75f:8.75f)+(graftSide?3.75f:6.0f)*growth;
  float fill=1.f-Smoother01(distance/max(.5f,fillRadius));
  float support=maximum*upper*fill;
  if(support>.0001f){
    // Lift the local minimum primarily in dorsal radius and secondarily in
    // forward projection.  This enlarges the collar/ramp, not the shaft body.
    float upperRadial=1.f+.360f*growth*support;
    value[0]+=.25f*growth*support;
    value[1]*=1.f+.070f*growth*support;
    value[2]=84.3f+(value[2]-84.3f)*upperRadial;
  }
}

static void ApplyShaftPoseAngle(float value[3],UINT i){
  float bw=min(1.f,phys_scrotum_weight[i]);
  float membership=max(phys_shaft_weight[i],phys_attachment_weight[i]);
  float active=membership*(1.f-bw);
  // Use the baked collar ramp before the fully rigid shaft.
  if(bw<.05f&&membership>.02f)active=1.f+(pelvicAngleFollow[i]-1.f)*preparedPelvicRampBlend;
  if(active<=.001f)return;
  float delta=sliderValues[4]*3.1415926535f/180.f*active;
  V3 root=ShaftRoot();
  float c=cosf(delta),s=sinf(delta),x=value[0]-root.x,z=value[2]-root.z;
  value[0]=root.x+c*x+s*z;
  value[2]=root.z-s*x+c*z;
}

static void FlareAttachment(float value[3],UINT i){
  // The duplicated body-side seam is the final ten graft vertices and has no
  // shaft/attachment membership, so it remains exactly welded to the torso.
  // Expand the donor collar most strongly beside that seam, then ease back to
  // the normal shaft over roughly its first third.  Scaling in the two axes
  // perpendicular to the posed shaft creates a true bell flare at every angle.
  float membership=max(phys_shaft_weight[i],phys_attachment_weight[i])*(1.f-min(1.f,phys_scrotum_weight[i]));
  if(membership<=.01f)return;
  float flex=phys_flex_coordinate[i];
  // The source topology jumps directly from attachment flex 0 to the first
  // shaft ring near .278.  Ending the flare at .34 collapsed almost its whole
  // radius change into that one ring, creating a second visible collar.  Carry
  // a C2-continuous bell across several rings so radius and curvature approach
  // the regular shaft without a ridge or constriction.
  float collar=1.f-Smoother01(flex/.70f);
  float influence=Smoother01(min(1.f,membership*2.5f))*collar;
  if(influence<=.001f)return;
  float angle=sliderValues[4]*3.1415926535f/180.f,sa=sinf(angle),ca=cosf(angle);
  V3 root=ShaftRoot();float dx=value[0]-root.x,dz=value[2]-root.z;
  // Match the shaft-local frame used by the centerline solver: X is axial;
  // Y and local Z are the true cross-section.  The former basis scaled X at
  // zero angle and was the source of the longitudinal buttress-like ridges.
  float axial=ca*dx-sa*dz,radial=sa*dx+ca*dz;
  // This is only a shallow precursor for the common section solver below.
  // It must not define an independent collar crown: stacking an attachment
  // bell on top of a separately scaled shaft caused the dorsal-left kink.
  float dorsal=Smoother01(max(0.f,min(1.f,radial/max(.5f,2.52f*ShaftWidthScale()))));
  float collarGrowth=PelvisCollarGrowth();
  float growthExtra=.035f*collarGrowth*(1.f-.82f*dorsal);
  float scale=1.f+(.042f+growthExtra)*influence;
  radial*=scale;value[1]*=scale;
  value[0]=root.x+ca*axial+sa*radial;
  value[2]=root.z-sa*axial+ca*radial;
}

static float HangOffset(){
  // Low values draw the lobes closer; high values lengthen the suspension.
  float u=effectiveHangUI<50.f?(effectiveHangUI-50.f)/49.f:(effectiveHangUI-50.f)/50.f;
  return -u*(u<0.f?1.f:4.5f)*sqrtf(max(.35f,BallShapeScale()));
}

static float SampleOverallWidthVertex(UINT q,float overall,float width){
  int oi0=overall<sliderSpecs[0].def?0:1,oi1=oi0+1,wi0=width<sliderSpecs[2].def?0:1,wi1=wi0+1;
  float omin=oi0==0?sliderSpecs[0].lo:sliderSpecs[0].def,omax=oi1==1?sliderSpecs[0].def:sliderSpecs[0].hi;
  float wmin=wi0==0?sliderSpecs[2].lo:sliderSpecs[2].def,wmax=wi1==1?sliderSpecs[2].def:sliderSpecs[2].hi;
  float ot=(overall-omin)/(omax-omin),wt=(width-wmin)/(wmax-wmin);
  float a=overallWidthTargets[oi0][wi0][q]*(1.f-wt)+overallWidthTargets[oi0][wi1][q]*wt;
  float b=overallWidthTargets[oi1][wi0][q]*(1.f-wt)+overallWidthTargets[oi1][wi1][q]*wt;
  return a*(1.f-ot)+b*ot;
}
static void BuildEarlyAuthoredStage(){
  float collarGrowth=PelvisCollarGrowth();
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
}
