// Exact source functions; see tools/extract_rest_frame.py.
static V3 RestShaftDirection(){float a=sliderValues[4]*3.1415926535f/180.f;return {cosf(a),0.f,-sinf(a)};}

static float ClosestRestShaftFlex(V3 point){
  float bestDistance=1e30f,bestT=0.f;
  for(int segment=0;segment<shaftRestSampleCount-1;segment++){
    V3 a=shaftRestCenters[segment],ab=shaftRestCenters[segment+1]-a;float denominator=Dot(ab,ab);
    float q=denominator>1e-8f?max(0.f,min(1.f,Dot(point-a,ab)/denominator)):0.f;
    V3 nearest=a+ab*q;float distance=Dot(point-nearest,point-nearest);
    if(distance<bestDistance){bestDistance=distance;bestT=(segment+q)/(shaftRestSampleCount-1);}
  }
  return bestT;
}

static void BuildShaftRestFrame(){
  const float sigma=.082f,invTwoSigma2=1.f/(2.f*sigma*sigma);
  static float weights[shaftRestSampleCount][graftCount];static bool weightsReady=false;
  if(!weightsReady){
    for(int sample=0;sample<shaftRestSampleCount;sample++)for(UINT i=0;i<graftCount;i++){
      float target=(float)sample/(shaftRestSampleCount-1),shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]),ball=min(1.f,phys_scrotum_weight[i]);
      float q=phys_flex_coordinate[i]-target;
      weights[sample][i]=shaft*shaft*(1.f-ball)*(1.f-ball)*expf(-q*q*invTwoSigma2);
    }
    weightsReady=true;
  }
  for(int sample=0;sample<shaftRestSampleCount;sample++){
    float target=(float)sample/(shaftRestSampleCount-1),sum=0.f;V3 center{};
    for(UINT i=0;i<graftCount;i++){
      float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]);
      float ball=min(1.f,phys_scrotum_weight[i]);
      if(shaft<.20f||suspensionWeight[i]>0.f)continue;
      float w=weights[sample][i];
      center=center+graftDeformedPositions[i]*w;sum+=w;
    }
    if(sum>1e-5f)center=center/sum;
    else {restFrameUsesPreviousLength=true;center=ShaftRoot()+RestShaftDirection()*(constraintRestLength*target);}
    center.y=0.f;shaftRestCenters[sample]=center;
  }
  shaftRestCenters[0]=ShaftRoot();
  // The donor has uneven vertex density around several cross-sections. Smooth
  // its measured centerline before it becomes the radial frame, while keeping
  // the anatomical root exact and the terminal location stable.
  for(int pass=0;pass<6;pass++){
    V3 copy[shaftRestSampleCount];memcpy(copy,shaftRestCenters,sizeof(copy));
    for(int i=1;i<shaftRestSampleCount-1;i++)shaftRestCenters[i]=copy[i]+((copy[i-1]+copy[i+1])*.5f-copy[i])*.34f;
  }
  shaftRestCenters[0]=ShaftRoot();
  // The scrotal donor contaminates the measured proximal centers.  The root
  // and first free shaft must share one centerline; otherwise even matching
  // radii read as a kink when adjacent rings drift vertically.  Re-project the
  // proximal samples onto the authored axis with a C2 release into the stable
  // body.  Full physics already uses a straight rest chain, so retain that
  // exact architecture for moving states.
  if(physicsState==0){
    V3 root=ShaftRoot(),axis=RestShaftDirection();float previous=0.f;
    for(int i=1;i<shaftRestSampleCount;i++){
      float t=(float)i/(shaftRestSampleCount-1),axial=max(previous+.10f,Dot(shaftRestCenters[i]-root,axis));previous=axial;
      V3 line=root+axis*axial;
      float follow=1.f-Smoother01(max(0.f,(t-.07f)/.53f));
      shaftRestCenters[i]=shaftRestCenters[i]*(1.f-follow)+line*follow;
    }
  }else{
    V3 axisEnd=shaftRestCenters[shaftRestSampleCount-1],axisSpan=axisEnd-ShaftRoot();axisSpan.y=0.f;
    if(Length(axisSpan)<8.f){restFrameUsesPreviousLength=true;axisSpan=RestShaftDirection()*constraintRestLength;}
    for(int i=0;i<shaftRestSampleCount;i++)shaftRestCenters[i]=ShaftRoot()+axisSpan*((float)i/(shaftRestSampleCount-1));
  }
  for(UINT i=0;i<graftCount;i++){
    graftRestFlex[i]=ClosestRestShaftFlex(graftDeformedPositions[i]);
    float original=Smoother01(graftRestFlex[i]/.18f);
    preparedPelvicRootFollow[i]=original+(pelvicRootFollow[i]-original)*(preparedPelvicRampBlend*pelvicRootMask[i]);
  }
  // Derive one body radius only from pure shaft skin.  Width and Overall may
  // change this value, while the independently scaled pouch is excluded.
  float radiusSum=0.f,weightSum=0.f;
  for(UINT i=0;i<graftCount;i++){
    float shaft=max(phys_shaft_weight[i],phys_attachment_weight[i]),ball=min(1.f,phys_scrotum_weight[i]),t=graftRestFlex[i];
    if(shaft<.72f||suspensionWeight[i]>0.f||t<.30f||t>.70f)continue;
    V3 center{},tangent{};SampleRestShaftFrame(t,center,tangent);V3 offset=graftDeformedPositions[i]-center;
    V3 radial=offset-tangent*Dot(offset,tangent);float weight=shaft*shaft;
    radiusSum+=Length(radial)*weight;weightSum+=weight;
  }
  logicalShaftBodyRadius=weightSum>1e-5f?radiusSum/weightSum:2.52f*ShaftWidthScale();
  float arc=0.f;for(int i=1;i<shaftRestSampleCount;i++)arc+=Length(shaftRestCenters[i]-shaftRestCenters[i-1]);
  float measured=max(8.f,min(60.f,arc));constraintRestLength=measured;
  shaftRestFrameReady=true;
}

static void SampleRestShaftFrame(float t,V3& center,V3& tangent){
  t=max(0.f,min(1.f,t));float u=t*(shaftRestSampleCount-1);int i=min(shaftRestSampleCount-2,(int)u);float q=u-i;
  center=shaftRestCenters[i]*(1.f-q)+shaftRestCenters[i+1]*q;
  V3 before=shaftRestCenters[max(0,i-1)],after=shaftRestCenters[min(shaftRestSampleCount-1,i+2)];
  tangent=Unit(after-before);
}
