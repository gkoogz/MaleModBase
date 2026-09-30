// Each lower lobe retains its authored ovoid and a bounded pressure transform.
// The solid core preserves volume; the attachment and central skin can yield.
static float firmLobeMask[r14Count];
static void PreserveRigidLobeSurfaces(){
  if(!constraintSolverReady||!eggRestReady)return;
  static unsigned revision=~0u;
  static V3 firmOffsets[r14NewStart][2];static float firmRight[r14NewStart];
  static GeometryFairPass<r14Count> fair;static bool fairReady=false;
  if(revision!=geometryRestRevision){
    float scale=sqrtf(max(.35f,BallShapeScale()));
  for(UINT i=0;i<r14NewStart;i++){
    V3 rest{r14Base[i*3],r14Base[i*3+1],r14Base[i*3+2]};float membership=0.f;
    for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++){
      UINT j=r14Sources[k];float w=r14Weight[k];
      rest=rest+(firmLobeRestSkin[j]-V3{r14Reference[j*3],r14Reference[j*3+1],r14Reference[j*3+2]})*w;
      membership+=w*phys_scrotum_weight[j];
    }
    float lower=Smoother01((81.f-r14Base[i*3+2])/5.f);
    float mask=lower*Smoother01((membership-.55f)/.35f);
    firmLobeMask[i]=mask;if(mask<=0.f)continue;
    firmRight[i]=Smoother01(.5f+r14Base[i*3+1]/1.4f);
    for(int side=0;side<2;side++){
      V3 offset=EggSurface(rest-constraintBallRest[side],eggRadii[side]);
      V3 originalOffset=offset;
      // A fuller shared skin envelope fills the medial hollow without
      // changing either rigid support or moving the lateral lobe surfaces.
      float medial=1.f-Smoother01((fabsf(r14Base[i*3+1])-.20f)/1.40f);
      float pouch=Smoother01((78.f-r14Base[i*3+2])/3.f);
      float fill=.80f*medial*pouch;
      V3 envelope=EggSurface(V3{offset.x,0.f,offset.z},eggRadii[side]);
      offset.x+=(envelope.x-offset.x)*fill;
      offset.z+=(envelope.z-offset.z)*fill;
      // No wraparound scrotal ridge: the shaft owns the axial continuation.
      float forward=(side?16.f:18.f)*.01745329252f,lateral=(side?10.f:-9.f)*.01745329252f;
      V3 up=Unit(V3{sinf(forward),sinf(lateral),cosf(forward)*cosf(lateral)});
      offset=RotateFromTo(offset,{0,0,1},up);
      offset.y=RotateFromTo(originalOffset,{0,0,1},up).y;offset.z+=(side?.05f:-.15f)*scale;
      firmOffsets[i][side]=offset;
    }
  }
    revision=geometryRestRevision;
  }
  if(!fairReady){for(UINT i=0;i<r14NewStart;i++){
    float m=firmLobeMask[i];if(m<=0.f||m>=.99999f)continue;
    fair.Add(i,undersideNeighbors+undersideRows[i],undersideNeighbors+undersideRows[i+1],.45f*4.f*m*(1.f-m));
  }fairReady=true;}
  V3 restAxis[2],liveAxis[2];for(int side=0;side<2;side++){
    restAxis[side]=Unit(constraintBallRest[side]-RestBallAnchor(side));liveAxis[side]=Unit(ballNodes[side]-BallAnchor(side));
  }
  GeometryRotation rotation[2]={{restAxis[0],liveAxis[0]},{restAxis[1],liveAxis[1]}};
  for(UINT i=0;i<r14NewStart;i++){
    float mask=firmLobeMask[i];if(mask<=0.f)continue;float right=firmRight[i];V3 target{};
    for(int side=0;side<2;side++)target=target+(ballNodes[side]+LobePressureOffset(rotation[side].Apply(firmOffsets[i][side]),side))*(side?right:1.f-right);
    r14Positions[i]=r14Positions[i]*(1.f-mask)+target*mask;
  }
  fair.Apply(r14Positions,24);
}
