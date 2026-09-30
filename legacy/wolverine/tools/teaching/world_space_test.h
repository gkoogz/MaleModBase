static float mockWorldScale=.02f;static bool mockUpper=false;static unsigned mockWorldCalls=0;
static bool MockWorldRay(V3 origin,V3 direction,float distance,V3& point,V3& normal){
 ++mockWorldCalls;V3 p=origin/mockWorldScale;float length=distance/mockWorldScale;
 if(p.x<1500||p.x>=1660||p.y<29000||p.y>31000||direction.z>=-.1f)return false;
 float ground=mockUpper&&p.z>=3200?3200.f:3120.f;
 float t=(ground-p.z)/direction.z;if(t<0||t>length)return false;
 point=(p+direction*t)*mockWorldScale;normal={0,0,1};return true;
}
static bool WorldSpaceTest(){
 ResetFluidCollision();tankCameraSceneTick=0;anatomyScene=0;
 D3DVERTEXELEMENT9 baked[]={
 {0,0,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_POSITION,0},
 {0,12,D3DDECLTYPE_UBYTE4,0,D3DDECLUSAGE_TANGENT,0},
 {0,16,D3DDECLTYPE_UBYTE4,0,D3DDECLUSAGE_NORMAL,0},
 {0,20,D3DDECLTYPE_FLOAT2,0,D3DDECLUSAGE_TEXCOORD,0},
 {0,28,D3DDECLTYPE_FLOAT2,0,D3DDECLUSAGE_COLOR,0},D3DDECL_END()};
 CHECK(FluidBspDeclaration(baked,6));baked[4].Offset=24;CHECK(!FluidBspDeclaration(baked,6));baked[4].Offset=28;
 D3DXMATRIX local;D3DXMatrixIdentity(&local);local._41=-1595.30273f;local._42=-29797.9277f;local._43=-3283.53271f;
 V3 camera;CHECK(FluidBspCamera((float*)&local,camera));CHECK(Length(camera-V3{1595.30273f,29797.9277f,3283.53271f})<.001f);
 local._11=.75f;CHECK(!FluidBspCamera((float*)&local,camera));
 for(int frame=0;frame<360;frame++){
  V3 actor{1600.f+frame*.2f,29800.f,3120.f},component{18,0,82};float angle=frame*.03f;
  V3 eye=actor+V3{250*cosf(angle),250*sinf(angle),130};
  D3DXMATRIX actorMatrix,translated,rotation,projection,relativeView;
  D3DXMatrixRotationZ(&actorMatrix,angle*.4f);actorMatrix._41=actor.x;actorMatrix._42=actor.y;actorMatrix._43=actor.z;
  V3 expected=FluidMatrixPoint((float*)&actorMatrix,component);
  local=actorMatrix;local._41-=eye.x;local._42-=eye.y;local._43-=eye.z;
  D3DXMatrixTranslation(&translated,-eye.x,-eye.y,-eye.z);CHECK(FluidBspCamera((float*)&translated,camera));
  D3DXVECTOR3 zero(0,0,0),aim(actor.x-eye.x,actor.y-eye.y,actor.z+60-eye.z),up(0,0,1);
  D3DXMatrixLookAtLH(&rotation,&zero,&aim,&up);D3DXMatrixPerspectiveFovLH(&projection,.85f,16.f/9.f,1,10000);
  D3DXMatrixMultiply(&relativeView,&rotation,&projection);
  memcpy(fluidCollisionLocal,&local,64);memcpy(fluidCollisionView,&relativeView,64);fluidCameraWorld=camera;fluidCollisionWorld=true;
  V3 actual,direction;float clip[16];FluidSimulationTransform(component,{1,0,0},actual,direction,clip);
  CHECK(Length(actual-expected)<.006f);
  V3 fixedWorld{1620,29830,3120};D3DXVECTOR4 input(fixedWorld.x,fixedWorld.y,fixedWorld.z,1),a,b;
  D3DXMATRIX independent;D3DXMatrixMultiply(&independent,&translated,&relativeView);
  D3DXVec4Transform(&a,&input,(D3DXMATRIX*)clip);D3DXVec4Transform(&b,&input,&independent);
  CHECK(fabsf(a.x/a.w-b.x/b.w)<1e-5f&&fabsf(a.y/a.w-b.y/b.w)<1e-5f);
 }
 printf("PASS captured BSP layout/origin, reject nonidentity mesh; 360 moving-camera/moving-actor world projection frames\n");
 fluidPhysicsRayTest=MockWorldRay;
 for(float scale:{.02f,1.f}){
  ResetFluidCollision();mockWorldScale=scale;mockUpper=false;renderFrameSerial++;
  fluidWorldReferences={{{1600,29800,3120},{0,0,1}},{{1620,29800,3120},{0,0,1}}};
  CHECK(FluidVerifyPhysicsScale());CHECK(fabsf(fluidPhysicsScale-scale)<1e-6f);
  V3 p,n;CHECK(FluidPhysicsRay({1600,29800,3130},{0,0,-1},20,p,n));CHECK(Length(p-V3{1600,29800,3120})<.01f);
  fluidGroundCells.clear();auto* lower=FluidGroundCellAt({1600,29800,3130},true);CHECK(lower&&fabsf(lower->point.z-3120)<.01f);
  mockUpper=true;auto* upper=FluidGroundCellAt({1600,29800,3210},true);CHECK(upper&&fabsf(upper->point.z-3200)<.01f);
  lower=FluidGroundCellAt({1600,29800,3130},true);CHECK(lower&&fabsf(lower->point.z-3120)<.01f);mockUpper=false;
  fluidCollisionWorld=true;float best=1.0001f;volumeFluid::FluidImpact hit{};
  CHECK(FluidGroundSweep({1600,29800,3121},{1600,29800,3119},.1f,hit,best));CHECK(fabsf(hit.p.z-3120)<.01f);
  // A cached plane spans this 6-unit cell, but the actual ledge ends at x=1660.
  fluidGroundCells.push_back({276,4966,{1658,29800,3120},{0,0,1},GetTickCount(),97});best=1.0001f;
  CHECK(!FluidGroundSweep({1661,29800,3121},{1661,29800,3119},.1f,hit,best));
 }
 ResetFluidCollision();mockWorldScale=.02f;renderFrameSerial++;
 fluidWorldReferences={{{1600,29800,3120},{0,0,1}},{{1600,29800,3120},{0,0,1}}};
 CHECK(!FluidVerifyPhysicsScale());renderFrameSerial++;CHECK(!FluidVerifyPhysicsScale());
 ResetFluidCollision();fluidPhysicsRayTest=nullptr;
 CHECK(fluidPhysicsScale==0&&fluidWorldReferences.empty()&&fluidWorldCameraFrame==-2);
 printf("PASS 1:50 and 1:1 physics calibration, distinct-reference requirement, stacked floors, exact ledge rejection, scene reset\n");return true;
}
