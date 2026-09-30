#ifdef CLINICAL_BASELINE
#include "C:/Users/Administrator/Documents/Codex/2026-09-28/i-am-an-educator-working-as-2/work/release-1.9/src/runtime/d3d9_proxy.cpp"
#else
#include "../../src/runtime/d3d9_proxy.cpp"
#endif
#include "../r14/cpu_buffer.h"
#include <fstream>
#include <map>
#include <tuple>
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);return false;}}while(0)
struct TeachingVertex {V3 p;D3DCOLOR color;};
static void InitMesh(){
 graftBuffer=new CpuVertexBuffer(50915*32);graftOffset=47050*32;void* raw=nullptr;graftBuffer->Lock(0,0,&raw,0);memset(raw,0,50915*32);
 for(UINT i=0;i<collarNormalTriangleCount*3;i++)memcpy((char*)raw+collarNormalTriangleIndices[i]*32,collarNormalTriangleBasePositions+i*3,12);
 for(UINT i=0;i<pelvisControlCount;i++)memcpy((char*)raw+pelvisControlIndices[i]*32,pelvisControlBasePositions+i*3,12);
 for(UINT i=0;i<graftCount;i++)memcpy((char*)raw+(47050+i)*32,morph_base+i*3,12);
 graftBuffer->Unlock();shapeDirty=true;ApplyShape();
}
static bool PulseTests(){
 ResetStudyControls();throbMode=0;teachingTimeline.Start();teachingTimeline.time=7;
 ApplyControlMapping();CHECK(fabsf(sliderValues[4]+37.f)<.001f&&effectiveShapeUI[1]==56&&effectiveShapeUI[2]==62&&effectiveGlansUI==54);
 CHECK(hangUI==50&&effectiveHangUI==15);
 CHECK(sliderUI[1]==50&&sliderUI[2]==50&&sliderUI[4]==50&&glansUI==50);
 for(float angle:{1.f,4.f,10.f,50.f,85.f,100.f}){
  sliderUI[4]=angle;ApplyControlMapping();CHECK(effectiveShapeUI[4]>=1&&effectiveShapeUI[4]<=100);
  if(angle<=4||angle>=85)CHECK(effectiveShapeUI[2]==sliderUI[2]);
 }
 for(float time:{6.5f,7.f,7.13f,8.f,8.5f,10.f,11.5f,11.63f,12.05f}){
  teachingTimeline.time=time;ApplyControlMapping();
  CHECK(sliderValues[4]<=-24.99f&&sliderValues[4]>=-37.01f);
 }
 for(float hang:{1.f,20.f,50.f,100.f}){
  hangUI=hang;teachingTimeline.time=7;ApplyControlMapping();CHECK(effectiveHangUI>=1&&effectiveHangUI<=hang&&hangUI==hang);
  for(int p=4;p<8;p++){
   teachingTimeline.time=teaching::peaks[p]+.36f;ApplyControlMapping();CHECK(fabsf(effectiveHangUI-min(100.f,hang+15.f))<.001f);
   teachingTimeline.time=teaching::peaks[p]+.9f;ApplyControlMapping();CHECK(fabsf(effectiveHangUI-min(100.f,hang+15.f))<.001f||p==7);
  }
 }
 CancelTeaching();ApplyControlMapping();CHECK(effectiveShapeUI[2]==sliderUI[2]&&effectiveShapeUI[4]==sliderUI[4]);
 CHECK(effectiveHangUI==hangUI);
 printf("PASS classroom pulse amplitude, stored controls, low/high-angle fold guards, cancellation\n");return true;
}
static bool IntegrationTests(){
 for(int state=0;state<3;state++){
  ResetStudyControls();physicsState=state;throbMode=2;throbSizePulse=.4f;throbTwitchPulse=.3f;
  sliderUI[0]=sliderUI[1]=100;ApplyControlMapping();InitMesh();
  float before[7];memcpy(before,sliderUI,sizeof(before));float beforePhysics[8];memcpy(beforePhysics,physUI,sizeof(beforePhysics));
  CHECK(teachingFluid.Prepare());teachingTimeline.Start();float minMode=10,minSide=0,maxSide=0;float identity[12]={1,0,0,0,0,1,0,0,0,0,1,0};
  V3 oldTip{},dir{};float worst=0,peakHeight[4]{},restHeight[4]{};
  for(int i=0;i<601;i++){
   teachingTimeline.Advance(1.f/30);ApplyControlMapping();UpdateConstraintSolver(1.f/30,0,0);ApplyShape();
   V3 tip;CHECK(TeachingEmitter(identity,tip,dir));if(i)worst=max(worst,Length(tip-oldTip));oldTip=tip;
   if(teachingTimeline.time>6.7&&teachingTimeline.time<12.5){minSide=min(minSide,dir.y);maxSide=max(maxSide,dir.y);}
   if(i>210&&i<390)minMode=min(minMode,shaftMode);
   for(int p=0;p<4;p++){
    float t=(float)teachingTimeline.time;
    if(fabsf(t-teaching::peaks[p+4])<.015f)peakHeight[p]=(ballNodes[0].z+ballNodes[1].z)*.5f;
    if(fabsf(t-teaching::peaks[p+4]-.8f)<.015f)restHeight[p]=(ballNodes[0].z+ballNodes[1].z)*.5f;
   }
   for(UINT v=0;v<r14Count;v++)CHECK(std::isfinite(r14Positions[v].x)&&std::isfinite(r14Positions[v].y)&&std::isfinite(r14Positions[v].z));
  }
  CHECK(!teachingTimeline.active&&minMode<.03f&&physicsState==state&&throbMode==2);
  printf("Actual mesh outlet lateral direction state=%d range %.6f..%.6f\n",state,minSide,maxSide);CHECK(minSide<-.005f&&maxSide>.005f);
  CHECK(!memcmp(before,sliderUI,sizeof(before))&&!memcmp(beforePhysics,physUI,sizeof(beforePhysics)));
  for(int p=0;p<4;p++){
   printf("Suspension state=%d pulse=%d peak/rest height %.4f/%.4f, drop %.4f\n",state,p+1,peakHeight[p],restHeight[p],peakHeight[p]-restHeight[p]);
   CHECK(peakHeight[p]-restHeight[p]>.05f);
  }
  teachingTimeline.Start();AdjustStudyControl(1,1,1);CHECK(!teachingTimeline.active);teachingTimeline.Start();ResetStudyControls();CHECK(!teachingTimeline.active);
  printf("PASS 20s integrated state=%d: finite surface, firmness reached %.6f, saved preset preserved, cancel/reset; max tip step %.3f\n",state,minMode,worst);
  graftBuffer->Release();graftBuffer=nullptr;
 }
 return true;
}
static bool IdleAnimationTest(){
 ResetStudyControls();physicsState=0;sliderUI[0]=100;sliderUI[1]=95;sliderUI[3]=72;glansUI=29;throbMode=3;ApplyControlMapping();InitMesh();float maximumStep=0,minimumGap=1e9f;
 for(int frame=0;frame<420;frame++){
  V3 before[2]={ballNodes[0],ballNodes[1]};float dt=1.f/30;
  throbSizeTime=fmodf(throbSizeTime+dt,3.f);throbTwitchTime=AdvanceTwitchClock(throbTwitchTime,dt);
  throbSizePulse=ThrobEnvelope(throbSizeTime,.20f,1.05f,false);throbTwitchPulse=ThrobEnvelope(throbTwitchTime,1.15f,4.6f,true);throbAngleSizePulse=ThrobEnvelope(throbTwitchTime,1.15f,1.05f,true);
  ApplyControlMapping();UpdateConstraintSolver(dt,0,0);ApplyShape();
  for(int s=0;s<2;s++){CHECK(std::isfinite(ballNodes[s].x)&&std::isfinite(ballNodes[s].y)&&std::isfinite(ballNodes[s].z));float step=Length(ballNodes[s]-before[s]);if(step>2)printf("IDLE frame=%d side=%d step=%.5f throb=%.3f/%.3f\n",frame,s,step,throbSizePulse,throbTwitchPulse);maximumStep=max(maximumStep,step);}minimumGap=min(minimumGap,cpLastGap);
  for(UINT i=0;i<rsCount;i++)CHECK(std::isfinite(rsPositions[i].x)&&std::isfinite(rsPositions[i].y)&&std::isfinite(rsPositions[i].z));
 }
 printf("IDLE intense animation: max step %.5f, min pair gap %.5f; intentional animation retained\n",maximumStep,minimumGap);// INTENSE is a driven size/pose animation: report its peak step rather than treating it as a resting test.
 CHECK(minimumGap>-.05f);graftBuffer->Release();graftBuffer=nullptr;return true;
}
static LRESULT CALLBACK Proc(HWND h,UINT m,WPARAM w,LPARAM l){return DefWindowProc(h,m,w,l);}
static bool RestTest(int scenario){
 ResetStudyControls();physicsState=scenario==2?2:0;
 if(scenario!=2){sliderUI[0]=100;sliderUI[1]=95;sliderUI[2]=50;sliderUI[3]=scenario==1?100:72;glansUI=29;}
 ApplyControlMapping();InitMesh();
 float maxSpeed=0,maxSpin=0,minGap=1e9f,maxForce=0;V3 lo[2]={{1e9f,1e9f,1e9f},{1e9f,1e9f,1e9f}},hi[2]={{-1e9f,-1e9f,-1e9f},{-1e9f,-1e9f,-1e9f}};
 for(int frame=0;frame<1200;frame++){
  UpdateConstraintSolver(1.f/60,0,0);
  for(int s=0;s<2;s++){CHECK(std::isfinite(ballNodes[s].x)&&std::isfinite(ballNodes[s].y)&&std::isfinite(ballNodes[s].z));if(frame>=900){maxSpeed=max(maxSpeed,Length(pdVelocity[pdBody0+s]));maxSpin=max(maxSpin,Length(cpOmega[s]));V3 p=ballNodes[s];lo[s]={min(lo[s].x,p.x),min(lo[s].y,p.y),min(lo[s].z,p.z)};hi[s]={max(hi[s].x,p.x),max(hi[s].y,p.y),max(hi[s].z,p.z)};}}
  if(frame>=900){minGap=min(minGap,cpLastGap);maxForce=max(maxForce,pdPairForce);}
 }
 printf("REST scenario=%d speed=%.6f spin=%.6f drift=%.6f,%.6f pairGap=%.6f pairForce=%.6f centresY=%.4f,%.4f radiiY=%.4f,%.4f\n",scenario,maxSpeed,maxSpin,Length(hi[0]-lo[0]),Length(hi[1]-lo[1]),minGap,maxForce,ballNodes[0].y,ballNodes[1].y,CPRadii(0).y,CPRadii(1).y);
 CHECK(maxSpeed<.05f&&maxSpin<.01f&&Length(hi[0]-lo[0])<.01f&&Length(hi[1]-lo[1])<.01f&&minGap>.1f&&maxForce<1.f);
 for(int s=0;s<2;s++){printf("Rest lateral lean %d = %.4f\n",s,cpBasis[s][2].y);CHECK(fabsf(cpBasis[s][2].y)<.15f);}
 V3 before=ballNodes[0];for(int frame=0;frame<15;frame++)UpdateConstraintSolver(1.f/60,0,1);CHECK(Length(ballNodes[0]-before)>.01f);
 for(int s=0;s<2;s++)printf("body%d rest=(%.3f %.3f %.3f) position=(%.3f %.3f %.3f) target=(%.3f %.3f %.3f)\n",s,constraintBallRest[s].x,constraintBallRest[s].y,constraintBallRest[s].z,ballNodes[s].x,ballNodes[s].y,ballNodes[s].z,PDMaterialTarget(s).x,PDMaterialTarget(s).y,PDMaterialTarget(s).z);
 graftBuffer->Release();graftBuffer=nullptr;return true;
}
static bool surfaceBenchmark=false;
static bool cameraMotionTest=false;
static bool spaceDiagnosticTest=false;
static bool worldOriginGpuTest=false;
static bool RenderTest(bool late=false,bool closeup=false,bool pool=false,bool first=false,bool resting=false,bool passive=false){
 if(pool){volumeFluid::config.catchPlane=true;volumeFluid::config.catchDepth=8;} ResetStudyControls();physicsState=0;sliderUI[4]=50;if(resting){sliderUI[0]=100;sliderUI[1]=95;sliderUI[3]=72;glansUI=29;}else if(!passive){teachingTimeline.Start();teachingTimeline.time=7;}ApplyControlMapping();InitMesh();
 if(resting){for(int i=0;i<1200;i++)UpdateConstraintSolver(1.f/60,0,0);shapeDirty=true;ApplyShape();}
 WNDCLASSA wc{};wc.lpfnWndProc=Proc;wc.hInstance=GetModuleHandle(nullptr);wc.lpszClassName="TeachingFluidTest";RegisterClassA(&wc);
 HWND window=CreateWindowA(wc.lpszClassName,"Teaching fluid offline test",0,0,0,1000,750,nullptr,nullptr,wc.hInstance,nullptr);
 LoadReal();CHECK(realCreate9);IDirect3D9* d9=realCreate9(D3D_SDK_VERSION);CHECK(d9);
 D3DPRESENT_PARAMETERS pp{};pp.Windowed=TRUE;pp.hDeviceWindow=window;pp.SwapEffect=D3DSWAPEFFECT_DISCARD;pp.BackBufferWidth=surfaceBenchmark?1920:1000;pp.BackBufferHeight=surfaceBenchmark?1080:750;pp.BackBufferFormat=D3DFMT_A8R8G8B8;pp.EnableAutoDepthStencil=TRUE;pp.AutoDepthStencilFormat=D3DFMT_D24S8;
 IDirect3DDevice9* d=nullptr;CHECK(SUCCEEDED(d9->CreateDevice(0,D3DDEVTYPE_HAL,window,D3DCREATE_HARDWARE_VERTEXPROCESSING,&pp,&d)));
 // Use the real game's captured shader to verify reflection and matrix convention.
 std::ifstream in("C:/Games/X-Men Origins Wolverine/Binaries/R14Capture_20260920_023631/draw_000_vs.bin",std::ios::binary);
 std::vector<char> bytes((std::istreambuf_iterator<char>(in)),{});CHECK(!bytes.empty());
 IDirect3DVertexShader9* gameVS=nullptr;CHECK(SUCCEEDED(d->CreateVertexShader((DWORD*)bytes.data(),&gameVS)));d->SetVertexShader(gameVS);
 ShaderLayout* layout=GetShaderLayout(d);CHECK(layout&&layout->valid&&layout->viewValid);
 if(worldOriginGpuTest){
  ResetFluidCollision();tankCameraSceneTick=0;anatomyScene=0;++renderFrameSerial;
  const char* shader="float4x4 LocalToWorld:register(c5);float4x4 ViewProjectionMatrix:register(c0);float4 main(float4 p:POSITION):POSITION{return mul(mul(p,LocalToWorld),ViewProjectionMatrix);}";
  ID3DXBuffer* code=nullptr;CHECK(SUCCEEDED(D3DXCompileShader(shader,(UINT)strlen(shader),nullptr,nullptr,"main","vs_3_0",0,&code,nullptr,nullptr)));
  IDirect3DVertexShader9* bspShader=nullptr;CHECK(SUCCEEDED(d->CreateVertexShader((DWORD*)code->GetBufferPointer(),&bspShader)));code->Release();d->SetVertexShader(bspShader);
  D3DVERTEXELEMENT9 elements[]={{0,0,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_POSITION,0},{0,12,D3DDECLTYPE_UBYTE4,0,D3DDECLUSAGE_TANGENT,0},{0,16,D3DDECLTYPE_UBYTE4,0,D3DDECLUSAGE_NORMAL,0},{0,20,D3DDECLTYPE_FLOAT2,0,D3DDECLUSAGE_TEXCOORD,0},{0,28,D3DDECLTYPE_FLOAT2,0,D3DDECLUSAGE_COLOR,0},D3DDECL_END()};
  IDirect3DVertexDeclaration9* declaration=nullptr;CHECK(SUCCEEDED(d->CreateVertexDeclaration(elements,&declaration)));d->SetVertexDeclaration(declaration);
  IDirect3DVertexBuffer9* vertices=nullptr;IDirect3DIndexBuffer9* indices=nullptr;
  CHECK(SUCCEEDED(d->CreateVertexBuffer(6*36,0,0,D3DPOOL_MANAGED,&vertices,nullptr)));CHECK(SUCCEEDED(d->CreateIndexBuffer(6*2,0,D3DFMT_INDEX16,D3DPOOL_MANAGED,&indices,nullptr)));
  V3 points[6]={{1590,29800,3120},{1600,29800,3120},{1590,29810,3120},{1620,29800,3120},{1630,29800,3120},{1620,29810,3120}};
  void* raw=nullptr;CHECK(SUCCEEDED(vertices->Lock(0,0,&raw,0)));memset(raw,0,216);for(int i=0;i<6;i++)memcpy((char*)raw+i*36,&points[i],12);vertices->Unlock();
  CHECK(SUCCEEDED(indices->Lock(0,0,&raw,0)));for(unsigned short i=0;i<6;i++)((unsigned short*)raw)[i]=i;indices->Unlock();d->SetStreamSource(0,vertices,0,36);d->SetIndices(indices);
  D3DXMATRIX translation,projection;D3DXMatrixTranslation(&translation,-1595,-29798,-3284);D3DXMatrixPerspectiveFovLH(&projection,1,1,1,10000);d->SetVertexShaderConstantF(5,(float*)&translation,4);d->SetVertexShaderConstantF(0,(float*)&projection,4);
  CaptureFluidWorldGeometry(d,D3DPT_TRIANGLELIST,0,0,6,0,2,vertices,0,36);
  CHECK(fluidWorldCameraFrame==renderFrameSerial&&Length(fluidCameraWorld-V3{1595,29798,3284})<.001f);CHECK(fluidWorldReferences.size()==2);
  d->SetVertexShader(gameVS);CHECK(!layout->cameraValid);CaptureFluidCamera(d);CHECK(Length(fluidCameraWorld-V3{1595,29798,3284})<.001f);
  ++renderFrameSerial;CaptureFluidCamera(d);CHECK(fluidWorldCameraFrame!=renderFrameSerial);
  ResetFluidCollision();d->SetStreamSource(0,nullptr,0,0);d->SetIndices(nullptr);d->SetVertexDeclaration(nullptr);d->SetVertexShader(nullptr);
  vertices->Release();indices->Release();declaration->Release();bspShader->Release();gameVS->Release();d->Release();d9->Release();DestroyWindow(window);
  printf("PASS D3D BSP origin and calibration-geometry capture; generic zero CameraPosition cannot overwrite or refresh origin\n");return true;
 }
 float bone[12]={1,0,0,0,0,1,0,0,0,0,1,0};float local[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
 float view[16]={1.f/130,0,0,0,0,0,1.f/200,0,0,1.f/97.5f,0,0,-.55f,-.5f,.5f,1};
 d->SetVertexShaderConstantF(layout->boneRegister,bone,3);d->SetVertexShaderConstantF(layout->localRegister,local,4);d->SetVertexShaderConstantF(layout->viewRegister,view,4);
 if(spaceDiagnosticTest){
  float before[1024],after[1024];CHECK(SUCCEEDED(d->GetVertexShaderConstantF(0,before,256)));
  for(unsigned i=0;i<100;i++)FluidSpaceDiagnostics(d,249804,4472);
  CHECK(fluidSpaceRecords==1&&fluidSpaceSampleCount==1&&fluidSpaceFile);
  CHECK(SUCCEEDED(d->GetVertexShaderConstantF(0,after,256)));CHECK(!memcmp(before,after,sizeof(before)));
  ReleaseFluidSpaceDiagnostics();CHECK(!fluidSpaceFile&&fluidSpaceSampleCount==0);
  gameVS->Release();d->Release();d9->Release();DestroyWindow(window);
  printf("PASS diagnostic samples captured game shader once across 100 draws; constant bank unchanged; resources released\n");return true;
 }
 float l[16],v[16],b[12];CHECK(TeachingMatrices(d,l,v,b));CHECK(!memcmp(local,l,sizeof(l))&&!memcmp(view,v,sizeof(v))&&!memcmp(bone,b,sizeof(b)));
 V3 tip,dir;CHECK(TeachingEmitter(bone,tip,dir));
 CHECK(Dot(dir,Unit(TeachingRing(r14RingCount-6)-TeachingRing(r14CrownRing)))>.8f);
 // Validate rotated/translated tip transport independently of shader rendering.
 float moved[12]={0,-1,0,12,1,0,0,25,0,0,1,-4};V3 mt,md;CHECK(TeachingEmitter(moved,mt,md));
 CHECK(Length(mt-V3{12-tip.y,25+tip.x,tip.z-4})<.001f&&Length(md-V3{-dir.y,dir.x,dir.z})<.001f);
 CHECK(EnsureTeachingShaders(d));
 IDirect3DVertexBuffer9* source=nullptr;d->CreateVertexBuffer(64,0,D3DFVF_XYZ|D3DFVF_DIFFUSE,D3DPOOL_MANAGED,&source,nullptr);CHECK(source);
 d->SetFVF(D3DFVF_XYZ|D3DFVF_DIFFUSE);d->SetStreamSource(0,source,0,16);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_CW);d->SetRenderState(D3DRS_ZWRITEENABLE,TRUE);d->SetRenderState(D3DRS_COLORWRITEENABLE,15);
 teaching::Fluid passivePreview;
 if(passive){CHECK(LoadVolumeConfig());CHECK(passivePreview.BeginPassive(tip));CHECK(passivePreview.TriggerPassiveClear());for(int i=0;i<90;i++)passivePreview.Advance(1.f/60,tip,dir,{});CHECK(!passivePreview.surface.empty());}
 if(!resting&&!passive){teachingTimeline.Start();teachingFluid.Begin(tip);teachingFluidTime=0;
 for(int frame=0;frame<=(first?444:late?954:870);frame++){
  teachingTimeline.Advance(1.f/120);float dt=(float)(teachingTimeline.time-teachingFluidTime);teachingFluidTime=teachingTimeline.time;
  teachingFluid.Advance(dt,tip,dir,{0,0,-98});
 }
 if(!teachingFluid.ready)printf("GPU error %s\n",teachingFluid.error.c_str()); CHECK(teachingFluid.Live()>0);}
 if(closeup&&!pool){
  const auto& preview=passive?passivePreview.surface:teachingFluid.surface;
  V3 lo{1e9f,1e9f,1e9f},hi{-1e9f,-1e9f,-1e9f};
  for(const auto& particle:preview)if(particle.live>0){V3 p=particle.p;lo.x=min(lo.x,p.x);lo.z=min(lo.z,p.z);hi.x=max(hi.x,p.x);hi.z=max(hi.z,p.z);}
  float half=max(first?1.5f:8.f,max((hi.x-lo.x)*.65f,(hi.z-lo.z)*.9f));
  view[0]=1.f/half;view[9]=1.f/(half*.75f);view[12]=-(lo.x+hi.x)*.5f/half;view[13]=-(lo.z+hi.z)*.5f/(half*.75f);
  d->SetVertexShaderConstantF(layout->viewRegister,view,4);
 }
 if(resting){D3DXMATRIX camera,projection,combined;D3DXVECTOR3 eye(95,8,68),aim(16,0,68),up(0,0,1);D3DXMatrixLookAtLH(&camera,&eye,&aim,&up);D3DXMatrixOrthoLH(&projection,60,45,1,200);D3DXMatrixMultiply(&combined,&camera,&projection);memcpy(view,&combined,64);}
 if(pool){ D3DXMATRIX camera,projection,combined; D3DXVECTOR3 eye(tip.x+40,tip.y-65,tip.z+35),aim(tip.x+20,tip.y,tip.z-8),up(0,0,1); D3DXMatrixLookAtLH(&camera,&eye,&aim,&up); D3DXMatrixPerspectiveFovLH(&projection,.65f,4.f/3,1,600); D3DXMatrixMultiply(&combined,&camera,&projection); memcpy(view,&combined,64); }
d->Clear(0,nullptr,D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER,D3DCOLOR_XRGB(26,36,46),1,0);CHECK(SUCCEEDED(d->BeginScene()));
 // Render the actual 1.3 refined anatomy in a neutral diagram material.
 std::vector<TeachingVertex> anatomy;anatomy.reserve(rsIndexCount);
 for(UINT i=0;i<rsIndexCount;i++){UINT id=rsIndices[i];float light=.4f+.6f*fabsf(Unit(rsNormals[id]).x);int c=(int)(light*180);anatomy.push_back({rsPositions[id],D3DCOLOR_XRGB(c,c,c)});}
 d->SetVertexShader(teachingVS);d->SetPixelShader(teachingPS);d->SetVertexShaderConstantF(0,local,4);d->SetVertexShaderConstantF(4,view,4);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);d->SetRenderState(D3DRS_ZENABLE,TRUE);
 CHECK(SUCCEEDED(d->DrawPrimitiveUP(D3DPT_TRIANGLELIST,(UINT)anatomy.size()/3,anatomy.data(),sizeof(TeachingVertex))));
 d->SetVertexShader(gameVS);d->SetVertexShaderConstantF(layout->boneRegister,bone,3);d->SetVertexShaderConstantF(layout->localRegister,local,4);d->SetVertexShaderConstantF(layout->viewRegister,view,4);
 d->SetStreamSource(0,source,0,16);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_CW);
 d->SetRenderState(D3DRS_ALPHABLENDENABLE,FALSE);d->SetRenderState(D3DRS_SRCBLEND,D3DBLEND_ONE);d->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_ZERO);
 RECT originalScissor{17,23,911,678};d->SetScissorRect(&originalScissor);d->SetRenderState(D3DRS_SCISSORTESTENABLE,FALSE);
 if(passive){teachingTimeline.Cancel();teachingFluid.Clear();throbMode=1;throbSizePulse=.7f;}
 ++renderFrameSerial;DrawTeachingFluid(d);
 if(passive){for(int frame=0;frame<90;frame++){Sleep(16);++renderFrameSerial;DrawTeachingFluid(d);}CHECK(teachingFluid.passiveMode&&teachingFluid.Live()>0&&!teachingTimeline.active);for(const auto& s:teachingFluid.streams)CHECK(s.nodes.empty()&&!s.live);CHECK(!teachingFluid.mesh.indices.empty());}
 if(FAILED(teachingLastDraw))printf("SURFACE %s\n",fluidSurface.error.c_str()); CHECK(resting||(teachingDraws>0&&SUCCEEDED(teachingLastDraw)));
 IDirect3DVertexShader9* check=nullptr;d->GetVertexShader(&check);CHECK(check==gameVS);check->Release();
 IDirect3DVertexBuffer9* checkBuffer=nullptr;UINT offset=0,stride=0;d->GetStreamSource(0,&checkBuffer,&offset,&stride);CHECK(checkBuffer==source&&offset==0&&stride==16);checkBuffer->Release();
 DWORD cull=0,write=0;d->GetRenderState(D3DRS_CULLMODE,&cull);d->GetRenderState(D3DRS_ZWRITEENABLE,&write);CHECK(cull==D3DCULL_CW&&write==TRUE); DWORD blend=0,src=0,dst=0;d->GetRenderState(D3DRS_ALPHABLENDENABLE,&blend);d->GetRenderState(D3DRS_SRCBLEND,&src);d->GetRenderState(D3DRS_DESTBLEND,&dst);CHECK(blend==FALSE&&src==D3DBLEND_ONE&&dst==D3DBLEND_ZERO);RECT restoredScissor{};DWORD scissors=1;d->GetScissorRect(&restoredScissor);d->GetRenderState(D3DRS_SCISSORTESTENABLE,&scissors);CHECK(scissors==FALSE&&!memcmp(&originalScissor,&restoredScissor,sizeof(RECT)));
 CHECK(TeachingMatrices(d,l,v,b));
 for(int i=0;i<16;i++){if(local[i]!=l[i])printf("local[%d] %g -> %g\n",i,local[i],l[i]);if(view[i]!=v[i])printf("view[%d] %g -> %g\n",i,view[i],v[i]);}
 for(int i=0;i<12;i++)if(bone[i]!=b[i])printf("bone[%d] %g -> %g\n",i,bone[i],b[i]);
 CHECK(!memcmp(local,l,sizeof(l))&&!memcmp(view,v,sizeof(v))&&!memcmp(bone,b,sizeof(b)));
 unsigned count=teachingDraws;DrawTeachingFluid(d);CHECK(count==teachingDraws);
 if(cameraMotionTest){
  // Reference simulation never sees camera matrices. Exercise production draw.
  teaching::Fluid reference=teachingFluid;
  D3DXMATRIX projection;D3DXMatrixPerspectiveFovLH(&projection,.7f,4.f/3,1,1000);
  for(int frame=0;frame<90;frame++){
   float angle=frame*.07f;D3DXMATRIX localMatrix,cameraMatrix,viewMatrix,combined,inverseLocal;
   D3DXMatrixRotationZ(&localMatrix,.3f*sinf(angle));
   localMatrix._41=300*sinf(angle);localMatrix._42=220*cosf(angle);localMatrix._43=80*sinf(angle*.7f);
   D3DXVECTOR3 eye(tip.x+100*cosf(angle),tip.y+100*sinf(angle),tip.z+25),aim(tip.x+10,tip.y,tip.z-5),up(0,0,1);
   D3DXMatrixLookAtLH(&cameraMatrix,&eye,&aim,&up);D3DXMatrixMultiply(&combined,&cameraMatrix,&projection);
   CHECK(D3DXMatrixInverse(&inverseLocal,nullptr,&localMatrix));D3DXMatrixMultiply(&viewMatrix,&inverseLocal,&combined);
   memcpy(local,&localMatrix,64);memcpy(view,&viewMatrix,64);
   d->SetVertexShaderConstantF(layout->localRegister,local,4);d->SetVertexShaderConstantF(layout->viewRegister,view,4);
   reference.Advance(1.f/120,tip,dir,{});teachingTimeline.Advance(1.f/120);
   d->Clear(0,nullptr,D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER,0,1,0);
   ++renderFrameSerial;DrawTeachingFluid(d);CHECK(teachingFluid.ready&&SUCCEEDED(teachingLastDraw));
   CHECK(Length(teachingFluid.lastTip-reference.lastTip)<.00001f);
   CHECK(teachingFluid.mesh.vertices.size()==reference.mesh.vertices.size());
   CHECK(teachingFluid.mesh.indices==reference.mesh.indices);
   for(size_t i=0;i<reference.mesh.vertices.size();i++){
    CHECK(Length(teachingFluid.mesh.vertices[i].p-reference.mesh.vertices[i].p)<.00001f);
    CHECK(Length(teachingFluid.mesh.vertices[i].n-reference.mesh.vertices[i].n)<.00001f);
   }
   float restoredLocal[16],restoredView[16],restoredBone[12];CHECK(TeachingMatrices(d,restoredLocal,restoredView,restoredBone));
   CHECK(!memcmp(local,restoredLocal,64)&&!memcmp(view,restoredView,64)&&!memcmp(bone,restoredBone,48));
  }
  printf("PASS 90 camera orbit/pre-view translation/rotation frames: simulation equals camera-independent reference, pipeline matrices restored\n");
 }

 d->EndScene();IDirect3DSurface9* back=nullptr;d->GetBackBuffer(0,0,D3DBACKBUFFER_TYPE_MONO,&back);CHECK(back);CHECK(SUCCEEDED(D3DXSaveSurfaceToFileA(resting?"anatomy-rest.png":passive?"teaching-fluid-passive.png":first?"teaching-fluid-first.png":pool?"teaching-fluid-pool.png":closeup?"teaching-fluid-close.png":late?"teaching-fluid-late.png":"teaching-fluid.png",D3DXIFF_PNG,back,nullptr,nullptr)));back->Release();
 if(surfaceBenchmark){
  IDirect3DQuery9* fence=nullptr;CHECK(SUCCEEDED(d->CreateQuery(D3DQUERYTYPE_EVENT,&fence)));LARGE_INTEGER frequency;QueryPerformanceFrequency(&frequency);std::vector<double> timings,gpuTimes;IDirect3DQuery9 *beginStamp=nullptr,*endStamp=nullptr,*gpuFrequency=nullptr;CHECK(SUCCEEDED(d->CreateQuery(D3DQUERYTYPE_TIMESTAMP,&beginStamp)));CHECK(SUCCEEDED(d->CreateQuery(D3DQUERYTYPE_TIMESTAMP,&endStamp)));CHECK(SUCCEEDED(d->CreateQuery(D3DQUERYTYPE_TIMESTAMPFREQ,&gpuFrequency)));gpuFrequency->Issue(D3DISSUE_END);
  for(int frame=0;frame<65;frame++){LARGE_INTEGER begin,end;QueryPerformanceCounter(&begin);d->Clear(0,nullptr,D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER,0,1,0);CHECK(SUCCEEDED(d->BeginScene()));beginStamp->Issue(D3DISSUE_END);CHECK(SUCCEEDED(fluidSurface.Draw(d,teachingFluid.mesh,view)));endStamp->Issue(D3DISSUE_END);CHECK(SUCCEEDED(d->EndScene()));CHECK(SUCCEEDED(fence->Issue(D3DISSUE_END)));HRESULT status;while((status=fence->GetData(nullptr,0,D3DGETDATA_FLUSH))==S_FALSE)Sleep(0);CHECK(SUCCEEDED(status));QueryPerformanceCounter(&end);if(frame>=5){timings.push_back(1000.*(end.QuadPart-begin.QuadPart)/frequency.QuadPart);UINT64 a=0,b=0,f=0;CHECK(beginStamp->GetData(&a,sizeof(a),0)==S_OK&&endStamp->GetData(&b,sizeof(b),0)==S_OK&&gpuFrequency->GetData(&f,sizeof(f),0)==S_OK&&f>0);gpuTimes.push_back(1000.*(b-a)/f);}}
  fence->Release();beginStamp->Release();endStamp->Release();gpuFrequency->Release();std::sort(gpuTimes.begin(),gpuTimes.end());printf("GPU timestamps median=%.3fms p95=%.3fms\n",gpuTimes[30],gpuTimes[57]);std::sort(timings.begin(),timings.end());printf("SURFACE GPU-complete 1920x1080 median=%.3fms p95=%.3fms samples=%u live=%d\n",timings[30],timings[57],(unsigned)timings.size(),teachingFluid.Live());
 }
 ReleaseTeaching();CHECK(!teachingTimeline.active&&!teachingVS&&!teachingPS&&teachingFluid.Live()==0);
 CHECK(SUCCEEDED(d->Reset(&pp)));CHECK(EnsureTeachingShaders(d));ReleaseTeaching();
 source->Release();gameVS->Release();graftBuffer->Release();graftBuffer=nullptr;
 for(UINT i=0;i<shaderLayoutCount;i++)if(shaderLayouts[i].shader)shaderLayouts[i].shader->Release();shaderLayoutCount=0;
 d->Release();d9->Release();DestroyWindow(window);
 printf("PASS real shader reflection, transformed emitter, D3D draw, pipeline restoration, duplicate-pass suppression, device reset/recreation; teaching-fluid.png\n");return true;
}


static bool VoiceCueTest(){
 teaching::PatientAudio audio;audio.count[0]=13;audio.count[1]=7;audio.Begin();int fired[2]={};float previous=0;
 for(int frame=1;frame<=600;frame++){float current=frame/30.f;for(int phase=0;phase<2;phase++)if(audio.Crossing(phase,previous,current)){++fired[phase];audio.fired[phase]=true;CHECK(fabsf(current-teaching::PatientAudio::Cue(phase))<.0001f);}previous=current;}
 CHECK(fired[0]==1&&fired[1]==1);CHECK(teaching::PatientAudio::Cue(0)==2.5f&&teaching::PatientAudio::Cue(1)==7.f);
 CHECK(!audio.Crossing(0,2.49f,2.51f));audio.Begin();CHECK(audio.Crossing(0,2.49f,2.51f));CHECK(audio.Crossing(1,6.99f,7.01f));
 bool seen1[13]{},seen2[7]{};for(int i=0;i<5000;i++){seen1[audio.Select(0)]=true;seen2[audio.Select(1)]=true;}for(bool v:seen1)CHECK(v);for(bool v:seen2)CHECK(v);
 printf("PASS patient audio phase triggers at 2.5s/7.0s once each; random selection covers 13/7 clips\n");return true;
}
static bool AudioAssetTest(const char* folder){
 WIN32_FIND_DATAA f{};char pattern[MAX_PATH];sprintf_s(pattern,"%s\\*.wav",folder);HANDLE h=FindFirstFileA(pattern,&f);CHECK(h!=INVALID_HANDLE_VALUE);int count=0;
 do{if(!(f.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)){char path[MAX_PATH];sprintf_s(path,"%s\\%s",folder,f.cFileName);CHECK(teaching::PatientAudio::ValidPcmWav(path));++count;}}while(FindNextFileA(h,&f));FindClose(h);CHECK(count==13);printf("PASS all13 installed study WAVs are supported PCM; phase1 count=%d\n",count);return true;
}
static bool MotionTest(){
 WNDCLASSA wc{};wc.lpfnWndProc=Proc;wc.hInstance=GetModuleHandle(nullptr);wc.lpszClassName="ThreadMotion";RegisterClassA(&wc);
 HWND window=CreateWindowA(wc.lpszClassName,"Fluid motion validation",0,0,0,960,720,nullptr,nullptr,wc.hInstance,nullptr);
 LoadReal();IDirect3D9* d9=realCreate9(D3D_SDK_VERSION);CHECK(d9);D3DPRESENT_PARAMETERS pp{};pp.Windowed=TRUE;pp.hDeviceWindow=window;pp.SwapEffect=D3DSWAPEFFECT_DISCARD;pp.BackBufferWidth=960;pp.BackBufferHeight=720;pp.BackBufferFormat=D3DFMT_A8R8G8B8;pp.EnableAutoDepthStencil=TRUE;pp.AutoDepthStencilFormat=D3DFMT_D24S8;
 IDirect3DDevice9* d=nullptr;CHECK(SUCCEEDED(d9->CreateDevice(0,D3DDEVTYPE_HAL,window,D3DCREATE_HARDWARE_VERTEXPROCESSING,&pp,&d)));
 volumeFluid::config={};teaching::Fluid f;CHECK(f.Begin({0,0,50}));f.lastDir=Unit({1,0,.45f});volumeFluid::Surface render;
 D3DXMATRIX camera,projection,view;D3DXVECTOR3 eye(29,-220,22),aim(29,0,22),up(0,0,1);D3DXMatrixLookAtLH(&camera,&eye,&aim,&up);D3DXMatrixOrthoLH(&projection,112,84,1,500);D3DXMatrixMultiply(&view,&camera,&projection);
 CHECK(EnsureTeachingShaders(d));CreateDirectoryA("motion-frames",nullptr);unsigned frame=0;
 std::vector<double> costs;LARGE_INTEGER freq;QueryPerformanceFrequency(&freq);
 for(int i=0;i<450;i++){
  float time=(i+1.f)/30;V3 tip{0,.25f*sinf(time*1.7f),50+.4f*sinf(time*1.1f)},dir=Unit({1,.015f*sinf(time*2),.45f+.025f*sinf(time*3)});
  f.Advance(1.f/30,tip,dir,{});CHECK(f.ready);if(time<6.8f)continue;
  d->Clear(0,nullptr,D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER,D3DCOLOR_XRGB(24,34,43),1,0);CHECK(SUCCEEDED(d->BeginScene()));
  // Neutral outlet and measurement grid, no game assets in this motion preview.
  std::vector<TeachingVertex> diagram;
  for(int x=-20;x<=90;x+=10){diagram.push_back({{float(x),2,-20},D3DCOLOR_XRGB(38,51,63)});diagram.push_back({{float(x),2,70},D3DCOLOR_XRGB(38,51,63)});}
  for(int z=-20;z<=70;z+=10){diagram.push_back({{-20,2,float(z)},D3DCOLOR_XRGB(38,51,63)});diagram.push_back({{90,2,float(z)},D3DCOLOR_XRGB(38,51,63)});}
  float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};d->SetVertexShader(teachingVS);d->SetPixelShader(teachingPS);d->SetVertexShaderConstantF(0,identity,4);d->SetVertexShaderConstantF(4,(float*)&view,4);d->SetFVF(D3DFVF_XYZ|D3DFVF_DIFFUSE);d->SetRenderState(D3DRS_ZENABLE,FALSE);d->SetRenderState(D3DRS_ALPHABLENDENABLE,FALSE);
  CHECK(SUCCEEDED(d->DrawPrimitiveUP(D3DPT_LINELIST,(UINT)diagram.size()/2,diagram.data(),sizeof(TeachingVertex))));
  TeachingVertex outlet[6]={{{tip.x-6,0,tip.z-2},0xff6096b0},{{tip.x,0,tip.z-2},0xff6096b0},{{tip.x,0,tip.z+2},0xff6096b0},{{tip.x-6,0,tip.z-2},0xff6096b0},{{tip.x,0,tip.z+2},0xff6096b0},{{tip.x-6,0,tip.z+2},0xff6096b0}};d->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);CHECK(SUCCEEDED(d->DrawPrimitiveUP(D3DPT_TRIANGLELIST,2,outlet,sizeof(TeachingVertex))));
  CHECK(SUCCEEDED(render.Draw(d,f.mesh,(float*)&view)));CHECK(SUCCEEDED(d->EndScene()));
  IDirect3DSurface9* back=nullptr;d->GetBackBuffer(0,0,D3DBACKBUFFER_TYPE_MONO,&back);CHECK(back);char path[120];sprintf_s(path,"motion-frames/frame-%04u.png",frame++);CHECK(SUCCEEDED(D3DXSaveSurfaceToFileA(path,D3DXIFF_PNG,back,nullptr,nullptr)));back->Release();
 }
 render.Release();ReleaseTeaching();d->Release();d9->Release();DestroyWindow(window);printf("PASS moving-outlet sequence, %u rendered frames at30FPS\n",frame);return true;
}
#include "world_space_test.h"
static bool MenuReturnTest(){
 ResetStudyControls();InitMesh();settingsLoaded=true;PrepareMenuTankSurface();
 std::vector<unsigned char> old(menuTankPacked,menuTankPacked+sizeof(menuTankPacked));
 CHECK(graftBuffer);tankCameraSceneTick=GetTickCount();sliderUI[0]=85;ApplyControlMapping();shapeDirty=true;
 UpdateActiveAnatomySurface();
 CHECK(memcmp(old.data(),menuTankPacked,sizeof(menuTankPacked))!=0);
 CHECK(!shapeDirty&&menuTankUploadPending);
 float saved=sliderUI[0];SelectAnatomyScene(false);CHECK(!TankCameraSceneActive()&&!graftBuffer);
 teachingTimeline.Start();fluidBodyPaletteCount[0]=75;fluidGroundCells.push_back({0,0,{},{0,0,1},0});
 SelectAnatomyScene(true);CHECK(TankCameraSceneActive()&&!teachingTimeline.active);
 CHECK(fluidBodyPaletteCount[0]==0&&fluidGroundCells.empty()&&sliderUI[0]==saved);
 teachingTimeline.Start();SelectAnatomyScene(true);CHECK(teachingTimeline.active);
 UpdateActiveAnatomySurface();CHECK(menuTankSurfaceReady&&r14Ready);
 SelectAnatomyScene(false);CHECK(!TankCameraSceneActive()&&!menuTankSurfaceReady);
 printf("PASS returning to tank updates visible mesh even with a retained gameplay buffer\n");return true;
}
#include "splat_contact_test.h"
#include "collision_budget_test.h"
int main(int argc,char** argv){
 if(argc>1&&strcmp(argv[1],"--collision-budget")==0)return CollisionBudgetTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--splat-contact")==0)return SplatContactTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--world-origin-gpu")==0){worldOriginGpuTest=true;return RenderTest()?0:1;}
 if(argc>1&&strcmp(argv[1],"--world-space")==0)return WorldSpaceTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--menu-return")==0)return MenuReturnTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--space-diagnostic")==0){spaceDiagnosticTest=true;return RenderTest()?0:1;}
 if(argc>1&&strcmp(argv[1],"--camera-motion-first")==0){cameraMotionTest=true;return RenderTest(false,true,false,true)?0:1;}
 if(argc>1&&strcmp(argv[1],"--camera-motion")==0){cameraMotionTest=true;return RenderTest()?0:1;}
 if(argc>1&&strcmp(argv[1],"--voice-cues")==0)return VoiceCueTest()?0:1;
 if(argc>2&&strcmp(argv[1],"--audio-assets")==0)return AudioAssetTest(argv[2])?0:1;
 if(argc>1&&strcmp(argv[1],"--motion")==0)return MotionTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--bench-surface")==0){surfaceBenchmark=true;return RenderTest(true)?0:1;}
 if(argc>1&&strcmp(argv[1],"--idle-animation")==0)return IdleAnimationTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--rest")==0)return RestTest(argc>2?atoi(argv[2]):0)?0:1;
 setvbuf(stdout,nullptr,_IONBF,0); if(argc>2)volumeFluid::config.viscosity=(float)atof(argv[2]); if(argc>3)volumeFluid::config.flowVariation=(float)atof(argv[3]);
 if(argc>1&&strcmp(argv[1],"--render-rest")==0)return RenderTest(false,false,false,false,true)?0:1;
 if(argc>1&&strcmp(argv[1],"--render-first")==0)return RenderTest(false,true,false,true)?0:1;
 if(argc>1&&strcmp(argv[1],"--render-pool")==0)return RenderTest(true,false,true)?0:1;
 if(argc>1&&strcmp(argv[1],"--render")==0)return RenderTest()?0:1;
 if(argc>1&&strcmp(argv[1],"--render-late")==0)return RenderTest(true)?0:1;


 if(argc>1&&strcmp(argv[1],"--render-close")==0)return RenderTest(true,true)?0:1;
 if(argc>1&&strcmp(argv[1],"--render-passive")==0)return RenderTest(false,true,false,false,false,true)?0:1;
 if(!PulseTests())return 1;if(!IntegrationTests())return 2;return 0;
}
