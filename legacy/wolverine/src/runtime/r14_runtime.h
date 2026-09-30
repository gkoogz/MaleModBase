// R14 render mesh. The original 2388-point cage still owns the established
// pelvis, shape and physics solver. This surface consumes its FINAL positions.
#include "r14_asset.h"
#include "rounded_render_data.h"
static unsigned char rsPacked[rsCount*32];
#include "neck_render_data.h"
static unsigned char nrPacked[nrCount*32];
static void UpdateNeckRender();
static IDirect3DVertexBuffer9* r14VB;
static IDirect3DIndexBuffer9* r14IB;
static V3 r14Positions[r14Count],r14Normals[r14Count],r14Tangents[r14Count];
static unsigned char r14Packed[r14Count*32];
#include "underside_blend.h"
#include "firm_lobes.h"
#include "ventral_tube.h"
#include "raphe_support.h"
#include "axial_raphe.h"
static bool r14Ready=false,r14UploadPending=false;
static UINT r14SuccessfulDraws=0;
static IDirect3DTexture9* r14SkinTexture[2]{};
static bool r14SkinTextureAttempted=false;
static void ReleaseR14SkinTextures(){
  for(auto& texture:r14SkinTexture)if(texture){texture->Release();texture=nullptr;}
  r14SkinTextureAttempted=false;
}
static void LoadR14SkinTextures(IDirect3DDevice9* d){
  if(r14SkinTextureAttempted)return;
  r14SkinTextureAttempted=true;
  const char* names[2]={"R14-skin-natural.png","R14-skin-erect.png"};
  for(int i=0;i<2;i++){
    char path[MAX_PATH];SiblingPath(path,names[i]);
    PerfScope perf(10);char cached[MAX_PATH];strcpy_s(cached,path);strcpy_s(strrchr(cached,'.'),5,".dds");
    HRESULT hr=D3DXCreateTextureFromFileExA(d,cached,D3DX_DEFAULT,D3DX_DEFAULT,D3DX_DEFAULT,0,D3DFMT_UNKNOWN,D3DPOOL_MANAGED,D3DX_FILTER_NONE,D3DX_FILTER_NONE,0,nullptr,nullptr,&r14SkinTexture[i]);
    if(FAILED(hr))hr=D3DXCreateTextureFromFileExA(d,path,D3DX_DEFAULT,D3DX_DEFAULT,D3DX_DEFAULT,0,
      D3DFMT_UNKNOWN,D3DPOOL_MANAGED,D3DX_FILTER_TRIANGLE,D3DX_FILTER_TRIANGLE,0,nullptr,nullptr,&r14SkinTexture[i]);
    Log("R14 skin texture %s load=%08X",names[i],hr);
  }
}
// Both body scene adapters use the same state-dependent color/SSS maps.
// As with the normal/specular profile, only touch the verified skin pass.
static HRESULT DrawSharedBodySkin(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count){
 LoadR14SkinTextures(d);
 return DrawSharedSkin(d,type,base,minv,nv,start,count,false,r14SkinTexture[physicsState==0?1:0]);
}
static float interLobeWebMask[r14NewStart];
static float obliqueLobeMask[r14NewStart];
static void ShapeObliqueLobes(){
  if(!constraintSolverReady||!eggRestReady)return;
  static V3 original[r14NewStart];memcpy(original,r14Positions,sizeof(original));
  static float rightWeight[r14NewStart];static bool prepared=false;
  static std::vector<UINT> active,faces;
  if(!prepared){
    for(UINT i=0;i<r14NewStart;i++){
      float membership=0.f;for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++)membership+=r14Weight[k]*phys_scrotum_weight[r14Sources[k]];
      obliqueLobeMask[i]=Smoother01((membership-.55f)/.35f)*Smoother01((79.f-r14Base[i*3+2])/3.f);
      rightWeight[i]=Smooth01(.5f+r14Base[i*3+1]/1.4f);if(obliqueLobeMask[i]>0.f)active.push_back(i);
    }
    for(UINT k=0;k<r14IndexCount;k+=3){UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
      if(a<r14NewStart&&b<r14NewStart&&c<r14NewStart&&(obliqueLobeMask[a]>0.f||obliqueLobeMask[b]>0.f||obliqueLobeMask[c]>0.f))faces.push_back(k);
    }prepared=true;
  }
  float scale=sqrtf(max(.35f,BallShapeScale()));
  V3 restAxis[2],liveAxis[2],up[2];
  for(int side=0;side<2;side++){
    restAxis[side]=Unit(constraintBallRest[side]-RestBallAnchor(side));liveAxis[side]=Unit(ballNodes[side]-BallAnchor(side));
    float forward=(side?16.f:18.f)*.01745329252f,lateral=(side?10.f:-9.f)*.01745329252f;
    up[side]=Unit(V3{sinf(forward),sinf(lateral),cosf(forward)*cosf(lateral)});
  }
  GeometryRotation toRest[2]={{liveAxis[0],restAxis[0]},{liveAxis[1],restAxis[1]}};
  GeometryRotation toLive[2]={{restAxis[0],liveAxis[0]},{restAxis[1],liveAxis[1]}};
  GeometryRotation tilt[2]={{{0,0,1},up[0]},{{0,0,1},up[1]}};
  for(UINT i:active){
    float mask=obliqueLobeMask[i],right=rightWeight[i];V3 shift{};
    for(int side=0;side<2;side++){
      V3 offset=toRest[side].Apply(original[i]-ballNodes[side]);
      V3 tilted=tilt[side].Apply(offset);tilted.z+=(side?.05f:-.15f)*scale;
      V3 change=toLive[side].Apply(tilted-offset);shift=shift+change*(side?right:1.f-right);
    }
    r14Positions[i]=original[i]+shift*mask;
  }
  // Restrict a strained attachment triangle locally, rather than letting one
  // short seam edge cancel the resting tilt of both complete lobes.
  static std::vector<PreparedSurfaceLimit> limits;
  limits.resize(faces.size());
  for(unsigned j=0;j<faces.size();j++){UINT k=faces[j];limits[j].Prepare(original[r14Indices[k]],original[r14Indices[k+1]],original[r14Indices[k+2]]);}
  for(int pass=0;pass<10;pass++){bool changed=false;for(unsigned j=0;j<faces.size();j++){UINT k=faces[j];
    UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    if(a>=r14NewStart||b>=r14NewStart||c>=r14NewStart)continue;
    float local=limits[j].Evaluate(r14Positions[a]-original[a],r14Positions[b]-original[b],r14Positions[c]-original[c],.35f);
    if(local<.9999f){changed=true;
      r14Positions[a]=original[a]+(r14Positions[a]-original[a])*local;
      r14Positions[b]=original[b]+(r14Positions[b]-original[b])*local;
      r14Positions[c]=original[c]+(r14Positions[c]-original[c])*local;
    }
  }
  if(!changed)break;}
  float fraction=1.f;
  for(unsigned j=0;j<faces.size();j++){UINT k=faces[j];
    UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    if(a>=r14NewStart||b>=r14NewStart||c>=r14NewStart)continue;
    fraction=min(fraction,limits[j].Evaluate(r14Positions[a]-original[a],r14Positions[b]-original[b],r14Positions[c]-original[c],.35f));
  }
  if(fraction<1.f)for(UINT i=0;i<r14NewStart;i++)r14Positions[i]=original[i]+(r14Positions[i]-original[i])*fraction;
}
static void TightenInterLobeWeb(){
  static bool initialized=false;
  static std::vector<UINT> active,normalFaces,safetyFaces;
  static std::vector<UINT> adjacent[r14NewStart];
  static V3 original[r14NewStart],next[r14NewStart],normals[r14NewStart];
  if(!initialized){
    for(UINT i=0;i<r14NewStart;i++){
      float membership=0.f;
      for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++)membership+=r14Weight[k]*phys_scrotum_weight[r14Sources[k]];
      float center=1.f-Smoother01((fabsf(r14Base[i*3+1])-.45f)/1.50f);
      interLobeWebMask[i]=center*Smoother01((membership-.55f)/.30f);
    }
    for(UINT k=0;k<r14IndexCount;k+=3)for(int a=0;a<3;a++){
      UINT i=r14Indices[k+a];if(i>=r14NewStart)continue;
      for(int b=0;b<3;b++)if(a!=b){UINT j=r14Indices[k+b];auto& list=adjacent[i];if(std::find(list.begin(),list.end(),j)==list.end())list.push_back(j);}
    }
    for(UINT i=0;i<r14NewStart;i++)if(interLobeWebMask[i]>0.f&&!adjacent[i].empty())active.push_back(i);
    for(UINT k=0;k<r14IndexCount;k+=3){UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
      if((a<r14NewStart&&interLobeWebMask[a]>0.f)||(b<r14NewStart&&interLobeWebMask[b]>0.f)||(c<r14NewStart&&interLobeWebMask[c]>0.f)){
        normalFaces.push_back(k);if(a<r14NewStart&&b<r14NewStart&&c<r14NewStart)safetyFaces.push_back(k);
      }
    }
    initialized=true;
  }
  memcpy(original,r14Positions,sizeof(original));
  // Fill only concave portions of the central skin. Convex lobe surfaces and
  // the midline ridge are not shrunk; no simulation state is written here.
  float cap=.48f*sqrtf(max(.35f,BallShapeScale()));
  for(int pass=0;pass<18;pass++){
    memset(normals,0,sizeof(normals));
    for(UINT k:normalFaces){
      UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
      if(a>=r14NewStart&&b>=r14NewStart&&c>=r14NewStart)continue;
      V3 n=Cross(r14Positions[c]-r14Positions[a],r14Positions[b]-r14Positions[a]);
      if(a<r14NewStart)normals[a]=normals[a]+n;if(b<r14NewStart)normals[b]=normals[b]+n;if(c<r14NewStart)normals[c]=normals[c]+n;
    }
    for(UINT i:active){
      next[i]=r14Positions[i];if(interLobeWebMask[i]<=0.f||adjacent[i].empty())continue;
      V3 mean{};for(UINT j:adjacent[i])mean=mean+r14Positions[j];mean=mean/float(adjacent[i].size());
      V3 n=Unit(normals[i]);float outward=max(0.f,Dot(mean-r14Positions[i],n));
      next[i]=next[i]+n*(outward*.28f*interLobeWebMask[i]);
      V3 delta=next[i]-original[i];
      if(Length(delta)>cap)next[i]=original[i]+Unit(delta)*cap;
    }
    for(UINT i:active)r14Positions[i]=next[i];
  }
  float fraction=1.f;
  for(UINT k:safetyFaces){
    UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    if(a>=r14NewStart||b>=r14NewStart||c>=r14NewStart)continue;
    fraction=min(fraction,SurfaceCorrectionLimit(original[a],original[b],original[c],r14Positions[a]-original[a],r14Positions[b]-original[b],r14Positions[c]-original[c],.35f));
  }
  if(fraction<1.f)for(UINT i=0;i<r14NewStart;i++)r14Positions[i]=original[i]+(r14Positions[i]-original[i])*fraction;
}
static float R14LegacyScale(float ui){return ui<=50.f?.85f+.003f*ui:1.f+.012f*(ui-50.f);}
static float R16PreviousScale(float ui){
  ui=max(0.f,min(100.f,ui));
  return ui<50.f?.80f+.05f*Smoother01(ui/50.f):R14LegacyScale(LegacyGlansControl(ui));
}
static float R14Scale(float ui){
  ui=max(0.f,min(100.f,ui));
  // New 50..100 is exactly the immediately preceding 0..100 response.
  // The lower half holds the preceding zero and uses a separate safe shrink.
  return ui<50.f?R16PreviousScale(0.f):R16PreviousScale((ui-50.f)*2.f);
}
static float R14LowerFactor(float ui){return ui<50.f?1.f-Smoother01(max(0.f,ui)/50.f):0.f;}
static void ReleaseR14(){
  if(r14VB){r14VB->Release();r14VB=nullptr;}
  if(r14IB){r14IB->Release();r14IB=nullptr;}
  r14UploadPending=true;
}
static void UpdateR14(const unsigned char* source){PerfScope perf(5);
  const float amount=R14Scale(GlansControlV4(glansUI))-1.f;
  const float lowerFactor=R14LowerFactor(GlansControlV4(glansUI));
  static V3 delta[graftCount];
  for(UINT i=0;i<graftCount;i++)delta[i]=graftDeformedPositions[i]-V3{r14Reference[i*3],r14Reference[i*3+1],r14Reference[i*3+2]};
  V3 columns[3]={{1,0,0},{0,1,0},{0,0,1}};
  for(UINT j=0;j<r14FrameCount;j++)for(int a=0;a<3;a++)columns[a]=columns[a]+delta[r14FrameSources[j]]*r14FrameLinear[a*r14FrameCount+j];
  // Glans-dependent coefficients are material bindings. The live cage delta
  // is the only input accumulated per vertex when controls are unchanged.
  static float preparedAmount=0.f;static bool mapReady=false;
  static V3 base[r14Count];static float weights[sizeof(r14Weight)/sizeof(r14Weight[0])];
  static __m128 deltas[graftCount];
  if(!mapReady||preparedAmount!=amount){
    for(UINT i=0;i<r14Count;i++)base[i]={r14Base[3*i]+amount*r14Growth[3*i],r14Base[3*i+1]+amount*r14Growth[3*i+1],r14Base[3*i+2]+amount*r14Growth[3*i+2]};
    for(UINT k=0;k<sizeof(weights)/sizeof(weights[0]);k++)weights[k]=r14Weight[k]+amount*r14GrowthWeight[k];
    preparedAmount=amount;mapReady=true;
  }
  for(UINT i=0;i<graftCount;i++)deltas[i]=_mm_set_ps(0.f,delta[i].z,delta[i].y,delta[i].x);
  volatile LONG invalidPosition=0;
  GeometryFor(r14Count,[&](unsigned i){
    __m128 position=_mm_set_ps(0.f,base[i].z,base[i].y,base[i].x);
    for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++)position=_mm_add_ps(position,_mm_mul_ps(deltas[r14Sources[k]],_mm_set1_ps(weights[k])));
    float xyz[4];_mm_storeu_ps(xyz,position);V3 p{xyz[0],xyz[1],xyz[2]};
    if(lowerFactor>0.f){
      V3 local{r14LowerDelta[3*i],r14LowerDelta[3*i+1],r14LowerDelta[3*i+2]};
      p=p+(columns[0]*local.x+columns[1]*local.y+columns[2]*local.z)*lowerFactor;
    }
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)){InterlockedExchange(&invalidPosition,1);return;}
    r14Positions[i]=p;
  });
  if(invalidPosition){r14Ready=false;return;}
  // Length changes the shaft, while Glans Size alone owns the head. Restore
  // the crown-to-tip span lost in the legacy length morph by moving complete
  // ring centers; their radial offsets and lowest side points remain intact.
  if(sliderUI[1]<50.f){
    V3 centers[r14RingCount];
    for(UINT ring=r14CrownRing;ring<r14RingCount;ring++){
      V3 center{0,0,0};UINT first=r14NewStart+ring*r14SegmentCount;
      for(UINT side=0;side<r14SegmentCount;side++)center=center+r14Positions[first+side];
      centers[ring]=center/float(r14SegmentCount);
    }
    V3 crown=centers[r14CrownRing];
    float currentArc=0.f;
    for(UINT ring=r14CrownRing+1;ring<r14RingCount;ring++)currentArc+=Length(centers[ring]-centers[ring-1]);
    float crownRadius=0.f;UINT crownFirst=r14NewStart+r14CrownRing*r14SegmentCount;
    for(UINT side=0;side<r14SegmentCount;side++)crownRadius+=Length(r14Positions[crownFirst+side]-crown);
    crownRadius/=float(r14SegmentCount);
    float axialScale=currentArc>1e-5f?(.985f*crownRadius/currentArc):1.f;
    for(UINT ring=r14CrownRing+1;ring<r14RingCount;ring++){
      V3 shift=crown+(centers[ring]-crown)*axialScale-centers[ring];
      UINT first=r14NewStart+ring*r14SegmentCount;
      for(UINT side=0;side<r14SegmentCount;side++)r14Positions[first+side]=r14Positions[first+side]+shift;
    }
    UINT cap=r14NewStart+r14RingCount*r14SegmentCount;
    for(UINT i=cap;i<r14Count;i++)r14Positions[i]=crown+(r14Positions[i]-crown)*axialScale;
  }
  // Fair only the proximal ventral transition. Fixed boundary and crown
  // vertices retain their exact positions; every pass reads a shared snapshot.
  static V3 rapheInput[r14Count];memcpy(rapheInput,r14Positions,sizeof(rapheInput));
  static GeometryFairPass<r14Count> rapheFair;static bool rapheReady=false;
  if(!rapheReady){for(UINT k=0;k<rapheSmoothCount;k++)rapheFair.Add(rapheSmoothIDs[k],rapheSmoothNeighbors+rapheSmoothOffsets[k],rapheSmoothNeighbors+rapheSmoothOffsets[k+1],.32f*rapheSmoothMask[k]);rapheReady=true;}
  rapheFair.Apply(r14Positions,18);
  float rapheFraction=1.f;
  for(UINT k=0;k<r14IndexCount;k+=3){
    UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    if(a>=r14NewStart&&b>=r14NewStart&&c>=r14NewStart)continue;
    rapheFraction=min(rapheFraction,SurfaceCorrectionLimit(rapheInput[a],rapheInput[b],rapheInput[c],r14Positions[a]-rapheInput[a],r14Positions[b]-rapheInput[b],r14Positions[c]-rapheInput[c],.35f));
  }
  debugSmoothFraction=rapheFraction;
  if(rapheFraction<1.f)for(UINT k=0;k<rapheSmoothCount;k++){
    UINT i=rapheSmoothIDs[k];r14Positions[i]=rapheInput[i]+(r14Positions[i]-rapheInput[i])*rapheFraction;
  }
  ShapeObliqueLobes();
  TightenInterLobeWeb();
  memset(undersideBlendMoved,0,sizeof(undersideBlendMoved));
  PreserveRigidLobeSurfaces();
  PreserveShaftRaphe();
  // Retain the authored neck placement; the final shared tube changes only
  // the shaft envelope and a bounded easing at the neck intersection.
  PreserveAxialRaphe();
  static float uv[r14IndexCount/3][5];static bool uvReady=false;
  if(!uvReady){for(UINT k=0;k<r14IndexCount;k+=3){UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];float* d=uv[k/3];
    d[0]=r14UV[b*2]-r14UV[a*2];d[1]=r14UV[b*2+1]-r14UV[a*2+1];d[2]=r14UV[c*2]-r14UV[a*2];d[3]=r14UV[c*2+1]-r14UV[a*2+1];d[4]=d[0]*d[3]-d[1]*d[2];
  }uvReady=true;}
  memset(r14Normals,0,sizeof(r14Normals));memset(r14Tangents,0,sizeof(r14Tangents));
  for(UINT k=0;k<r14IndexCount;k+=3){
    UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
    V3 e1=r14Positions[b]-r14Positions[a],e2=r14Positions[c]-r14Positions[a];
    V3 n=Cross(e2,e1);r14Normals[a]=r14Normals[a]+n;r14Normals[b]=r14Normals[b]+n;r14Normals[c]=r14Normals[c]+n;
    const float* d=uv[k/3];float dv1=d[1],dv2=d[3],det=d[4];
    if(fabsf(det)>1e-10f){V3 t=(e1*dv2-e2*dv1)/det;r14Tangents[a]=r14Tangents[a]+t;r14Tangents[b]=r14Tangents[b]+t;r14Tangents[c]=r14Tangents[c]+t;}
  }
  const V3 normalColumns[3]={Cross(columns[1],columns[2]),Cross(columns[2],columns[0]),Cross(columns[0],columns[1])};
  for(UINT i=0;i<r14Count;i++){
    V3 n;
    if(firmLobeMask[i]<=1e-6f&&undersideBlendMoved[i]<=1e-6f&&(i>=r14NewStart||(interLobeWebMask[i]<=1e-4f&&obliqueLobeMask[i]<=1e-4f))&&rapheSmoothFull[i]<=1e-4f&&(r14CustomNormals[i*3]!=0.f||r14CustomNormals[i*3+1]!=0.f||r14CustomNormals[i*3+2]!=0.f)){
      V3 custom{r14CustomNormals[i*3],r14CustomNormals[i*3+1],r14CustomNormals[i*3+2]};
      n=Unit(normalColumns[0]*custom.x+normalColumns[1]*custom.y+normalColumns[2]*custom.z);
    }else n=Unit(r14Normals[i]);
    V3 t=r14Tangents[i]-n*Dot(n,r14Tangents[i]);
    if(Length(t)<1e-6f)t=Cross(fabsf(n.z)<.9f?V3{0,0,1}:V3{0,1,0},n);
    t=Unit(t);unsigned char* v=r14Packed+i*32;memcpy(v,&r14Positions[i],12);
    v[12]=PackSigned(t.x);v[13]=PackSigned(t.y);v[14]=PackSigned(t.z);v[15]=127;
    v[16]=PackSigned(n.x);v[17]=PackSigned(n.y);v[18]=PackSigned(n.z);v[19]=0;
    // Preserve exact normals at the body attachment, including its weld locks.
    if(r14Flex[i]<.04f&&r14Offsets[i+1]-r14Offsets[i]==1&&fabsf(r14Weight[r14Offsets[i]]-1.f)<1e-6f)
      memcpy(v+12,source+r14Sources[r14Offsets[i]]*32+12,8);
  }
  // Authored skin palettes and UVs do not depend on pose or dimensions. CPU
  // packed storage survives D3D device resets; only GPU resources are recreated.
  static bool materialReady=false;
  if(!materialReady){for(UINT i=0;i<r14Count;i++){
    unsigned char* v=r14Packed+i*32;
    memcpy(v+20,r14Bones+i*4,4);memcpy(v+24,r14Weights+i*4,4);
    D3DXFloat32To16Array(reinterpret_cast<D3DXFLOAT16*>(v+28),r14UV+i*2,2);
  }materialReady=true;}
  r14Ready=true;r14UploadPending=true;
}
static bool EnsureR14(IDirect3DDevice9* d){
  if(!r14Ready)return false;
  if(!r14VB){
    if(FAILED(d->CreateVertexBuffer(sizeof(nrPacked),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&r14VB,nullptr)))return false;
    r14UploadPending=true;
  }
  if(!r14IB){
    if(FAILED(d->CreateIndexBuffer(sizeof(nrIndices),D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&r14IB,nullptr)))return false;
    void* raw=nullptr;if(FAILED(r14IB->Lock(0,0,&raw,0))){ReleaseR14();return false;}
    memcpy(raw,nrIndices,sizeof(nrIndices));r14IB->Unlock();
  }
  if(r14UploadPending){
    void* raw=nullptr;if(FAILED(r14VB->Lock(0,sizeof(nrPacked),&raw,D3DLOCK_DISCARD)))return false;
    memcpy(raw,nrPacked,sizeof(nrPacked));r14VB->Unlock();r14UploadPending=false;
  }
  return true;
}
static HRESULT DrawR14(IDirect3DDevice9* d,IDirect3DVertexBuffer9* original,UINT offset,UINT stride){
  IDirect3DIndexBuffer9* ib=nullptr;
  if(FAILED(d->GetIndices(&ib)))return E_FAIL;
  LoadR14SkinTextures(d);
  IDirect3DBaseTexture9* oldDiffuse=nullptr,*oldSkin=nullptr;
  d->GetTexture(2,&oldDiffuse);d->GetTexture(10,&oldSkin);
  bool replaced=false;
  IDirect3DTexture9* chosen=r14SkinTexture[physicsState==0?1:0];
  if(chosen&&oldDiffuse&&oldDiffuse==oldSkin&&oldDiffuse->GetType()==D3DRTYPE_TEXTURE){
    D3DSURFACE_DESC desc{};
    if(SUCCEEDED(static_cast<IDirect3DTexture9*>(oldDiffuse)->GetLevelDesc(0,&desc))&&desc.Width==4096&&desc.Height==4096){
      d->SetTexture(2,chosen);d->SetTexture(10,chosen);replaced=true;
    }
  }
  HRESULT hr=d->SetStreamSource(0,r14VB,0,32);
  if(SUCCEEDED(hr))hr=d->SetIndices(r14IB);
  if(SUCCEEDED(hr)){
    bool capture=captureRemaining>0&&captureDraw<16;
    if(capture){CaptureDraw(d,D3DPT_TRIANGLELIST,0,0,nrCount,0,nrIndexCount/3,nrPacked,sizeof(nrPacked),nrIndices,sizeof(nrIndices));CaptureSurface(d,"before");}
    hr=capture?origDIP(d,D3DPT_TRIANGLELIST,0,0,nrCount,0,nrIndexCount/3):DrawSharedSkin(d,D3DPT_TRIANGLELIST,0,0,nrCount,0,nrIndexCount/3,true);
    if(capture)CaptureSurface(d,"after");
  }
  if(replaced){d->SetTexture(2,oldDiffuse);d->SetTexture(10,oldSkin);}
  if(oldDiffuse)oldDiffuse->Release();if(oldSkin)oldSkin->Release();
  d->SetStreamSource(0,original,offset,stride);d->SetIndices(ib);if(ib)ib->Release();
  if(SUCCEEDED(hr)){if(r14SuccessfulDraws++==0)Log("R14 rounded replacement draw active: %u vertices, %u triangles",nrCount,nrIndexCount/3);}
  return hr;
}
