#pragma once
static bool IsHdrSceneColorPass(IDirect3DDevice9* d);
#include "anatomy_basis_data.h"
#include "body_attachments.h"
// WStart uses a dedicated 12,082-vertex cinematic mesh and a 152-bone rig.
// The anatomy surface uses only pelvis, spine, spine1, and the two thigh-twist
// bones.  WStart section 4 carries all five, so the authored surface can use
// the live tank animation after remapping its compact section palette.
static unsigned char menuTankPacked[nrCount*32];
static IDirect3DVertexBuffer9* menuTankVB;
#include "menu_shell_eye_data.h"
#include "menu_body_split_data.h"
#include "menu_chest_data.h"
#include "menu_retarget_body_data.h"
static IDirect3DIndexBuffer9* menuShellFaceIB;
static IDirect3DIndexBuffer9* menuBodySplitIB[2][4]{};
static IDirect3DVertexBuffer9* menuChestVB;
static IDirect3DVertexBuffer9* menuRetargetBodyVB[2]{};
static IDirect3DIndexBuffer9* menuRetargetBodyIB[2]{};
static IDirect3DTexture9* menuBlendTexture[2]{};
static bool menuBlendTextureAttempted=false;
static bool menuTankSurfaceReady=false,menuTankUploadPending=true;
static bool menuTankDrawnThisFrame=false;
static UINT menuTankSuccessfulDraws=0;

static void ReleaseMenuTank(){
  ReleaseBodyAttachments();
  if(menuTankVB){menuTankVB->Release();menuTankVB=nullptr;}
  if(menuShellFaceIB){menuShellFaceIB->Release();menuShellFaceIB=nullptr;}
  if(menuChestVB){menuChestVB->Release();menuChestVB=nullptr;}
  for(auto& buffer:menuRetargetBodyVB)if(buffer){buffer->Release();buffer=nullptr;}
  for(auto& buffer:menuRetargetBodyIB)if(buffer){buffer->Release();buffer=nullptr;}
  for(auto& texture:menuBlendTexture)if(texture){texture->Release();texture=nullptr;}
  menuBlendTextureAttempted=false;
  for(int section=0;section<2;section++)for(int pass=0;pass<4;pass++)if(menuBodySplitIB[section][pass]){menuBodySplitIB[section][pass]->Release();menuBodySplitIB[section][pass]=nullptr;}
  menuTankUploadPending=true;menuRetargetBodyUploadPending[0]=menuRetargetBodyUploadPending[1]=true;menuTankDrawnThisFrame=false;
}

// Preserve only the two disconnected eyeball islands. The complete shell head
// uses a different model/skin and must not be overlaid on the tank actor.
static bool EnsureMenuShellFace(IDirect3DDevice9* d){
  if(menuShellFaceIB)return true;
  if(FAILED(d->CreateIndexBuffer(sizeof(menuShellEyeIndices),D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&menuShellFaceIB,nullptr)))return false;
  void* raw=nullptr;if(FAILED(menuShellFaceIB->Lock(0,0,&raw,0))){menuShellFaceIB->Release();menuShellFaceIB=nullptr;return false;}
  memcpy(raw,menuShellEyeIndices,sizeof(menuShellEyeIndices));menuShellFaceIB->Unlock();return true;
}

static HRESULT DrawMenuShellFace(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base){
  if(!EnsureMenuShellFace(d))return D3D_OK;
  IDirect3DIndexBuffer9* originalIB=nullptr;if(FAILED(d->GetIndices(&originalIB)))return D3D_OK;
  HRESULT hr=d->SetIndices(menuShellFaceIB);
  if(SUCCEEDED(hr))hr=origDIP(d,type,base,0,14498,0,menuShellEyeTriangleCount);
  d->SetIndices(originalIB);originalIB->Release();
  static volatile LONG logged=0;
  if(SUCCEEDED(hr)&&InterlockedCompareExchange(&logged,1,0)==0)Log("menu tank shell eyes restored: %u triangles",menuShellEyeTriangleCount);
  return hr;
}

static void PrepareMenuTankSurface(){PerfScope perf(3);
  bool report=shapeDirty||!menuTankSurfaceReady;
  // Reuse the production shape pipeline with a private CPU donor buffer.  It
  // contains the exact authored graft and pelvic attachment bases but never
  // becomes the gameplay graftBuffer, so WStart cannot poison gameplay mesh
  // discovery.  paBasisSaved deliberately remains untouched until the real
  // gameplay buffer is available.
  static unsigned char donor[(47050u+graftCount)*graftStride]{};
  auto* controlled=donor+pelvisControlFirstVertex*graftStride;
  auto* p=donor+47050u*graftStride;
  for(UINT i=0;i<graftCount;i++){
    V3 position={morph_base[i*3],morph_base[i*3+1],morph_base[i*3+2]};
    memcpy(p+i*graftStride,&position,12);
    memcpy(p+i*graftStride+12,anatomyDonorBasis+i*8,8);
  }
  PrepareMenuRetargetBodyBasis(donor);
  for(unsigned k=0;k<paBodyNormalCount;k++){
    V3 position=PARead(paBodyNormalBase,k);memcpy(donor+paBodyNormalIDs[k]*graftStride,&position,12);
  }

  if(report){preparedShapeReady=false;shaftRestFrameReady=false;eggRestReady=false;constraintSolverReady=false;constraintSolverState=-1;}
  EvaluateAnatomy(donor,47050u);
  memcpy(menuTankPacked,nrPacked,sizeof(menuTankPacked));
  // Gameplay section 7 slots 0..4 are global bones 3,4,5,113,121.  WStart
  // section 4 stores the matching bones 3,4,5,139,149 in slots below.
  static const unsigned char remap[5]={30,24,23,31,32};
  for(UINT i=0;i<nrCount;i++)for(int j=0;j<4;j++){
    unsigned char& bone=menuTankPacked[i*32+20+j];if(bone<5)bone=remap[bone];
  }
  menuTankSurfaceReady=true;menuTankUploadPending=true;shapeDirty=false;
  if(report)Log("menu tank anatomy prepared: %u vertices, %u triangles, state=%d shape=%.0f %.0f %.0f %.0f %.0f %.0f %.0f",nrCount,nrIndexCount/3,physicsState,sliderUI[0],sliderUI[1],sliderUI[2],sliderUI[3],sliderUI[4],sliderUI[5],sliderUI[6]);
}

static bool EnsureMenuTank(IDirect3DDevice9* d){
  if(!menuTankSurfaceReady)return false;
  if(!menuTankVB){if(FAILED(d->CreateVertexBuffer(sizeof(menuTankPacked),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&menuTankVB,nullptr)))return false;menuTankUploadPending=true;}
  if(!r14IB){if(FAILED(d->CreateIndexBuffer(sizeof(nrIndices),D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&r14IB,nullptr)))return false;void* raw=nullptr;if(FAILED(r14IB->Lock(0,0,&raw,0))){ReleaseMenuTank();return false;}memcpy(raw,nrIndices,sizeof(nrIndices));r14IB->Unlock();}
  if(menuTankUploadPending){void* raw=nullptr;if(FAILED(menuTankVB->Lock(0,sizeof(menuTankPacked),&raw,D3DLOCK_DISCARD)))return false;memcpy(raw,menuTankPacked,sizeof(menuTankPacked));menuTankVB->Unlock();menuTankUploadPending=false;}
  return true;
}

static bool EnsureMenuBodySplit(IDirect3DDevice9* d){
  const unsigned short* data[2][4]={{menuBodyCinematicIndices0,menuBodyBlend33Indices0,menuBodyBlend67Indices0,menuBodyPelvisIndices0},{menuBodyCinematicIndices1,menuBodyBlend33Indices1,menuBodyBlend67Indices1,menuBodyPelvisIndices1}};
  const UINT bytes[2][4]={{sizeof(menuBodyCinematicIndices0),sizeof(menuBodyBlend33Indices0),sizeof(menuBodyBlend67Indices0),sizeof(menuBodyPelvisIndices0)},{sizeof(menuBodyCinematicIndices1),sizeof(menuBodyBlend33Indices1),sizeof(menuBodyBlend67Indices1),sizeof(menuBodyPelvisIndices1)}};
  for(int section=0;section<2;section++)for(int pass=0;pass<4;pass++)if(!menuBodySplitIB[section][pass]){
    if(FAILED(d->CreateIndexBuffer(bytes[section][pass],D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&menuBodySplitIB[section][pass],nullptr)))return false;
    void* raw=nullptr;if(FAILED(menuBodySplitIB[section][pass]->Lock(0,0,&raw,0)))return false;
    memcpy(raw,data[section][pass],bytes[section][pass]);menuBodySplitIB[section][pass]->Unlock();
  }
  return true;
}

static void LoadMenuBlendTextures(IDirect3DDevice9* d){
  if(menuBlendTextureAttempted)return;menuBlendTextureAttempted=true;
  const char* names[2]={"MenuTank-skin-blend33.png","MenuTank-skin-blend67.png"};
  for(int i=0;i<2;i++){char path[MAX_PATH];SiblingPath(path,names[i]);HRESULT hr=D3DXCreateTextureFromFileExA(d,path,D3DX_DEFAULT,D3DX_DEFAULT,D3DX_DEFAULT,0,D3DFMT_UNKNOWN,D3DPOOL_MANAGED,D3DX_FILTER_TRIANGLE,D3DX_FILTER_TRIANGLE,0,nullptr,nullptr,&menuBlendTexture[i]);Log("menu tank blend texture %s load=%08X",names[i],hr);}
}

static bool EnsureMenuChestBuffer(IDirect3DDevice9* d,IDirect3DVertexBuffer9* source){
  if(menuChestVB)return true;D3DVERTEXBUFFER_DESC desc{};if(FAILED(source->GetDesc(&desc)))return false;
  void* src=nullptr;if(FAILED(source->Lock(0,0,&src,D3DLOCK_READONLY))){static volatile LONG once=0;if(InterlockedCompareExchange(&once,1,0)==0)Log("menu chest source buffer lock failed");return false;}
  HRESULT created=d->CreateVertexBuffer(desc.Size,D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&menuChestVB,nullptr);if(FAILED(created)){source->Unlock();return false;}
  void* dst=nullptr;if(FAILED(menuChestVB->Lock(0,0,&dst,0))){source->Unlock();menuChestVB->Release();menuChestVB=nullptr;return false;}
  memcpy(dst,src,desc.Size);source->Unlock();
  auto* bytes=(unsigned char*)dst;for(UINT i=0;i<menuChestOverrideCount;i++)memcpy(bytes+menuChestOverrideIDs[i]*32,menuChestOverridePositions+i*3,12);
  // Necklace clearance is applied to its animated tag-bone palette at draw
  // time. Editing skinned vertices here rotates the desired forward offset
  // through four differently oriented dog-tag bones.
  menuChestVB->Unlock();Log("menu tank overdeveloped chest active: %u body vertices",menuChestOverrideCount);return true;
}

static bool EnsureMenuRetargetBody(IDirect3DDevice9* d,int section){
  if(section<0||section>1)return false;
  const unsigned char* vertexData[2]={menuRetargetBodyDynamic0,menuRetargetBodyDynamic1};
  const UINT vertexBytes[2]={sizeof(menuRetargetBodyPacked0),sizeof(menuRetargetBodyPacked1)};
  const unsigned short* indexData[2]={menuRetargetBodyIndices0,menuRetargetBodyIndices1};
  const UINT indexBytes[2]={sizeof(menuRetargetBodyIndices0),sizeof(menuRetargetBodyIndices1)};
  if(!menuRetargetBodyVB[section]){
    if(FAILED(d->CreateVertexBuffer(vertexBytes[section],D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&menuRetargetBodyVB[section],nullptr)))return false;
    menuRetargetBodyUploadPending[section]=true;
  }
  if(menuRetargetBodyUploadPending[section]){
    void* raw=nullptr;if(FAILED(menuRetargetBodyVB[section]->Lock(0,vertexBytes[section],&raw,D3DLOCK_DISCARD)))return false;
    memcpy(raw,vertexData[section],vertexBytes[section]);menuRetargetBodyVB[section]->Unlock();menuRetargetBodyUploadPending[section]=false;
  }
  if(!menuRetargetBodyIB[section]){
    if(FAILED(d->CreateIndexBuffer(indexBytes[section],D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&menuRetargetBodyIB[section],nullptr)))return false;
    void* raw=nullptr;if(FAILED(menuRetargetBodyIB[section]->Lock(0,0,&raw,0))){menuRetargetBodyIB[section]->Release();menuRetargetBodyIB[section]=nullptr;return false;}
    memcpy(raw,indexData[section],indexBytes[section]);menuRetargetBodyIB[section]->Unlock();
  }
  return true;
}

// The gameplay and WStart rigs have matching bind transforms to within 1.1e-5
// model units.  The generated buffers preserve the mod body's full geometry,
// UVs, normals and weights while translating each chunk-local bone slot by
// bone name into the compact palette active for this WStart draw.
static HRESULT DrawMenuRetargetBody(IDirect3DDevice9* d,int section,D3DPRIMITIVETYPE type){
  if(type!=D3DPT_TRIANGLELIST||!EnsureMenuRetargetBody(d,section))return E_FAIL;
  IDirect3DVertexBuffer9* originalVB=nullptr;IDirect3DIndexBuffer9* originalIB=nullptr;UINT originalOffset=0,originalStride=0;
  if(FAILED(d->GetStreamSource(0,&originalVB,&originalOffset,&originalStride))||!originalVB)return E_FAIL;
  if(FAILED(d->GetIndices(&originalIB))){originalVB->Release();return E_FAIL;}
  const UINT vertices[2]={menuRetargetBodyVertexCount0,menuRetargetBodyVertexCount1};
  const UINT triangles[2]={menuRetargetBodyTriangleCount0,menuRetargetBodyTriangleCount1};
  HRESULT hr=d->SetStreamSource(0,menuRetargetBodyVB[section],0,32);
  if(SUCCEEDED(hr))hr=d->SetIndices(menuRetargetBodyIB[section]);
  if(SUCCEEDED(hr))hr=DrawSharedBodySkin(d,D3DPT_TRIANGLELIST,0,0,vertices[section],0,triangles[section]);
  d->SetStreamSource(0,originalVB,originalOffset,originalStride);d->SetIndices(originalIB);
  originalVB->Release();if(originalIB)originalIB->Release();
  static bool logged[2]{};if(SUCCEEDED(hr)&&!logged[section]){logged[section]=true;Log("menu tank exact gameplay body retarget %d active: %u vertices, %u triangles",section,vertices[section],triangles[section]);}
  return hr;
}

static HRESULT DrawMenuBodySplit(IDirect3DDevice9* d,int section,D3DPRIMITIVETYPE type,INT base){
  if(!EnsureMenuBodySplit(d))return E_FAIL;
  IDirect3DIndexBuffer9* originalIB=nullptr;if(FAILED(d->GetIndices(&originalIB)))return E_FAIL;
  const UINT counts[2][4]={{menuBodyCinematicTriangleCount0,menuBodyBlend33TriangleCount0,menuBodyBlend67TriangleCount0,menuBodyPelvisTriangleCount0},{menuBodyCinematicTriangleCount1,menuBodyBlend33TriangleCount1,menuBodyBlend67TriangleCount1,menuBodyPelvisTriangleCount1}};
  LoadR14SkinTextures(d);LoadMenuBlendTextures(d);IDirect3DBaseTexture9* oldPrimary=nullptr;d->GetTexture(2,&oldPrimary);
  IDirect3DTexture9* chosen=r14SkinTexture[physicsState==0?1:0];
  IDirect3DBaseTexture9* textures[4]={oldPrimary,menuBlendTexture[0],menuBlendTexture[1],chosen};HRESULT hr=D3D_OK;
  for(int pass=0;pass<4;pass++){if(textures[pass])d->SetTexture(2,textures[pass]);HRESULT passHr=d->SetIndices(menuBodySplitIB[section][pass]);if(SUCCEEDED(passHr))passHr=origDIP(d,type,base,0,12082,0,counts[section][pass]);if(FAILED(passHr)&&SUCCEEDED(hr))hr=passHr;}
  d->SetTexture(2,oldPrimary);if(oldPrimary)oldPrimary->Release();
  d->SetIndices(originalIB);originalIB->Release();
  static bool logged[2]{};if(SUCCEEDED(hr)&&!logged[section]){logged[section]=true;Log("menu tank feathered body split %d: cinematic=%u transition=%u/%u pelvis=%u",section,counts[section][0],counts[section][1],counts[section][2],counts[section][3]);}
  return hr;
}

static HRESULT DrawMenuTank(IDirect3DDevice9* d,IDirect3DVertexBuffer9* original,UINT offset,UINT stride){
  IDirect3DIndexBuffer9* originalIB=nullptr;if(FAILED(d->GetIndices(&originalIB)))return E_FAIL;
  LoadR14SkinTextures(d);
  IDirect3DTexture9* chosen=r14SkinTexture[physicsState==0?1:0];
  HRESULT hr=d->SetStreamSource(0,menuTankVB,0,32);if(SUCCEEDED(hr))hr=d->SetIndices(r14IB);
  if(SUCCEEDED(hr)&&captureTankMaterials&&IsHdrSceneColorPass(d))CaptureDraw(d,D3DPT_TRIANGLELIST,0,0,nrCount,0,nrIndexCount/3,menuTankPacked,sizeof(menuTankPacked),nrIndices,sizeof(nrIndices));
  if(SUCCEEDED(hr))hr=DrawSharedSkin(d,D3DPT_TRIANGLELIST,0,0,nrCount,0,nrIndexCount/3,true,chosen);
  d->SetStreamSource(0,original,offset,stride);d->SetIndices(originalIB);if(originalIB)originalIB->Release();
  if(SUCCEEDED(hr)){menuTankDrawnThisFrame=true;if(menuTankSuccessfulDraws++==0)Log("menu tank anatomy draw active; WStart body/electrodes retained");}
  return hr;
}
