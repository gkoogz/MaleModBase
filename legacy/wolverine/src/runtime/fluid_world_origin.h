#pragma once
// Provenance: diagnostic v2 distinguishes Wolverine's baked BSP vertices
// (36-byte position/tangent/normal/UV/lightmap-UV stream) from static meshes
// (12-byte position stream) and skinned meshes (32 bytes). BSP positions are
// already absolute; its identity LocalToWorld basis adds only pre-view shift.
struct FluidWorldDeclaration {IDirect3DVertexDeclaration9* declaration;bool bsp;};
static std::vector<FluidWorldDeclaration> fluidWorldDeclarations;
struct FluidWorldReference {V3 p,n;};
static std::vector<FluidWorldReference> fluidWorldReferences;
static LONG fluidWorldCameraFrame=-2,fluidWorldReadFrame=-2;
static unsigned fluidWorldReadCount=0;static bool fluidWorldOriginLogged=false;
struct FluidWorldReadKey {void* buffer;UINT start,count;};static std::vector<FluidWorldReadKey> fluidWorldReadKeys;
static bool FluidBspDeclaration(const D3DVERTEXELEMENT9* e,UINT count){
 bool position=false,tangent=false,normal=false,uv=false,lightmap=false;
 for(UINT i=0;i<count&&e[i].Stream!=0xff;i++){
  if(e[i].Usage==D3DDECLUSAGE_BLENDINDICES||e[i].Usage==D3DDECLUSAGE_BLENDWEIGHT)return false;
  if(e[i].Stream!=0)continue;
  position|=e[i].Offset==0&&e[i].Type==D3DDECLTYPE_FLOAT3&&e[i].Usage==D3DDECLUSAGE_POSITION;
  tangent|=e[i].Offset==12&&e[i].Type==D3DDECLTYPE_UBYTE4&&e[i].Usage==D3DDECLUSAGE_TANGENT;
  normal|=e[i].Offset==16&&e[i].Type==D3DDECLTYPE_UBYTE4&&e[i].Usage==D3DDECLUSAGE_NORMAL;
  uv|=e[i].Offset==20&&e[i].Type==D3DDECLTYPE_FLOAT2&&e[i].Usage==D3DDECLUSAGE_TEXCOORD;
  lightmap|=e[i].Offset==28&&e[i].Type==D3DDECLTYPE_FLOAT2&&e[i].Usage==D3DDECLUSAGE_COLOR;
 }return position&&tangent&&normal&&uv&&lightmap;
}
static bool FluidBspCamera(const float* local,V3& camera){
 for(int i=0;i<16;i++){if(!_finite(local[i]))return false;if(i>=12&&i<=14)continue;
  float expected=i%5==0?1.f:0.f;if(fabsf(local[i]-expected)>1e-5f)return false;
 }camera={-local[12],-local[13],-local[14]};return true;
}
static void CaptureFluidWorldGeometry(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count,IDirect3DVertexBuffer9* vb,UINT offset,UINT stride){
 if(TankCameraSceneActive()||type!=D3DPT_TRIANGLELIST||!vb||stride!=36)return;
 ShaderLayout* layout=GetShaderLayout(d);if(!layout||layout->boneCount||layout->localCount!=4||!layout->viewValid)return;
 float local[16],view[16];if(FAILED(d->GetVertexShaderConstantF(layout->localRegister,local,4))||FAILED(d->GetVertexShaderConstantF(layout->viewRegister,view,4)))return;
 // Reject shadow/light projections, which use another origin and viewport.
 D3DVIEWPORT9 vp{};if(FAILED(d->GetViewport(&vp))||vp.X||vp.Y||vp.Width<640||vp.Height<360||fabsf(view[15])>.05f)return;
 IDirect3DVertexDeclaration9* decl=nullptr;if(FAILED(d->GetVertexDeclaration(&decl))||!decl)return;
 bool bsp=false,known=false;for(const auto& entry:fluidWorldDeclarations)if(entry.declaration==decl){bsp=entry.bsp;known=true;break;}
 if(!known){D3DVERTEXELEMENT9 e[MAXD3DDECLLENGTH+1];UINT n=MAXD3DDECLLENGTH+1;bsp=SUCCEEDED(decl->GetDeclaration(e,&n))&&FluidBspDeclaration(e,n);
  if(fluidWorldDeclarations.size()<128){decl->AddRef();fluidWorldDeclarations.push_back({decl,bsp});}}
 decl->Release();V3 camera;if(!bsp||!FluidBspCamera(local,camera))return;
 fluidCameraWorld=camera;fluidCameraTick=GetTickCount();fluidWorldCameraFrame=renderFrameSerial;
 if(!fluidWorldOriginLogged){fluidWorldOriginLogged=true;Log("fluid absolute origin from baked BSP: camera=(%.3f %.3f %.3f)",camera.x,camera.y,camera.z);}
 // A few rendered, upward-facing triangles verify the physics unit conversion.
 // Read at most one bounded buffer range per frame and 64 ranges per scene.
 if(fluidWorldReferences.size()>=24||fluidWorldReadCount>=64||fluidWorldReadFrame==renderFrameSerial||!count||count>100000||!nv||nv>100000)return;
 for(const auto& key:fluidWorldReadKeys)if(key.buffer==vb&&key.start==start&&key.count==count)return;
 fluidWorldReadKeys.push_back({vb,start,count});
 fluidWorldReadFrame=renderFrameSerial;++fluidWorldReadCount;
 IDirect3DIndexBuffer9* ib=nullptr;if(FAILED(d->GetIndices(&ib))||!ib)return;
 D3DINDEXBUFFER_DESC id{};D3DVERTEXBUFFER_DESC vd{};
 if(FAILED(ib->GetDesc(&id))||FAILED(vb->GetDesc(&vd))||(id.Format!=D3DFMT_INDEX16&&id.Format!=D3DFMT_INDEX32)){ib->Release();return;}
 UINT indexSize=id.Format==D3DFMT_INDEX16?2:4;unsigned long long first=(unsigned long long)start*indexSize,bytes=(unsigned long long)count*3*indexSize;
 long long firstVertex=(long long)base+minv,vertexAt=(long long)offset+firstVertex*stride;unsigned long long vertexBytes=(unsigned long long)nv*stride;
 if(first+bytes>id.Size||firstVertex<0||vertexAt<0||vertexAt+vertexBytes>vd.Size||vertexBytes>4*1024*1024){ib->Release();return;}
 void *indices=nullptr,*vertices=nullptr;if(FAILED(ib->Lock((UINT)first,(UINT)bytes,&indices,D3DLOCK_READONLY))){ib->Release();return;}
 if(SUCCEEDED(vb->Lock((UINT)vertexAt,(UINT)vertexBytes,&vertices,D3DLOCK_READONLY))){
  size_t before=fluidWorldReferences.size();
  for(UINT sample=0;sample<32&&sample<count&&fluidWorldReferences.size()<24;sample++){
   UINT triangle=sample*(count-1)/min(31u,count-1?count-1:1u);V3 p[3];bool valid=true;
   for(UINT j=0;j<3;j++){UINT index=indexSize==2?((unsigned short*)indices)[triangle*3+j]:((UINT*)indices)[triangle*3+j];if(index<minv||index-minv>=nv){valid=false;break;}memcpy(&p[j],(unsigned char*)vertices+(index-minv)*stride,12);}
   if(!valid)continue;V3 n=Cross(p[1]-p[0],p[2]-p[0]);if(Length(n)<4)continue;n=Unit(n);if(fabsf(n.z)<.65f)continue;if(n.z<0)n=n*-1;
   V3 center=(p[0]+p[1]+p[2])/3.f;if(!std::isfinite(center.x)||!std::isfinite(center.y)||!std::isfinite(center.z)||Length(center-camera)>1800)continue;
   bool duplicate=false;for(const auto& ref:fluidWorldReferences)if(Length(ref.p-center)<2)duplicate=true;
   if(!duplicate)fluidWorldReferences.push_back({center,n});
  }vb->Unlock();if(fluidWorldReferences.size()!=before)Log("fluid BSP calibration references: %u from %u bounded reads",(unsigned)fluidWorldReferences.size(),fluidWorldReadCount);
 }ib->Unlock();ib->Release();
}
static void ResetFluidWorldOrigin(){
 for(auto& entry:fluidWorldDeclarations)entry.declaration->Release();fluidWorldDeclarations.clear();fluidWorldReferences.clear();
 fluidWorldReadKeys.clear();fluidWorldCameraFrame=fluidWorldReadFrame=-2;fluidWorldReadCount=0;fluidWorldOriginLogged=false;
}
