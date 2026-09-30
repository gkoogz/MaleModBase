#pragma once
#include "fluid_splat_model.h"
#ifndef SPLAT_SHADER_COMPILER
#include "stain_shader_bytecode.h"
#endif
namespace volumeFluid {
struct StainVertex {V3 p,n,side;float x,y,fade;};
static const char* stainShader=R"HLSL(
float4 VP[4]:register(c0);float4 IP[4]:register(c4);float4 Screen:register(c8);float4 Viewport:register(c9);
float4 Tile:register(c10);
sampler2D Scene:register(s0);sampler2D Deposit:register(s1);
struct O{float4 p:POSITION;float3 world:TEXCOORD0;float3 n:TEXCOORD1;float3 side:TEXCOORD2;float3 shape:TEXCOORD3;float4 clip:TEXCOORD4;};
float3 safe(float3 v){return v*rsqrt(max(dot(v,v),1e-12));}
O Vertex(float3 p:POSITION,float3 n:NORMAL,float3 side:TANGENT,float3 shape:TEXCOORD0){O o;o.p=p.x*VP[0]+p.y*VP[1]+p.z*VP[2]+VP[3];o.world=p;o.n=n;o.side=side;o.shape=shape;o.clip=o.p;return o;}
float4 Pixel(O o):COLOR0{
 float2 localUV=clamp(o.shape.xy+.5,.5/128.,127.5/128.);
 float4 wet=tex2D(Deposit,Tile.xy+localUV*Tile.zw);
 float alpha=wet.a*o.shape.z;clip(alpha-.003);
 float3 base=safe(o.n),x=safe(o.side-base*dot(o.side,base)),y=safe(cross(base,x));
 float3 tangentNormal=safe(wet.rgb*2-1);
 float3 n=safe(x*tangentNormal.x+y*tangentNormal.y+base*tangentNormal.z);
 float2 uv=(o.clip.xy/o.clip.w*float2(.5,-.5)+.5)*Viewport.xy+Viewport.zw;
 float3 scene=Screen.z>.5?max(0,tex2D(Scene,uv).rgb):float3(.45,.45,.45);
 float lum=dot(scene,float3(.2126,.7152,.0722));float illumination=clamp(.08+.92*sqrt(max(0,lum)),.08,1.2);
 float3 tint=lerp(float3(1,1,1),clamp((scene+.08)/(lum+.08),.55,1.5),.5);
 float2 ndc=o.clip.xy/o.clip.w;float4 q=ndc.x*IP[0]+ndc.y*IP[1]+IP[3];float3 eye=safe(q.xyz/q.w-o.world);
 if(dot(n,eye)<0)n=-n;
 float3 key=safe(float3(-.15,.5,.85));float diffuse=.76+.24*saturate(dot(n,key));
 float sheen=pow(saturate(dot(n,safe(key+eye))),95)*.28+pow(saturate(dot(n,safe(float3(-.35,-.5,.8)+eye))),70)*.55;
 float3 color=float3(.965,.96,.95)*tint*illumination*diffuse+sheen*sqrt(illumination);
 return float4(min(color,float3(1.2,1.2,1.18)),alpha);
}
)HLSL";
class StainSurface : public SplatModel {
public:
 IDirect3DVertexShader9* vs=nullptr;IDirect3DPixelShader9* ps=nullptr;IDirect3DVertexDeclaration9* decl=nullptr;
 IDirect3DVertexBuffer9* vb=nullptr;UINT capacity=0;IDirect3DTexture9* scene=nullptr;IDirect3DTexture9* atlas=nullptr;unsigned uploadedSerial[splatMaxMarks]{},uploadedRevision[splatMaxMarks]{};D3DSURFACE_DESC sceneDesc{};
 void Release(){Drop(vs);Drop(ps);Drop(decl);Drop(vb);Drop(scene);Drop(atlas);memset(uploadedSerial,0,sizeof(uploadedSerial));memset(uploadedRevision,0,sizeof(uploadedRevision));capacity=0;sceneDesc={};marks.clear();}
 bool Initialize(IDirect3DDevice9* d){
  project=FluidProjectSplat;resolve=FluidResolveSplat;
  if(!splatBakes.Open())return false;if(vs&&ps&&decl&&atlas)return true;
#ifdef SPLAT_SHADER_COMPILER
  return false; // The build-only compiler below never initializes a device.
#else
  // No HLSL compilation on the game/render thread. Both shaders are compiled
  // by the build tool; Create*Shader only uploads their validated bytecode.
  HRESULT hr=vs?S_OK:d->CreateVertexShader(stainVertexBytecode,&vs);
  if(SUCCEEDED(hr))hr=ps?S_OK:d->CreatePixelShader(stainPixelBytecode,&ps);
  if(FAILED(hr)){Drop(vs);Drop(ps);return false;}
#endif

  D3DVERTEXELEMENT9 elements[]={{0,0,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_POSITION,0},{0,12,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_NORMAL,0},{0,24,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_TANGENT,0},{0,36,D3DDECLTYPE_FLOAT3,0,D3DDECLUSAGE_TEXCOORD,0},D3DDECL_END()};
  if(!decl&&FAILED(d->CreateVertexDeclaration(elements,&decl)))return false;
  if(!atlas&&FAILED(d->CreateTexture(1024,512,1,0,D3DFMT_A8R8G8B8,D3DPOOL_MANAGED,&atlas,nullptr)))return false;
  return true;
 }
 bool CaptureScene(IDirect3DDevice9* d){
  IDirect3DSurface9* target=nullptr;if(FAILED(d->GetRenderTarget(0,&target))||!target)return false;D3DSURFACE_DESC desc{};target->GetDesc(&desc);
  if(!scene||desc.Width!=sceneDesc.Width||desc.Height!=sceneDesc.Height||desc.Format!=sceneDesc.Format){Drop(scene);sceneDesc=desc;d->CreateTexture(desc.Width,desc.Height,1,D3DUSAGE_RENDERTARGET,desc.Format,D3DPOOL_DEFAULT,&scene,nullptr);}
  IDirect3DSurface9* copy=nullptr;bool okay=scene&&SUCCEEDED(scene->GetSurfaceLevel(0,&copy));
  if(okay)okay=SUCCEEDED(d->StretchRect(target,nullptr,copy,nullptr,D3DTEXF_NONE));Drop(copy);Drop(target);return okay;
 }
 void Build(const StainMark& m,std::vector<StainVertex>& vertices,DWORD now){
  float age=DWORD(now-m.touched)*.001f,fade=age<15?1:max(0.f,(20-age)/5);
  V3 origin,normal,xaxis,yaxis;if(!Frame(m,origin,normal,xaxis,yaxis))return;
  auto vertex=[&](int i,StainVertex& out){V3 p,n;if(m.samples[i].state!=1||!Resolve(m.samples[i].anchor,p,n))return false;
   V3 side=Unit(xaxis-n*Dot(xaxis,n));out={p+n*.027f,n,side,Coordinate(i%splatGrid,m.span)/m.span,Coordinate(i/splatGrid,m.span)/m.span,fade};return true;};
  for(int y=m.y0;y<m.y1;y++)for(int x=m.x0;x<m.x1;x++){
   int ids[4]={y*splatGrid+x,y*splatGrid+x+1,(y+1)*splatGrid+x,(y+1)*splatGrid+x+1};
   if(max(max(m.samples[ids[0]].field,m.samples[ids[1]].field),max(m.samples[ids[2]].field,m.samples[ids[3]].field))<=.0001f)continue;
   StainVertex v[4];bool ok[4];for(int j=0;j<4;j++)ok[j]=vertex(ids[j],v[j]);
   const int triangles[6]={0,1,2,1,3,2};for(int j=0;j<6;j+=3){int a=triangles[j],b=triangles[j+1],c=triangles[j+2];
    if(ok[a]&&ok[b]&&ok[c]&&Length(v[a].p-v[b].p)<m.span/(splatGrid-1)*3&&Length(v[a].p-v[c].p)<m.span/(splatGrid-1)*3){vertices.push_back(v[a]);vertices.push_back(v[b]);vertices.push_back(v[c]);}
   }
  }
 }
 HRESULT Draw(IDirect3DDevice9* d,const float* vp,DWORD now){
  if(marks.empty())return S_FALSE;if(!Initialize(d))return E_FAIL;Update(now);
  for(const auto& m:marks){if(uploadedSerial[m.slot]==m.serial&&uploadedRevision[m.slot]==m.revision)continue;
   RECT rect{LONG((m.slot%8)*128),LONG((m.slot/8)*128),LONG((m.slot%8+1)*128),LONG((m.slot/8+1)*128)};D3DLOCKED_RECT lock{};
   HRESULT upload=atlas->LockRect(0,&lock,&rect,0);if(FAILED(upload))return upload;
   for(int y=0;y<128;y++)memcpy((unsigned char*)lock.pBits+y*lock.Pitch,m.pixels.data()+y*128,128*4);
   atlas->UnlockRect(0);uploadedSerial[m.slot]=m.serial;uploadedRevision[m.slot]=m.revision;
  }
  std::vector<StainVertex> vertices;std::vector<UINT> offsets;
  for(const auto& m:marks){offsets.push_back((UINT)vertices.size());Build(m,vertices,now);}offsets.push_back((UINT)vertices.size());
  UINT count=(UINT)vertices.size();if(!count)return S_FALSE;
  if(!vb||capacity<count){Drop(vb);capacity=count+1024;if(FAILED(d->CreateVertexBuffer(capacity*sizeof(StainVertex),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&vb,nullptr)))return E_FAIL;}
  void* raw=nullptr;HRESULT hr=vb->Lock(0,count*sizeof(StainVertex),&raw,D3DLOCK_DISCARD);if(FAILED(hr))return hr;memcpy(raw,vertices.data(),count*sizeof(StainVertex));vb->Unlock();
  D3DXMATRIX matrix,inv;memcpy(&matrix,vp,64);if(!D3DXMatrixInverse(&inv,nullptr,&matrix))return E_FAIL;
  bool sampled=CaptureScene(d);float constants[40]{};memcpy(constants,vp,64);memcpy(constants+16,&inv,64);constants[32]=1.f/max(1u,sceneDesc.Width);constants[33]=1.f/max(1u,sceneDesc.Height);constants[34]=sampled?1.f:0;
  D3DVIEWPORT9 viewport{};d->GetViewport(&viewport);constants[36]=viewport.Width*constants[32];constants[37]=viewport.Height*constants[33];constants[38]=viewport.X*constants[32];constants[39]=viewport.Y*constants[33];
  IDirect3DStateBlock9* saved=nullptr;if(FAILED(d->CreateStateBlock(D3DSBT_ALL,&saved)))return E_FAIL;float bank[1024];if(FAILED(d->GetVertexShaderConstantF(0,bank,256))){Drop(saved);return E_FAIL;}
  IDirect3DSurface9* extra[3]{};for(int i=1;i<4;i++){d->GetRenderTarget(i,&extra[i-1]);d->SetRenderTarget(i,nullptr);}
  d->SetVertexShader(vs);d->SetPixelShader(ps);d->SetVertexDeclaration(decl);d->SetStreamSource(0,vb,0,sizeof(StainVertex));d->SetIndices(nullptr);for(int i=0;i<4;i++)d->SetStreamSourceFreq(i,1);d->SetVertexShaderConstantF(0,constants,10);d->SetPixelShaderConstantF(0,constants,10);d->SetTexture(0,sampled?scene:nullptr);d->SetTexture(1,atlas);
  d->SetSamplerState(0,D3DSAMP_MINFILTER,D3DTEXF_LINEAR);d->SetSamplerState(0,D3DSAMP_MAGFILTER,D3DTEXF_LINEAR);d->SetSamplerState(0,D3DSAMP_MIPFILTER,D3DTEXF_NONE);d->SetSamplerState(0,D3DSAMP_ADDRESSU,D3DTADDRESS_CLAMP);d->SetSamplerState(0,D3DSAMP_ADDRESSV,D3DTADDRESS_CLAMP);d->SetSamplerState(0,D3DSAMP_SRGBTEXTURE,FALSE);
  d->SetSamplerState(1,D3DSAMP_MINFILTER,D3DTEXF_LINEAR);d->SetSamplerState(1,D3DSAMP_MAGFILTER,D3DTEXF_LINEAR);d->SetSamplerState(1,D3DSAMP_MIPFILTER,D3DTEXF_NONE);d->SetSamplerState(1,D3DSAMP_ADDRESSU,D3DTADDRESS_CLAMP);d->SetSamplerState(1,D3DSAMP_ADDRESSV,D3DTADDRESS_CLAMP);d->SetSamplerState(1,D3DSAMP_SRGBTEXTURE,FALSE);
  d->SetRenderState(D3DRS_ZENABLE,TRUE);d->SetRenderState(D3DRS_ZWRITEENABLE,FALSE);d->SetRenderState(D3DRS_ZFUNC,D3DCMP_LESSEQUAL);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);d->SetRenderState(D3DRS_ALPHABLENDENABLE,TRUE);d->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE,FALSE);d->SetRenderState(D3DRS_BLENDOP,D3DBLENDOP_ADD);d->SetRenderState(D3DRS_SRCBLEND,D3DBLEND_SRCALPHA);d->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_INVSRCALPHA);d->SetRenderState(D3DRS_ALPHATESTENABLE,FALSE);d->SetRenderState(D3DRS_SCISSORTESTENABLE,FALSE);d->SetRenderState(D3DRS_STENCILENABLE,FALSE);d->SetRenderState(D3DRS_CLIPPLANEENABLE,0);d->SetRenderState(D3DRS_FILLMODE,D3DFILL_SOLID);d->SetRenderState(D3DRS_SRGBWRITEENABLE,FALSE);d->SetRenderState(D3DRS_COLORWRITEENABLE,7);d->SetRenderState(D3DRS_SLOPESCALEDEPTHBIAS,0);float depthBias=-.00001f;DWORD packedBias;memcpy(&packedBias,&depthBias,4);d->SetRenderState(D3DRS_DEPTHBIAS,packedBias);
  for(size_t i=0;i<marks.size();i++){if(offsets[i+1]==offsets[i])continue;const auto& m=marks[i];
   float tile[4]={float(m.slot%8)*.125f,float(m.slot/8)*.25f,.125f,.25f};
   d->SetPixelShaderConstantF(10,tile,1);hr=d->DrawPrimitive(D3DPT_TRIANGLELIST,offsets[i],(offsets[i+1]-offsets[i])/3);if(FAILED(hr))break;
  }
  for(int i=1;i<4;i++){d->SetRenderTarget(i,extra[i-1]);Drop(extra[i-1]);}saved->Apply();d->SetVertexShaderConstantF(0,bank,256);Drop(saved);return hr;
 }
};
}
