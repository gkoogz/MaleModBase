#pragma once
// MIT. Indexed D3D9 liquid mesh, one opaque and one transparent material pass.
namespace volumeFluid {
static const char* liquidShader=R"HLSL(
float4 VP[4]:register(c0);float4 IP[4]:register(c4);float4 Material:register(c8);
struct O {float4 p:POSITION;float3 world:TEXCOORD0;float3 normal:TEXCOORD1;float4 clip:TEXCOORD2;};
float3 SafeUnit(float3 v){return v*rsqrt(max(dot(v,v),1e-12));}
O Vertex(float3 p:POSITION,float3 n:NORMAL){O o;o.p=p.x*VP[0]+p.y*VP[1]+p.z*VP[2]+VP[3];o.world=p;o.normal=n;o.clip=o.p;return o;}
float4 Pixel(O o):COLOR0{
 float2 ndc=o.clip.xy/o.clip.w;float4 nearPoint=ndc.x*IP[0]+ndc.y*IP[1]+IP[3];float3 eye=SafeUnit(nearPoint.xyz/nearPoint.w-o.world);
 float3 n=SafeUnit(o.normal);if(dot(n,eye)<0)n=-n;
 float3 key=normalize(float3(-.35,-.65,.8)),fill=normalize(float3(.6,.3,.6));
 float diffuse=.62+.24*max(0,dot(n,key))+.10*max(0,dot(n,fill));
 float broad=pow(saturate(dot(n,SafeUnit(key+eye))),18)*.23;
 float wet=pow(saturate(dot(n,SafeUnit(key+eye))),90)*.32;
 float edge=pow(1-saturate(dot(n,eye)),5);
 float tank=step(.5,Material.y);
 float3 base=lerp(float3(.97,.97,.97),float3(.94,.91,.84),tank);
 // Gentle shading and restrained highlights retain shape without HDR glare.
 float opaqueLight=lerp(diffuse,.80+.16*diffuse,tank);
 float3 color=base*opaqueLight+broad*(1-.80*tank)+wet*(1-.90*tank)+.035*edge*(1-.6*tank);
 if(tank>.5)color=min(color,float3(.98,.97,.94));
 // PC UE3 Common.usf EncodeFloatW stores projected device depth in scene
 // color alpha. One is far-plane depth, NOT an opaque/compositor mask.
 float alpha=tank>.5?saturate(o.clip.z/o.clip.w):1;
 if(Material.x>.5){color=(Material.y>.5?float3(.86,.84,.79):float3(.79,.79,.79))+broad*(1-.6*tank)+wet*(1-.75*tank);alpha=saturate(.05+.27*edge+wet*.75+broad*.2);}
 return float4(color,alpha);
}
)HLSL";
class Surface {
public:
 IDirect3DVertexShader9* vs=nullptr;IDirect3DPixelShader9* ps=nullptr;IDirect3DVertexDeclaration9* decl=nullptr;
 IDirect3DVertexBuffer9* vertices=nullptr;IDirect3DIndexBuffer9* indices=nullptr;UINT vertexCapacity=0,indexCapacity=0;std::string error;
 void Release(){Drop(vs);Drop(ps);Drop(decl);Drop(vertices);Drop(indices);vertexCapacity=indexCapacity=0;}
 bool Fail(const char* where,HRESULT hr){char s[180];sprintf_s(s,"%s 0x%08X",where,(unsigned)hr);error=s;return false;}
 bool Initialize(IDirect3DDevice9* d){
  if(vs&&ps&&decl)return true;Release();
  for(int i=0;i<2;i++){ID3DXBuffer *code=nullptr,*err=nullptr;HRESULT hr=D3DXCompileShader(liquidShader,(UINT)strlen(liquidShader),nullptr,nullptr,i?"Pixel":"Vertex",i?"ps_3_0":"vs_3_0",D3DXSHADER_OPTIMIZATION_LEVEL3,&code,&err,nullptr);if(FAILED(hr)){if(err)error.assign((char*)err->GetBufferPointer(),err->GetBufferSize());Drop(err);Drop(code);return false;}Drop(err);hr=i?d->CreatePixelShader((DWORD*)code->GetBufferPointer(),&ps):d->CreateVertexShader((DWORD*)code->GetBufferPointer(),&vs);Drop(code);if(FAILED(hr))return Fail("liquid shader",hr);}
  D3DVERTEXELEMENT9 elements[]={{0,0,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_POSITION,0},{0,12,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_NORMAL,0},D3DDECL_END()};HRESULT hr=d->CreateVertexDeclaration(elements,&decl);return SUCCEEDED(hr)||Fail("liquid declaration",hr);
 }
 HRESULT Draw(IDirect3DDevice9* d,const teaching::LiquidMesh& mesh,const float* vp,bool tankScene=false){
  if(mesh.indices.empty())return S_FALSE;if(!Initialize(d))return E_FAIL;HRESULT hr;
  UINT vc=(UINT)mesh.vertices.size(),ic=(UINT)mesh.indices.size();
  if(!vertices||vertexCapacity<vc){Drop(vertices);vertexCapacity=vc+4096;hr=d->CreateVertexBuffer(vertexCapacity*sizeof(teaching::LiquidVertex),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&vertices,nullptr);if(FAILED(hr)){Fail("mesh vertex buffer",hr);return hr;}}
  if(!indices||indexCapacity<ic){Drop(indices);indexCapacity=ic+8192;hr=d->CreateIndexBuffer(indexCapacity*sizeof(unsigned),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,D3DFMT_INDEX32,D3DPOOL_DEFAULT,&indices,nullptr);if(FAILED(hr)){Fail("mesh index buffer",hr);return hr;}}
  void* raw=nullptr;hr=vertices->Lock(0,vc*sizeof(teaching::LiquidVertex),&raw,D3DLOCK_DISCARD);if(FAILED(hr))return hr;memcpy(raw,mesh.vertices.data(),vc*sizeof(teaching::LiquidVertex));vertices->Unlock();
  hr=indices->Lock(0,ic*sizeof(unsigned),&raw,D3DLOCK_DISCARD);if(FAILED(hr))return hr;memcpy(raw,mesh.indices.data(),ic*sizeof(unsigned));indices->Unlock();
  D3DXMATRIX matrix,inv;memcpy(&matrix,vp,64);if(!D3DXMatrixInverse(&inv,nullptr,&matrix))return E_FAIL;
  float c[36]{};memcpy(c,vp,64);memcpy(c+16,&inv,64);c[33]=tankScene?1.f:0.f;
  IDirect3DStateBlock9* saved=nullptr;hr=d->CreateStateBlock(D3DSBT_ALL,&saved);if(FAILED(hr))return hr;
  float bank[1024];hr=d->GetVertexShaderConstantF(0,bank,256);if(FAILED(hr)){Drop(saved);return hr;}
  IDirect3DSurface9* rt[3]{};for(int i=1;i<4;i++){d->GetRenderTarget(i,&rt[i-1]);d->SetRenderTarget(i,nullptr);}
  d->SetVertexShader(vs);d->SetPixelShader(ps);d->SetVertexDeclaration(decl);d->SetStreamSource(0,vertices,0,sizeof(teaching::LiquidVertex));d->SetIndices(indices);for(int i=0;i<4;i++)d->SetStreamSourceFreq(i,1);
  d->SetVertexShaderConstantF(0,c,9);d->SetPixelShaderConstantF(0,c,9);
  d->SetRenderState(D3DRS_ZENABLE,TRUE);d->SetRenderState(D3DRS_ZWRITEENABLE,TRUE);d->SetRenderState(D3DRS_ZFUNC,D3DCMP_LESSEQUAL);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);d->SetRenderState(D3DRS_ALPHABLENDENABLE,FALSE);d->SetRenderState(D3DRS_ALPHATESTENABLE,FALSE);d->SetRenderState(D3DRS_STENCILENABLE,FALSE);d->SetRenderState(D3DRS_SCISSORTESTENABLE,FALSE);d->SetRenderState(D3DRS_CLIPPLANEENABLE,0);d->SetRenderState(D3DRS_SRGBWRITEENABLE,FALSE);
  // Opaque tank fluid writes its own projected scene depth in alpha. Clear
  // fluid blends RGB only and preserves destination depth, like translucency.
  d->SetRenderState(D3DRS_COLORWRITEENABLE,0xF);d->SetRenderState(D3DRS_FILLMODE,D3DFILL_SOLID);d->SetRenderState(D3DRS_DEPTHBIAS,0);d->SetRenderState(D3DRS_SLOPESCALEDEPTHBIAS,0);
  if(mesh.opaqueIndices)hr=d->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,0,0,vc,0,mesh.opaqueIndices/3);else hr=S_OK;
  if(SUCCEEDED(hr)&&ic>mesh.opaqueIndices){c[32]=1;d->SetPixelShaderConstantF(0,c,9);d->SetRenderState(D3DRS_ZWRITEENABLE,FALSE);d->SetRenderState(D3DRS_COLORWRITEENABLE,7);d->SetRenderState(D3DRS_ALPHABLENDENABLE,TRUE);d->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE,FALSE);d->SetRenderState(D3DRS_BLENDOP,D3DBLENDOP_ADD);d->SetRenderState(D3DRS_SRCBLEND,D3DBLEND_SRCALPHA);d->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_INVSRCALPHA);
   // Front surfaces only avoid doubled opacity on a closed transparent tube.
   d->SetRenderState(D3DRS_CULLMODE,D3DCULL_CW);hr=d->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,0,0,vc,mesh.opaqueIndices,(ic-mesh.opaqueIndices)/3);
  }
  for(int i=1;i<4;i++){d->SetRenderTarget(i,rt[i-1]);Drop(rt[i-1]);}saved->Apply();d->SetVertexShaderConstantF(0,bank,256);Drop(saved);return hr;
 }
};
}
