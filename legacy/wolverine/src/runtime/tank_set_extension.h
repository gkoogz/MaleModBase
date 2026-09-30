#pragma once
// A tiny authored extension for camera poses that see beyond WStart's partial
// tank set.  Five static faces close the far end of the set; no clears,
// postprocess buffers, screen-space masks, or per-pixel scene copies are used.
struct TankSetVertex {float x,y,z;};
static IDirect3DVertexShader9* tankSetVS;
static IDirect3DPixelShader9* tankSetPS;
static IDirect3DVertexDeclaration9* tankSetDecl;
static IDirect3DVertexBuffer9* tankSetVB;
static IDirect3DIndexBuffer9* tankSetIB;
static LONG tankSetDrawSerial=-2;

static void ReleaseTankSetExtension(){
  if(tankSetVS){tankSetVS->Release();tankSetVS=nullptr;}
  if(tankSetPS){tankSetPS->Release();tankSetPS=nullptr;}
  if(tankSetDecl){tankSetDecl->Release();tankSetDecl=nullptr;}
  if(tankSetVB){tankSetVB->Release();tankSetVB=nullptr;}
  if(tankSetIB){tankSetIB->Release();tankSetIB=nullptr;}
  tankSetDrawSerial=-2;
}

static bool EnsureTankSetExtension(IDirect3DDevice9* d){
  if(tankSetVS&&tankSetPS&&tankSetDecl&&tankSetVB&&tankSetIB)return true;
  ReleaseTankSetExtension();
  const char* shader=R"HLSL(
float4 VP[4]:register(c0);
struct O{float4 p:POSITION;float3 w:TEXCOORD0;};
O Vertex(float3 p:POSITION){O o;o.p=p.x*VP[0]+p.y*VP[1]+p.z*VP[2]+VP[3];o.w=p;return o;}
float4 Pixel(O o):COLOR0{
  float height=saturate((o.w.z+120.0)/440.0);
  float side=saturate(abs(o.w.y)/360.0);
  float3 tank=float3(.026,.075,.145)*(0.74+0.20*height+0.06*side);
  return float4(tank,1);
})HLSL";
  for(int pixel=0;pixel<2;pixel++){
    ID3DXBuffer *code=nullptr,*errors=nullptr;
    HRESULT hr=D3DXCompileShader(shader,(UINT)strlen(shader),nullptr,nullptr,pixel?"Pixel":"Vertex",pixel?"ps_3_0":"vs_3_0",D3DXSHADER_OPTIMIZATION_LEVEL3,&code,&errors,nullptr);
    if(errors)errors->Release();
    if(FAILED(hr)||!code){if(code)code->Release();ReleaseTankSetExtension();return false;}
    hr=pixel?d->CreatePixelShader((DWORD*)code->GetBufferPointer(),&tankSetPS):d->CreateVertexShader((DWORD*)code->GetBufferPointer(),&tankSetVS);
    code->Release();if(FAILED(hr)){ReleaseTankSetExtension();return false;}
  }
  D3DVERTEXELEMENT9 elements[]={{0,0,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_POSITION,0},D3DDECL_END()};
  if(FAILED(d->CreateVertexDeclaration(elements,&tankSetDecl))){ReleaseTankSetExtension();return false;}
  // Open toward the camera (-X). Far wall, floor, ceiling and both side walls
  // overlap beyond their edges so the widest saved FOV cannot reveal a crack.
  static const TankSetVertex vertices[]={
    {230,-360,-120},{230,360,-120},{230,360,320},{230,-360,320},
    {-190,-360,-120},{230,-360,-120},{230,360,-120},{-190,360,-120},
    {-190,-360,320},{-190,360,320},{230,360,320},{230,-360,320},
    {-190,-360,-120},{-190,-360,320},{230,-360,320},{230,-360,-120},
    {-190,360,-120},{230,360,-120},{230,360,320},{-190,360,320}
  };
  static const WORD indices[]={
    0,1,2,0,2,3, 4,5,6,4,6,7, 8,9,10,8,10,11,
    12,13,14,12,14,15, 16,17,18,16,18,19
  };
  if(FAILED(d->CreateVertexBuffer(sizeof(vertices),D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&tankSetVB,nullptr))||
     FAILED(d->CreateIndexBuffer(sizeof(indices),D3DUSAGE_WRITEONLY,D3DFMT_INDEX16,D3DPOOL_DEFAULT,&tankSetIB,nullptr))){ReleaseTankSetExtension();return false;}
  void* raw=nullptr;if(FAILED(tankSetVB->Lock(0,0,&raw,0))){ReleaseTankSetExtension();return false;}memcpy(raw,vertices,sizeof(vertices));tankSetVB->Unlock();
  if(FAILED(tankSetIB->Lock(0,0,&raw,0))){ReleaseTankSetExtension();return false;}memcpy(raw,indices,sizeof(indices));tankSetIB->Unlock();
  return true;
}

static void DrawTankSetExtension(IDirect3DDevice9* d){
  if(tankSetDrawSerial==renderFrameSerial)return;
  ShaderLayout* layout=GetShaderLayout(d);if(!layout||!layout->valid||!layout->viewValid||!EnsureTankSetExtension(d))return;
  float local[16],view[16];
  if(FAILED(d->GetVertexShaderConstantF(layout->localRegister,local,4))||FAILED(d->GetVertexShaderConstantF(layout->viewRegister,view,4)))return;
  D3DXMATRIX l,v,clip;memcpy(&l,local,sizeof(l));memcpy(&v,view,sizeof(v));D3DXMatrixMultiply(&clip,&l,&v);
  IDirect3DStateBlock9* saved=nullptr;if(FAILED(d->CreateStateBlock(D3DSBT_ALL,&saved)))return;
  float oldConstants[16];if(FAILED(d->GetVertexShaderConstantF(0,oldConstants,4))){saved->Release();return;}
  IDirect3DSurface9* extra[3]{};for(int i=1;i<4;i++){d->GetRenderTarget(i,&extra[i-1]);d->SetRenderTarget(i,nullptr);}
  d->SetVertexShader(tankSetVS);d->SetPixelShader(tankSetPS);d->SetVertexDeclaration(tankSetDecl);d->SetStreamSource(0,tankSetVB,0,sizeof(TankSetVertex));d->SetIndices(tankSetIB);d->SetVertexShaderConstantF(0,(float*)&clip,4);
  d->SetRenderState(D3DRS_ZENABLE,TRUE);d->SetRenderState(D3DRS_ZWRITEENABLE,TRUE);d->SetRenderState(D3DRS_ZFUNC,D3DCMP_LESSEQUAL);d->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);d->SetRenderState(D3DRS_ALPHABLENDENABLE,FALSE);d->SetRenderState(D3DRS_ALPHATESTENABLE,FALSE);d->SetRenderState(D3DRS_STENCILENABLE,FALSE);d->SetRenderState(D3DRS_SCISSORTESTENABLE,FALSE);d->SetRenderState(D3DRS_COLORWRITEENABLE,7);
  HRESULT hr=d->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,0,0,20,0,10);
  for(int i=1;i<4;i++){d->SetRenderTarget(i,extra[i-1]);if(extra[i-1])extra[i-1]->Release();}
  saved->Apply();d->SetVertexShaderConstantF(0,oldConstants,4);saved->Release();
  if(SUCCEEDED(hr)){tankSetDrawSerial=renderFrameSerial;static bool logged=false;if(!logged){logged=true;Log("menu tank set extension active: 20 vertices, 10 triangles, HDR geometry pass");}}
}
