#include "skin_basis_shaders.h"
// Correct only the demonstrated shader pair, only while rendering R14.
// The skin lobe retains its original strength/exponent and sampled normal.
// Carry all three transformed basis vectors to the pixel shader instead of
// reconstructing a missing row using interpolated tangent handedness.
static bool skinBasisEnabled=true;
static IDirect3DVertexShader9* skinBasisVS=nullptr;
static IDirect3DPixelShader9* skinBasisPS=nullptr;
struct SkinPairMatch {IDirect3DVertexShader9* vs;IDirect3DPixelShader9* ps;bool matched;};
static std::vector<SkinPairMatch> skinPairs;
static UINT skinBasisDraws=0;
template<class T,size_t N> static bool SkinShaderMatches(T* shader,const DWORD (&expected)[N]){
 if(!shader)return false;UINT bytes=0;
 if(FAILED(shader->GetFunction(nullptr,&bytes))||bytes!=sizeof(expected))return false;
 std::vector<DWORD> code(N);return SUCCEEDED(shader->GetFunction(code.data(),&bytes))&&memcmp(code.data(),expected,bytes)==0;
}
static void ReleaseSkinBasis(){
 for(auto& pair:skinPairs){pair.vs->Release();pair.ps->Release();}skinPairs.clear();
 if(skinBasisVS){skinBasisVS->Release();skinBasisVS=nullptr;}
 if(skinBasisPS){skinBasisPS->Release();skinBasisPS=nullptr;}
}
static bool MatchSkinPair(IDirect3DVertexShader9* vs,IDirect3DPixelShader9* ps){
 if(!vs||!ps)return false;
 for(auto& pair:skinPairs)if(pair.vs==vs&&pair.ps==ps)return pair.matched;
 bool match=SkinShaderMatches(vs,skinOriginalVS)&&SkinShaderMatches(ps,skinOriginalPS);
 if(skinPairs.size()<64){vs->AddRef();ps->AddRef();skinPairs.push_back({vs,ps,match});Log("R14 skin basis shader pair %s",match?"MATCHED":"unmatched; original retained");}
 return match;
}
static HRESULT DrawWithSkinBasis(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count){
 if(!skinBasisEnabled)return origDIP(d,type,base,minv,nv,start,count);
 IDirect3DVertexShader9* vs=nullptr;IDirect3DPixelShader9* ps=nullptr;d->GetVertexShader(&vs);d->GetPixelShader(&ps);
 bool ready=MatchSkinPair(vs,ps);
 if(ready&&!skinBasisVS)ready=SUCCEEDED(d->CreateVertexShader(skinFixedVS,&skinBasisVS));
 if(ready&&!skinBasisPS)ready=SUCCEEDED(d->CreatePixelShader(skinFixedPS,&skinBasisPS));
 bool applied=false;
 if(ready){
  HRESULT a=d->SetVertexShader(skinBasisVS),b=SUCCEEDED(a)?d->SetPixelShader(skinBasisPS):E_FAIL;
  applied=SUCCEEDED(a)&&SUCCEEDED(b);
  if(!applied){d->SetVertexShader(vs);d->SetPixelShader(ps);}
 }
 HRESULT result=origDIP(d,type,base,minv,nv,start,count);
 if(applied){d->SetVertexShader(vs);d->SetPixelShader(ps);if(skinBasisDraws++==0)Log("R14 skin basis correction ACTIVE; original lighting constants retained");}
 if(vs)vs->Release();if(ps)ps->Release();return result;
}
