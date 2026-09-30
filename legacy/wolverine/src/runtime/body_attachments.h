#pragma once
#include "electrode_bindings_data.h"
// Rigid pieces follow a barycentric body anchor and its changing tangent frame.
// Native compact bone weights, UVs, material and animation remain authored.
struct BodyAttachmentFrame {V3 p,x,y,z;bool valid;};
static BodyAttachmentFrame ElectrodeFrame(unsigned k,bool rest){
 const auto& b=electrodeBindings[k];V3 p[3];
 for(int i=0;i<3;i++){int s;UINT local;if(!MenuRetargetBodyVertex(b.vertex[i],s,local))return {};memcpy(&p[i],(rest?sharedBodyBase[s]:sharedBodyOutput[s])+local*32,12);}
 V3 x=p[1]-p[0],z=Cross(x,p[2]-p[0]);if(Length(x)<1e-5f||Length(z)<1e-5f)return {};
 x=Unit(x);z=Unit(z);return {p[0]*b.bary[0]+p[1]*b.bary[1]+p[2]*b.bary[2],x,Unit(Cross(z,x)),z,true};
}
static V3 TransportAttachmentVector(V3 v,const BodyAttachmentFrame& a,const BodyAttachmentFrame& b){return b.x*Dot(v,a.x)+b.y*Dot(v,a.y)+b.z*Dot(v,a.z);}
static void DeformElectrodes(unsigned char* dst,const unsigned char* authored){
 EnsureSharedBody();BodyAttachmentFrame rest[16],now[16];bool changed[16]{};
 for(unsigned k=0;k<16;k++){rest[k]=ElectrodeFrame(k,true);now[k]=ElectrodeFrame(k,false);changed[k]=rest[k].valid&&now[k].valid&&(Length(rest[k].p-now[k].p)>1e-6f||Length(rest[k].x-now[k].x)>1e-6f||Length(rest[k].z-now[k].z)>1e-6f);}
 for(unsigned i=0;i<2162;i++){
  const unsigned char* src=authored+(9920+i)*32;unsigned char* out=dst+(9920+i)*32;memcpy(out,src,32);unsigned k=electrodeMembership[i];if(!changed[k])continue;
  V3 p;memcpy(&p,src,12);p=now[k].p+TransportAttachmentVector(p-rest[k].p,rest[k],now[k]);memcpy(out,&p,12);
  for(int offset:{12,16}){V3 v={(src[offset]-127.5f)/127.5f,(src[offset+1]-127.5f)/127.5f,(src[offset+2]-127.5f)/127.5f};v=Unit(TransportAttachmentVector(v,rest[k],now[k]));out[offset]=PackSigned(v.x);out[offset+1]=PackSigned(v.y);out[offset+2]=PackSigned(v.z);}
 }
}
static IDirect3DVertexBuffer9* electrodeVB=nullptr;
static unsigned electrodeRevision=~0u;
static unsigned char electrodeSource[12082*32];
static void ReleaseBodyAttachments(){if(electrodeVB){electrodeVB->Release();electrodeVB=nullptr;}electrodeRevision=~0u;}
static HRESULT DrawBodyAttachments(IDirect3DDevice9* d,IDirect3DVertexBuffer9* source,UINT offset,UINT stride,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count){
 if(!electrodeVB){
  void* raw=nullptr;if(FAILED(source->Lock(0,sizeof(electrodeSource),&raw,D3DLOCK_READONLY)))return origDIP(d,type,base,minv,nv,start,count);
  memcpy(electrodeSource,raw,sizeof(electrodeSource));source->Unlock();
  if(FAILED(d->CreateVertexBuffer(sizeof(electrodeSource),D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&electrodeVB,nullptr)))return origDIP(d,type,base,minv,nv,start,count);
 }
 if(electrodeRevision!=sharedBodyRevision){
  void* raw=nullptr;if(FAILED(electrodeVB->Lock(0,sizeof(electrodeSource),&raw,D3DLOCK_DISCARD)))return origDIP(d,type,base,minv,nv,start,count);
  memcpy(raw,electrodeSource,sizeof(electrodeSource));DeformElectrodes((unsigned char*)raw,electrodeSource);electrodeVB->Unlock();electrodeRevision=sharedBodyRevision;
 }
 IDirect3DVertexBuffer9* previous=nullptr;UINT prevOffset,prevStride;if(FAILED(d->GetStreamSource(0,&previous,&prevOffset,&prevStride)))return origDIP(d,type,base,minv,nv,start,count);
 HRESULT hr=d->SetStreamSource(0,electrodeVB,offset,stride);if(SUCCEEDED(hr))hr=origDIP(d,type,base,minv,nv,start,count);
 d->SetStreamSource(0,previous,prevOffset,prevStride);if(previous)previous->Release();return hr;
}
