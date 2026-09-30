#pragma once
// New fluid subsystem. MIT license, see FLUID-LICENSE.txt. Windows 10 / D3D11.
#include <d3d11.h>
#include <d3dcompiler.h>
#include <algorithm>
#include <string>
#include "fluid_types.h"
namespace volumeFluid {
struct Buffer {
 ID3D11Buffer* b=nullptr;ID3D11ShaderResourceView* srv=nullptr;ID3D11UnorderedAccessView* uav=nullptr;
 void Release(){Drop(srv);Drop(uav);Drop(b);}
};
static const char* computeSource=R"HLSL(
struct Particle {float3 p;float live;float3 v;float age;};
cbuffer Params:register(b0){uint count;uint hashMask;float dt;float spacing;float h;float particleVolume;float viscosity;float tension;float3 gravity;float lifetime;float3 origin;float floorZ;float3 tip;float maxSpeed;float3 direction;float enableFloor;float4 collider[8];uint useCache;float3 cachePadding;};
StructuredBuffer<Particle> particles:register(t0);
StructuredBuffer<float4> positions:register(t1);
StructuredBuffer<float4> auxiliary:register(t2);
StructuredBuffer<uint> heads:register(t3);
StructuredBuffer<uint> links:register(t4);
StructuredBuffer<float4> velocity:register(t5);
StructuredBuffer<float4> baseVelocity:register(t6);
RWStructuredBuffer<float4> output:register(u0);
RWStructuredBuffer<float4> auxOut:register(u1);
RWStructuredBuffer<uint> headOut:register(u2);
RWStructuredBuffer<uint> linkOut:register(u3);
RWStructuredBuffer<Particle> particleOut:register(u4);
struct Neighbor{float3 n;float beta;uint index;};
struct Matrix{float4 row0,row1,row2;uint size;float3 padding;};
StructuredBuffer<Neighbor> neighbors:register(t7);StructuredBuffer<Matrix> matrices:register(t8);
RWStructuredBuffer<Neighbor> neighborOut:register(u5);RWStructuredBuffer<Matrix> matrixOut:register(u6);
uint key(int3 c){return ((uint(c.x)*73856093u)^(uint(c.y)*19349663u)^(uint(c.z)*83492791u))&hashMask;}
float W(float r){float q=r/h;float a=max(0,1-q*q);return 1.56668147*a*a*a/(h*h*h);}
float3 grad(float3 r){float l=length(r);float a=max(0,1-l/h);return l>1e-6 ? r*(-14.3239449*a*a/(h*h*h*h*l)):0;}
float3 contact(float3 p){
 if(enableFloor>0.5)p.z=max(p.z,floorZ+spacing*.45);
 [unroll]for(uint k=0;k<8;k++){float r=collider[k].w+spacing*.4;float3 delta=p-collider[k].xyz;float l=length(delta);if(collider[k].w>0 && l<r && l>1e-6)p=collider[k].xyz+delta*(r/l);}
 return p;
}
[numthreads(128,1,1)]void Predict(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;Particle a=particles[i];bool active=a.live>0&&a.age+dt<lifetime&&a.p.z>origin.z-180&&all(abs(a.p-origin)<1000);output[i]=float4(contact(a.p+(a.v+gravity*dt)*dt),active?a.live:0);}
[numthreads(128,1,1)]void Grid(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;float4 a=positions[i];if(a.w==0){linkOut[i]=0xffffffff;return;}uint old;InterlockedExchange(headOut[key((int3)floor(a.xyz/h))],i,old);linkOut[i]=old;}
[numthreads(128,1,1)]void Density(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;float4 p=positions[i];if(p.w==0){auxOut[i]=0;return;}float density=0,denom=0;float3 gi=0;int3 cell=(int3)floor(p.xyz/h);
 [loop]for(int z=-1;z<=1;z++)[loop]for(int y=-1;y<=1;y++)[loop]for(int x=-1;x<=1;x++){
  int3 c=cell+int3(x,y,z);uint j=heads[key(c)];[loop]while(j!=0xffffffff){float4 b=positions[j];float3 r=p.xyz-b.xyz;float l=length(r);if(all((int3)floor(b.xyz/h)==c)&&l<h){density+=particleVolume*W(l);float3 g=particleVolume*grad(r);gi+=g;denom+=dot(g,g);}j=links[j];}
 }
 float lambda=-max(0,density-1)/(denom+dot(gi,gi)+.01/(spacing*spacing));auxOut[i]=float4(lambda,density,0,0);
}
[numthreads(128,1,1)]void Project(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;float4 p=positions[i];if(p.w==0){output[i]=p;return;}float3 delta=0;int3 cell=(int3)floor(p.xyz/h);
 [loop]for(int z=-1;z<=1;z++)[loop]for(int y=-1;y<=1;y++)[loop]for(int x=-1;x<=1;x++){
  int3 c=cell+int3(x,y,z);uint j=heads[key(c)];[loop]while(j!=0xffffffff){float4 b=positions[j];float3 r=p.xyz-b.xyz;float l=length(r);if(i!=j&&all((int3)floor(b.xyz/h)==c)&&l<h){delta+=(auxiliary[i].x+auxiliary[j].x)*particleVolume*grad(r);}j=links[j];}
 }
 float l=length(delta);if(l>spacing*.3)delta*=spacing*.3/l;output[i]=float4(contact(p.xyz+delta),p.w);
}
[numthreads(128,1,1)]void BaseVelocity(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;float4 p=positions[i];float3 v=(p.xyz-particles[i].p)/dt;float3 attraction=0;int3 cell=(int3)floor(p.xyz/h);
 if(p.w>0 && tension>0){[loop]for(int z=-1;z<=1;z++)[loop]for(int y=-1;y<=1;y++)[loop]for(int x=-1;x<=1;x++){
  int3 c=cell+int3(x,y,z);uint j=heads[key(c)];[loop]while(j!=0xffffffff){float4 b=positions[j];float3 r=b.xyz-p.xyz;float l=length(r);if(i!=j&&all((int3)floor(b.xyz/h)==c)&&l>spacing*.85&&l<h){float q=l/h;float w=(1-q)*(1-q);attraction+=r/l*w;}j=links[j];}
 }}
 v+=dt*tension*attraction;float speed=length(v);if(speed>maxSpeed)v*=maxSpeed/speed;output[i]=float4(v,particles[i].age+dt);
}
// Neighbors and the block inverse depend on positions, not Jacobi iteration.
// Store up to 64 neighbors in transposed slots for coalesced iteration reads.
// Crowded particles use the original full-neighborhood path; none are dropped.
[numthreads(128,1,1)]void CacheViscosity(uint3 id:SV_DispatchThreadID){
 uint i=id.x;if(i>=count)return;float4 p=positions[i];float3 a=float3(1,0,0),b=float3(0,1,0),c0=float3(0,0,1);uint size=0;int3 cell=(int3)floor(p.xyz/h);
 if(p.w>0){[loop]for(int z=-1;z<=1;z++)[loop]for(int y=-1;y<=1;y++)[loop]for(int x=-1;x<=1;x++){
  int3 c=cell+int3(x,y,z);uint j=heads[key(c)];[loop]while(j!=0xffffffff){float3 r=p.xyz-positions[j].xyz;float l=length(r);
   if(i!=j&&all((int3)floor(positions[j].xyz/h)==c)&&l>1e-5&&l<h){float3 n=r/l;float beta=dt*viscosity*10*particleVolume*(-dot(r,grad(r)))/(l*l+.01*h*h);
    a+=beta*n.x*n;b+=beta*n.y*n;c0+=beta*n.z*n;if(size<64){Neighbor item;item.n=n;item.beta=beta;item.index=j;neighborOut[size*count+i]=item;}size++;
   }j=links[j];
  }
 }}
 float3 bc=cross(b,c0);float det=max(dot(a,bc),1e-10);Matrix result;result.row0=float4(bc/det,0);result.row1=float4(cross(c0,a)/det,0);result.row2=float4(cross(a,b)/det,0);result.size=size>64?0xffffffff:size;result.padding=0;matrixOut[i]=result;
}
// Implicit pairwise longitudinal viscosity: symmetric, positive coefficients.
// Solve a 3x3 block Jacobi system. Unlike global drag, translation is preserved.
[numthreads(128,1,1)]void Viscosity(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return; if(useCache!=0){Matrix m=matrices[i];if(m.size!=0xffffffff){float3 rhs=baseVelocity[i].xyz;
  [loop]for(uint k=0;k<m.size;k++){Neighbor item=neighbors[k*count+i];rhs+=item.beta*item.n*dot(item.n,velocity[item.index].xyz);}
  output[i]=float4(dot(m.row0.xyz,rhs),dot(m.row1.xyz,rhs),dot(m.row2.xyz,rhs),baseVelocity[i].w);return;
 }}float4 p=positions[i];float3 rhs=baseVelocity[i].xyz;float3 a=float3(1,0,0),b=float3(0,1,0),c0=float3(0,0,1);int3 cell=(int3)floor(p.xyz/h);
 if(p.w>0){[loop]for(int z=-1;z<=1;z++)[loop]for(int y=-1;y<=1;y++)[loop]for(int x=-1;x<=1;x++){
  int3 c=cell+int3(x,y,z);uint j=heads[key(c)];[loop]while(j!=0xffffffff){float4 other=positions[j];float3 r=p.xyz-other.xyz;float l=length(r);if(i!=j&&all((int3)floor(other.xyz/h)==c)&&l>1e-5&&l<h){float3 n=r/l;float beta=dt*viscosity*10*particleVolume*(-dot(r,grad(r)))/(l*l+.01*h*h);a+=beta*n.x*n;b+=beta*n.y*n;c0+=beta*n.z*n;rhs+=beta*n*dot(n,velocity[j].xyz);}j=links[j];}
 }}
 float3 bc=cross(b,c0);float determinant=dot(a,bc);float3 v=float3(dot(rhs,bc),dot(a,cross(rhs,c0)),dot(a,cross(b,rhs)))/max(determinant,1e-10);output[i]=float4(v,baseVelocity[i].w);
}
[numthreads(128,1,1)]void Finish(uint3 id:SV_DispatchThreadID){uint i=id.x;if(i>=count)return;Particle p;p.p=positions[i].xyz;p.live=positions[i].w;p.v=velocity[i].xyz;p.age=velocity[i].w;if(enableFloor>.5&&p.p.z<=floorZ+spacing*.451){p.v.z=max(0,p.v.z);p.v.xy*=exp(-dt*4);}particleOut[i]=p;}
)HLSL";
struct Constants {UINT count,mask;float dt,spacing,h,volume,viscosity,tension;V3 gravity;float lifetime;V3 origin;float floorZ;V3 tip;float maxSpeed;V3 direction;float floor;F4 collider[8];UINT useCache;float cachePadding[3];};
static_assert(sizeof(Constants)%16==0,"constant alignment");
class Solver {
public:
 ID3D11Device* device=nullptr;ID3D11DeviceContext* context=nullptr;ID3D11Buffer *constants=nullptr,*staging=nullptr;
 Buffer particles,pos[2],aux,heads,links,vel[3],neighbors,matrices;ID3D11ComputeShader* shader[8]{};
 UINT capacity=0,count=0,active=0;V3 origin{};std::vector<Particle> snapshot;std::string error;bool valid=false,releaseAtDestruction=true;
 ~Solver(){if(releaseAtDestruction)Release();}
 void Release(){valid=false;count=capacity=active=0;snapshot.clear();if(context)context->ClearState();for(auto& s:shader)Drop(s);particles.Release();for(auto& b:pos)b.Release();for(auto& b:vel)b.Release();aux.Release();heads.Release();links.Release();neighbors.Release();matrices.Release();Drop(staging);Drop(constants);Drop(context);Drop(device);}
 bool Fail(const char* where,HRESULT hr){char b[200];sprintf_s(b,"%s (0x%08X)",where,(unsigned)hr);error=b;valid=false;return false;}
 bool Make(Buffer& b,UINT elements,UINT stride){D3D11_BUFFER_DESC d{};d.ByteWidth=elements*stride;d.Usage=D3D11_USAGE_DEFAULT;d.BindFlags=D3D11_BIND_SHADER_RESOURCE|D3D11_BIND_UNORDERED_ACCESS;d.MiscFlags=D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;d.StructureByteStride=stride;
  HRESULT hr=device->CreateBuffer(&d,nullptr,&b.b);if(FAILED(hr))return Fail("buffer",hr);hr=device->CreateShaderResourceView(b.b,nullptr,&b.srv);if(FAILED(hr))return Fail("SRV",hr);hr=device->CreateUnorderedAccessView(b.b,nullptr,&b.uav);return SUCCEEDED(hr)||Fail("UAV",hr);
 }
 bool Initialize(UINT requested){if(valid&&capacity==requested)return true;Release();error.clear();D3D_FEATURE_LEVEL level;HRESULT hr=D3D11CreateDevice(nullptr,D3D_DRIVER_TYPE_HARDWARE,nullptr,0,nullptr,0,D3D11_SDK_VERSION,&device,&level,&context);if(FAILED(hr))return Fail("D3D11 hardware device",hr);if(level<D3D_FEATURE_LEVEL_11_0)return Fail("requires D3D11 feature level 11",E_NOINTERFACE);
  const char* names[]={"Predict","Grid","Density","Project","BaseVelocity","Viscosity","Finish","CacheViscosity"};for(int i=0;i<8;i++){ID3DBlob *code=nullptr,*errors=nullptr;hr=D3DCompile(computeSource,strlen(computeSource),"fluid_compute",nullptr,nullptr,names[i],"cs_5_0",D3DCOMPILE_OPTIMIZATION_LEVEL3,0,&code,&errors);if(FAILED(hr)){if(errors)error.assign((char*)errors->GetBufferPointer(),errors->GetBufferSize());Drop(errors);Drop(code);return false;}Drop(errors);hr=device->CreateComputeShader(code->GetBufferPointer(),code->GetBufferSize(),nullptr,&shader[i]);Drop(code);if(FAILED(hr))return Fail("compute shader",hr);}
  capacity=requested;if(!Make(particles,capacity,sizeof(Particle))||!Make(aux,capacity,16)||!Make(heads,65536,4)||!Make(links,capacity,4))return false;for(auto& b:pos)if(!Make(b,capacity,16))return false;for(auto& b:vel)if(!Make(b,capacity,16))return false;if(!Make(neighbors,capacity*64,20)||!Make(matrices,capacity,64))return false;
  D3D11_BUFFER_DESC bd{};bd.ByteWidth=sizeof(Constants);bd.Usage=D3D11_USAGE_DEFAULT;bd.BindFlags=D3D11_BIND_CONSTANT_BUFFER;hr=device->CreateBuffer(&bd,nullptr,&constants);if(FAILED(hr))return Fail("constants",hr);
  bd={};bd.ByteWidth=capacity*sizeof(Particle);bd.Usage=D3D11_USAGE_STAGING;bd.CPUAccessFlags=D3D11_CPU_ACCESS_READ;hr=device->CreateBuffer(&bd,nullptr,&staging);if(FAILED(hr))return Fail("readback",hr);valid=true;Reset({});return true;
 }
 void Reset(V3 center){count=active=0;origin=center;snapshot.clear();}
 bool Add(const std::vector<Particle>& data){if(data.empty())return true;if(!valid||count+data.size()>capacity){error="Particle capacity exceeded; reduce volume or increase spacing/capacity";return false;}D3D11_BOX box{};box.left=count*sizeof(Particle);box.right=(count+(UINT)data.size())*sizeof(Particle);box.bottom=box.back=1;context->UpdateSubresource(particles.b,0,&box,data.data(),0,0);count+=(UINT)data.size();return true;}
 void Run(int pass,Buffer* p,Buffer* dst,Buffer* v=nullptr){
  ID3D11ShaderResourceView* s[9]{};ID3D11UnorderedAccessView* u[7]{};
  if(pass==0){s[0]=particles.srv;u[0]=dst->uav;}
  else if(pass==1){s[1]=p->srv;u[2]=heads.uav;u[3]=links.uav;}
  else if(pass==2||pass==3||pass==4||pass==5||pass==7){s[1]=p->srv;s[3]=heads.srv;s[4]=links.srv;
   if(pass==2)u[1]=aux.uav;
   if(pass==3){s[2]=aux.srv;u[0]=dst->uav;}
   if(pass==4){s[0]=particles.srv;u[0]=dst->uav;}
   if(pass==5){s[5]=v->srv;s[6]=vel[0].srv;s[7]=neighbors.srv;s[8]=matrices.srv;u[0]=dst->uav;}
   if(pass==7){u[5]=neighbors.uav;u[6]=matrices.uav;}
  }else {s[1]=p->srv;s[5]=v->srv;u[4]=particles.uav;}
  context->CSSetShader(shader[pass],nullptr,0);context->CSSetShaderResources(0,9,s);context->CSSetUnorderedAccessViews(0,7,u,nullptr);context->Dispatch((count+127)/128,1,1);
  memset(s,0,sizeof(s));memset(u,0,sizeof(u));context->CSSetShaderResources(0,9,s);context->CSSetUnorderedAccessViews(0,7,u,nullptr);
 }
 void Grid(Buffer* p){UINT zero[4]={0xffffffff,0xffffffff,0xffffffff,0xffffffff};context->ClearUnorderedAccessViewUint(heads.uav,zero);Run(1,p,nullptr);}
 bool Step(float dt,const Settings& cfg,V3 emitter,V3 direction,const F4* collision=nullptr){if(!valid)return false;if(!count)return true;Constants c{};c.count=count;c.mask=65535;c.dt=dt;c.spacing=cfg.spacing;c.h=cfg.spacing*2;c.volume=cfg.spacing*cfg.spacing*cfg.spacing;c.viscosity=cfg.viscosity;c.tension=cfg.tension;c.gravity={0,0,-cfg.gravity};c.lifetime=cfg.lifetime;c.origin=origin;c.floorZ=origin.z-cfg.catchDepth;c.tip=emitter;c.direction=direction;c.maxSpeed=cfg.speedLimit;c.floor=cfg.catchPlane?1.f:0.f;c.useCache=cfg.cacheViscosity?1:0;if(collision)memcpy(c.collider,collision,sizeof(c.collider));context->UpdateSubresource(constants,0,nullptr,&c,0,0);context->CSSetConstantBuffers(0,1,&constants);
  Buffer *a=&pos[0],*b=&pos[1];Run(0,nullptr,a);for(int n=0;n<cfg.iterations;n++){Grid(a);Run(2,a,nullptr);Run(3,a,b);std::swap(a,b);}Grid(a);Run(4,a,&vel[0]);if(cfg.viscosity>0&&cfg.cacheViscosity)Run(7,a,nullptr);Buffer* v=&vel[0];for(int n=0;n<cfg.viscIterations&&cfg.viscosity>0;n++){Buffer* out=&vel[1+n%2];Run(5,a,out,v);v=out;}Run(6,a,nullptr,v);return true;
 }
 bool Read(){if(!valid)return false;snapshot.clear();active=0;if(!count)return true;D3D11_BOX box{};box.right=count*sizeof(Particle);box.bottom=box.back=1;context->CopySubresourceRegion(staging,0,0,0,0,particles.b,0,&box);D3D11_MAPPED_SUBRESOURCE map{};HRESULT hr=context->Map(staging,0,D3D11_MAP_READ,0,&map);if(FAILED(hr))return Fail("particle readback",hr);snapshot.resize(count);memcpy(snapshot.data(),map.pData,count*sizeof(Particle));context->Unmap(staging,0);for(auto& p:snapshot){if(!std::isfinite(p.p.x)||!std::isfinite(p.p.y)||!std::isfinite(p.p.z)||!std::isfinite(p.v.x)||!std::isfinite(p.v.y)||!std::isfinite(p.v.z)||!std::isfinite(p.age))return Fail("nonfinite simulation",E_FAIL);if(p.live>0)++active;}return true;}
};
}


