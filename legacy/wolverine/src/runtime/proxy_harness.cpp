#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <d3dx9.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include "physics_weights.h"

struct P3{float x,y,z;};
static const unsigned HARNESS_GRAFT_COUNT=2388u;
static float Distance(P3 a,P3 b){float x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;return sqrtf(x*x+y*y+z*z);}
static void ReadGraft(IDirect3DVertexBuffer9* vb,std::vector<P3>& out){void* raw=nullptr;out.resize(HARNESS_GRAFT_COUNT);vb->Lock(47050u*32u,HARNESS_GRAFT_COUNT*32u,&raw,D3DLOCK_READONLY);for(unsigned i=0;i<HARNESS_GRAFT_COUNT;i++)memcpy(&out[i],(unsigned char*)raw+i*32u,12);vb->Unlock();}
static void LobeCenters(const std::vector<P3>& points,P3 centers[2]){unsigned counts[2]{};centers[0]=centers[1]=P3{};for(unsigned i=0;i<HARNESS_GRAFT_COUNT;i++){if(phys_scrotum_weight[i]<.82f)continue;int side=points[i].y<0?0:1;centers[side].x+=points[i].x;centers[side].y+=points[i].y;centers[side].z+=points[i].z;counts[side]++;}for(int side=0;side<2;side++){centers[side].x/=counts[side];centers[side].y/=counts[side];centers[side].z/=counts[side];}}
static void LobeIntegrity(const std::vector<P3>& before,const std::vector<P3>& after,float& rms,float& maximum){float maxY=0;for(unsigned i=0;i<HARNESS_GRAFT_COUNT;i++)if(phys_scrotum_weight[i]>.82f)maxY=max(maxY,fabsf(before[i].y));float coreCutoff=maxY*.40f;double error2=0;unsigned count=0;maximum=0;for(unsigned i=0;i<HARNESS_GRAFT_COUNT;i+=3){if(phys_scrotum_weight[i]<.82f||fabsf(before[i].y)<coreCutoff)continue;for(unsigned j=i+3;j<HARNESS_GRAFT_COUNT;j+=7){if(phys_scrotum_weight[j]<.82f||fabsf(before[j].y)<coreCutoff||(before[i].y<0)!=(before[j].y<0))continue;float d0=Distance(before[i],before[j]);if(d0<.35f)continue;float relative=fabsf(Distance(after[i],after[j])-d0)/d0;maximum=max(maximum,relative);error2+=relative*relative;count++;}}rms=count?sqrtf((float)(error2/count)):999.f;}

static LRESULT CALLBACK Proc(HWND h,UINT m,WPARAM w,LPARAM l){return m==WM_DESTROY?(PostQuitMessage(0),0):DefWindowProc(h,m,w,l);}
int main(){HINSTANCE hi=GetModuleHandleA(nullptr);
  WNDCLASSA wc{};wc.lpfnWndProc=Proc;wc.hInstance=hi;wc.lpszClassName="ProxyHarness";RegisterClassA(&wc);
  HWND hw=CreateWindowA(wc.lpszClassName,"Proxy Harness",WS_OVERLAPPEDWINDOW,0,0,800,600,nullptr,nullptr,hi,nullptr);
  IDirect3D9* d9=Direct3DCreate9(D3D_SDK_VERSION);if(!d9)return 10;
  D3DPRESENT_PARAMETERS pp{};pp.Windowed=TRUE;pp.SwapEffect=D3DSWAPEFFECT_DISCARD;pp.hDeviceWindow=hw;pp.BackBufferWidth=800;pp.BackBufferHeight=600;pp.BackBufferFormat=D3DFMT_A8R8G8B8;pp.PresentationInterval=D3DPRESENT_INTERVAL_IMMEDIATE;
  IDirect3DDevice9* d=nullptr;HRESULT hr=d9->CreateDevice(D3DADAPTER_DEFAULT,D3DDEVTYPE_HAL,hw,D3DCREATE_SOFTWARE_VERTEXPROCESSING,&pp,&d);if(FAILED(hr))return 11;
  IDirect3DVertexBuffer9* vb=nullptr;hr=d->CreateVertexBuffer(50915u*32u,D3DUSAGE_DYNAMIC|D3DUSAGE_WRITEONLY,0,D3DPOOL_DEFAULT,&vb,nullptr);if(FAILED(hr))return 12;
  void* raw=nullptr;vb->Lock(0,0,&raw,0);memset(raw,0,50915u*32u);float known[3][3]={{14.075339f,-2.7741053f,83.369194f},{12.718411f,-2.7821724f,84.149310f},{12.683769f,-2.4691443f,83.122450f}};for(int i=0;i<3;i++)memcpy((unsigned char*)raw+(47050+i)*32,known[i],12);vb->Unlock();
  d->SetStreamSource(0,vb,0,32);keybd_event(VK_F7,0,0,0);d->Clear(0,nullptr,D3DCLEAR_TARGET,D3DCOLOR_XRGB(12,25,38),1,0);d->BeginScene();d->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,0,0,3,0,1);hr=d->EndScene();keybd_event(VK_F7,0,KEYEVENTF_KEYUP,0);d->Present(nullptr,nullptr,nullptr,nullptr);
  float before[3]{},after[3]{},shaftBefore[3]{},shaftAfter[3]{},ballsBefore[3]{},ballsAfter[3]{};std::vector<P3> graftBefore,graftAfter;ReadGraft(vb,graftBefore);vb->Lock(47050u*32u,12,&raw,D3DLOCK_READONLY);memcpy(before,raw,12);vb->Unlock();vb->Lock((47050u+507u)*32u,12,&raw,D3DLOCK_READONLY);memcpy(shaftBefore,raw,12);vb->Unlock();vb->Lock((47050u+213u)*32u,12,&raw,D3DLOCK_READONLY);memcpy(ballsBefore,raw,12);vb->Unlock();keybd_event('W',0,0,0);
  for(int frame=0;frame<600;frame++){d->Clear(0,nullptr,D3DCLEAR_TARGET,D3DCOLOR_XRGB(12,25,38),1,0);d->BeginScene();d->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,0,0,3,0,1);d->EndScene();d->Present(nullptr,nullptr,nullptr,nullptr);Sleep(8);}keybd_event('W',0,KEYEVENTF_KEYUP,0);
  ReadGraft(vb,graftAfter);vb->Lock(47050u*32u,12,&raw,D3DLOCK_READONLY);memcpy(after,raw,12);vb->Unlock();vb->Lock((47050u+507u)*32u,12,&raw,D3DLOCK_READONLY);memcpy(shaftAfter,raw,12);vb->Unlock();vb->Lock((47050u+213u)*32u,12,&raw,D3DLOCK_READONLY);memcpy(ballsAfter,raw,12);vb->Unlock();
  IDirect3DSurface9* back=nullptr;d->GetBackBuffer(0,0,D3DBACKBUFFER_TYPE_MONO,&back);HRESULT save=D3DXSaveSurfaceToFileA("proxy_harness.png",D3DXIFF_PNG,back,nullptr,nullptr);if(back)back->Release();
  float lobeRms=0,lobeMax=0;P3 centersBefore[2],centersAfter[2];LobeIntegrity(graftBefore,graftAfter,lobeRms,lobeMax);LobeCenters(graftBefore,centersBefore);LobeCenters(graftAfter,centersAfter);bool shaftMoved=memcmp(shaftBefore,shaftAfter,12)!=0,ballsMoved=memcmp(ballsBefore,ballsAfter,12)!=0;std::printf("EndScene=%08X Save=%08X shaftMoved=%d ballsMoved=%d shaftX=%.4f->%.4f shaftZ=%.4f->%.4f ballsZ=%.4f->%.4f lobeRms=%.5f lobeMax=%.5f separation=%.3f->%.3f centerX=(%.2f,%.2f) centerZ=(%.2f,%.2f)\n",hr,save,shaftMoved,ballsMoved,shaftBefore[0],shaftAfter[0],shaftBefore[2],shaftAfter[2],ballsBefore[2],ballsAfter[2],lobeRms,lobeMax,Distance(centersBefore[0],centersBefore[1]),Distance(centersAfter[0],centersAfter[1]),centersAfter[0].x,centersAfter[1].x,centersAfter[0].z,centersAfter[1].z);vb->Release();d->Release();d9->Release();return FAILED(hr)||FAILED(save)||!shaftMoved||!ballsMoved||lobeRms>.035f?20:0;
}
