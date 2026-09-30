#include <d3dx9tex.h>
#include <set>
#include <string>
// Read-only, bounded capture of actual Wolverine draws. F10 requests one frame.
static bool captureKey=false;
static int captureRemaining=0, captureDraw=0;
static char captureDirectory[MAX_PATH];
static std::set<void*> captureTextures;
static unsigned long long captureBytes=0;
static bool captureComplete=false;
static bool captureTankMaterials=false;
static UINT captureWaitFrames=0;
static void CaptureWrite(const char* name,const void* data,size_t size){
  char path[MAX_PATH];sprintf_s(path,"%s\\%s",captureDirectory,name);
  FILE* f=nullptr;fopen_s(&f,path,"wb");if(f){fwrite(data,1,size,f);fclose(f);}
}
static void CapturePoll(){
  bool down=(GetAsyncKeyState(VK_F10)&0x8000)!=0;
  bool requested=false;static DWORD materialPollTick=0,tankWarmupTick=0;DWORD now=GetTickCount();
  if(!TankCameraSceneActive())tankWarmupTick=0;else if(!tankWarmupTick)tankWarmupTick=now;
  if(tankWarmupTick&&DWORD(now-tankWarmupTick)>3000&&DWORD(now-materialPollTick)>1000&&captureRemaining==0){
    materialPollTick=now;char request[MAX_PATH];SiblingPath(request,"TankMaterialCapture.request");
    if(GetFileAttributesA(request)!=INVALID_FILE_ATTRIBUTES)requested=DeleteFileA(request)!=FALSE;
  }
  if(((down&&!captureKey)||requested)&&captureRemaining==0){
    captureTankMaterials=requested||TankCameraSceneActive();
    GetModuleFileNameA((HMODULE)&__ImageBase,captureDirectory,MAX_PATH);
    char* slash=strrchr(captureDirectory,'\\');if(slash)*slash=0;
    SYSTEMTIME t;GetLocalTime(&t);char leaf[64];sprintf_s(leaf,"\\R14Capture_%04u%02u%02u_%02u%02u%02u",t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute,t.wSecond);strcat_s(captureDirectory,leaf);
    CreateDirectoryA(captureDirectory,nullptr);captureRemaining=1;captureComplete=false;captureWaitFrames=0;captureDraw=0;captureTextures.clear();captureBytes=0;
    Log("R14 full material capture requested: %s",captureDirectory);
  }captureKey=down;
}
template<class T> static void CaptureShader(T* shader,const char* name){
  if(!shader)return;UINT n=0;if(FAILED(shader->GetFunction(nullptr,&n))||n>1024*1024)return;
  std::vector<unsigned char> bytes(n);if(FAILED(shader->GetFunction(bytes.data(),&n)))return;
  char file[96];sprintf_s(file,"%s.bin",name);CaptureWrite(file,bytes.data(),n);
  ID3DXBuffer* assembly=nullptr;if(SUCCEEDED(D3DXDisassembleShader((DWORD*)bytes.data(),FALSE,nullptr,&assembly))){sprintf_s(file,"%s.txt",name);CaptureWrite(file,assembly->GetBufferPointer(),assembly->GetBufferSize());assembly->Release();}
}
static void CaptureDraw(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count,const void* vertexData,UINT vertexBytes,const void* indexData,UINT indexBytes){
  if(captureRemaining<=0||captureDraw>=16)return;
  char prefix[40];sprintf_s(prefix,"draw_%03d",captureDraw++);
  char path[MAX_PATH];sprintf_s(path,"%s\\%s.txt",captureDirectory,prefix);FILE* f=nullptr;fopen_s(&f,path,"w");if(!f)return;
  fprintf(f,"type=%u base=%d min=%u vertices=%u start=%u triangles=%u\n",type,base,minv,nv,start,count);
  fprintf(f,"controls:");for(float v:sliderUI)fprintf(f," %.9g",v);fprintf(f,"\n");
  D3DVIEWPORT9 vp{};d->GetViewport(&vp);fprintf(f,"viewport %u %u %u %u %.9g %.9g\n",vp.X,vp.Y,vp.Width,vp.Height,vp.MinZ,vp.MaxZ);
  for(int i=0;i<4;i++){IDirect3DSurface9* rt=nullptr;if(SUCCEEDED(d->GetRenderTarget(i,&rt))&&rt){D3DSURFACE_DESC desc{};rt->GetDesc(&desc);fprintf(f,"RT%d %p %u %u format=%u\n",i,rt,desc.Width,desc.Height,desc.Format);rt->Release();}}
  char name[100];float constants[256*4];
  d->GetVertexShaderConstantF(0,constants,256);sprintf_s(name,"%s_vs_constants.bin",prefix);CaptureWrite(name,constants,sizeof(constants));
  d->GetPixelShaderConstantF(0,constants,224);sprintf_s(name,"%s_ps_constants.bin",prefix);CaptureWrite(name,constants,224*16);
  IDirect3DVertexShader9* vs=nullptr;d->GetVertexShader(&vs);sprintf_s(name,"%s_vs",prefix);CaptureShader(vs,name);if(vs)vs->Release();
  IDirect3DPixelShader9* ps=nullptr;d->GetPixelShader(&ps);sprintf_s(name,"%s_ps",prefix);CaptureShader(ps,name);if(ps)ps->Release();
  IDirect3DVertexDeclaration9* decl=nullptr;d->GetVertexDeclaration(&decl);if(decl){D3DVERTEXELEMENT9 elements[MAXD3DDECLLENGTH+1];UINT n=MAXD3DDECLLENGTH+1;if(SUCCEEDED(decl->GetDeclaration(elements,&n))){sprintf_s(name,"%s_decl.bin",prefix);CaptureWrite(name,elements,n*sizeof(elements[0]));for(UINT i=0;i<n;i++)fprintf(f,"decl %u %u %u %u %u %u\n",elements[i].Stream,elements[i].Offset,elements[i].Type,elements[i].Method,elements[i].Usage,elements[i].UsageIndex);}decl->Release();}
  sprintf_s(name,"%s_stream0.bin",prefix);CaptureWrite(name,vertexData,vertexBytes);
  fprintf(f,"stream0 offset=0 stride=32 size=%u cpu_mirror=1\n",vertexBytes);
  sprintf_s(name,"%s_indices.bin",prefix);CaptureWrite(name,indexData,indexBytes);
  fprintf(f,"indices format=%u size=%u cpu_mirror=1\n",D3DFMT_INDEX16,indexBytes);
  for(UINT r=0;r<210;r++){DWORD value;if(SUCCEEDED(d->GetRenderState((D3DRENDERSTATETYPE)r,&value)))fprintf(f,"renderstate %u %u\n",r,value);}
  for(UINT s=0;s<16;s++){
    for(UINT r=1;r<=13;r++){DWORD value;if(SUCCEEDED(d->GetSamplerState(s,(D3DSAMPLERSTATETYPE)r,&value)))fprintf(f,"sampler %u %u %u\n",s,r,value);}
    IDirect3DBaseTexture9* tex=nullptr;d->GetTexture(s,&tex);if(!tex)continue;
    fprintf(f,"texture %u %p type=%u\n",s,tex,tex->GetType());
    D3DSURFACE_DESC desc{};bool supported=false;unsigned faces=1;
    if(tex->GetType()==D3DRTYPE_TEXTURE){supported=SUCCEEDED(((IDirect3DTexture9*)tex)->GetLevelDesc(0,&desc));}
    if(tex->GetType()==D3DRTYPE_CUBETEXTURE){supported=SUCCEEDED(((IDirect3DCubeTexture9*)tex)->GetLevelDesc(0,&desc));faces=6;}
    if(supported){
      fprintf(f,"texture_desc %u %u %u fmt=%u usage=%u levels=%u faces=%u\n",s,desc.Width,desc.Height,desc.Format,desc.Usage,tex->GetLevelCount(),faces);
      unsigned long long estimate=(unsigned long long)desc.Width*desc.Height*4*faces*2;
      bool dynamic=(desc.Usage&D3DUSAGE_RENDERTARGET)!=0;
      // Render-target textures may change between passes; save them per draw.
      if(captureBytes+estimate<512ull*1024*1024&&(dynamic||captureTextures.insert(tex).second)){
        captureBytes+=estimate;char file[MAX_PATH];
        if(dynamic)sprintf_s(file,"%s\\%s_texture_%p.dds",captureDirectory,prefix,tex);
        else sprintf_s(file,"%s\\texture_%p.dds",captureDirectory,tex);
        HRESULT hr=D3DXSaveTextureToFileA(file,D3DXIFF_DDS,tex,nullptr);
        fprintf(f,"save_dds %u %08X %s\n",s,hr,strrchr(file,'\\')+1);
      }
    }tex->Release();
  }fclose(f);
}
static void CaptureSurface(IDirect3DDevice9* d,const char* suffix){
 if(captureRemaining<=0||captureDraw==0||captureDraw>16)return;
 IDirect3DSurface9* rt=nullptr;if(FAILED(d->GetRenderTarget(0,&rt))||!rt)return;
 char file[MAX_PATH];sprintf_s(file,"%s\\draw_%03d_%s.dds",captureDirectory,captureDraw-1,suffix);
 HRESULT hr=D3DXSaveSurfaceToFileA(file,D3DXIFF_DDS,rt,nullptr,nullptr);
 Log("R14 capture RT %s hr=%08X",suffix,hr);rt->Release();
}
// Diagnostic-only copy of currently bound native geometry and material state.
// Never lock or read the buffers outside an explicitly requested capture.
static void CaptureBoundDraw(IDirect3DDevice9* d,D3DPRIMITIVETYPE type,INT base,UINT minv,UINT nv,UINT start,UINT count){
 if(!captureTankMaterials||captureRemaining<=0||captureDraw>=16)return;
 IDirect3DVertexBuffer9* vb=nullptr;IDirect3DIndexBuffer9* ib=nullptr;UINT offset=0,stride=0;
 if(FAILED(d->GetStreamSource(0,&vb,&offset,&stride))||!vb)return;
 if(FAILED(d->GetIndices(&ib))||!ib){vb->Release();return;}
 D3DVERTEXBUFFER_DESC vd{};D3DINDEXBUFFER_DESC id{};void *v=nullptr,*i=nullptr;
 if(SUCCEEDED(vb->GetDesc(&vd))&&SUCCEEDED(ib->GetDesc(&id))&&stride==32&&offset==0&&vd.Size<4*1024*1024&&id.Size<4*1024*1024&&id.Format==D3DFMT_INDEX16&&SUCCEEDED(vb->Lock(0,0,&v,D3DLOCK_READONLY))){
  if(SUCCEEDED(ib->Lock(0,0,&i,D3DLOCK_READONLY))){CaptureDraw(d,type,base,minv,nv,start,count,v,vd.Size,i,id.Size);ib->Unlock();}vb->Unlock();
 }
 ib->Release();vb->Release();
}
static void CaptureFinish(IDirect3DDevice9* d){
 if(captureRemaining<=0)return;
 if(captureDraw==0&&++captureWaitFrames<180)return;
 IDirect3DSurface9* back=nullptr;HRESULT hr=d->GetBackBuffer(0,0,D3DBACKBUFFER_TYPE_MONO,&back);
 if(SUCCEEDED(hr)&&back){char file[MAX_PATH];sprintf_s(file,"%s\\frame.png",captureDirectory);hr=D3DXSaveSurfaceToFileA(file,D3DXIFF_PNG,back,nullptr,nullptr);back->Release();}
 captureRemaining=0;captureComplete=captureDraw>0;
 char summary[300];sprintf_s(summary,"draws=%d screenshot=%08X original_shading=1 estimatedTextureBytes=%llu\n",captureDraw,hr,captureBytes);CaptureWrite("complete.txt",summary,strlen(summary));
 Log("R14 capture complete: %d draws, frame hr=%08X, %s",captureDraw,hr,captureDirectory);
}
