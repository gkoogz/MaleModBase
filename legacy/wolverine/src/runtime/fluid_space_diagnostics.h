#pragma once
// Bounded, read-only shader sampling. No physics calls or render-state writes.
// CameraPosition is zero in captured translated-world passes; its name alone
// cannot establish the absolute world origin required by the PhysX scene.
struct FluidSpaceSample {void* shader;DWORD tick;unsigned scene;};
static FluidSpaceSample fluidSpaceSamples[256]{};
static unsigned fluidSpaceSampleCount=0,fluidSpaceRecords=0;
static unsigned fluidSpaceSceneRecords[2]{},fluidSpaceFrameRecords=0;
static LONG fluidSpaceFrame=-2;
static void* fluidSpacePositionBuffers[64]{};static unsigned fluidSpacePositionBufferCount=0;
static FILE* fluidSpaceFile=nullptr;
static bool fluidSpaceOpenAttempted=false;
static void FluidSpaceDiagnostics(IDirect3DDevice9* d,UINT start,UINT count,INT base=0,UINT minv=0,UINT nv=0){
 if(!teachingTimeline.active||fluidSpaceRecords>=512)return;
 IDirect3DVertexShader9* shader=nullptr;if(FAILED(d->GetVertexShader(&shader))||!shader)return;
 const DWORD now=GetTickCount();const unsigned scene=TankCameraSceneActive()?1:0;
 if(fluidSpaceFrame!=renderFrameSerial){fluidSpaceFrame=renderFrameSerial;fluidSpaceFrameRecords=0;}
 if(fluidSpaceSceneRecords[scene]>=256||fluidSpaceFrameRecords>=2){shader->Release();return;}
 unsigned slot=0;for(;slot<fluidSpaceSampleCount;slot++)if(fluidSpaceSamples[slot].shader==shader&&fluidSpaceSamples[slot].scene==scene)break;
 bool first=slot==fluidSpaceSampleCount;
 if(first){if(slot>=256){shader->Release();return;}fluidSpaceSamples[slot]={shader,now,scene};++fluidSpaceSampleCount;shader->AddRef();}
 else if(DWORD(now-fluidSpaceSamples[slot].tick)<1000u){shader->Release();return;}
 fluidSpaceSamples[slot].tick=now;
 if(!fluidSpaceOpenAttempted){fluidSpaceOpenAttempted=true;char path[MAX_PATH];SiblingPath(path,"FluidSpaceDiagnostic.log");fopen_s(&fluidSpaceFile,path,"w");if(fluidSpaceFile)fprintf(fluidSpaceFile,"Fluid space diagnostic 2: geometry provenance plus spatial shader constants\n");}
 if(!fluidSpaceFile){shader->Release();return;}
 UINT bytes=0;ID3DXConstantTable* table=nullptr;
 if(FAILED(shader->GetFunction(nullptr,&bytes))||!bytes||bytes>1024*1024){shader->Release();return;}
 std::vector<DWORD> code((bytes+3)/4);
 if(FAILED(shader->GetFunction(code.data(),&bytes))||FAILED(D3DXGetShaderConstantTable(code.data(),&table))||!table){shader->Release();return;}
 fprintf(fluidSpaceFile,"record=%u frame=%ld time=%.3f tank=%u shader=%u start=%u triangles=%u\n",++fluidSpaceRecords,renderFrameSerial,teachingTimeline.time,scene,slot,start,count);
 ++fluidSpaceSceneRecords[scene];++fluidSpaceFrameRecords;
 D3DVIEWPORT9 viewport{};d->GetViewport(&viewport);fprintf(fluidSpaceFile,"viewport=%u,%u,%u,%u\n",viewport.X,viewport.Y,viewport.Width,viewport.Height);
 // Only inspect a few existing vertices, once per buffer. No full buffer
 // readback and no mutation of buffer contents or device bindings.
 if(nv){
  IDirect3DVertexDeclaration9* declaration=nullptr;
  if(SUCCEEDED(d->GetVertexDeclaration(&declaration))&&declaration){
   D3DVERTEXELEMENT9 elements[MAXD3DDECLLENGTH+1];UINT n=MAXD3DDECLLENGTH+1;
   if(SUCCEEDED(declaration->GetDeclaration(elements,&n)))for(UINT e=0;e<n&&elements[e].Stream!=0xff;e++){
    const auto& element=elements[e];fprintf(fluidSpaceFile,"decl stream=%u offset=%u type=%u usage=%u index=%u\n",element.Stream,element.Offset,element.Type,element.Usage,element.UsageIndex);
    if(element.Usage!=D3DDECLUSAGE_POSITION||element.UsageIndex||element.Type!=D3DDECLTYPE_FLOAT3)continue;
    IDirect3DVertexBuffer9* buffer=nullptr;UINT offset=0,stride=0;
    if(SUCCEEDED(d->GetStreamSource(element.Stream,&buffer,&offset,&stride))&&buffer){
     D3DVERTEXBUFFER_DESC bd{};if(SUCCEEDED(buffer->GetDesc(&bd))&&stride){
      fprintf(fluidSpaceFile,"positions buffer=%p bytes=%u stride=%u base=%d min=%u count=%u usage=%u\n",buffer,bd.Size,stride,base,minv,nv,bd.Usage);
      bool seen=false;for(unsigned b=0;b<fluidSpacePositionBufferCount;b++)if(fluidSpacePositionBuffers[b]==buffer)seen=true;
      bool sampleBuffer=!seen&&fluidSpacePositionBufferCount<64;
      if(sampleBuffer)fluidSpacePositionBuffers[fluidSpacePositionBufferCount++]=buffer;
      for(UINT sample=0;sampleBuffer&&sample<3;sample++){
       long long index=(long long)base+minv+(sample==0?0:sample==1?nv/2:nv-1);
       long long at=(long long)offset+index*stride+element.Offset;
       if(index<0||at<0||at+12>bd.Size)continue;void* raw=nullptr;
       HRESULT hr=buffer->Lock((UINT)at,12,&raw,D3DLOCK_READONLY);
       if(SUCCEEDED(hr)){float p[3];memcpy(p,raw,12);buffer->Unlock();fprintf(fluidSpaceFile,"position %lld %.9g %.9g %.9g\n",index,p[0],p[1],p[2]);}
       else fprintf(fluidSpaceFile,"position-read-failed %08X\n",(unsigned)hr);
      }
     }buffer->Release();
    }
   }declaration->Release();
  }
 }
 D3DXCONSTANTTABLE_DESC td{};
 if(SUCCEEDED(table->GetDesc(&td)))for(UINT i=0;i<td.Constants;i++){
  D3DXHANDLE handle=table->GetConstant(nullptr,i);D3DXCONSTANT_DESC desc{};UINT n=1;
  if(!handle||FAILED(table->GetConstantDesc(handle,&desc,&n))||!desc.Name||desc.RegisterSet!=D3DXRS_FLOAT4)continue;
  if(!strcmp(desc.Name,"BoneMatrices"))continue;
  bool spatial=strstr(desc.Name,"World")||strstr(desc.Name,"View")||strstr(desc.Name,"Camera")||strstr(desc.Name,"Position")||strstr(desc.Name,"Origin")||strstr(desc.Name,"Projection")||strstr(desc.Name,"Translation");
  if(!spatial||desc.RegisterIndex>=256)continue;
  UINT registers=min(16u,min(desc.RegisterCount,256u-desc.RegisterIndex));float values[64]{};
  if(FAILED(d->GetVertexShaderConstantF(desc.RegisterIndex,values,registers)))continue;
  fprintf(fluidSpaceFile,"%s c%u class=%u count=%u",desc.Name,desc.RegisterIndex,desc.Class,registers);
  for(UINT j=0;j<registers*4;j++)fprintf(fluidSpaceFile," %.9g",values[j]);fprintf(fluidSpaceFile,"\n");
 }
 if(first){ID3DXBuffer* assembly=nullptr;if(SUCCEEDED(D3DXDisassembleShader(code.data(),FALSE,nullptr,&assembly))){fprintf(fluidSpaceFile,"shader-disassembly-begin %u\n",slot);fwrite(assembly->GetBufferPointer(),1,assembly->GetBufferSize(),fluidSpaceFile);fprintf(fluidSpaceFile,"\nshader-disassembly-end\n");assembly->Release();}}
 fflush(fluidSpaceFile);table->Release();shader->Release();
}
static void ReleaseFluidSpaceDiagnostics(){
 if(fluidSpaceFile){fclose(fluidSpaceFile);fluidSpaceFile=nullptr;}
 for(unsigned i=0;i<fluidSpaceSampleCount;i++)if(fluidSpaceSamples[i].shader)((IDirect3DVertexShader9*)fluidSpaceSamples[i].shader)->Release();
 fluidSpaceSampleCount=0; // Keep the session-wide record cap and file-open latch.
}
