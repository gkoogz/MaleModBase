#pragma once
namespace volumeFluid {
struct SplatBakes {
 const unsigned short* pixels=nullptr;int width=0,frames[2]{};
 bool attempted=false;
 bool Open(){
  if(attempted)return pixels!=nullptr;attempted=true;
  static const int resourceAnchor=0;HMODULE module=nullptr;
  if(!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCSTR)&resourceAnchor,&module))return false;
  HRSRC resource=FindResourceA(module,MAKEINTRESOURCEA(201),RT_RCDATA);if(!resource)return false;
  DWORD bytes=SizeofResource(module,resource);HGLOBAL memory=LoadResource(module,resource);if(!memory||bytes<16)return false;
  auto header=(const unsigned*)LockResource(memory);if(!header||header[0]!=0x314B5053||header[1]!=192||header[2]!=26||header[3]!=64)return false;
  if(bytes!=16+192u*192u*90u*2u)return false;
  width=192;frames[0]=26;frames[1]=64;pixels=(const unsigned short*)(header+4);return true;
 }
 float Sample(int kind,int frame,float u,float v)const{
  if(!pixels||u<0||v<0||u>width-1||v>width-1)return 0;
  int x=(int)u,y=(int)v,xx=min(width-1,x+1),yy=min(width-1,y+1);float fx=u-x,fy=v-y;
  const auto p=pixels+((kind?frames[0]:0)+frame)*width*width;
  return ((p[y*width+x]*(1-fx)+p[y*width+xx]*fx)*(1-fy)+(p[yy*width+x]*(1-fx)+p[yy*width+xx]*fx)*fy)/65535.f;
 }
};
static SplatBakes splatBakes;
}
