// Generated full numerical kernel; see tools/extract_surface_runtime.py.
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <xmmintrin.h>
#include <Eigen/SparseCholesky>
#include <Eigen/Geometry>
#include <malemod/surface/pelvic_frame.hpp>

#include <malemod/surface/garment_support.hpp>
#include <malemod/surface/garment_impulse.hpp>
#include <malemod/surface/root_contact.hpp>
#include <malemod/controls/presentation.hpp>
#include <malemod/garments/pouch_budget.hpp>
#include <ppl.h>
#include <chrono>
#include <string>
#ifndef _MSC_VER
#define __forceinline inline
#define _finite std::isfinite
#endif
namespace malemod::surface::source {
using UINT=unsigned; using LONG=long;
using std::min;using std::max;
using D3DXFLOAT16=uint16_t;
static void D3DXFloat16To32Array(float* dst,const uint16_t* src,unsigned n){
 for(unsigned i=0;i<n;i++){uint32_t h=src[i],sign=(h&0x8000)<<16,e=(h>>10)&31,m=h&1023,bits;
  if(e==0){if(m==0)bits=sign;else{int exp=-14;while(!(m&1024)){m<<=1;--exp;}bits=sign|((exp+127)<<23)|((m&1023)<<13);}}
  else bits=sign|((e==31?255:e+112)<<23)|(m<<13);
  memcpy(dst+i,&bits,4);
 }
}
static void D3DXFloat32To16Array(uint16_t* dst,const float* src,unsigned n){
 for(unsigned i=0;i<n;i++){uint32_t bits;memcpy(&bits,src+i,4);unsigned sign=(bits>>16)&0x8000,m=bits&0x7fffff;int e=((bits>>23)&255)-112;
  if(e>=31)dst[i]=uint16_t(sign|0x7c00|(m?0x200:0));
  else if(e<=0){if(e<-10)dst[i]=uint16_t(sign);else{m|=0x800000;unsigned shift=14-e,rounded=(m+((1u<<(shift-1))-1)+((m>>shift)&1))>>shift;dst[i]=uint16_t(sign|rounded);}}
  else{m+=0xfff+((m>>13)&1);if(m&0x800000){m=0;++e;}dst[i]=uint16_t(sign|(e<<10)|(m>>13));}
 }
}
static LONG InterlockedExchange(volatile LONG* p,LONG value){LONG old=*p;*p=value;return old;}
using DWORD=std::uint32_t;
// The adapter supplies the same unsigned millisecond clock used by Wolverine;
// no host clock or engine API is allowed inside the numerical worker.
static DWORD sourceMilliseconds=0;
static DWORD GetTickCount(){return sourceMilliseconds;}
static bool sourceRootJointRequested=false;
@CANONICAL_TIMELINE@
static teaching::Timeline teachingTimeline;
namespace teaching {
struct SourceFluidProjection {
 float angleGain[4]={1,1,1,1},lateralGain[4]{};
 struct {float lateralWobbleDegrees=0;} settings;
 @CANONICAL_EVENT_TIME@
 @CANONICAL_LATERAL_YAW@
};
}
static teaching::SourceFluidProjection teachingFluid;

