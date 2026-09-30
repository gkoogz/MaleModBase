#include <algorithm>
#include <cmath>
extern "C" __declspec(dllexport) void raster(int count,const int* indices,const float* x,const float* y,const float* z,const float* n,int w,int h,float* depth,float* normal) {
 for(int t=0;t<count;t++) {
  int a=indices[t*3],b=indices[t*3+1],c=indices[t*3+2];
  int x0=std::max(0,int(floor(std::min({x[a],x[b],x[c]})))),x1=std::min(w-1,int(ceil(std::max({x[a],x[b],x[c]}))));
  int y0=std::max(0,int(floor(std::min({y[a],y[b],y[c]})))),y1=std::min(h-1,int(ceil(std::max({y[a],y[b],y[c]}))));
  float den=(y[b]-y[c])*(x[a]-x[c])+(x[c]-x[b])*(y[a]-y[c]);
  if(fabs(den)<1e-8f)continue;
  for(int j=y0;j<=y1;j++)for(int i=x0;i<=x1;i++) {
   float u=((y[b]-y[c])*(i+.5f-x[c])+(x[c]-x[b])*(j+.5f-y[c]))/den;
   float v=((y[c]-y[a])*(i+.5f-x[c])+(x[a]-x[c])*(j+.5f-y[c]))/den;
   float k=1-u-v;
   if(u<-.00001f||v<-.00001f||k<-.00001f)continue;
   float d=u*z[a]+v*z[b]+k*z[c]; int p=j*w+i;
   if(d<=depth[p])continue;
   depth[p]=d;
   for(int q=0;q<3;q++)normal[p*3+q]=u*n[a*3+q]+v*n[b*3+q]+k*n[c*3+q];
  }
 }
}
