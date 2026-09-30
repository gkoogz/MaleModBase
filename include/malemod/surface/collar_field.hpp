#pragma once
// Generated active-source metric; see provenance/collar.json.
#include <malemod/math.hpp>
#include <algorithm>
#include <cmath>
namespace malemod::collar {
using std::max;using std::min;
inline float Smoother01(float value){value=max(0.f,min(1.f,value));return value*value*value*(value*(value*6.f-15.f)+10.f);}
struct MetricRow {double mask,screen,area;};
inline MetricRow EvaluateMetricRow(V3 p,V3 root,V3 axis,V3 up,float radius,float length,double area){
 float growth=Smoother01((radius-2.9f)/4.72f);
  V3 q=p-root;float s=Dot(q,axis),y=q.y,z=Dot(q,up),rho=sqrtf(y*y+z*z),upper=(z/max(rho,1e-8f)+1)*.5f;
  float reach=5+growth*(2.5f+2.5f*upper);
  double w=Smoother01((s+reach)/3)*(1-Smoother01((s/length-.12f)/.26f));
  w*=1-Smoother01((rho-(radius*1.55f+2))/3);
  float ventralReach=4+4*Smoother01((radius-2.7f)/1.1f);
  w*=Smoother01((p.x-(2+ventralReach*(1-upper)))/3);
  w*=Smoother01((p.z-(root.z-radius*2.1f-2))/4);
  area=(std::max)(area,.005);double screen=area*(2.5+2*pow(1-w,4));
 return {w,screen,area};
}
}
