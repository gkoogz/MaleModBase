#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
// Experimental fitted garment: a fixed sewn edge and a small elliptical loft
// around current measured support samples. No cloth integration, plane search,
// iterative triangle projection or previous-pose state. The adapter supplies
// source-space ease. Sample enclosure is not a continuous collision certificate.
struct PouchLoftReceipt {unsigned sections=0,samples=0;float length=0,maxRadius=0;};
inline PouchLoftReceipt FitPouchLoft(std::vector<Vec>& points,unsigned columns,unsigned rows,
 unsigned supportFirst,unsigned supportEnd,float ease=.12f,unsigned sections=16){
 if(columns<3||rows<3||points.size()<=std::size_t(columns)*rows||supportFirst>=supportEnd||supportEnd>points.size()||sections<4||sections>64||!std::isfinite(ease)||ease<0)
  throw std::runtime_error("Invalid pouch loft inputs");
 const Vec center=AttachmentCentroid(points,columns);Vec area{},mean{};
 for(unsigned i=0;i<columns;i++)area=Add(area,Cross(Sub(points[i],center),Sub(points[(i+1)%columns],center)));
 Vec axis=Unit(area);
 for(unsigned i=supportFirst;i<supportEnd;i++){for(float v:points[i])if(!std::isfinite(v))throw std::runtime_error("Nonfinite pouch support");mean=Add(mean,Sub(points[i],center));}
 if(Dot(axis,mean)<0)axis=Mul(axis,-1);
 Vec x=Sub(points[columns/2],points[0]);x=Unit(Sub(x,Mul(axis,Dot(x,axis))));Vec y=Cross(axis,x);
 struct Sample {float h,x,y;};std::vector<Sample> samples;samples.reserve(supportEnd-supportFirst);
 float height=0;Sample end{};
 for(unsigned i=supportFirst;i<supportEnd;i++){auto d=Sub(points[i],center);Sample s{Dot(d,axis),Dot(d,x),Dot(d,y)};samples.push_back(s);if(s.h>height){height=s.h;end=s;}}
 height+=ease;if(height<=1e-5f)throw std::runtime_error("Collapsed pouch loft");
 struct Section {float x=0,y=0,rx=0,ry=0;};std::vector<Section> cage(sections+1);
 const float step=height/sections;
 for(unsigned j=0;j<sections;j++){
  const float h=j*step;float lowX=INFINITY,highX=-INFINITY,lowY=INFINITY,highY=-INFINITY;unsigned count=0;
  for(auto s:samples)if(std::abs(s.h-h)<=step+ease){lowX=(std::min)(lowX,s.x);highX=(std::max)(highX,s.x);lowY=(std::min)(lowY,s.y);highY=(std::max)(highY,s.y);++count;}
  auto& c=cage[j];
  if(!count){c=j?cage[j-1]:Section{};continue;}
  c.x=(lowX+highX)*.5f;c.y=(lowY+highY)*.5f;c.rx=(std::max)(ease,(highX-lowX)*.5f);c.ry=(std::max)(ease,(highY-lowY)*.5f);
  float scale=1;
  for(auto s:samples)if(std::abs(s.h-h)<=step+ease){float dx=(s.x-c.x)/(std::max)(1e-5f,c.rx),dy=(s.y-c.y)/(std::max)(1e-5f,c.ry);scale=(std::max)(scale,dx*dx+dy*dy);}
  scale=std::sqrt(scale);c.rx=c.rx*scale+ease;c.ry=c.ry*scale+ease;
 }
 cage[sections]={end.x,end.y,0,0};
 // An outward-only radius filter removes isolated ridges without shrinking a
 // sampled support envelope. The seam stays exact and is blended over 3 rows
 // of the cage, independently of display tessellation.
 auto original=cage;
 for(unsigned j=1;j<sections;j++){cage[j].rx=(std::max)(original[j].rx,(original[j-1].rx+original[j+1].rx)*.5f);cage[j].ry=(std::max)(original[j].ry,(original[j-1].ry+original[j+1].ry)*.5f);}
 PouchLoftReceipt receipt{sections,unsigned(samples.size()),height,0};
 for(auto c:cage)receipt.maxRadius=(std::max)({receipt.maxRadius,c.rx,c.ry});
 const auto seam=std::vector<Vec>(points.begin(),points.begin()+columns);
 auto start=Sub(seam[0],center);const float phase=std::atan2(Dot(start,y),Dot(start,x));
 const float winding=Dot(area,axis)>=0?1.f:-1.f;
 for(unsigned col=0;col<columns;col++){
  // Material columns keep their cyclic order even when the measured sewn edge
  // is locally concave. Deriving every interior ray from that edge can fold
  // multiple columns onto the same direction and produce a pleated side view.
  const float angle=phase+winding*6.28318531f*col/columns;
  const float dx=std::cos(angle),dy=std::sin(angle);
  for(unsigned row=1;row<rows;row++){
   float t=float(row)/rows,u=t*sections;unsigned j=(std::min)(sections-1,unsigned(u));float f=u-j;
   auto a=cage[j],b=cage[j+1];Section c{a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f,a.rx+(b.rx-a.rx)*f,a.ry+(b.ry-a.ry)*f};
   Vec desired=Add(center,Add(Mul(axis,height*t),Add(Mul(x,c.x+c.rx*dx),Mul(y,c.y+c.ry*dy))));
   float blend=(std::min)(1.f,t*sections/3);blend=blend*blend*(3-2*blend);
   Vec sewn=Add(seam[col],Mul(axis,height*t));points[row*columns+col]=Add(Mul(sewn,1-blend),Mul(desired,blend));
  }
 }
 points[columns*rows]=Add(center,Add(Mul(axis,height),Add(Mul(x,end.x),Mul(y,end.y))));
 return receipt;
}
}
