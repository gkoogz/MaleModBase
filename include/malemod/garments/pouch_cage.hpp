#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
struct PouchCageReceipt {unsigned sections=0,samples=0,corrections=0;float length=0,maxRadius=0;};
// A pre-shaped cap with a coherent axis and fixed material rays. Only a small
// radial cage receives contact corrections; display density does not multiply
// collision work. The concave radius envelope bridges anatomy rather than
// reproducing every lobe. Input distances and ease use the adapter's units.
inline PouchCageReceipt FitPouchCage(std::vector<Vec>& points,unsigned columns,unsigned rows,
 unsigned first,unsigned end,Vec tip,float ease=.12f,unsigned sections=16){
 if(columns<3||rows<3||points.size()<=size_t(columns)*rows||first<size_t(columns)*rows+1||first>=end||end>points.size()||sections<4||sections>32||!std::isfinite(ease)||ease<0)
  throw std::runtime_error("Invalid pouch cage inputs");
 for(float v:tip)if(!std::isfinite(v))throw std::runtime_error("Nonfinite pouch tip");
 for(unsigned c=0;c<columns;c++)for(float v:points[c])if(!std::isfinite(v))throw std::runtime_error("Nonfinite pouch seam");
 Vec center=AttachmentCentroid(points,columns),area{},mean{};
 for(unsigned c=0;c<columns;c++)area=Add(area,Cross(Sub(points[c],center),Sub(points[(c+1)%columns],center)));
 Vec axis=Unit(area);for(unsigned i=first;i<end;i++)mean=Add(mean,Sub(points[i],center));if(Dot(axis,mean)<0)axis=Mul(axis,-1);
 Vec x=Sub(points[columns/2],points[0]);x=Unit(Sub(x,Mul(axis,Dot(x,axis))));Vec y=Cross(axis,x);
 Vec offset=Sub(tip,center),shift=Sub(offset,Mul(axis,Dot(offset,axis)));
 float height=0,rx=0,ry=0;
 for(unsigned c=0;c<columns;c++){auto p=Sub(points[c],center);rx=(std::max)(rx,std::abs(Dot(p,x)));ry=(std::max)(ry,std::abs(Dot(p,y)));}
 for(unsigned i=first;i<end;i++){for(float v:points[i])if(!std::isfinite(v))throw std::runtime_error("Nonfinite pouch cage support");height=(std::max)(height,Dot(Sub(points[i],center),axis));}
 height+=(std::max)(ease*3,.035f*height);rx=(std::max)(rx,ease);ry=(std::max)(ry,ease);
 if(height<1e-5f||rx<1e-5f||ry<1e-5f)throw std::runtime_error("Collapsed pouch cage");
 constexpr unsigned rays=24;constexpr float tau=6.28318530718f;
 std::vector<float> radius((sections+1)*rays);
 auto at=[&](unsigned j,unsigned c)->float&{return radius[j*rays+c%rays];};
 for(unsigned j=0;j<=sections;j++){float t=float(j)/sections;float seed=std::sqrt((std::max)(0.f,1-t*t))*(.45f+.55f*std::exp(-8*t));for(unsigned c=0;c<rays;c++)at(j,c)=seed;}
 PouchCageReceipt receipt{sections,end-first,0,height,0};
 // Local splats constrain both endpoints of the containing cage interval.
 // This is a sampled contact fit, not a continuous triangle certificate;
 // the final bicubic display and sewn transition require separate auditing.
 for(unsigned i=first;i<end;i++){
  auto p=Sub(points[i],center);float t=Dot(p,axis)/height;if(t<0||t>=1)continue;
  p=Sub(p,Mul(shift,t));float a=Dot(p,x)/rx,b=Dot(p,y)/ry;
  float required=std::sqrt(a*a+b*b)+ease/(std::min)(rx,ry);
  float u=t*sections,angle=std::atan2(b,a);if(angle<0)angle+=tau;float v=angle/tau*rays;
  int j=int(u),col=int(v);required*=1.025f;
  if(j==int(sections)-1)required/=std::sqrt((std::max)(1e-5f,float(sections)*(1-t)));
  auto weight=[](float distance){float q=(std::max)(0.f,(std::min)(1.f,2-std::abs(distance)));return q*q*(3-2*q);};
  // A continuous splat avoids a contact jumping between bins under tiny pose
  // changes. Its central plateau keeps both interpolation donors outside.
  for(int k=j-1;k<=j+2;k++)if(k>=0&&k<int(sections))for(int c=col-1;c<=col+2;c++){
   float value=required*weight(k-u)*weight(c-v);unsigned wrapped=unsigned(c+int(rays))%rays;
   if(at(unsigned(k),wrapped)<value){at(unsigned(k),wrapped)=value;++receipt.corrections;}
  }
 }
 // Least concave majorant: an outward-only rope over the corrected cage.
 for(unsigned c=0;c<rays;c++){
  std::vector<unsigned> hull;
  for(unsigned j=0;j<=sections;j++){
   while(hull.size()>1){unsigned a=hull[hull.size()-2],b=hull.back();if((at(b,c)-at(a,c))/(b-a)>(at(j,c)-at(b,c))/(j-b))break;hull.pop_back();}hull.push_back(j);
  }
  for(unsigned k=1;k<hull.size();k++){unsigned a=hull[k-1],b=hull[k];for(unsigned j=a+1;j<b;j++)at(j,c)=at(a,c)+(at(b,c)-at(a,c))*float(j-a)/(b-a);}
 }
 // Outward smoothing across cage kinks; endpoints retain attachment/pole.
 for(unsigned pass=0;pass<8;pass++){auto old=radius;for(unsigned j=1;j<sections;j++)for(unsigned c=0;c<rays;c++)at(j,c)=(std::max)(old[j*rays+c],.5f*old[j*rays+c]+.125f*(old[(j-1)*rays+c]+old[(j+1)*rays+c]+old[j*rays+(c+1)%rays]+old[j*rays+(c+rays-1)%rays]));}
 auto seam=std::vector<Vec>(points.begin(),points.begin()+columns);
 auto cubic=[](float a,float b,float c,float d,float t){return b+.5f*t*(c-a+t*(2*a-5*b+4*c-d+t*(3*(b-c)+d-a)));};
 auto sample=[&](int j,int c){j=(std::max)(0,(std::min)(int(sections),j));return at(unsigned(j),unsigned(c+int(rays)*2)%rays);};
 for(unsigned c=0;c<columns;c++){
  auto d=Sub(seam[c],center);float angle=std::atan2(Dot(d,y)/ry,Dot(d,x)/rx),dx=std::cos(angle),dy=std::sin(angle);if(angle<0)angle+=tau;float angular=angle/tau*rays;unsigned k=unsigned(angular)%rays;float af=angular-unsigned(angular);
  for(unsigned row=1;row<rows;row++){
   float t=std::sin(1.57079632679f*float(row)/rows),u=t*sections;unsigned j=(std::min)(sections-1,unsigned(u));float f=u-j;
   float along[4];for(int q=0;q<4;q++)along[q]=cubic(sample(int(j)+q-1,int(k)-1),sample(int(j)+q-1,int(k)),sample(int(j)+q-1,int(k)+1),sample(int(j)+q-1,int(k)+2),af);
   float r=(std::max)(0.f,cubic(along[0],along[1],along[2],along[3],f));
   if(j==sections-1)r=(std::max)(r,along[1]*std::sqrt(1-f));
   Vec p=Add(center,Add(Mul(axis,height*t),Add(Mul(shift,t),Add(Mul(x,rx*r*dx),Mul(y,ry*r*dy)))));
   float w=(std::min)(1.f,t*20);w=w*w*(3-2*w);
   Vec sewn=Add(seam[c],Add(Mul(axis,height*t),Mul(shift,t)));points[row*columns+c]=Add(Mul(sewn,1-w),Mul(p,w));
  }
 }
 points[columns*rows]=Add(center,Add(Mul(axis,height),shift));
 for(float r:radius)receipt.maxRadius=(std::max)(receipt.maxRadius,r*(std::max)(rx,ry));
 return receipt;
}
}
