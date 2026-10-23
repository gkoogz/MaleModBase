#pragma once
#include "presentation.hpp"
#include <algorithm>
#include <vector>

namespace malemod::surface {
// A material cross-section, from the retained sac toward the mounted rim.
// Endpoint values and first derivatives remain exact. Only generated skin is
// included: neither the complete shaft nor either retained lobe is reshaped.
struct SuspensionSection {double t=0;std::vector<unsigned> vertices;};
inline double SuspensionSectionScale(double t,double hang){
 if(!std::isfinite(t)||t<0||t>1||!std::isfinite(hang)||hang<1||hang>100)
  throw std::invalid_argument("Invalid suspended neck profile coordinate");
 const double u=(hang-1)/99.,h=u*u*(3-2*u);
 // The waist is above the lobes, at two thirds of the neck. The broad lower
 // transition and upper trumpet have zero displacement and slope at the joins.
 const double bell=(729./16.)*t*t*t*t*(1-t)*(1-t);
 return 1-(.38*h-.10*(1-h))*bell;
}
inline void ShapeSuspensionSections(std::vector<PresentationPoint>& points,
 const std::vector<SuspensionSection>& sections,double hang){
 for(const auto& section:sections){
  const double scale=SuspensionSectionScale(section.t,hang);
  const auto& ids=section.vertices;
  if(ids.size()<3)throw std::invalid_argument("Suspended neck section is not closed");
  PresentationPoint center{};double perimeter=0;
  for(unsigned j=0;j<ids.size();j++){
   const auto a=points.at(ids[j]),b=points.at(ids[(j+1)%ids.size()]);
   const double length=std::sqrt(Dot(Sub(b,a),Sub(b,a)));
   center=Add(center,Scale(Add(a,b),.5*length));perimeter+=length;
  }
  if(perimeter<1e-12)throw std::invalid_argument("Collapsed suspended neck section");
  center=Scale(center,1/perimeter);
  for(auto id:ids)points[id]=Add(center,Scale(Sub(points[id],center),scale));
 }
}
// Profile the active compound skin rather than the retired asset loft. The
// retained lower rim and mounted upper rim stay exact, including a tangent
// band at each end. Fractions describe already-profiled material: applying
// target/from, rather than another absolute contraction, avoids accumulation
// when the immediate control view catches up with the authored shape.
inline double SuspensionStripScale(double t,double hang,double aspect){
 if(!std::isfinite(t)||t<0||t>1||!std::isfinite(hang)||hang<1||hang>100||!std::isfinite(aspect)||aspect<0)
  throw std::invalid_argument("Invalid compound suspension profile");
 // Keep the lower join and upper mounting tangent bands outside the waist.
 // The source derivative is fitted before this reversible width profile.
 if(t<=.20||t>=.90)return 1;
 auto smooth=[](double x){x=std::clamp(x,0.,1.);return x*x*x*(10+x*(-15+6*x));};
 const double u=(t-.20)/.70;
 // C2 shoulders and a waist just above the retained contents. A compact,
 // short gap has no room for an hourglass; let its geometry suppress it.
 const double peak=3./7.;
 const double normalization=peak*peak*peak*std::pow(1-peak,4);
 const double bell=u*u*u*std::pow(1-u,4)/normalization;
 return 1-.22*smooth((hang-1)/99.)*smooth((aspect-.08)/.62)*bell;
}
template<class Index> inline double ShapeSuspensionStripIndices(std::vector<PresentationPoint>& points,
 const std::vector<double>& parameters,unsigned count,Index index,double fromHang,double toHang,double fromFraction=1,PresentationPoint forward={}){
 using Vec=PresentationPoint;
 const unsigned rows=unsigned(parameters.size());
 if(rows<3||count<3||!std::isfinite(fromFraction)||fromFraction<0||fromFraction>1)
  throw std::invalid_argument("Compound suspension strip layout differs");
 for(auto value:forward)if(!std::isfinite(value))throw std::invalid_argument("Invalid compound suspension forward axis");
 const double forwardLength=std::hypot(forward[0],forward[1],forward[2]);
 const bool widthOnly=forwardLength>0;
 if(widthOnly)forward=Scale(forward,1/forwardLength);
 std::vector<Vec> center(rows);
 for(unsigned row=0;row<rows;row++){
  if(!std::isfinite(parameters[row])||parameters[row]<0||parameters[row]>1||(row&&parameters[row]<=parameters[row-1]))
   throw std::invalid_argument("Invalid compound suspension ring");
  for(unsigned j=0;j<count;j++)center[row]=Add(center[row],points.at(index(row,j)));
  center[row]=Scale(center[row],1./count);
 }
 if(parameters.front()!=0||parameters.back()!=1)throw std::invalid_argument("Compound suspension endpoints missing");
 auto axis=Sub(center.back(),center.front());const double span=std::sqrt(Dot(axis,axis));
 if(span<=1e-12)throw std::invalid_argument("Compound suspension centerline collapsed");
 axis=Scale(axis,1/span);
 double radius2=0;
 for(unsigned row:{0u,rows-1})for(unsigned j=0;j<count;j++){auto id=index(row,j);
  auto q=Sub(points[id],center[row]);q=Sub(q,Scale(axis,Dot(q,axis)));radius2+=Dot(q,q)/(2*count);
 }
 if(radius2<=1e-24)throw std::invalid_argument("Compound suspension rim collapsed");
 const double aspect=span/(2*std::sqrt(radius2));
 // Validate controls even for an identity request or protected-only layout.
 SuspensionStripScale(.5,fromHang,aspect);SuspensionStripScale(.5,toHang,aspect);
 if(fromHang==toHang&&fromFraction==1)return 1;
 std::vector<Vec> before,delta;before.reserve(count*rows);delta.reserve(count*rows);
 for(unsigned row=0;row<rows;row++){
  const unsigned a=row?row-1:row,b=row+1<rows?row+1:row;
  auto tangent=Sub(center[b],center[a]);double length=std::sqrt(Dot(tangent,tangent));
  tangent=length>1e-12?Scale(tangent,1/length):axis;
  auto side=Cross(tangent,forward);const double sideLength=std::sqrt(Dot(side,side));
  side=sideLength>1e-12?Scale(side,1/sideLength):Vec{};
  const double fromScale=1+(SuspensionStripScale(parameters[row],fromHang,aspect)-1)*fromFraction;
  const double ratio=SuspensionStripScale(parameters[row],toHang,aspect)/fromScale;
  for(unsigned j=0;j<count;j++){auto id=index(row,j);
   auto p=points[id],q=Sub(p,center[row]);
   // Compound boundaries are oblique, nonplanar attachment contours, not
   // transverse tube sections. When a measured forward axis is available,
   // narrow only lateral width: retain the anterior/posterior silhouette and
   // height exactly. A parallel centerline has no defined side, so skip it.
   q=widthOnly?Scale(side,Dot(q,side)):Sub(q,Scale(tangent,Dot(q,tangent)));
   before.push_back(p);delta.push_back(Scale(q,ratio-1));
  }
 }
 auto triangles=[&](auto visit){for(unsigned row=0;row+1<rows;row++)for(unsigned j=0;j<count;j++){
  unsigned k=(j+1)%count,a=row*count+j,b=row*count+k,c=(row+1)*count+j,d=(row+1)*count+k;
  visit(a,b,c);visit(b,d,c);
 }};
 // Bound the full displacement path, not just its endpoint: a quadratic
 // face-area curve can cross zero and recover before the requested profile.
 double fraction=1;
 triangles([&](unsigned i,unsigned j,unsigned k){
  auto a=Sub(before[j],before[i]),b=Sub(before[k],before[i]),n=Cross(a,b);
  const double nn=Dot(n,n);if(nn<=1e-28)return;
  auto da=Sub(delta[j],delta[i]),db=Sub(delta[k],delta[i]);
  double c=.90*nn,linear=Dot(Add(Cross(da,b),Cross(a,db)),n),quadratic=Dot(Cross(da,db),n);
  // Retain at least 10% of each face's projected area. A broad, modest
  // radial change is fine; a tiny cut ear cannot be collapsed by the profile.
  auto root=[&](double x){if(x>=0&&x<fraction)fraction=x;};
  double scale=(std::max)({std::abs(c),std::abs(linear),std::abs(quadratic),1e-300});
  if(std::abs(quadratic)<1e-14*scale){if(linear<0)root(-c/linear);}
  else{double discriminant=linear*linear-4*quadratic*c;if(discriminant>0){double q=-.5*(linear+std::copysign(std::sqrt(discriminant),linear));root(q/quadratic);if(q!=0)root(c/q);}}
 });
 if(fraction<1)fraction*=.98;
 for(unsigned row=1;row+1<rows;row++)for(unsigned j=0;j<count;j++){
  const unsigned flat=row*count+j;
  points[index(row,j)]=Add(before[flat],Scale(delta[flat],fraction));
 }
 return fraction;
}
inline double ShapeSuspensionStrip(std::vector<PresentationPoint>& points,
 const std::vector<std::vector<unsigned>>& rings,const std::vector<double>& parameters,
 double fromHang,double toHang,double fromFraction=1,PresentationPoint forward={}){
 if(rings.empty()||rings.size()!=parameters.size())throw std::invalid_argument("Compound suspension strip layout differs");
 const unsigned count=unsigned(rings.front().size());
 for(const auto& ring:rings)if(ring.size()!=count)throw std::invalid_argument("Compound suspension ring size differs");
 return ShapeSuspensionStripIndices(points,parameters,count,[&](unsigned row,unsigned col){return rings[row][col];},fromHang,toHang,fromFraction,forward);
}
inline double ShapeSuspensionStrip(std::vector<PresentationPoint>& points,
 const std::vector<double>& parameters,unsigned columnCount,double fromHang,double toHang,double fromFraction=1,PresentationPoint forward={}){
 if(points.size()!=parameters.size()*std::size_t(columnCount))throw std::invalid_argument("Compound suspension flat layout differs");
 return ShapeSuspensionStripIndices(points,parameters,columnCount,[&](unsigned row,unsigned col){return row*columnCount+col;},fromHang,toHang,fromFraction,forward);
}
}
