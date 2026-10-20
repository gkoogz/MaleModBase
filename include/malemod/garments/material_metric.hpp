#pragma once
#include <malemod/garments/jockstrap.hpp>

namespace malemod::garments {
// Intrinsic length of a straight material-coordinate edge on a piecewise
// affine simulation surface. A 3D chord across a rest fold is shorter than
// the actual fabric and would report fictitious strain when it unfolds.
inline double MaterialEdgeLength(const std::vector<Point>& uv,
                                const std::vector<std::array<unsigned,3>>& faces,
                                const std::vector<Point>& rest,Point from,Point to){
 if(uv.size()!=rest.size()||!Finite(from)||!Finite(to))throw std::invalid_argument("Invalid material metric inputs");
 for(unsigned i=0;i<uv.size();i++)if(!Finite(uv[i])||!Finite(rest[i]))throw std::invalid_argument("Nonfinite material surface");
 struct Interval {double low,high,speed;};std::vector<Interval> intervals;
 for(auto face:faces){
  const auto a=uv.at(face[0]),b=uv.at(face[1]),c=uv.at(face[2]);
  const double determinant=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  if(std::abs(determinant)<1e-14)throw std::invalid_argument("Degenerate material triangle");
  auto barycentric=[&](Point p){double x=p[0]-a[0],y=p[1]-a[1];double v=(x*(c[1]-a[1])-y*(c[0]-a[0]))/determinant,w=((b[0]-a[0])*y-(b[1]-a[1])*x)/determinant;return Point{1-v-w,v,w};};
  auto first=barycentric(from),last=barycentric(to);double low=0,high=1;
  for(unsigned k=0;k<3;k++){
   double slope=last[k]-first[k];
   if(std::abs(slope)<1e-14){if(first[k]<-1e-12)high=-1;}
   else if(slope>0)low=(std::max)(low,-first[k]/slope);
   else high=(std::min)(high,-first[k]/slope);
  }
  if(high<=low+1e-12)continue;
  Point velocity{};for(unsigned k=0;k<3;k++)velocity=Add(velocity,Mul(rest.at(face[k]),last[k]-first[k]));
  intervals.push_back({low,high,Length(velocity)});
 }
 std::sort(intervals.begin(),intervals.end(),[](auto a,auto b){return a.low<b.low;});
 double cursor=0,length=0;
 for(auto interval:intervals){
  if(interval.low>cursor+1e-9)throw std::invalid_argument("Material edge leaves simulation surface");
  if(interval.high>cursor){length+=(interval.high-cursor)*interval.speed;cursor=interval.high;}
 }
 if(cursor<1-1e-9)throw std::invalid_argument("Material edge is not covered by simulation surface");
 return length;
}
}
