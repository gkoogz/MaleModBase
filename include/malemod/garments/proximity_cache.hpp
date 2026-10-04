#pragma once
// Conservative geometric separation certificates. These do not classify
// volume membership: the caller must first verify the outside side using its
// closed measured surface. Exact contact queries remain the fallback.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace malemod::garments::proximity {
using Point=std::array<double,3>;
struct Stamp {std::uint64_t topology=0;double surfaceTravel=0;};
class Motion {
 Stamp stamp_{};
public:
 void Advance(double maximumVertexTravel,bool topologyChanged=false){
  if(!std::isfinite(maximumVertexTravel)||maximumVertexTravel<0)throw std::invalid_argument("Invalid surface motion bound");
  if(topologyChanged||!stamp_.topology){++stamp_.topology;stamp_.surfaceTravel=0;}
  else {stamp_.surfaceTravel+=maximumVertexTravel;if(!std::isfinite(stamp_.surfaceTravel)||stamp_.surfaceTravel>1e12){++stamp_.topology;stamp_.surfaceTravel=0;}}
 }
 Stamp Current()const{return stamp_;}
};
template<unsigned N>class Certificate {
 std::array<Point,N> points_{};Stamp stamp_{};double distance_=0;bool outside_=false;
 static double Distance(Point a,Point b){double square=0;for(unsigned i=0;i<3;i++){double d=a[i]-b[i];square+=d*d;}return std::sqrt(square);}
 static bool Finite(Point a){return std::isfinite(a[0])&&std::isfinite(a[1])&&std::isfinite(a[2]);}
public:
 void Clear(){outside_=false;}
 void Remember(const std::array<Point,N>& points,double unsignedSeparationLowerBound,bool verifiedOutside,Stamp stamp){
  if(!std::isfinite(unsignedSeparationLowerBound)||unsignedSeparationLowerBound<0||!std::isfinite(stamp.surfaceTravel)||stamp.surfaceTravel<0)throw std::invalid_argument("Invalid proximity certificate");
  for(auto p:points)if(!Finite(p))throw std::invalid_argument("Nonfinite proximity point");
  points_=points;distance_=unsignedSeparationLowerBound;outside_=verifiedOutside;stamp_=stamp;
 }
 double LowerBound(const std::array<Point,N>& points,Stamp stamp)const{
  if(!std::isfinite(stamp.surfaceTravel)||stamp.surfaceTravel<0)throw std::invalid_argument("Invalid proximity query");
  for(auto p:points)if(!Finite(p))throw std::invalid_argument("Nonfinite proximity point");
  if(!outside_||!stamp.topology||stamp.topology!=stamp_.topology||stamp.surfaceTravel<stamp_.surfaceTravel)return 0;
  double travel=0;for(unsigned i=0;i<N;i++)travel=(std::max)(travel,Distance(points[i],points_[i]));
  double surfaceTravel=stamp.surfaceTravel-stamp_.surfaceTravel;
  double slack=64*std::numeric_limits<double>::epsilon()*(1+distance_+travel+surfaceTravel);
  // Corresponding triangle vertices bound its entire linear interior. The
  // distance between two moving sets changes by at most their Hausdorff travel.
  return (std::max)(0.,distance_-travel-surfaceTravel-slack);
 }
 bool ProvesClear(const std::array<Point,N>& points,double margin,Stamp stamp)const{
  if(!std::isfinite(margin)||margin<0)throw std::invalid_argument("Invalid proximity query");
  return LowerBound(points,stamp)>margin;
 }
};
using PointCertificate=Certificate<1>;
using FaceCertificate=Certificate<3>;
// Certifies coverage of a collected broad-phase neighborhood, not separation
// or volume membership. An excluded triangle cannot enter the smaller current
// query unless cloth/surface travel consumes this neighborhood's padding.
class NeighborhoodBound {
 std::array<Point,3> points_{};Stamp stamp_{};double radius_=0;std::uintptr_t owner_=0;
public:
 void Remember(const std::array<Point,3>& points,double radius,Stamp stamp,std::uintptr_t owner){
  if(!std::isfinite(radius)||radius<=0||!std::isfinite(stamp.surfaceTravel)||stamp.surfaceTravel<0||!owner)throw std::invalid_argument("Invalid surface neighborhood bound");
  for(auto p:points)for(auto x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite neighborhood point");
  points_=points;stamp_=stamp;radius_=radius;owner_=owner;
 }
 bool Covers(const std::array<Point,3>& points,double margin,Stamp stamp,std::uintptr_t owner)const{
  if(!std::isfinite(margin)||margin<=0||!std::isfinite(stamp.surfaceTravel)||stamp.surfaceTravel<0)throw std::invalid_argument("Invalid surface neighborhood query");
  double travel=0;for(unsigned i=0;i<3;i++){double square=0;for(unsigned j=0;j<3;j++){if(!std::isfinite(points[i][j]))throw std::invalid_argument("Nonfinite neighborhood point");double d=points[i][j]-points_[i][j];square+=d*d;}travel=(std::max)(travel,std::sqrt(square));}
  if(!owner_||owner!=owner_||!stamp.topology||stamp.topology!=stamp_.topology||stamp.surfaceTravel<stamp_.surfaceTravel)return false;
  double surface=stamp.surfaceTravel-stamp_.surfaceTravel,slack=64*std::numeric_limits<double>::epsilon()*(1+radius_+travel+surface);
  return margin+travel+surface+slack<radius_;
 }
};
}
