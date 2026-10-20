#include <malemod/surface/dilating_collar.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace malemod::surface;
int main(){try{
 std::vector<PrecisePoint> ring;std::vector<unsigned> mouth;
 for(unsigned i=0;i<32;i++){double a=i*6.283185307179586/32;ring.push_back({0,2*cos(a),2*sin(a)});mouth.push_back(i);}
 auto points=ring;points.push_back({0,100,0});DilatingCollar field(points,ring,mouth);
 for(double radius:{.5,1.,2.,4.,8.}){
  for(unsigned i=0;i<32;i++){
   auto p=points[i],d=field.Displacement(i,{0,0,0},radius,2);double u=p[2]/2,expected=radius*(1+.1*std::pow(std::max(0.,-u),2));
   if(std::abs(std::hypot(p[1]+d[1],p[2]+d[2])-expected)>1e-10)throw std::runtime_error("Opening does not track radius");
  }
  if(field.Displacement(32,{0,0,0},radius,2)!=PrecisePoint{})throw std::runtime_error("Distant body changed");
 }
 bool invalid=false;try{field.Displacement(0,{0,0,0},0,2);}catch(const std::invalid_argument&){invalid=true;}
 if(!invalid)throw std::runtime_error("Invalid radius accepted");
 // Donor vertices straddle the cut; neither may be collapsed onto the cut.
 auto donorPoints=ring;donorPoints.push_back({0,1.9,0});donorPoints.push_back({0,2.1,0});
 auto donors=mouth;donors.push_back(32);donors.push_back(33);DilatingCollar donorField(donorPoints,ring,donors);
 for(double radius:{.5,2.,8.})for(double angle:{-.5,0.,.7}){
  auto a=donorField.Displacement(32,{0,0,0},radius,2,angle),b=donorField.Displacement(33,{0,0,0},radius,2,angle);
  if(1.9+a[1]>=2.1+b[1])throw std::runtime_error("Radial donor spacing collapsed");
  for(unsigned i=0;i<32;i++){
   auto d=donorField.Displacement(i,{0,0,0},radius,2,angle),p=ring[i];for(unsigned k=0;k<3;k++)p[k]+=d[k];
   const double expected=radius*(1+.1*std::pow(std::max(0.,-ring[i][2]/2),2));
   const double transverse=p[2]*std::cos(angle)-p[0]*std::sin(angle);
   if(std::abs(std::hypot(p[1],transverse)-expected)>1e-9)throw std::runtime_error("Tilted opening changed girth");
  }
 }
 std::cout<<"160 dimensional samples pass; native attachment not established\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
