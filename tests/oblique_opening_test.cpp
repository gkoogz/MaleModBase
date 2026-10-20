#include <malemod/surface/root_transition.hpp>
#include <iostream>
using namespace malemod::surface;
static double length(PrecisePoint p){return std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);}
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){try{
 constexpr double pi=3.14159265358979323846;
 // An explicitly measured origin must not be replaced by a tessellation-
 // dependent average, and must preserve the exact neutral body response.
 {
  PrecisePoint anchor{1,2,3};std::vector<PrecisePoint> opening{{1,5,3},{1,2,6},{1,-1,3},{1,2,0},{1,4,5}};
  auto rest=opening;rest.push_back({1,30,3});RootTransition explicitAnchor(rest,5,opening,{1,0,0},{0,0,1},anchor);
  require(explicitAnchor.Root()==anchor,"Measured attachment anchor was averaged away");
  GraftFrame neutral{anchor,{1,0,0},{0,0,1},2,20,1};
  for(unsigned i=0;i<rest.size();i++)require(length(explicitAnchor.BodyDisplacement(i,neutral,2))==0,"Measured anchor changed neutral body");
  bool rejected=false;try{RootTransition invalid(rest,5,opening,{1,0,0},{0,0,1},PrecisePoint{NAN,2,3});}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"Nonfinite anchor accepted");
 }
 for(double scale:{.01,1.,100.})for(double degrees:{-60.,-30.,0.,15.,30.,45.,60.}){
  double theta=degrees*pi/180.;GraftFrame pose{{0,0,0},{std::cos(theta),0,std::sin(theta)},{-std::sin(theta),0,std::cos(theta)},6*scale,30*scale,scale};
  std::vector<PrecisePoint> opening;for(unsigned i=0;i<96;i++){double phi=2*pi*i/96;opening.push_back({0,3*scale*std::cos(phi),3*scale*std::sin(phi)});}
  auto rest=opening;rest.push_back({0,100*scale,0});RootTransition law(rest,96,opening);
  for(unsigned i=0;i<96;i++){
   auto budget=law.ArcAnteriorLimit(i,pose,2*scale);
   require(budget>=0&&budget<=2*scale,"Arc budget is unbounded");
   auto d=law.ObliqueBodyDisplacement(i,pose,2*scale,budget),p=rest[i];for(unsigned a=0;a<3;a++)p[a]+=d[a];
   // Distance to the live centreline is the actual transported section,
   // independent of the opening's elliptical appearance in the body plane.
   double axial=0;for(unsigned a=0;a<3;a++)axial+=p[a]*pose.axis[a];for(unsigned a=0;a<3;a++)p[a]-=axial*pose.axis[a];
   double inferior=std::pow(std::max(0.,-std::sin(2*pi*i/96)),2),expected=pose.radius*(1.03+.20*inferior);
   require(std::abs(length(p)-expected)<1e-9*scale,"Opening does not accommodate full shaft section and raphe allowance");
  }
  auto exterior=law.ObliqueBodyDisplacement(96,pose,2*scale);require(length(exterior)==0,"Exterior pelvis moved outside recruitment support");
  pose.radius=2*scale;for(unsigned i=0;i<rest.size();i++)require(length(law.ObliqueBodyDisplacement(i,pose,2*scale))==0,"Neutral pelvis changed");
 }
 std::cout<<"Oblique opening: full circumference, lower allowance, neutral/exterior preservation and unit covariance passed\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
