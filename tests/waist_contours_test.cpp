#include <malemod/garments/waist_contours.hpp>
#include <iostream>
using namespace malemod::garments::waist;
struct Sample {Point position,normal;};
struct Frame {
 Point origin{},x{1,0,0},y{0,1,0},z{0,0,1};
 Point Local(Point p)const{p=detail::Sub(p,origin);auto dot=[](Point a,Point b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};return {dot(p,x),dot(p,y),dot(p,z)};}
 Point World(Point p)const{return detail::Add(origin,detail::Add(detail::Mul(x,p[0]),detail::Add(detail::Mul(y,p[1]),detail::Mul(z,p[2]))));}
 void Validate()const{}
};
struct Body {std::vector<Sample> samples;std::vector<std::array<unsigned,3>> faces;};
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';return false;}}while(0)
Body Frustum(){
 Body b;constexpr unsigned n=32;constexpr double pi=3.14159265358979323846;
 // Two independent resources with duplicated waist and UV seam vertices.
 for(unsigned part=0;part<2;part++){unsigned first=unsigned(b.samples.size());
  for(double height:{.8+.2*part,1.+.2*part})for(unsigned i=0;i<=n;i++){double theta=2*pi*(i%n)/n,a=.2+.08*(height-1),c=std::cos(theta),s=std::sin(theta);b.samples.push_back({{a*c,a*.75*s,height},detail::Unit({c,s/.75,-.08})});}
  for(unsigned i=0;i<n;i++){unsigned a=first+i;b.faces.push_back({a,a+1,a+n+2});b.faces.push_back({a,a+n+2,a+n+1});}
 }
 return b;
}
bool ExactCuts(){auto body=Frustum();Frame frame;auto band=Intersect(body.samples,body.faces,frame,.94,1.06,64);
 for(auto* contour:{&band.bottom,&band.top}){CHECK(contour->points.size()==64);double a=.2+.08*(contour->height-1),expectedArea=32*.5*a*a*.75*std::sin(2*3.14159265358979323846/32);CHECK(std::abs(contour->signedArea-expectedArea)<1e-12);
  for(auto anchor:contour->points){CHECK(std::abs(anchor.position[2]-contour->height)<1e-12);double weight=0;Point reconstructed{};for(unsigned k=0;k<4;k++){CHECK(anchor.weights[k]>=0);CHECK(anchor.vertices[k]<body.samples.size());weight+=anchor.weights[k];reconstructed=detail::Add(reconstructed,detail::Mul(body.samples[anchor.vertices[k]].position,anchor.weights[k]));}CHECK(std::abs(weight-1)<1e-12);CHECK(detail::Length(detail::Sub(reconstructed,anchor.position))<1e-12);CHECK(std::abs(detail::Length(anchor.normal)-1)<1e-12);}
 }
 CHECK(band.top.circumference>band.bottom.circumference);CHECK(std::abs(band.top.circumference/band.bottom.circumference-(.2+.08*.06)/(.2-.08*.06))<1e-12);
 // A plane exactly at the shared part seam must close once, not branch.
 auto seam=Intersect(body.samples,body.faces,frame,1.,1.06,64);CHECK(seam.bottom.points.size()==64);
 std::reverse(body.faces.begin(),body.faces.end());auto reordered=Intersect(body.samples,body.faces,frame,.94,1.06,64);for(unsigned k=0;k<64;k++)CHECK(detail::Length(detail::Sub(reordered.top.points[k].position,band.top.points[k].position))<1e-12);
 std::cout<<"PASS two exact tapered-body transverse cuts, closed part/UV aliases, four-donor attachment reconstruction and triangle-order invariance\n";return true;
}
bool Covariance(){auto body=Frustum(),moved=body;Frame a,b;double angle=.71;b.origin={1000,-300,80};b.x={std::cos(angle),0,std::sin(angle)};b.y={0,1,0};b.z={-std::sin(angle),0,std::cos(angle)};
 for(auto& s:moved.samples){s.position=b.World(s.position);s.normal=detail::Add(detail::Mul(b.x,s.normal[0]),detail::Add(detail::Mul(b.y,s.normal[1]),detail::Mul(b.z,s.normal[2])));}
 auto x=Intersect(body.samples,body.faces,a,.94,1.06,64),y=Intersect(moved.samples,moved.faces,b,.94,1.06,64);for(unsigned k=0;k<64;k++)CHECK(detail::Length(detail::Sub(b.World(x.bottom.points[k].position),y.bottom.points[k].position))<1e-9);
 auto scaled=body;for(auto& s:scaled.samples)s.position=detail::Mul(s.position,100);auto z=Intersect(scaled.samples,scaled.faces,a,94,106,64);CHECK(std::abs(z.bottom.circumference/x.bottom.circumference-100)<1e-10);for(unsigned k=0;k<64;k++)CHECK(detail::Length(detail::Sub(detail::Mul(x.top.points[k].position,100),z.top.points[k].position))<1e-10);
 std::cout<<"PASS rotated/transformed character frame and explicit100x unit covariance\n";return true;
}
bool Reject(){auto body=Frustum();Frame frame;for(unsigned mode=0;mode<4;mode++){auto altered=body;double low=.94,high=1.06;if(mode==0)std::swap(low,high);if(mode==1)altered.faces[0][0]=999999;if(mode==2)altered.samples[0].position[0]=std::nan("");if(mode==3)altered.faces.erase(altered.faces.begin()+32,altered.faces.begin()+34);bool rejected=false;try{Intersect(altered.samples,altered.faces,frame,low,high,64);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);}
 std::cout<<"PASS unordered planes, missing topology, nonfinite surface and open intersection rejection\n";return true;
}
int main(){try{return ExactCuts()&&Covariance()&&Reject()?0:1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
