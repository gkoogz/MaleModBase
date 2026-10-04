#include <malemod/garments/jockstrap.hpp>
#include <iostream>
using namespace malemod::garments;
using Side=detail::BodyCollider::Side;
static void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 std::vector<Sample> vertices(12);std::array<Point,6> outline{{{0,0,0},{2,0,0},{2,1,0},{1,1,0},{1,2,0},{0,2,0}}};
 for(unsigned i=0;i<6;i++){vertices[i].position=outline[i];vertices[i].position[2]=-.4;vertices[i+6].position=outline[i];vertices[i+6].position[2]=.4;}
 std::vector<std::array<std::uint32_t,3>> faces;std::array<std::array<unsigned,3>,4> cap{{{0,1,3},{1,2,3},{0,3,5},{3,4,5}}};
 for(auto f:cap){faces.push_back({f[2],f[1],f[0]});faces.push_back({f[0]+6,f[1]+6,f[2]+6});}
 for(unsigned i=0;i<6;i++){unsigned j=(i+1)%6;faces.push_back({i,j,j+6});faces.push_back({i,j+6,i+6});}
 detail::BodyCollider collider;collider.Update(vertices,faces);
 Check(collider.Classify({.5,.5,0})==Side::Indeterminate,"Open/unverified classification implicitly trusted");
 Check(collider.Classify({1.5,1.5,0},true)==Side::Outside,"Concave notch classified inside");
 Check(collider.Classify({1,.5,0},true)==Side::Inside,"Interior point on cap triangulation ray lost parity");
 Check(collider.Classify({1,1.5,0},true)==Side::Boundary,"Exact concave wall boundary missed");
 Check(collider.Classify({0,0,-.4},true)==Side::Boundary,"Exact vertex boundary missed");
 for(unsigned x=0;x<40;x++)for(unsigned y=0;y<40;y++)for(double z:{-.41,0.,.41}){
  Point p{-.1+x*.059,-.1+y*.057,z};bool inside=p[0]>0&&p[1]>0&&p[0]<2&&p[1]<2&&(p[0]<1||p[1]<1)&&std::abs(z)<.4;
  auto expected=inside?Side::Inside:Side::Outside;Check(collider.Classify(p,true)==expected,"Ray parity disagrees with analytic concave prism");
 }
 Point origin{1000,-900,800};auto transformed=vertices;for(auto& v:transformed)v.position=Add(origin,Mul(v.position,100));collider.Update(transformed,faces,origin,100);
 Check(collider.Classify({1.5,1.5,0},true)==Side::Outside&&collider.Classify({.2,1.2,0},true)==Side::Inside,"Membership unit/world transform covariance failed");
 for(auto& f:faces)std::swap(f[0],f[2]);collider.Update(vertices,faces);Check(collider.Classify({.2,1.2,0},true)==Side::Inside,"Parity depends on nearest face normal/winding");
 collider.Clear();Check(collider.Classify({0,0,0},true)==Side::Indeterminate,"Empty collider invents membership");
 std::cout<<"PASS closed concave surface membership against4800analyticqueries, boundary/ray degeneracy, unit transform, winding and explicit closure precondition\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
