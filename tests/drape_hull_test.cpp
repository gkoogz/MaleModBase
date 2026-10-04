#include <malemod/garments/jockstrap.hpp>
#include <iostream>
#include <map>
#include <set>
using namespace malemod::garments;
static void Check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
static Sample At(Point p,unsigned id){Sample s;s.position=p;s.normal=Unit(Add(p,{.11,.23,.37}));s.lineage.donors[0]={Surface::Anatomy,id,1};return s;}
static void Audit(const drape::HullSurface& hull,const std::vector<Sample>& input,double scale){
 Check(hull.samples.size()==input.size(),"Hull lost retained source samples");
 for(unsigned i=0;i<input.size();i++)Check(hull.samples[i].position==input[i].position&&hull.samples[i].lineage.donors[0].vertex==input[i].lineage.donors[0].vertex,"Hull reordered or replaced source lineage");
 Check(hull.faces.size()>=4,"Hull lacks a closed volume");
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 std::set<std::array<unsigned,3>> triangles;double signedVolume=0;
 auto origin=input.front().position;
 for(auto face:hull.faces){
  Check(face[0]<input.size()&&face[1]<input.size()&&face[2]<input.size(),"Hull index exceeds source");
  auto ordered=face;std::sort(ordered.begin(),ordered.end());Check(triangles.insert(ordered).second,"Hull duplicated a triangle");
  auto a=hull.samples[face[0]].position,b=hull.samples[face[1]].position,c=hull.samples[face[2]].position;
  auto cross=Cross(Sub(b,a),Sub(c,a));Check(Length(cross)>scale*scale*1e-12,"Hull contains a degenerate face");
  auto normal=Unit(cross);
  for(auto s:input)Check(Dot(normal,Sub(s.position,a))<=scale*2e-9,"Measured source prominence lies outside tension hull");
  signedVolume+=Dot(Sub(a,origin),Cross(Sub(b,origin),Sub(c,origin)))/6;
  for(unsigned k=0;k<3;k++){auto x=face[k],y=face[(k+1)%3];auto& edge=edges[std::minmax(x,y)];edge.first++;edge.second+=x<y?1:-1;}
 }
 Check(signedVolume>scale*scale*scale*1e-10,"Hull volume is inward or degenerate");
 for(auto e:edges)Check(e.second.first==2&&e.second.second==0,"Hull is open, nonmanifold or inconsistently oriented");
}
static Point Transform(Point p){return Add(Mul({p[2],-p[0],-p[1]},100),{870,-932,512});}
int main(){try{
 std::vector<Sample> box;
 for(unsigned i=0;i<8;i++)box.push_back(At({(i&1)?1.:-1.,(i&2)?2.:-2.,(i&4)?3.:-3.},i));
 box.push_back(At({0,0,0},8));box.push_back(At(box[0].position,9));box.push_back(At({0,0,3},10));
 auto hull=drape::TensionHull(box);Audit(hull,box,6);
 std::vector<Sample> cloud;
 for(unsigned i=0;i<400;i++){double y=1.-2.*(i+.5)/400.,radius=std::sqrt(1-y*y),angle=i*2.399963229728653;cloud.push_back(At({radius*std::cos(angle),y,radius*std::sin(angle)},i));}
 cloud.push_back(At({0,0,0},400));cloud.push_back(cloud[11]);cloud.back().lineage.donors[0].vertex=401;
 auto sphere=drape::TensionHull(cloud);Audit(sphere,cloud,2);
 auto moved=cloud;for(auto& s:moved){s.position=Transform(s.position);s.normal={s.normal[2],-s.normal[0],-s.normal[1]};}
 auto transformed=drape::TensionHull(moved);Audit(transformed,moved,200);
 // Compare analytic support values rather than a particular triangulation of
 // coplanar faces. Rotation may choose another valid initial tetrahedron.
 for(Point direction:std::vector<Point>{{1,0,0},{0,1,0},{0,0,1},Unit({.2,-.3,.5})}){
  double first=-1e100,second=-1e100;auto turned=Point{direction[2],-direction[0],-direction[1]};
  for(auto face:sphere.faces)for(auto id:face)first=(std::max)(first,Dot(cloud[id].position,direction));
  for(auto face:transformed.faces)for(auto id:face)second=(std::max)(second,Dot(Sub(moved[id].position,{870,-932,512}),turned)/100);
  Check(std::abs(first-second)<1e-10,"Hull changes measured support under frame and unit conversion");
 }
 auto rejected=[](std::vector<Sample> samples){try{drape::TensionHull(samples);return false;}catch(const std::invalid_argument&){return true;}};
 Check(rejected({At({0,0,0},0),At({1,0,0},1),At({2,0,0},2),At({3,0,0},3)}),"Hull accepts collinear source");
 Check(rejected({At({0,0,0},0),At({1,0,0},1),At({1,1,0},2),At({0,1,0},3)}),"Hull accepts planar source");
 auto invalid=box;invalid.back().position[1]=std::numeric_limits<double>::quiet_NaN();
 Check(rejected(invalid),"Hull accepts nonfinite retained source");
 std::cout<<"PASS measured hull enclosure, closed oriented topology, retained source lineage, duplicates, interior points, rigid/unit covariance and invalid inputs\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
