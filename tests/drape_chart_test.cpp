#include "malemod/garments/jockstrap.hpp"
#include <iostream>
#include <limits>
using namespace malemod::garments;
static void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static Sample MeasuredPoint(Point p,unsigned id){Sample s;s.position=p;s.normal=Unit(p);s.lineage.donors[0]={Surface::Anatomy,id,1};return s;}
int main(){try{
 std::vector<Sample> seam{MeasuredPoint({-1,.1,.5},0),MeasuredPoint({0,.5,.5},1),MeasuredPoint({1,.1,.5},2)};Frame coverageFrame;
 const double seamMeanForward=.7/3;
 for(auto fixture:std::array<std::array<double,2>,5>{std::array<double,2>{1,0},{1.25,0},{1.375,.15},{1.5,.3},{2,.3}}){
  std::vector<Sample> tissue{MeasuredPoint({.2,2*fixture[0]+seamMeanForward,.2},3),MeasuredPoint({30,-90,2},4)};
  auto g=drape::MeasureCoverageGrowth(seam,tissue,coverageFrame,.3);Check(std::abs(g.frontArcWidth-2)<1e-14&&std::abs(g.reach-2*fixture[0])<1e-14&&std::abs(g.aspect-fixture[0])<1e-14,"Coverage growth did not measure complete tissue against actual mean sewn arc");Check(std::abs(g.effectiveExtra-fixture[1])<1e-14,"Coverage growth quarter-width transition is incorrect");
  auto disabled=drape::MeasureCoverageGrowth(seam,tissue,coverageFrame,0);Check(disabled.effectiveExtra==0,"Zero requested coverage changed the pattern");
 }
 std::vector<Sample> tissue{MeasuredPoint({.2,2*1.375+seamMeanForward,.2},3),MeasuredPoint({30,-90,2},4)};
 auto referenceGrowth=drape::MeasureCoverageGrowth(seam,tissue,coverageFrame,.3);
 for(double units:{.001,1.,1000.}){Frame f;f.origin={910,-180,56};f.lateral={0,1,0};f.forward={-1,0,0};auto transform=[&](std::vector<Sample> values){for(auto& s:values)s.position=f.World(Mul(s.position,units));return values;};auto g=drape::MeasureCoverageGrowth(transform(seam),transform(tissue),f,.3);Check(std::abs(g.aspect-referenceGrowth.aspect)<1e-9&&std::abs(g.effectiveExtra-referenceGrowth.effectiveExtra)<1e-9,"Coverage growth depends on game units, translation or rotation");}
 for(double invalid:{-.001,1.5001,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){bool rejected=false;try{drape::MeasureCoverageGrowth(seam,tissue,coverageFrame,invalid);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Coverage growth accepted invalid maximum");}
 auto collapsedSeam=seam;collapsedSeam.back().position=collapsedSeam.front().position;bool rejected=false;try{drape::MeasureCoverageGrowth(collapsedSeam,tissue,coverageFrame,.3);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Coverage growth accepted zero native sewn width");
 auto malformedTissue=tissue;malformedTissue.back().position[1]=std::numeric_limits<double>::quiet_NaN();rejected=false;try{drape::MeasureCoverageGrowth(seam,malformedTissue,coverageFrame,.3);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Coverage growth omitted a nonfinite native tissue sample");
 // Different nonlinear side speeds must still share a single row map. Their
 // mean measured physical length, not either projected angular coordinate,
 // defines every common material interval.
 std::vector<std::array<Point,2>> paths(1025);for(unsigned i=0;i<paths.size();i++){double t=double(i)/(paths.size()-1);paths[i]={Point{t*t,0,0},Point{0,3*t*t*t,0}};}
 auto parameters=drape::RadialChart::CommonRows(paths,32);Check(parameters.front()==0&&parameters.back()==1,"Arc-length chart moved its sewn ends");
 auto measure=[](double t){return .5*(t*t+3*t*t*t);};
 for(unsigned i=1;i<parameters.size();i++){Check(parameters[i]>parameters[i-1],"Common physical row map is not strictly monotone");Check(std::abs(measure(parameters[i])-2*double(i)/32)<2e-6,"Common row map does not equalize measured mean side length");}
 auto movedPaths=paths;for(auto& sides:movedPaths)for(auto& p:sides)p={100*p[1]+910,-100*p[0]-180,56};auto movedParameters=drape::RadialChart::CommonRows(movedPaths,32);for(unsigned i=0;i<parameters.size();i++)Check(std::abs(parameters[i]-movedParameters[i])<1e-12,"Physical row map depends on origin, rotation or units");
 std::vector<Sample> cloud;for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})cloud.push_back(MeasuredPoint({double(x),double(y),double(z)},unsigned(cloud.size())));
 auto hull=drape::TensionHull(cloud);Frame frame;drape::RadialChart chart(hull,frame,.03);
 for(unsigned i=0;i<300;i++){double a=i*.213,b=i*.917;auto s=chart.Cast({std::sin(a)*std::cos(b),std::cos(a)*std::cos(b),std::sin(b)});double clearance=(std::max)({std::abs(s.position[0])-1,std::abs(s.position[1])-1,std::abs(s.position[2])-1});Check(std::abs(clearance-.03)<1e-10,"Chart ray does not lie on exact expanded measured hull");double sum=0;for(auto d:s.lineage.donors){Check(d.weight>=0&&std::isfinite(d.weight),"Invalid chart donor");if(d.weight)Check(d.vertex<8,"Chart invented source lineage");sum+=d.weight;}Check(std::abs(sum-1)<1e-12,"Chart lost source interpolation weights");}
 std::vector<Sample> top;for(unsigned i=0;i<=64;i++)top.push_back(MeasuredPoint({-.65+1.3*i/64.,-1.,.85+.03*std::cos(i*.1)},i%8));std::array<Sample,2> bottom{MeasuredPoint({-.12,-.4,-1},0),MeasuredPoint({.12,-.4,-1},1)};auto sheet=chart.Make(top,bottom,32);
 Check(Parameters{}.sideCoverageExtra==.45,"Common coverage maximum differs from declared pattern setting");
 for(double extra:{-.001,1.5001,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
  bool rejected=false;try{chart.Make(top,bottom,32,extra);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Direct chart accepted invalid side coverage setting");
  rejected=false;try{Parameters p;p.sideCoverageExtra=extra;Session session(p);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Garment session accepted invalid side coverage setting");
 }
 auto zero=chart.Make(top,bottom,32,0);for(unsigned i=0;i<sheet.vertices.size();i++){const auto& a=sheet.vertices[i];const auto& b=zero.vertices[i];Check(a.position==b.position&&a.normal==b.normal,"Default zero extra changes the reference rest sheet");for(unsigned k=0;k<a.lineage.donors.size();k++){auto x=a.lineage.donors[k],y=b.lineage.donors[k];Check(x.surface==y.surface&&x.vertex==y.vertex&&x.weight==y.weight,"Explicit zero extra changes measured source donors");}}
 for(auto f:cloth_detail::SheetTriangles(0,32,64)){auto a=sheet.vertices[f[0]].position,b=sheet.vertices[f[1]].position,c=sheet.vertices[f[2]].position;if(Dot(Cross(Sub(b,a),Sub(c,a)),Add(Add(a,b),c))<=0){for(auto p:{a,b,c})std::cerr<<p[0]<<','<<p[1]<<','<<p[2]<<'\n';throw std::runtime_error("Coherent chart reverses exterior material triangle "+std::to_string(f[0])+","+std::to_string(f[1])+","+std::to_string(f[2]));}}
 for(unsigned col=0;col<top.size();col++){Check(sheet.vertices[col].position==top[col].position,"Chart moved a measured waistband seam");for(unsigned k=0;k<top[col].lineage.donors.size();k++){auto a=sheet.vertices[col].lineage.donors[k],b=top[col].lineage.donors[k];Check(a.surface==b.surface&&a.vertex==b.vertex&&a.weight==b.weight,"Chart changed measured waistband donors");}}
 Frame moved;moved.origin={910,-180,56};moved.lateral={0,1,0};moved.forward={-1,0,0};double scale=100;auto transform=[&](Sample s){s.position=moved.World(Mul(s.position,scale));s.normal=Add(Mul(moved.lateral,s.normal[0]),Add(Mul(moved.forward,s.normal[1]),Mul(moved.up,s.normal[2])));return s;};auto points=cloud;for(auto& s:points)s=transform(s);auto transformedHull=drape::TensionHull(points);drape::RadialChart transformed(transformedHull,moved,.03*scale);auto transformedTop=top;for(auto& s:transformedTop)s=transform(s);auto transformedBottom=bottom;for(auto& s:transformedBottom)s=transform(s);auto other=transformed.Make(transformedTop,transformedBottom,32);
 for(unsigned i=0;i<sheet.vertices.size();i++)Check(Length(Sub(other.vertices[i].position,transform(sheet.vertices[i]).position))<1e-9,"Chart depends on game units, origin or rotation");
 std::cout<<"Measured coherent radial chart tests passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
