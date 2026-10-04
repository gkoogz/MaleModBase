#include <malemod/garments/jockstrap.hpp>
#include <iostream>
#include <string>
using namespace malemod::garments;
static void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static bool Near(Point a,Point b,double tolerance=1e-9){return Length(Sub(a,b))<tolerance;}
static Sample At(Point p,unsigned id){Sample s;s.position=p;s.normal=Unit(Add(p,{.13,.07,.21}));s.lineage.donors[0]={Surface::Anatomy,id,1};return s;}
static Input Fixture(){
 Input in;const Point centers[]={{0,.62,-.25},{0,1.,-.35},{-.2,.32,-.7},{.2,.32,-.7}};
 for(unsigned group=0;group<4;group++){
  unsigned start=unsigned(in.anatomy.size());
  for(unsigned corner=0;corner<8;corner++){auto p=Add(centers[group],{(corner&1)?.13:-.13,(corner&2)?.11:-.11,(corner&4)?.12:-.12});auto s=At(p,start+corner);in.anatomy.push_back(s);in.anatomyRegions[group].push_back(start+corner);}
  in.anatomyTriangles.push_back({start,start+1,start+2});
 }return in;
}
static Point Along(const std::vector<Sample>& path,double fraction){
 double total=0;for(unsigned i=1;i<path.size();i++)total+=Length(Sub(path[i].position,path[i-1].position));double remaining=total*fraction;
 for(unsigned i=1;i<path.size();i++){double length=Length(Sub(path[i].position,path[i-1].position));if(remaining<=length)return Add(path[i-1].position,Mul(Sub(path[i].position,path[i-1].position),length?remaining/length:0));remaining-=length;}return path.back().position;
}
int main(){try{
 auto pathSample=[](Point p,unsigned id){auto s=At(p,id);s.normal={0,1,0};return s;};
 auto clear=[](Point,Point)->std::optional<Sample>{return {};};
 std::vector<Sample> valley{pathSample({-1,0,0},0),pathSample({0,-1,0},1),pathSample({1,0,0},2)};
 auto bridge=drape::Tauten(valley,.025,clear);Check(bridge.size()==2&&bridge.front().position==valley.front().position&&bridge.back().position==valley.back().position,"Taut cloth did not bridge a clear concave gap while preserving attachments");
 auto prominence=valley;prominence[1].position={0,1,0};auto retained=drape::Tauten(prominence,.025,clear);Check(retained.size()==3&&retained[1].position==prominence[1].position,"Collision-free shortcut went behind a protruding tissue support");
 auto blocked=drape::Tauten(valley,.025,[](Point,Point)->std::optional<Sample>{return Sample{};});Check(blocked.size()==3,"Tautening shortcut crossed a physical contact");
 std::vector<Sample> peaks{pathSample({-2,0,0},0),pathSample({-1,1,0},1),pathSample({0,.2,0},2),pathSample({1,1,0},3),pathSample({2,0,0},4)};
 auto outer=drape::Tauten(peaks,.025,clear);Check(outer.size()==4&&outer[1].position==peaks[1].position&&outer[2].position==peaks[3].position,"Cloth must retain both prominences and bridge their inward valley");
 auto scaledPeaks=peaks;for(auto& s:scaledPeaks)s.position=Add(Mul(s.position,100),{1000,-1200,750});auto scaledOuter=drape::Tauten(scaledPeaks,2.5,clear);Check(scaledOuter.size()==outer.size(),"Exterior gap bridging depends on units or world origin");for(unsigned i=0;i<outer.size();i++)Check(Near(scaledOuter[i].position,Add(Mul(outer[i].position,100),{1000,-1200,750})),"Tautening changed exterior supports under unit conversion");
 auto input=Fixture();auto original=input.anatomy;std::vector<Sample> top;for(unsigned i=0;i<9;i++){double angle=(-40.+10*i)*3.14159265358979323846/180;top.push_back(At({.5*std::sin(angle),.5*std::cos(angle),0},100+i));}
 std::array<Sample,2> bottom{At({-.2,.08,-.75},120),At({.2,.08,-.75},121)};unsigned queries=0;
 auto sheet=drape::Walk(input,top,bottom,24,.025,[&](Point a,Point b)->std::optional<Sample>{Check(Finite(a)&&Finite(b),"Contact query lost finite material coordinates");queries++;return {};},[](Sample&){});
 Check(sheet.rows==24&&sheet.columns==8&&sheet.vertices.size()==25*9&&sheet.walks.size()==9&&queries>0,"Continuous sheet dimensions/contact traversal wrong");
 for(unsigned col=0;col<9;col++){
  Check(Near(sheet.vertices[col].position,top[col].position),"Broad top seam moved off its supplied attachment");
  auto lower=Add(Mul(bottom[0].position,1.-double(col)/8),Mul(bottom[1].position,double(col)/8));Check(Near(sheet.vertices[24*9+col].position,lower),"Lower sheet edge did not return to underside seams");
  for(unsigned row=0;row<=24;row++){const auto& s=sheet.vertices[row*9+col];Check(Near(s.position,Along(sheet.walks[col],double(row)/24)),"Material rows do not follow equal arc-length walking progression");double sum=0;for(auto d:s.lineage.donors){Check(d.weight>=0&&std::isfinite(d.weight),"Invalid material lineage weight");sum+=d.weight;}Check(std::abs(sum-1)<1e-12,"Walking fabric lost exact source ancestry");}
 }
 for(unsigned i=0;i<original.size();i++)Check(input.anatomy[i].position==original[i].position&&input.anatomy[i].lineage.donors[0].vertex==original[i].lineage.donors[0].vertex,"Clothing removed or changed underlying anatomy");
 // Rotate and translate the supplied character frame and explicitly convert
 // all source dimensions. The same authored supports must produce the same
 // sheet in transformed coordinates, independently of game axes and units.
 const Point offset{1000,-1200,750};auto turn=[](Point p){return Point{-p[1],p[0],p[2]};};auto world=[&](Point p){return Add(offset,Mul(turn(p),100));};
 auto moved=input;moved.frame.origin=offset;moved.frame.lateral=turn({1,0,0});moved.frame.forward=turn({0,1,0});moved.frame.up=turn({0,0,1});
 for(auto& s:moved.anatomy){s.position=world(s.position);s.normal=turn(s.normal);}auto movedTop=top;auto movedBottom=bottom;for(auto& s:movedTop){s.position=world(s.position);s.normal=turn(s.normal);}for(auto& s:movedBottom){s.position=world(s.position);s.normal=turn(s.normal);}
 auto transformed=drape::Walk(moved,movedTop,movedBottom,24,2.5,[](Point,Point)->std::optional<Sample>{return {};},[](Sample&){});
 for(unsigned i=0;i<sheet.vertices.size();i++)Check(Near(transformed.vertices[i].position,world(sheet.vertices[i].position),1e-8),"Walking sheet depends on world origin, axes or units");
 auto invalid=[&](Input bad){bool rejected=false;try{drape::Walk(bad,top,bottom,24,.025,[](Point,Point)->std::optional<Sample>{return {};},[](Sample&){});}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Malformed authored tissue contract accepted");};
 auto bad=input;bad.anatomyRegions[1][0]=bad.anatomyRegions[0][0];invalid(bad);bad=input;bad.anatomyRegions[0][0]=unsigned(input.anatomy.size());invalid(bad);bad=input;bad.anatomyRegions[3].clear();invalid(bad);bad=input;bad.anatomyTriangles.clear();invalid(bad);
 Lineage a,b;for(unsigned i=0;i<8;i++){a.donors[i]={Surface::Body,i,.125};b.donors[i]={Surface::Anatomy,i,.125};}auto blended=detail::Blend(a,b,.5);unsigned nonzero=0;for(auto d:blended.donors)if(d.weight){Check(d.weight==.0625,"Exact sixteen-donor interpolation was renormalized or dropped");nonzero++;}Check(nonzero==16,"Expanded ancestry was truncated");
 auto repeated=detail::Blend(blended,blended,.5);for(unsigned i=0;i<16;i++)Check(repeated.donors[i].surface==blended.donors[i].surface&&repeated.donors[i].vertex==blended.donors[i].vertex&&repeated.donors[i].weight==blended.donors[i].weight,"Repeated material donors were not merged exactly");
 Lineage extra;extra.donors[0]={Surface::Body,1000,1};bool overflow=false;try{detail::Blend(blended,extra,.5);}catch(const std::invalid_argument&){overflow=true;}Check(overflow,"Material interpolation silently dropped donors beyond capacity");
 std::cout<<"PASS exterior concavity bridges, prominence preservation, physical contact rejection, continuous sheet seams, exact arc-length rows, retained anatomy, rotated/translated100x units, semantics and sixteen-donor ancestry\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
