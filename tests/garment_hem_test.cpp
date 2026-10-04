#include "garment_hem_checks.hpp"
#include <iostream>
using namespace malemod::garments;
static Output Fixture(double scale=1,Point offset={}){
 Output out;out.style=Style::WhiteJockstrap;out.measuredCircumference=scale;out.layout.revision=2;out.layout.sheet={0,8,8};Frame frame;frame.origin=offset;
 for(unsigned row=0;row<=8;row++)for(unsigned col=0;col<=8;col++){Vertex v;v.position=Add(offset,Mul({double(col)/8,0,1.-double(row)/8},scale));v.normal={0,1,0};v.tangent={1,0,0};out.mesh.vertices.push_back(v);}
 for(unsigned col=0;col<=8;col++)out.layout.topSeam.push_back(col);
 out.layout.bottomSeams[0]={72,73,74};out.layout.bottomSeams[1]={78,79,80};Parameters parameters;
 for(unsigned side=0;side<2;side++){std::vector<Sample> path;for(unsigned row=0;row<=8;row++){unsigned id=row*9+(side?8:0);out.layout.sideBoundary[side].push_back(id);Sample s;s.position=Add(out.mesh.vertices[id].position,{0,.0009*scale,0});s.normal={0,1,0};path.push_back(s);}unsigned start=unsigned(out.mesh.vertices.size());detail::Tube(out.mesh,path,parameters.hemWidth*scale,parameters.hemThickness*scale,false,frame,MaterialSlot::WhiteElastic);out.layout.sideHems[side]={start,9,4};}detail::Shading(out.mesh);return out;
}
int main(){try{
 auto rest=Fixture();auto measurements=hem_checks::Check(rest,{},true);if(std::abs(measurements.maximumCenterOffset-.0009)>1e-12)throw std::runtime_error("Outward hem thickness offset changed");
 auto transformed=Fixture(100,{1000,-1200,750});hem_checks::Check(transformed,{},true);
 auto reject=[&](Output bad,bool smooth=false){bool failed=false;try{hem_checks::Check(bad,{},smooth);}catch(const std::runtime_error&){failed=true;}if(!failed)throw std::runtime_error("Invalid or discontinuous side hem accepted");};
 auto bad=rest;bad.layout.sideBoundary[0][4]++;reject(bad);bad=rest;bad.layout.sideHems[1].sections--;reject(bad);bad=rest;bad.mesh.vertices[bad.layout.sideHems[0].start].position[1]+=.02;reject(bad);
 bad=rest;bad.layout.topSeam.front()++;reject(bad);bad=rest;bad.layout.bottomSeams[1].back()--;reject(bad);
 bad=rest;auto hem=bad.layout.sideHems[0];bad.mesh.triangles.erase(std::remove_if(bad.mesh.triangles.begin(),bad.mesh.triangles.end(),[&](auto t){bool left=false,right=false;for(auto id:t.vertices){left=left||(id>=hem.start+12&&id<hem.start+16);right=right||(id>=hem.start+16&&id<hem.start+20);}return left&&right;}),bad.mesh.triangles.end());reject(bad);
 bad=rest;unsigned row=4;bad.mesh.vertices[row*9].position[0]+=1;for(unsigned k=0;k<4;k++)bad.mesh.vertices[hem.start+row*4+k].position[0]+=1;reject(bad,true);
 std::cout<<"PASS two continuous side hems, exact sheet binding, outward thickness offset, section integrity, topology gaps, kink rejection and translated100x unit contract\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
