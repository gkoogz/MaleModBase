#include <malemod/garments/jockstrap.hpp>
#include <iostream>
using namespace malemod::garments;
static void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static Input Fixture(){
 Input input;std::array<Point,8> body{{{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}};
 for(unsigned i=0;i<body.size();i++){Sample s;s.position=body[i];s.normal=Unit(body[i]);s.lineage.donors[0]={Surface::Body,i,1};input.bodySurface.push_back(s);}
 // Actual body has its open graft at the positive-forward face.
 input.bodyTriangles={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{3,0,4},{3,4,7}};
 std::array<Point,8> perimeter{{{-1,0,-1},{-1,0,0},{-1,0,1},{0,0,1},{1,0,1},{1,0,0},{1,0,-1},{0,0,-1}}};
 for(unsigned layer=0;layer<2;layer++)for(auto p:perimeter){Sample s;s.position={p[0],layer?2.5:1.,p[2]};s.normal=Unit({p[0],0,p[2]});s.lineage.donors[0]={Surface::Anatomy,unsigned(input.anatomy.size()),1};input.anatomy.push_back(s);}
 Sample tip;tip.position={0,2.5,0};tip.normal={0,1,0};tip.lineage.donors[0]={Surface::Anatomy,16,1};input.anatomy.push_back(tip);
 auto face=[&](unsigned a,unsigned b,unsigned c,Point outward){if(Dot(Cross(Sub(input.anatomy[b].position,input.anatomy[a].position),Sub(input.anatomy[c].position,input.anatomy[a].position)),outward)<0)std::swap(b,c);input.anatomyTriangles.push_back({a,b,c});};
 for(unsigned i=0;i<8;i++){unsigned j=(i+1)%8;auto n=Add(input.anatomy[i].normal,input.anatomy[j].normal);face(i,j,8+j,n);face(i,8+j,8+i,n);face(8+i,8+j,16,{0,1,0});input.opening.push_back(input.anatomy[i]);}
 return input;
}
int main(){try{
 auto input=Fixture();auto originalBody=input.bodyTriangles,originalAnatomy=input.anatomyTriangles;
 bool bodyOpen=false;try{waist::Intersect(input.bodySurface,input.bodyTriangles,input.frame,.2,.4,32);}catch(const std::invalid_argument&){bodyOpen=true;}Check(bodyOpen,"Fixture body-only waist cut was not genuinely open at graft");
 auto joined=waist::JoinedPhysicalSurface(input,8);Check(joined.samples.size()==25&&joined.triangles.size()==38&&joined.refinedBodyTriangles==14,"Joined physical waist invented cap vertices or faces");
 auto band=waist::Intersect(joined.samples,joined.triangles,input.frame,.2,.4,32);
 unsigned anatomyAttachments=0;
 for(auto* contour:{&band.bottom,&band.top}){
  Check(std::abs(contour->circumference-11)<1e-12&&std::abs(contour->signedArea-7)<1e-12,"Waistband did not follow actual protruding tissue surface");
  for(auto attachment:contour->points){Point p{};for(unsigned k=0;k<4;k++)if(attachment.weights[k]){const auto& sample=joined.samples.at(attachment.vertices[k]);p=Add(p,Mul(sample.position,attachment.weights[k]));anatomyAttachments+=sample.lineage.donors[0].surface==Surface::Anatomy;}Check(Length(Sub(p,attachment.position))<1e-12,"Joined cut lost native donor reconstruction");}
 }
 Check(anatomyAttachments>0&&input.bodyTriangles==originalBody&&input.anatomyTriangles==originalAnatomy,"Joined cut omitted tissue lineage or changed native topology");
 auto moved=input;Point origin{1000,-1200,750};moved.frame.origin=origin;for(auto* samples:{&moved.bodySurface,&moved.anatomy,&moved.opening})for(auto& s:*samples)s.position=Add(origin,Mul(s.position,100));auto expanded=waist::JoinedPhysicalSurface(moved,800);auto other=waist::Intersect(expanded.samples,expanded.triangles,moved.frame,20,40,32);Check(expanded.triangles==joined.triangles,"Physical graft refinement depends on origin or units");
 for(unsigned i=0;i<32;i++)Check(Length(Sub(other.bottom.points[i].position,Add(origin,Mul(band.bottom.points[i].position,100))))<1e-9,"Physical waist cut changed under explicit100x conversion");
 auto bent=input;for(auto row:std::array<RootSubdivision,4>{{{1,0,2,.5},{3,2,4,.5},{5,4,6,.5},{7,6,0,.5}}}){bent.rootSubdivisions.push_back(row);bent.anatomy[row.vertex].position[1]+=.01;bent.opening[row.vertex]=bent.anatomy[row.vertex];}auto bentJoined=waist::JoinedPhysicalSurface(bent,8);Check(bentJoined.samples.size()==joined.samples.size()&&bentJoined.refinedBodyTriangles==joined.refinedBodyTriangles,"Animated waist cut lost authored edge ownership");for(unsigned i=0;i<bent.anatomy.size();i++)Check(bentJoined.samples[input.bodySurface.size()+i].position==bent.anatomy[i].position,"Animated cut altered native tissue");
 auto bad=input;bad.opening.pop_back();bool rejected=false;try{waist::JoinedPhysicalSurface(bad,8);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Missing original tissue boundary accepted");
 std::cout<<"PASS actual body+graft transverse cuts, coarse-to-refined edge weld, no physical caps, exact mixed tissue lineage, unchanged native topology and translated100x units\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
