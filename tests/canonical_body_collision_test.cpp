#include <malemod/clinical/canonical_body_collision.hpp>
#include <iostream>
#include <limits>
#ifdef MALEMOD_BODY_COLLISION_REFERENCE
#include MALEMOD_BODY_COLLISION_REFERENCE
#endif
using namespace malemod;
using Body=clinical::CanonicalBodyCollision;
#define REQUIRE(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';return 1;}}while(0)
int main(){
 Body body;std::vector<V3> vertices;std::vector<Body::Triangle> triangles;
 for(unsigned i=0;i<64;i++){float x=float(i%8)*4,y=float(i/8)*4;unsigned base=unsigned(vertices.size());vertices.insert(vertices.end(),{{x,y,0},{x+3,y,0},{x,y+3,0}});triangles.push_back({base,base+1,base+2,static_cast<unsigned char>(i%3),100+i});}
 body.SetTopology(triangles);body.Refit(vertices);body.SetEmitter({0,0,0},false);
#ifdef MALEMOD_BODY_COLLISION_REFERENCE
 original::fluidCollisionVertices=vertices;original::fluidCollisionTriangles.clear();for(auto t:triangles)original::fluidCollisionTriangles.push_back({t.a,t.b,t.c,t.section,t.source});original::fluidTriangleOrder.resize(triangles.size());for(unsigned i=0;i<triangles.size();i++)original::fluidTriangleOrder[i]=i;original::FluidBuildBvh(0,unsigned(triangles.size()));original::FluidRefitBvh(0);
#endif
 for(unsigned i=0;i<256;i++){
  const float x=float(i%32),y=float((i*7)%32);clinical::volumeFluid::FluidImpact impact;float best=1.0001f;const bool hit=body.Sweep({x+.2f,y+.2f,5},{x+.2f,y+.2f,-5},.1f,impact,best);
#ifdef MALEMOD_BODY_COLLISION_REFERENCE
  clinical::volumeFluid::FluidImpact reference;float oldBest=1.0001f;const bool old=original::FluidBodySweep({x+.2f,y+.2f,5},{x+.2f,y+.2f,-5},.1f,reference,oldBest);REQUIRE(hit==old&&best==oldBest);if(hit)REQUIRE(Length(impact.p-reference.p)==0&&Length(impact.n-reference.n)==0&&impact.triangle==reference.triangle&&impact.baryU==reference.baryU&&impact.baryV==reference.baryV&&impact.section==reference.section&&impact.sourceTriangle==reference.sourceTriangle);
#endif
  if(hit)REQUIRE(impact.kind==clinical::volumeFluid::FLUID_IMPACT_BODY&&std::abs(impact.p.z)<1e-6f);
 }
 clinical::volumeFluid::FluidImpact anchor;float best=1.0001f;REQUIRE(body.Sweep({.5f,.5f,3},{.5f,.5f,-3},.1f,anchor,best));REQUIRE(anchor.sourceTriangle==100);
 V3 point,normal;REQUIRE(body.Resolve(anchor,point,normal)&&Length(point-anchor.p)<1e-6f);
 body.SetEmitter(anchor.p);best=1.0001f;clinical::volumeFluid::FluidImpact out;REQUIRE(!body.Sweep({.5f,.5f,3},{.5f,.5f,-3},.1f,out,best));best=1.0001f;REQUIRE(body.Sweep({.5f,.5f,3},{.5f,.5f,-3},.1f,out,best,false));
 REQUIRE(body.Project(anchor,{.6f,.6f,0},out));
 for(auto& p:vertices)p.z=2;body.Refit(vertices);REQUIRE(body.Resolve(anchor,point,normal)&&point.z==2);REQUIRE(body.Project(anchor,{.6f,.6f,2},out)&&out.p.z==2);
 std::reverse(triangles.begin(),triangles.end());body.SetTopology(triangles);body.Refit(vertices);REQUIRE(body.Resolve(anchor,point,normal)&&point.z==2);
 body.Clear();REQUIRE(!body.Resolve(anchor,point,normal));best=1.0001f;REQUIRE(!body.Sweep({0,0,1},{0,0,-1},1,out,best));
 body.SetTopology({{0,1,2,0,0}});bool rejected=false;try{body.Refit({{0,0,0}});}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 std::cout<<"PASS exact source BVH/sweep differential, moving barycentric body anchors, topology reorder, outlet exclusion, body projection, empty and invalid snapshots.\n";
}
