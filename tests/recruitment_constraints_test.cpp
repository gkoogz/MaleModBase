#include <malemod/surface/recruitment_constraints.hpp>
#include <iostream>
using namespace malemod::surface;
static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 for(double scale:{.01,1.,100.}){
  GraftDomain d;d.points={{{0,0,0}},{{scale,0,0}},{{0,scale,0}},{{scale,scale,0}},{{scale*.5,0,0}}};
  d.triangles={{{0,4,2}},{{4,1,2}},{{1,3,2}}};d.seams={{4,0,1,.5}};d.protectedVertices={0,2,3};d.orientationTriangles={0,1,2};
  RecruitmentConstraints law(d);auto out=law.Project(d.points,d.points,{{4,{0,0,1},.4*scale}});
  require(out[4][2]>=.4*scale-1e-7,"Bounded seam remained below its required support");
  for(unsigned a=0;a<3;a++)require(std::abs(out[4][a]-.5*(out[0][a]+out[1][a]))<1e-10*scale,"Original-edge donor relation changed");
  for(auto i:d.protectedVertices)require(out[i]==d.points[i],"Protected surrounding tissue changed");
  bool rejected=false;try{law.Project(d.points,d.points,{{0,{0,0,1},scale}});}catch(const std::runtime_error&){rejected=true;}
  require(rejected,"An infeasible fixed support was accepted");
  for(unsigned steps:{1u,32u}){
   auto exact=law.Project(d.points,d.points,{{1,{1,0,0},scale},{1,{-1,0,0},-scale},
      {1,{0,0,1},.4*scale},{1,{0,0,-1},-.4*scale}},steps);
   require(std::abs(exact[1][0]-scale)<1e-10*scale&&std::abs(exact[1][2]-.4*scale)<1e-10*scale,
           "Opposed planes did not preserve exact section coordinates");
   require(exact[0]==d.points[0]&&exact[2]==d.points[2],"Exact section moved fixed tissue");
  }
  bool badCount=false;try{law.Project(d.points,d.points,{},0);}catch(const std::invalid_argument&){badCount=true;}
  require(badCount,"Zero continuation steps were accepted");
  bool captured=false;try{law.Project(d.points,d.points,{{0,{0,0,1},scale}},1);}
  catch(const RecruitmentConstraintFailure& e){captured=e.candidate.size()==d.points.size();}
  require(captured,"Rejected surface did not preserve diagnostic evidence");
  d.points.push_back({.75*scale,0,0});d.seams.push_back({5,4,1,.5});
  auto nested=RecruitmentConstraints(d).Project(d.points,d.points,{{5,{0,0,1},.3*scale}});
  for(unsigned a=0;a<3;a++)require(std::abs(nested[5][a]-.5*(nested[4][a]+nested[1][a]))<1e-10*scale,
      "Nested original-edge donors changed");
 }
 GraftDomain d;d.points={{{0,0,0}},{{1,0,0}},{{0,1,0}}};d.triangles={{{0,1,2}}};d.orientationTriangles={0};d.protectedVertices={0,2};
 auto reference=d.points,target=d.points;reference[1][0]=20;target[1][0]=1e-6;
 RecruitmentConstraints law(d);auto out=law.Project(reference,target);
 require(out[1][0]>=.020001*20,"Projected body bound did not preserve the intrinsic area floor");
 require(out[0]==reference[0]&&out[2]==reference[2],"Body area recovery changed fixed tissue");
 std::cout<<"Recruitment constraints: directed body area, support floor, exact donors, fixed tissue and unit covariance passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
