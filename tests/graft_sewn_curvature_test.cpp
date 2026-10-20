#include <malemod/surface/graft_runtime.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace malemod::surface;

static GraftDomain FlatJoinedStrips(){
 GraftDomain d;
 for(double x:{-1.,0.,0.,2.})for(double y:{-1.,0.,1.})
  d.points.push_back({x,y,.3*x+.2*y});
 d.triangles={{{0,3,4}},{{0,4,1}},{{1,4,5}},{{1,5,2}},
              {{6,9,10}},{{6,10,7}},{{7,10,11}},{{7,11,8}}};
 d.seams={{6,3,4,0},{7,4,5,0},{8,5,4,0}};
 d.protectedVertices={0,1,2,3,5,9,10,11};
 d.preserveTargetDifferential=true;
 d.surfaceFairingDistance=1;d.surfaceFairingStrength=64;
 return d;
}
static double Maximum(const std::vector<PrecisePoint>& p){
 double value=0;for(auto q:p)for(double x:q)value=std::max(value,std::abs(x));return value;
}
int main(){try{
 auto domain=FlatJoinedStrips();GraftFrame frame{{0,0,0},{1,0,0},{0,0,1},2.9,10};
 frame.seamSupportRadius=2;
 std::vector<PrecisePoint> zero(domain.points.size());
 const auto old=GraftPlan(domain,frame).SolveDisplacement(zero);
 domain.surfaceFairingRetention.assign(domain.points.size(),1);
 if(GraftPlan(domain,frame).SolveDisplacement(zero)!=old)throw std::runtime_error("Unit retention changed replay");
 domain.surfaceFairingRetention.assign(domain.points.size(),0);
 if(Maximum(GraftPlan(domain,frame).SolveDisplacement(zero))>1e-11)throw std::runtime_error("Released fairing still contracts flat target");
 domain.surfaceFairingRetention[0]=1.01;
 bool invalid=false;try{GraftPlan rejected(domain,frame);}catch(const std::invalid_argument&){invalid=true;}
 if(!invalid)throw std::runtime_error("Out-of-range retention accepted");
 domain.surfaceFairingRetention.clear();
 domain.coupledCurvature=true;
 const auto joined=GraftPlan(domain,frame).SolveDisplacement(zero);
 std::cout<<"Flat sewn interface: old drift="<<Maximum(old)<<", joined drift="<<Maximum(joined)<<std::endl;
 if(Maximum(old)<1e-5)throw std::runtime_error("Fixture does not expose separate seam flux penalty");
 if(Maximum(joined)>1e-11)throw std::runtime_error("Flat sewn surface acquired curvature");
 for(auto i:domain.protectedVertices)if(joined[i]!=zero[i])throw std::runtime_error("Protected edge moved");
 for(auto e:domain.seams)for(unsigned a=0;a<3;a++)
  if(std::abs(joined[e.slave][a]-(1-e.weight)*joined[e.a][a]-e.weight*joined[e.b][a])>1e-13)
   throw std::runtime_error("Original-edge relation changed");
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
