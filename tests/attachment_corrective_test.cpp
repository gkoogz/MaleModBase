#include <malemod/surface/attachment_corrective.hpp>
#include <malemod/surface/endpoint_constraints.hpp>
#include <malemod/surface/section_clearance.hpp>
#include <iostream>
using namespace malemod::surface::corrective;
void Require(bool v){if(!v)throw std::runtime_error("Corrective regression failed");}
int main(){try{
 std::vector<Point> p={{0,0,0},{1,0,0},{0,1,0},{1,1,0}};std::vector<Face> f={{0,1,2},{1,3,2}};
 const auto frame=Frame::FromAxis({1,0,0});
 Plan translation(4,f,{}, {0,1,2,3},{{1,{{0,0,.2},{0,0,.2},{0,0,.2},{0,0,.2}}}});
 Require(translation.Apply(p,frame,{1,1,1},0).positions==p);
 auto lifted=translation.Apply(p,frame,{1,1,1},1);Require(lifted.appliedWeight==1&&std::abs(lifted.positions[3][2]-.2)<1e-12);
 auto relaxedTranslation=translation.Apply(p,frame,{1,1,1},1,true);
 Require(relaxedTranslation.appliedWeight==1&&relaxedTranslation.positions==lifted.positions);
 Plan fold(4,f,{0,1,2},{3},{{1,{{-1,-1,0}}}});auto bounded=fold.Apply(p,frame,{1,1,1},1);
 Require(bounded.appliedWeight>.48&&bounded.appliedWeight<.5);for(unsigned i=0;i<3;i++)Require(bounded.positions[i]==p[i]);
 auto normal=Cross(Sub(bounded.positions[3],p[1]),Sub(p[2],p[1]));Require(normal[2]>=.0199);
 auto relaxedFold=fold.Apply(p,frame,{1,1,1},1,true);
 for(unsigned i=0;i<3;i++)Require(relaxedFold.positions[i]==p[i]);
 auto relaxedNormal=Cross(Sub(relaxedFold.positions[3],p[1]),Sub(p[2],p[1]));
 Require(relaxedNormal[2]>=.0199&&Finite(relaxedFold.positions[3]));
 std::vector<Point> half={{0,0,0},{-2,0,0},{0,-2,0},{-2,-2,0}},quarter={{0,0,0},{-1,1,0},{-1,-1,0},{-2,0,0}};
 Plan direct(4,f,{}, {0,1,2,3},{{1,half}});Require(direct.Apply(p,frame,{1,1,1},1).appliedWeight<1);
 auto immediate=direct.EvaluateEndpoint(p,frame,{1,1,1},1);
 Require(immediate.appliedWeight==1);for(unsigned i=0;i<4;i++)Require(immediate.positions[i]==Add(p[i],half[i]));
 Require(fold.EvaluateEndpoint(p,frame,{1,1,1},1).appliedWeight==0);
 Plan curved(4,f,{}, {0,1,2,3},{{.5,quarter},{1,half}});auto complete=curved.Apply(p,frame,{1,1,1},1);
 Require(complete.appliedWeight==1);for(unsigned i=0;i<4;i++)Require(complete.positions[i]==Add(p[i],half[i]));
 bool rejected=false;try{Plan bad(4,f,{3},{3},{{1,{{0,0,1}}}});}catch(const std::invalid_argument&){rejected=true;}Require(rejected);
 // Fields follow the supplied material frame, rather than a fixed world axis.
 auto rotated=translation.Apply(p,Frame::FromAxis({0,0,1},{0,1,0}),{1,1,1},1);
 Require(std::abs(rotated.positions[3][0]-.8)<1e-12&&rotated.positions[3][2]==0);
 // Pose changes retain topology preparation but replace the entire path.
 std::vector<Knot> many;
 for(unsigned i=1;i<=40;i++){double t=i/40.;many.push_back({t,{{0,0,.4*t},{0,0,.4*t},{0,0,.4*t},{0,0,.4*t}}});}
 translation.UpdateKnots(many);
 auto updated=translation.Apply(p,frame,{1,1,1},1);
 Require(updated.appliedWeight==1&&std::abs(updated.positions[3][2]-.4)<1e-12);
 rejected=false;try{translation.UpdateKnots({{.5,{{},{},{},{}}}});}catch(const std::invalid_argument&){rejected=true;}Require(rejected);
 Require(translation.Apply(p,frame,{1,1,1},1).positions==updated.positions);
 // A near-folded endpoint is repaired through independent seam donors.
 // The fixed triangle and the exact interpolated seam must survive together.
 std::vector<Point> sewn={{0,0,0},{1,0,0},{0,1,0},{1,1,0},{1,.5,0}};
 std::vector<Face> sf={{0,1,2},{1,4,2},{4,3,2}};
 EndpointConstraints constraints(5,sf,{3,4},{{4,1,3,.5}});
 auto candidate=sewn;candidate[3]={.45,.45,.2};candidate[4]=Scale(Add(candidate[1],candidate[3]),.5);
 Require(constraints.Project(sewn,candidate));
 for(unsigned i=0;i<3;i++)Require(candidate[i]==sewn[i]);
 Require(std::sqrt(Dot(Sub(candidate[4],Scale(Add(candidate[1],candidate[3]),.5)),Sub(candidate[4],Scale(Add(candidate[1],candidate[3]),.5))))<1e-12);
 auto unchanged=sewn;Require(constraints.Project(sewn,unchanged)&&unchanged==sewn);
 // A cached correction may accelerate a new pose but may not move fixed
 // rows, break a donor, or expand the cold target's correction budget.
 auto cold=sewn;cold[3]={.45,.45,.2};cold[4]=Scale(Add(cold[1],cold[3]),.5);
 std::vector<Point> seed(5);for(unsigned i=0;i<5;i++)seed[i]=Sub(candidate[i],cold[i]);
 seed[0]={100,100,100};seed[4]={100,100,100};
 auto warm=cold;Require(constraints.ProjectWarm(sewn,warm,seed));
 for(unsigned i=0;i<3;i++)Require(warm[i]==sewn[i]);
 Require(std::sqrt(Dot(Sub(warm[4],Scale(Add(warm[1],warm[3]),.5)),Sub(warm[4],Scale(Add(warm[1],warm[3]),.5))))<1e-12);
 auto checkWarm=warm;Require(constraints.Project(sewn,checkWarm)&&checkWarm==warm);
 seed[3]={100,100,100};warm=cold;Require(!constraints.ProjectWarm(sewn,warm,seed)&&warm==cold);
 warm=sewn;seed.assign(5,{0,0,.1});Require(!constraints.ProjectWarm(sewn,warm,seed)&&warm==sewn);
 rejected=false;try{EndpointConstraints broken(5,sf,{3},{{4,1,3,.5}});}catch(const std::invalid_argument&){rejected=true;}Require(rejected);
 // An observed opening deficit, rather than a second size attenuation,
 // determines whether an otherwise scaled corrective needs full activation.
 Require(UpperClearanceResponse({{2,0,0}},{0},{{0,0,1}},{0,0,0},{1,0,0},1,1)==1);
 Require(UpperClearanceResponse({{2,0,2}},{0},{{0,0,1}},{0,0,0},{1,0,0},1,1)==0);
 Require(UpperClearanceResponse({{0,0,0}},{0},{{0,0,1}},{0,0,0},{1,0,0},1,1)==0);
 Require(UpperClearanceResponse({{7,3,4}},{0},{{0,0,1}},{5,3,4},{1,0,0},1,6)==1);
 std::cout<<"Attachment corrective: endpoint, fixed-row, continuous-fold and frame tests passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
