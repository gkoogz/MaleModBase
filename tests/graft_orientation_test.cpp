#include <malemod/surface/graft_runtime.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace malemod::surface;
static void Check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
static double Area(const std::vector<PrecisePoint>& p){return p[1][1]*p[2][2]-p[1][2]*p[2][1];}
int main(){try{
 GraftDomain d;d.points={{0,0,0},{0,1,0},{0,0,1},{0,.5,0}};
 d.triangles={{0,3,2},{3,1,2}};d.seams={{3,0,1,.5}};d.protectedVertices={0,1};
 GraftFrame f{{0,0,0},{1,0,0},{0,0,1},7,25,1};
 std::vector<PrecisePoint> input(4);input[2]={0,0,-20};
 auto positions=[&](const std::vector<PrecisePoint>& delta){auto p=d.points;for(unsigned i=0;i<p.size();i++)for(unsigned a=0;a<3;a++)p[i][a]+=delta[i][a];return p;};
 GraftPlan original(d,f);const auto before=original.SolveDisplacement(input);
 Check(Area(positions(before))<0,"Fixture does not exercise a folded body face");
 d.orientationTriangles={0,1};GraftPlan guarded(d,f);const auto corrected=guarded.SolveDisplacement(input);
 Check(Area(positions(corrected))>.001,"Body triangle remains folded");
 Check(corrected[0]==PrecisePoint{}&&corrected[1]==PrecisePoint{},"Protected attachment donors moved");
 for(unsigned a=0;a<3;a++)Check(corrected[3][a]==.5*(corrected[0][a]+corrected[1][a]),"Original-edge weld opened");
 Check(guarded.SolveDisplacement(input)==corrected,"Constraint projection is not deterministic");
 GraftPlan nativeGuard(d);const auto native=nativeGuard.ProjectDisplacement(input);
 Check(Area(positions(native))>.001,"Constraint-only cooked surface remains folded");
 for(unsigned a=0;a<3;a++)Check(native[3][a]==.5*(native[0][a]+native[1][a]),"Constraint-only projection opens original-edge donors");
 std::vector<PrecisePoint> valid(4);valid[2]={.3,0,.1};valid[3]={0,0,0};
 Check(nativeGuard.ProjectDisplacement(valid)==valid,"Valid cooked surface changed by constraint-only projection");
 // An infeasible fixed boundary must fail instead of emitting broken geometry.
 d.prescribedVertices={2};bool rejected=false;
 try{GraftPlan impossible(d,f);impossible.SolveDisplacement(input);}catch(const std::runtime_error&){rejected=true;}
 Check(rejected,"Infeasible prescribed body field was accepted");
 // Resource joins prescribe exact targets. Flattening neighboring differential
 // coordinates around those rows introduces a shelf despite a smooth field.
 GraftDomain smooth;smooth.points={{0,-1,0},{0,0,0},{0,1,0},{0,-1,1},{0,0,1},{0,1,1}};
 smooth.triangles={{0,1,3},{1,4,3},{1,2,4},{2,5,4}};smooth.prescribedVertices={1,4};
 smooth.preserveTargetDifferential=true;GraftPlan preserve(smooth,f);
 std::vector<PrecisePoint> loft={{.1,0,0},{.2,0,0},{.1,0,0},{.2,0,0},{.4,0,0},{.2,0,0}};
 const auto fitted=preserve.SolveDisplacement(loft);
 for(unsigned i=0;i<loft.size();i++)for(unsigned a=0;a<3;a++)Check(std::abs(fitted[i][a]-loft[i][a])<1e-10,"Smooth measured body loft attenuated beside prescribed resource join");
 std::cout<<"PASS body orientation, hard donors, seam elimination, repeatability and infeasible rejection\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
