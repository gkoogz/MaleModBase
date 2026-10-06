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
 // An infeasible fixed boundary must fail instead of emitting broken geometry.
 d.prescribedVertices={2};bool rejected=false;
 try{GraftPlan impossible(d,f);impossible.SolveDisplacement(input);}catch(const std::runtime_error&){rejected=true;}
 Check(rejected,"Infeasible prescribed body field was accepted");
 std::cout<<"PASS body orientation, hard donors, seam elimination, repeatability and infeasible rejection\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
