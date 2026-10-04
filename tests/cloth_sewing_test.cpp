#include <malemod/garments/jockstrap.hpp>
#include <iostream>
using namespace malemod::garments;
struct Edge {unsigned a,b;double rest;bool bend=false,tether=false;};
struct Seam {std::array<unsigned,2> nodes;std::array<double,2> weights;Point residual;unsigned count=2;};
static void Check(bool v,const char* m){if(!v)throw std::runtime_error(m);}
static Point Center(const std::vector<Point>& x,const std::vector<double>& w){Point p{};double mass=0;for(unsigned i=0;i<x.size();i++)if(w[i]){p=Add(p,Mul(x[i],1/w[i]));mass+=1/w[i];}return Mul(p,1/mass);}
int main(){try{
 std::vector<Edge> edges{{1,2,.01}};std::vector<Seam> seams{{{1,0},{1,-1},{-.02,0,0}}};
 auto adjacency=cloth_stretch::Adjacency(4,edges);
 std::vector<Point> original{{0,0,0},{.02,0,0},{.04,0,0},{.3,.2,.1}};
 auto x=original;std::vector<double>w{0,10,1,2};auto solve=[&](auto& positions,const auto& masses,const auto& sew){return cloth_stretch::ProjectCoupled(positions,masses,edges,adjacency,sew,[](Point p){return p;},1.05);};
 auto result=solve(x,w,seams);Check(!result.exhausted,"Compatible sewn extension did not converge");
 Check(Length(Sub(x[1],x[2]))<=.01*1.05*1.00101,"Sewn endpoint undid material extension");
 Check(Length(Sub(Sub(x[1],x[0]),Point{.02,0,0}))<=1.001e-6,"Finite material sewing offset collapsed");
 Check(x[0]==original[0]&&x[3]==original[3],"Pinned or disconnected material moved");Check(edges[0].rest==.01,"Projection changed material rest length");
 w[0]=3;x=original;auto before=Center(x,w);result=solve(x,w,seams);Check(!result.exhausted,"Free elastic-band coupling did not converge");Check(Length(Sub(Center(x,w),before))<1e-12,"Internal sewing/material correction failed reciprocal mass conservation");
 auto transform=[](Point p){return Add(Point{-p[1],p[0],p[2]},{.4,-.2,.7});};auto rotated=original;for(auto& p:rotated)p=transform(p);auto rs=seams;rs[0].residual={0,-.02,0};solve(rotated,w,rs);for(unsigned i=0;i<x.size();i++)Check(Length(Sub(rotated[i],transform(x[i])))<2e-6,"Constraint coupling depends on frame orientation/origin");
 std::cout<<"PASS stitched extension, finite offset, pinned/disconnected state, reciprocal masses and rigid frame covariance\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
