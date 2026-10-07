#include <malemod/garments/cpu_cloth.hpp>
#include <iostream>
using namespace malemod::garments;
int main(){try{
 const unsigned rows=12,cols=12;std::vector<Point> rest;std::vector<bool> pins;
 for(unsigned y=0;y<=rows;y++)for(unsigned x=0;x<=cols;x++){rest.push_back({double(x)/cols-.5,double(y)/rows-.5,.35});pins.push_back(y==0);}
 auto faces=cloth_detail::SheetTriangles(0,rows,cols);CpuCloth cloth;cloth.Initialize(rest,faces,pins);
 double worst=0,stretch=1,motion=0;auto begin=std::chrono::steady_clock::now();
 for(unsigned step=0;step<600;step++){
  auto target=rest;double phase=step/60.;for(auto& p:target)p[0]+=.06*std::sin(phase*2);
  Capsule capsule{{0,0,0},{0,.1,0},.25};cloth.Step(1./60,target,{capsule},{0,0,-1});auto points=cloth.Positions();
  for(unsigned i=0;i<points.size();i++){if(pins[i]&&Length(Sub(points[i],target[i]))>1e-6)throw std::runtime_error("Pinned cloth attachment drifted");worst=(std::min)(worst,detail::CapsuleDistance(points[i],capsule));motion=(std::max)(motion,Length(Sub(points[i],rest[i])));}
  for(auto f:faces)for(unsigned k=0;k<3;k++){unsigned a=f[k],b=f[(k+1)%3];stretch=(std::max)(stretch,Length(Sub(points[a],points[b]))/Length(Sub(rest[a],rest[b])));}
 }
 auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
 std::cout<<"frames=600 seconds="<<elapsed<<" min_vertex_gap="<<worst<<" max_stretch="<<stretch<<" movement="<<motion<<'\n';
 if(worst<-.005||stretch>1.1||motion<.1)throw std::runtime_error("Moving pinned cloth/capsule regression failed");
 cloth.Initialize(rest,faces,pins);
 const std::vector<std::array<Point,3>> floor{{Point{-2,-2,0},Point{2,-2,0},Point{2,2,0}},{Point{-2,-2,0},Point{2,2,0},Point{-2,2,0}}};
 for(unsigned step=0;step<180;step++)cloth.Step(1./60,rest,{},Point{0,0,-1},{},{},floor);
 double lowest=1;for(auto p:cloth.Positions())lowest=(std::min)(lowest,p[2]);
 if(lowest<-.0001||lowest>.05)throw std::runtime_error("Cloth triangle floor contact failed");
 cloth.Initialize(rest,faces,pins);std::vector<CpuCloth::MotionLimit> backstops;for(auto p:rest)backstops.push_back({Point{p[0],p[1],0},.2});
 for(unsigned step=0;step<180;step++)cloth.Step(1./60,rest,{},Point{0,0,-1},{},backstops);
 auto supported=cloth.Positions();for(unsigned i=0;i<supported.size();i++)if(Length(Sub(supported[i],backstops[i].center))<.1999)throw std::runtime_error("Cloth separation backstop failed");
 bool rejected=false;try{cloth.Step(-1,rest,{},{});}catch(const std::invalid_argument&){rejected=true;}if(!rejected)throw std::runtime_error("Negative elapsed time was accepted");
 std::cout<<"Triangle floor, body backstop and invalid timing checks passed\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
