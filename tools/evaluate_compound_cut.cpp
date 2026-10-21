#include <malemod/surface/convex_surface_cut.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=3)throw std::invalid_argument("Usage: evaluate_compound_cut input.bin output.obj");
 std::ifstream in(argv[1],std::ios::binary);
 auto read=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::runtime_error("Truncated compound cut input");};
 std::array<unsigned,3> counts{};read(counts.data(),12);
 if(!counts[0]||counts[0]>500000||!counts[1]||counts[1]>1000000||counts[2]<3||counts[2]>4096)throw std::runtime_error("Invalid compound cut dimensions");
 std::vector<PresentationPoint> p(counts[0]);std::vector<std::array<unsigned,3>> f(counts[1]);
 std::vector<std::array<double,2>> chart(counts[0]),footprint(counts[2]);std::vector<unsigned char> eligible(counts[1]);
 read(p.data(),p.size()*24);read(f.data(),f.size()*12);read(chart.data(),chart.size()*16);read(eligible.data(),eligible.size());read(footprint.data(),footprint.size()*16);
 if(in.peek()!=EOF)throw std::runtime_error("Trailing compound cut data");
 auto start=std::chrono::steady_clock::now();
 auto cut=ConvexSurfaceCut(p,f,std::vector<bool>(eligible.begin(),eligible.end()),chart,1e-8).CutSimple(footprint);
 double error=0;
 for(auto face:cut.faces)for(auto c:face.corners){PresentationPoint expected{};for(unsigned k=0;k<3;k++)expected=Add(expected,Scale(p[f[face.parent][k]],c.donor[k]));auto d=Sub(expected,cut.points[c.vertex]);error=(std::max)(error,std::sqrt(Dot(d,d)));}
 const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
 std::ofstream out(argv[2]);out<<std::setprecision(17);for(auto v:cut.points)out<<"v "<<v[0]<<' '<<v[1]<<' '<<v[2]<<'\n';
 for(auto face:cut.faces)out<<"f "<<face.corners[0].vertex+1<<' '<<face.corners[1].vertex+1<<' '<<face.corners[2].vertex+1<<'\n';
 std::cout<<"vertices="<<cut.points.size()<<" faces="<<cut.faces.size()<<" rim="<<cut.opening.size()<<" donorError="<<error<<" seconds="<<seconds<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
