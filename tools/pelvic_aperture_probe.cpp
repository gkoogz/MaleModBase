#include <malemod/surface/pelvic_aperture.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=6)throw std::invalid_argument("domain scale angle output-vertices output-faces");
 std::ifstream in(argv[1],std::ios::binary);auto read=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::runtime_error("Short aperture domain");};unsigned nv,nf,nr;read(&nv,4);read(&nf,4);read(&nr,4);
 if(nv>500000||nf>1000000||nr>4096)throw std::invalid_argument("Invalid aperture domain counts");
 std::vector<PresentationPoint> p(nv),r(nr);std::vector<std::array<unsigned,3>> f(nf);PresentationPoint origin;PresentationBasis basis;read(p.data(),nv*24);read(f.data(),nf*12);read(&origin,24);read(&basis,72);read(r.data(),nr*24);
 const auto start=std::chrono::steady_clock::now();PelvicAperturePlan plan(p,f,origin,basis,r,1e-8);const double angle=std::stod(argv[3])*3.14159265358979323846/180;auto out=plan.Solve(Add(Scale(basis[0],std::cos(angle)),Scale(basis[2],std::sin(angle))),std::stod(argv[2]));
 std::ofstream vertices(argv[4],std::ios::binary);vertices.write(reinterpret_cast<const char*>(out.body.points.data()),out.body.points.size()*24);std::ofstream faces(argv[5],std::ios::binary);for(auto face:out.body.faces){std::array<unsigned,3> ids;for(unsigned k=0;k<3;k++)ids[k]=face.corners[k].vertex;faces.write(reinterpret_cast<const char*>(ids.data()),12);}
 std::cout<<"{\"vertices\":"<<out.body.points.size()<<",\"faces\":"<<out.body.faces.size()<<",\"knots\":"<<out.body.opening.size()<<",\"anchor\":["<<out.anchor[0]<<','<<out.anchor[1]<<','<<out.anchor[2]<<"],\"milliseconds\":"<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<",\"originalBodyDisplacement\":0}";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
