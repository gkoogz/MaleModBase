// SDK-free authoring bridge: use the same bounded seam fit as native adapters.
#include <malemod/garments/meridian_clearance.hpp>
#include <fstream>
#include <iostream>
using namespace malemod::garments::meridian;
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("Usage: fitter input.bin output.bin");
 std::ifstream input(argv[1],std::ios::binary);
 auto read=[&](void* p,std::size_t bytes){if(!input.read(static_cast<char*>(p),bytes))throw std::runtime_error("Truncated seam input");};
 unsigned columns=0,hullCount=0;read(&columns,4);read(&hullCount,4);
 if(columns<8||columns>256||hullCount>64)throw std::runtime_error("Invalid seam dimensions");
 Vec pole{},axis{};read(pole.data(),12);read(axis.data(),12);
 std::vector<Vec> points(columns);read(points.data(),columns*sizeof(Vec));
 std::vector<Hull> hulls(hullCount);
 for(auto& hull:hulls){unsigned count=0;read(&count,4);if(count>16384)throw std::runtime_error("Invalid support count");hull.resize(count);for(auto& p:hull){read(p.normal.data(),12);read(&p.offset,4);}}
 auto delta=FitSeam(points,columns,pole,axis,hulls,.12f,6.f);
 std::ofstream output(argv[2],std::ios::binary);output.write((const char*)points.data(),points.size()*sizeof(Vec));
 if(!output)throw std::runtime_error("Cannot write fitted seam");
 float maximum=0;for(auto d:delta)maximum=(std::max)(maximum,std::sqrt(Dot(d,d)));
 std::cout<<"Maximum seam adjustment: "<<maximum<<" source units\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
