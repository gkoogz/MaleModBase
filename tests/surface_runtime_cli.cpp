#include <malemod/surface/runtime.hpp>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
using namespace malemod::surface;
template<class T>static bool Write(const char* prefix,const char* suffix,const std::vector<T>& data){
 std::ofstream f(std::string(prefix)+suffix,std::ios::binary);f.write(reinterpret_cast<const char*>(data.data()),data.size()*sizeof(T));return bool(f);
}
int main(int argc,char** argv){
 if(argc!=3&&argc!=4)return 2;
 Controls controls;std::ifstream prefs(argv[1]);
 // Original oracle files order scrotum/angle/offsets before glans/hang.
 const unsigned order[18]={0,1,2,3,5,7,8,9,4,6,10,11,12,13,14,15,16,17};
 for(unsigned i:order)if(!(prefs>>controls.values[i]))return 3;
 const unsigned steps=argc==4?unsigned(atoi(argv[3])):120;
 Session session(controls);for(unsigned i=0;i<steps;i++)session.Step();
 auto out=session.Read();std::ofstream file(argv[2],std::ios::binary);
 file.write(reinterpret_cast<const char*>(out.anatomy.positions.data()),out.anatomy.positions.size()*sizeof(Point));
 if(!Write(argv[2],".normals",out.anatomy.normals)||!Write(argv[2],".tangents",out.anatomy.tangents)||!Write(argv[2],".uv",out.anatomy.uv)||!Write(argv[2],".indices",out.anatomyIndices)||!Write(argv[2],".body0",out.body[0].positions)||!Write(argv[2],".body1",out.body[1].positions))return 4;
 printf("session vertices=%zu indices=%zu radius=%.9g length=%.9g\n",out.anatomy.positions.size(),out.anatomyIndices.size(),out.proximalRadius,out.restLength);
 return file?0:4;
}
