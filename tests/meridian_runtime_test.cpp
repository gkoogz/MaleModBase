#include <malemod/garments/meridian_runtime.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
using namespace malemod::garments::meridian;
int main(){
 Vec source[3]={{0,0,0},{1,0,0},{0,1,0}};
 Binding bind[3]={{{0,1,2},{1,0,0},{0,0,.2f},0},{{0,1,2},{0,1,0},{0,0,.2f},0},{{0,1,2},{0,0,1},{0,0,.2f},0}};
 Face face={0,1,2};Mesh mesh;
 mesh.Update(bind,3,&face,1,[&](unsigned,unsigned id){return source[id];});
 if(std::abs(mesh.Vertices()[0].position[2]-.2f)>1e-6)return 1;
 // Translate and rotate the donor triangle; thickness must rotate with it.
 for(auto& p:source)p={p[0]+3,-p[2]+4,p[1]+5};
 mesh.Update(bind,3,&face,1,[&](unsigned,unsigned id){return source[id];});
 auto p=mesh.Vertices()[0].position;
 if(std::abs(p[0]-3)>1e-6||std::abs(p[1]-3.8f)>1e-6||std::abs(p[2]-5)>1e-6)return 2;
 // Invalid geometry must reject instead of publishing NaNs.
 source[2]=source[1];bool rejected=false;
 try{mesh.Update(bind,3,&face,1,[&](unsigned,unsigned id){return source[id];});}catch(const std::exception&){rejected=true;}
 if(!rejected)return 3;
 std::puts("PASS: reference placement, rotating offset, degenerate donor rejection");
}
