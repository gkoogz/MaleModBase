#include <malemod/surface/lighting.hpp>
#include <iostream>
using namespace malemod::surface;
int main(){try{
 std::vector<LightingPoint> p{{0,0,0},{2,0,0},{0,3,0},{0,0,0},{0,3,0},{-2,0,0}};
 std::vector<std::array<double,2>> uv{{0,0},{1,0},{0,1},{0,0},{0,1},{1,0}};
 std::vector<std::array<std::uint32_t,3>> f{{0,1,2},{3,4,5}};std::vector<std::uint32_t> groups{0,1,2,0,2,5};
 std::vector<LightingFrame> fallback(6,{{0,0,1},{1,0,0},1});
 auto a=RebuildLighting(p,uv,f,groups,fallback);
 if(a[0].normal!=a[3].normal||a[0].sign!=1||a[3].sign!=-1)return 1;
 for(auto& v:p){auto y=v[1];v[1]=-v[2];v[2]=y;}
 auto b=RebuildLighting(p,uv,f,groups,fallback);
 for(unsigned i=0;i<6;i++)if(std::abs(b[i].normal[1]+1)>1e-12||std::abs(LightingDot(b[i].normal,b[i].tangent))>1e-12||std::abs(LightingDot(b[i].tangent,b[i].tangent)-1)>1e-12||b[i].sign!=a[i].sign)return 2;
 uv.assign(6,{0,0});auto degenerate=RebuildLighting(p,uv,f,groups,fallback);if(degenerate.size()!=6)return 3;
 f[0][0]=99;try{RebuildLighting(p,uv,f,groups,fallback);return 4;}catch(const std::invalid_argument&){}
 std::cout<<"PASS: final geometry normals, welded UV aliases, mirrored tangent signs, rotated/nonuniform geometry and degenerate UV fallback\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 5;}}
