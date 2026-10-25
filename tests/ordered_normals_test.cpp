#include <malemod/surface/ordered_normals.hpp>
#include <iostream>
using namespace malemod::surface;
int main(){try{
 std::vector<PresentationPoint> a(4096),b(4096);
 for(unsigned i=0;i<a.size();i++){
  a[i]={double(i%64),double(i/64),std::sin(i*.13)};
  b[i]={a[i][0]*1.1,a[i][1]*.9,a[i][2]+std::cos(i*.07)*.1};
 }
 std::vector<std::array<unsigned,3>> faces;
 for(unsigned y=0;y<63;y++)for(unsigned x=0;x<63;x++){unsigned i=y*64+x;faces.push_back({i,i+1,i+64});faces.push_back({i+1,i+65,i+64});}
 faces.push_back({0,0,1}); // Repeated corner must retain serial multiplicity.
 OrderedNormals normals(unsigned(a.size()),faces);malemod::TaskArena arena;
 auto serial=normals.Pair(a,b,nullptr,false,false),parallel=normals.Pair(a,b,&arena,false,false);
 std::vector<PresentationPoint> reference(a.size());
 for(auto f:faces){auto n=Cross(Sub(a[f[1]],a[f[0]]),Sub(a[f[2]],a[f[0]]));for(auto i:f)reference[i]=Add(reference[i],n);}
 if(serial!=parallel||serial.first!=reference)throw std::runtime_error("Ordered normal accumulation differs");
 auto normalized=normals.Pair(a,b,&arena);
 if(normals.Field(a)!=normalized.first||normals.Field(b,&arena)!=normalized.second)throw std::runtime_error("Single live normal field differs from paired reference");
 std::vector<unsigned> subset;for(unsigned i=1;i<b.size();i+=7)subset.push_back(i);
 const auto partial=normals.FieldSubset(b,subset,&arena);
 if(partial!=normals.FieldSubset(b,subset))throw std::runtime_error("Parallel subset normal field differs");
 for(unsigned i=0;i<b.size();i++)if(partial[i]!=(i%7==1?normalized.second[i]:PresentationPoint{}))throw std::runtime_error("Subset normals changed retained values or unrequested entries");
 auto inverted=a;for(auto& p:inverted)p[0]=-p[0];
 bool rejected=false;try{normals.Pair(a,inverted,&arena,true);}catch(const std::runtime_error&){rejected=true;}
 if(!rejected)throw std::runtime_error("Reversed face was accepted");
 if(normals.Pair(a,b,&arena,false,false)!=serial)throw std::runtime_error("Normal arena did not recover after rejection");
 std::cout<<"PASS ordered normals match serial, reject reversals and recover\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
