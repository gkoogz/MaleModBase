// SDK-free exact arithmetic checks for precomputed query geometry reuse.
#include <malemod/garments/jockstrap.hpp>
#include <random>
#include <iostream>
int main(){try{
 namespace G=malemod::garments;namespace D=G::detail;
 std::mt19937_64 random(0x636c6f7468706c61ull);std::uniform_real_distribution<double> coordinate(-2,2),positive(0,1);
 unsigned boxAccept=0,boxReject=0,satAccept=0,satReject=0;
 for(double scale:{.001,1.,1000.})for(double offset:{0.,1e4})for(unsigned i=0;i<20000;i++){
  std::array<G::Point,3>a,b;for(auto* triangle:{&a,&b})for(auto& p:*triangle)for(unsigned k=0;k<3;k++)p[k]=coordinate(random)*scale+offset;
  if(i%17==0)a[2]=a[1]; // same degeneracy branch as old oracle
  G::Point low,high;for(unsigned k=0;k<3;k++){low[k]=coordinate(random)*scale+offset;high[k]=low[k]+positive(random)*scale;}
  double margin=positive(random)*scale*.03;
  D::SurfaceFaceQuery query(a);
  bool boxOld=D::FaceBoxesOverlap(a,low,high,margin),boxNew=D::FaceBoxesOverlap(query,low,high,margin);
  bool satOld=D::SeparatedTriangles(a,b,margin),satNew=D::SeparatedTriangles(query,b,margin);
  if(boxOld!=boxNew||satOld!=satNew)throw std::runtime_error("Exact query geometry arithmetic changed a separation decision");
  (boxNew?boxAccept:boxReject)++;(satNew?satReject:satAccept)++;
 }
 if(!boxAccept||!boxReject||!satAccept||!satReject)throw std::runtime_error("Equivalence fixture omitted acceptance or rejection");
 std::cout<<"PASS 120000 SDK-free exact box/SAT comparisons, translated, scale-varied, degeneracy branches; box "<<boxAccept<<'/'<<boxReject<<" sat "<<satAccept<<'/'<<satReject<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
