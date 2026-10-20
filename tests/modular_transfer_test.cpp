#include <malemod/surface/modular_transfer.hpp>
#include <chrono>
#include <iostream>
#include <string>
using namespace malemod::surface;
using namespace malemod::surface::modular;
int main(){
 const auto dir=std::filesystem::temp_directory_path()/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
 std::filesystem::create_directory(dir);
 try{
  const std::vector<Vec> points{{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
  {
   std::ofstream f(dir/"asset",std::ios::binary);
   auto u=[&](unsigned n){f.write(reinterpret_cast<char*>(&n),4);};
   auto d=[&](double n){f.write(reinterpret_cast<char*>(&n),8);};
   auto v=[&](Vec p){for(double x:p)d(x);};
   f.write("MMMOD002",8);u(4);u(2);u(0);d(1);d(11);v({});v({1,1,1});v({1,1,1});
   for(unsigned i=0;i<15;i++){v({});v({1,0,0});v({0,1,0});v({0,0,1});}
   for(auto p:points){v(p);v({0,0,1});u(1);d(1);v(p);u(0);d(0);v({});}
   for(unsigned i:{0,1,2,0,2,3})u(i);u(0);
  }
  {
   std::ofstream f(dir/"transfer",std::ios::binary);
   auto u=[&](unsigned n){f.write(reinterpret_cast<char*>(&n),4);};
   auto d=[&](double n){f.write(reinterpret_cast<char*>(&n),8);};
   f.write("MMXFER02",8);f<<std::string(64,'0');u(4);u(4);u(4);
   for(unsigned i=0;i<4;i++){u(i);u(1);u(1);d(double(i)/3);u(i);d(1);}
  }
  Asset asset(dir/"asset");auto plan=asset.MakePlan();SurfaceTransfer transfer(dir/"transfer");
  Output source;for(auto p:points){source.anatomy.positions.push_back({float(p[0]+2),float(p[1]),float(p[2])});source.anatomy.normals.push_back({0,0,1});}
  for(unsigned i=0;i<12;i++)source.shaftGuide[i]={float(i),0,0};
  auto identity=[](Vec p){return p;};State rest;
  transfer.Apply(asset,plan,rest,source,identity,true,true);
  auto material=plan.Evaluate(rest);
  for(unsigned i=0;i<4;i++)if(material[i]!=Add(points[i],Vec{2,0,0}))throw std::runtime_error("Rest morphology loses axial shape across crown weights");
  State live;transfer.Apply(asset,plan,live,source,identity);
  auto legacy=plan.Evaluate(live);
  if(legacy[0]!=points[0]||legacy[3]!=Add(points[3],Vec{2,0,0}))throw std::runtime_error("Legacy live axial isolation changed");
  State independent;transfer.Apply(asset,plan,independent,source,identity,false);
  if(plan.Evaluate(independent)!=points)throw std::runtime_error("Live legacy shaft skin leaked into material-owned motion");
  {
   std::ofstream f(dir/"skin-transfer",std::ios::binary);
   auto u=[&](unsigned n){f.write(reinterpret_cast<char*>(&n),4);};
   auto d=[&](double n){f.write(reinterpret_cast<char*>(&n),8);};
   f.write("MMXFER02",8);f<<std::string(64,'0');u(4);u(4);u(4);
   for(unsigned i=0;i<4;i++){u(i);u(2);u(1);d(0);u(i);d(1);}
  }
  SurfaceTransfer skin(dir/"skin-transfer");
  for(unsigned i=0;i<4;i++)asset.bindings[i].influences={Influence{13+i%2,1,points[i]},Influence{}};
  auto skinPlan=asset.MakePlan();State skinMaterial,skinLive;
  skinMaterial.surfaceNormals=std::make_shared<const std::vector<Vec>>(4,Unit(Vec{1,0,1}));
  skin.ApplyPrepared(asset,skinLive,skinMaterial,points);
  if(skinPlan.Evaluate(skinLive)!=points)throw std::runtime_error("Prepared skin loses authored rest shape");
  skinLive.frames[13].origin={2,-1,3};skinLive.frames[14].origin={-1,2,-3};
  skinLive.scales[13]={1.2,.8,1};skinLive.scales[14]={.9,1,1.1};
  skin.ApplyPrepared(asset,skinLive,skinMaterial,points);
  auto posed=skinPlan.Evaluate(skinLive);
  for(unsigned i=0;i<4;i++){
   const auto frame=13+i%2;
   const auto expected=Add(skinLive.frames[frame].origin,Components(points[i],skinLive.scales[frame]));
   if(Dot(Sub(posed[i],expected),Sub(posed[i],expected))>1e-24)throw std::runtime_error("Independent prepared skin follows wrong body");
   auto normal=(*skinMaterial.surfaceNormals)[i];for(unsigned k=0;k<3;k++)normal[k]/=skinLive.scales[frame][k];
   auto error=Sub((*skinLive.surfaceNormals)[i],Unit(normal));
   if(Dot(error,error)>1e-24)throw std::runtime_error("Prepared skin normal ignores anisotropic scale");
  }
  std::filesystem::remove_all(dir);
  std::cout<<"PASS rest morphology exact across crown blend; legacy live and independent motion isolated\n";
 }catch(const std::exception& e){std::filesystem::remove_all(dir);std::cerr<<e.what()<<'\n';return 1;}
}
