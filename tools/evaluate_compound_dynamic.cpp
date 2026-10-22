#include <malemod/surface/compound_dynamic.hpp>
#include <malemod/surface/wire.hpp>
#include <malemod/surface/modular_transfer.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=5&&argc!=6)throw std::invalid_argument("Usage: evaluate_compound_dynamic asset dynamic source-wire output.obj [surface-transfer]");
 modular::Asset asset(argv[1]);auto plan=asset.MakePlan();modular::DynamicMount mount(argv[2]);CompoundDynamicPlan compound(mount.compoundData,asset,mount);
 std::ifstream in(argv[3],std::ios::binary);wire::Bytes bytes((std::istreambuf_iterator<char>(in)),{});auto source=wire::DecodeOutput(bytes);
 auto state=modular::DrivenState(asset.reference,asset.dimensions,source);if(argc==6){modular::SurfaceTransfer transfer(argv[5]);if(transfer.geometryHash!=mount.geometryHash)throw std::invalid_argument("Transfer identity differs");transfer.Apply(asset,plan,state,source,[](auto p){return p;});}auto shape=state;if(state.hasShapeFrames)shape.frames=state.shapeFrames;
 auto start=std::chrono::steady_clock::now();std::cerr<<"Evaluating dynamic support\n";auto support=mount.Evaluate(asset,plan,shape);std::cerr<<"Building compound cut\n";
 auto posed=mount.Pose(asset,plan,shape,support.fitTranslation,support.advances,support.suspensionTranslation,support.clearedBindings,support.materialRoot);
 auto cache=compound.Build(support,asset,shape,posed);
 auto points=cache.Evaluate(compound.Source(posed,shape),support.surface.points);
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;for(auto f:cache.faces)for(unsigned k=0;k<3;k++){auto a=f[k].vertex,b=f[(k+1)%3].vertex;auto& e=edges[std::minmax(a,b)];e.first++;e.second+=a<b?1:-1;}
 unsigned boundary=0;for(auto item:edges){auto e=item.second;if(e.first==1)boundary++;else if(e.first!=2||e.second!=0)throw std::runtime_error("Dynamic compound assembly lost manifold winding");}
 std::ofstream out(argv[4]);out<<std::setprecision(17);for(auto p:points)out<<"v "<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';for(auto f:cache.faces)out<<"f "<<f[0].vertex+1<<' '<<f[1].vertex+1<<' '<<f[2].vertex+1<<'\n';
 std::ofstream domains(std::string(argv[4])+".domains",std::ios::binary);for(auto f:cache.faces){unsigned domain=f[0].domain==0?support.surface.faces[f[0].parent][0].domain:f[0].domain+2;domains.write(reinterpret_cast<const char*>(&domain),4);}
 std::ofstream rings(std::string(argv[4])+".rings",std::ios::binary);unsigned rows=unsigned(cache.rings.size()),columns=unsigned(cache.rings[0].size());rings.write(reinterpret_cast<const char*>(&rows),4);rings.write(reinterpret_cast<const char*>(&columns),4);for(auto r:cache.rings)rings.write(reinterpret_cast<const char*>(r.data()),r.size()*4);
 std::cout<<"vertices="<<points.size()<<" faces="<<cache.faces.size()<<" boundary="<<boundary<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
