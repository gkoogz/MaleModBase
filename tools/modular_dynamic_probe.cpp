#include <malemod/surface/modular_dynamic.hpp>
#include <malemod/surface/wire.hpp>
#include <malemod/surface/modular_transfer.hpp>
#include <malemod/surface/root_articulation.hpp>
#include <chrono>
#include <iostream>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=5&&argc!=7&&argc!=8&&argc!=9)throw std::invalid_argument("asset dynamic-mount source-wire output-prefix [surface-transfer angle-slider [captured-upward-degrees [trial-upward-degrees]]]");modular::Asset asset(argv[1]);auto plan=asset.MakePlan();modular::DynamicMount mount(argv[2]);
 std::ifstream in(argv[3],std::ios::binary);wire::Bytes bytes((std::istreambuf_iterator<char>(in)),{});auto source=wire::DecodeOutput(bytes);double radians=0;
 if(argc>=7){double slider=std::stod(argv[6]);radians=(50-slider)/(slider<50?49.:50.)*(slider<50?(argc>=8?std::stod(argv[7]):55.):30.)*3.141592653589793/180.;
  if(argc==9&&slider<50){double desired=(50-slider)/49.*std::stod(argv[8])*3.141592653589793/180.;root_articulation::Rotation adjust(Precise(source.shaftGuide[0]),{1,0,0},{0,0,1},desired-radians);for(auto* guide:{&source.shaftGuide,&source.restGuide})for(auto& p:*guide){auto q=adjust.Apply(Precise(p));p={float(q[0]),float(q[1]),float(q[2])};}radians=desired;}
 }
 auto state=modular::DrivenState(asset.reference,asset.dimensions,source);
 if(argc>=7){modular::SurfaceTransfer transfer(argv[5]);if(transfer.geometryHash!=mount.geometryHash)throw std::invalid_argument("Transfer identity differs");root_articulation::Rotation rotation(Precise(source.shaftGuide[0]),{1,0,0},{0,0,1},radians);transfer.Apply(asset,plan,state,source,[&](auto p){return rotation.Apply(p);});}
 auto start=std::chrono::steady_clock::now();auto shape=state;if(state.hasShapeFrames)shape.frames=state.shapeFrames;auto result=mount.Evaluate(asset,plan,shape);auto& out=result.surface;
 auto repeated=mount.Repose(result,asset,plan,shape);double error=0;for(unsigned i=0;i<out.points.size();i++)error=(std::max)(error,std::sqrt(Dot(Sub(repeated[i],out.points[i]),Sub(repeated[i],out.points[i]))));
 if(error>1e-7)throw std::runtime_error("Cached dynamic pose does not reproduce its authored surface: "+std::to_string(error));
 out.points=mount.Repose(result,asset,plan,state);
 auto write=[&](const char* suffix,const void* data,std::size_t size){std::ofstream f(std::string(argv[4])+suffix,std::ios::binary);if(!f.write(static_cast<const char*>(data),size))throw std::runtime_error("Failed dynamic mesh export");};
 write("-points.bin",out.points.data(),out.points.size()*24);std::vector<std::array<unsigned,3>> faces;for(auto f:out.faces)faces.push_back({f[0].vertex,f[1].vertex,f[2].vertex});write("-faces.bin",faces.data(),faces.size()*12);
 std::vector<unsigned> rings;for(auto row:out.rings)rings.insert(rings.end(),row.begin(),row.end());write("-rings.bin",rings.data(),rings.size()*4);
 std::cout<<"{\"vertices\":"<<out.points.size()<<",\"faces\":"<<faces.size()<<",\"rows\":"<<out.rings.size()<<",\"knots\":"<<out.rings[0].size()<<",\"anchor\":["<<result.anchor[0]<<','<<result.anchor[1]<<','<<result.anchor[2]<<"],\"rootAdvance\":"<<result.rootAdvance<<",\"milliseconds\":"<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<",\"originalBodyDisplacement\":0}";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
