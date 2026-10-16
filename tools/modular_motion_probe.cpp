#include <malemod/surface/modular_asset.hpp>
#include <malemod/surface/wire.hpp>
#include <chrono>
#include <iostream>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=4)throw std::invalid_argument("asset source-wire output-points");
 modular::Asset asset(argv[1]);auto plan=asset.MakePlan();
 std::ifstream f(argv[2],std::ios::binary);wire::Bytes bytes((std::istreambuf_iterator<char>(f)),{});auto source=wire::DecodeOutput(bytes);
 auto state=modular::DrivenState(asset.reference,asset.dimensions,source);
 const auto start=std::chrono::steady_clock::now();std::vector<modular::Vec> points;
 for(unsigned i=0;i<20;i++)points=asset.Evaluate(plan,state);
 const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/20;
 std::ofstream output(argv[3],std::ios::binary);output.write(reinterpret_cast<const char*>(points.data()),points.size()*sizeof(points[0]));
 if(!output)throw std::runtime_error("Failed to save evaluated surface");
 std::cout<<"{\"vertices\":"<<points.size()<<",\"evaluationMs\":"<<ms<<"}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
