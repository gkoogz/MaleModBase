#include <malemod/garments/meridian_clearance.hpp>
#include <malemod/garments/meridian_rig.hpp>
#include "meridian_recipe.h"
#include <fstream>
#include <cstdio>
#include <chrono>
using namespace malemod::garments::meridian;
int main(int argc,char** argv){
 if(argc!=3)return 1;std::vector<Vec> points(MeridianRecipe::sampleCount);
 std::ifstream in(argv[1],std::ios::binary);in.read((char*)points.data(),points.size()*sizeof(Vec));if(!in)return 2;
 std::vector<Vec> normals;
 for(unsigned i=0;i<128;i++){float z=1-2*(i+.5f)/128,r=std::sqrt(1-z*z),a=i*2.39996323f;normals.push_back({r*std::cos(a),r*std::sin(a),z});}
 auto began=std::chrono::steady_clock::now();
 CircularSection rings[7];for(unsigned h=0;h<7;h++){auto c=MeridianRecipe::ringControls[h];rings[h]=FitCircularSection(points[c[0]],points[c[1]],points[c[2]],points[c[3]]);}
 Vec apex=points[MeridianRecipe::domeApex];AlignDomeRim(rings[6],apex);Vec lobes[2][6];for(unsigned h=0;h<2;h++)for(unsigned k=0;k<6;k++)lobes[h][k]=points[MeridianRecipe::lobeControls[h][k]];
 for(unsigned h=0;h<6;h++)WriteLink(points.data()+MeridianRecipe::proxyRanges[h][0],rings[h],rings[h+1],64);
 WriteDome(points.data()+MeridianRecipe::proxyRanges[6][0],rings[6],apex,24,64);
 for(unsigned h=0;h<2;h++){auto c=lobes[h];WriteOvoid(points.data()+MeridianRecipe::proxyRanges[h+7][0],c[0],c[1],c[2],c[3],c[4],c[5],24,48);}
 auto liveAxis=PreparePole(points,MeridianRecipe::clothCount-1,MeridianRecipe::count,MeridianRecipe::sampleCount-1,AttachmentCentroid(points,MeridianRecipe::columns));
 std::vector<Hull> hulls;
 for(unsigned h=0;h<9;h++){
  auto* frame=MeridianRecipe::proxyFrames[h];Vec e=Unit(Sub(points[frame[1]],points[frame[0]])),n=Unit(Cross(e,Sub(points[frame[2]],points[frame[0]]))),v=Cross(n,e);
  auto* range=MeridianRecipe::normalRanges[h];normals.clear();normals.push_back(liveAxis);normals.push_back(Mul(liveAxis,-1));for(unsigned k=0;k<range[1];k++){auto q=MeridianRecipe::supportNormals[range[0]+k];normals.push_back(Add(Mul(e,q[0]),Add(Mul(v,q[1]),Mul(n,q[2]))));}
  auto* proxy=MeridianRecipe::proxyRanges[h];Hull hull;for(auto n:normals){n=Unit(n);float support=h<6?LinkSupport(rings[h],rings[h+1],n):(h==6?DomeSupport(rings[6],apex,n):OvoidSupport(lobes[h-7],n));hull.push_back({n,support+.0002f});}hulls.push_back(std::move(hull));
 }
 Vec center{};for(unsigned i=0;i<80;i++)center=Add(center,Mul(points[i],1.f/80));
 auto original=points;std::vector<unsigned> cache;double steady=0;float worst=0;
 try{for(unsigned frame=0;frame<31;frame++){
  points=original;auto start=std::chrono::steady_clock::now();FitSeam(points,MeridianRecipe::columns,points[MeridianRecipe::clothCount-1],liveAxis,hulls,.12f,6.f);
  SeedMeridians(points,MeridianRecipe::columns,MeridianRecipe::rows,MeridianRecipe::rowHeights,liveAxis);
  auto receipt=ClearMeridians(points,MeridianRecipe::columns,MeridianRecipe::rows,MeridianRecipe::clothFaces,MeridianRecipe::clothFaceCount,hulls,liveAxis,.04f,12,&cache);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();if(frame){steady+=ms;worst=(std::max)(worst,float(ms));}
  if(!frame)std::printf("clear passes=%u separation=%f maximumDisplacement=%f firstWrapMs=%f\n",receipt.iterations,receipt.minimumSeparation,receipt.maximumDisplacement,ms);
 }
 std::printf("cachedWrapMeanMs=%f maxMs=%f samples=30 excludesSupportBuild=1\n",steady/30,worst);
 std::ofstream out(argv[2],std::ios::binary);out.write((char*)points.data(),points.size()*sizeof(Vec));
 }catch(const std::exception& e){std::printf("REJECT: %s\n",e.what());return 3;}
}
