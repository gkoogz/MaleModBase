#include <malemod/surface/runtime.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <future>
#include <stdexcept>
using namespace malemod::surface;
static bool Same(const Output& a,const Output& b){
 return a.anatomy.positions.size()==b.anatomy.positions.size()&&
  !std::memcmp(a.anatomy.positions.data(),b.anatomy.positions.data(),a.anatomy.positions.size()*sizeof(Point));
}
static void Advance(Session& s){for(int i=0;i<12;i++){Frame f;f.pitchForce=std::sin(i*.3f)*.25f;f.yawForce=std::cos(i*.2f)*.15f;s.Step(f);}}
int main(){
 Controls large;large.values[1]=100;large.values[2]=100;large.values[3]=100;large.values[4]=100;large.values[5]=100;
 Session baseline,first,other(large);Advance(baseline);
 auto run=std::async(std::launch::async,[&]{Advance(other);});Advance(first);run.get();
 auto expected=baseline.Read(),actual=first.Read();
 if(!Same(expected,actual)){std::puts("FAIL: another session changed the first session");return 1;}
 if(Same(actual,other.Read())){std::puts("FAIL: different preferences produced identical surfaces");return 2;}
 if(actual.anatomy.positions.size()!=17528||actual.anatomyIndices.size()!=105000||actual.body[0].positions.empty()||actual.body[1].positions.empty())return 3;
 for(unsigned i=0;i<actual.anatomy.positions.size();i++)if(actual.anatomy.sourceVertexIDs[i]!=i)return 4;
 // The guide and surface must be captured from the same isolated worker state.
 if(std::memcmp(expected.shaftGuide.data(),actual.shaftGuide.data(),sizeof(actual.shaftGuide))||std::memcmp(expected.lobeCenters.data(),actual.lobeCenters.data(),sizeof(actual.lobeCenters)))return 7;
 const auto metric=actual.collarMetric;
 if(!metric.generation||metric.radius<=0||metric.length<=0)return 10;
 for(unsigned i=1;i<12;i++){
  auto a=actual.shaftGuide[i-1],b=actual.shaftGuide[i];float d=std::sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y)+(b.z-a.z)*(b.z-a.z));
  if(!std::isfinite(d)||std::abs(d-actual.restLength/11.f)>.02f)return 8;
 }
 for(float multiplier:actual.bendMultipliers)if(!(multiplier>0&&multiplier<=1))return 9;
 Controls invalid;invalid.values[1]=NAN;
 try{first.SetControls(invalid);return 5;}catch(const std::invalid_argument&){}
 if(!Same(actual,first.Read()))return 6;
 // A rejected request must leave the worker usable and its state unchanged.
 first.Step();
 const auto moved=first.Read();
 if(moved.collarMetric.generation!=metric.generation||std::memcmp(&moved.collarMetric.root,&metric.root,sizeof(Point)*3))return 11;
 first.SetControls(large);first.Step();
 if(first.Read().collarMetric.generation<=metric.generation)return 12;
 std::puts("PASS: simultaneous instance isolation, full topology/body output and rejected input recovery");
}
