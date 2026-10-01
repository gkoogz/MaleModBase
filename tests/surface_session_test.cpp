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
 Controls invalid;invalid.values[1]=NAN;
 try{first.SetControls(invalid);return 5;}catch(const std::invalid_argument&){}
 if(!Same(actual,first.Read()))return 6;
 // A rejected request must leave the worker usable and its state unchanged.
 first.Step();
 std::puts("PASS: simultaneous instance isolation, full topology/body output and rejected input recovery");
}
