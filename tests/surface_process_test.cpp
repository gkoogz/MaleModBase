#include <malemod/surface/wire.hpp>
#include <cmath>
#include <cstdio>
using namespace malemod::surface;
int main(){
 {
  Session session;auto initial=session.Read();
  if(!initial.collarMetric.generation)return 1;
  for(unsigned i=0;i<12;i++){Frame f;f.pitchForce=.2f*std::sin(i*.15f);f.yawForce=.15f*std::cos(i*.1f);session.Step(f);}
  auto moved=session.Read();
  if(moved.collarMetric.generation!=initial.collarMetric.generation)return 2;
  if(wire::Encode(wire::DecodeOutput(wire::Encode(moved)))!=wire::Encode(moved))return 3;
  Controls changed;changed.values[1]=100;session.SetControls(changed);session.Step();
  if(session.Read().collarMetric.generation<=initial.collarMetric.generation)return 4;
  try{Session second;return 5;}catch(const std::logic_error&){}
 }
 try{Session replacement;return 6;}catch(const std::logic_error&){}
 std::puts("PASS process lifetime isolation, source metric lifecycle and complete wire output");
}
