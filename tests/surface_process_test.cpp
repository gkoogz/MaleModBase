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
  if(std::memcmp(&moved.collarMetric.axis,&initial.collarMetric.axis,sizeof(Point)*2))return 7;
  if(wire::Encode(wire::DecodeOutput(wire::Encode(moved)))!=wire::Encode(moved))return 3;
  Controls changed;changed.values[1]=100;session.SetControls(changed);session.Step();
  if(session.Read().collarMetric.generation<=initial.collarMetric.generation)return 4;
  Frame queries;queries.collarQueries={initial.body[0].positions[0],initial.body[1].positions[0],initial.shaftGuide[0]};
  changed.values[7]=1;session.SetControls(changed);for(unsigned i=0;i<60;i++)session.Step(queries);auto low=session.Read();
  changed.values[7]=100;session.SetControls(changed);for(unsigned i=0;i<60;i++)session.Step(queries);auto high=session.Read();
  if(std::memcmp(&low.collarMetric.axis,&high.collarMetric.axis,sizeof(Point)*2))return 8;
  if(std::abs(low.rootDirection.x-high.rootDirection.x)+std::abs(low.rootDirection.z-high.rootDirection.z)<.5f)return 9;
  if(high.collarDisplacements.size()!=queries.collarQueries.size())return 10;
  for(auto p:high.collarDisplacements)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return 11;
  try{Session second;return 5;}catch(const std::logic_error&){}
 }
 try{Session replacement;return 6;}catch(const std::logic_error&){}
 std::puts("PASS process lifetime isolation, source metric lifecycle and complete wire output");
}
