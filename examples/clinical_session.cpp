#include <malemod/clinical/session.hpp>
#include <iostream>

// CPU integration example. An engine supplies a live outlet pose, collision
// callbacks and uploads the resulting mesh/deposit data to its own renderer.
int main(int argc,char** argv){
 malemod::clinical::Session session;
 if(argc==2&&!session.deposits.splatBakes.LoadFile(argv[1]))return 1;
 session.fluid.collisionQuery.callback=[](malemod::V3,malemod::V3,float,malemod::clinical::volumeFluid::FluidImpact&){
  return malemod::clinical::volumeFluid::SweepResult::miss;
 };
 if(!session.Begin({0,0,100})){std::cerr<<session.fluid.error<<'\n';return 1;}
 for(unsigned frame=0;frame<1320;frame++){
  if(!session.Advance(1.f/60,{{0,0,100},{1,0,0},{}},frame*1000/60))return 1;
  // session.timeline.Get(): deformation envelopes for the character adapter.
  // session.fluid.mesh: positions/normals and triangle indices.
  // session.fluid.impacts: stable receiver anchors and volume bookkeeping.
  // session.deposits.marks: CPU density and packed normal/coverage fields.
 }
 std::cout<<"Simulated volume: "<<session.fluid.emittedVolume<<" source model units cubed\n";
}
