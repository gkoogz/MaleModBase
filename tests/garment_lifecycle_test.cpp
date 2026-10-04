// This gate exercises rest-material ownership, not native rendering or the
// character-specific walked-sheet fit. Reuse the measured numerical fixture.
#define main garment_reference_main
#include "garment_test.cpp"
#undef main

bool PersistentRest(){
 auto in=Fixture();in.gravity={};in.deltaTime=0;
 // This legacy synthetic fixture contains three pole-truncated ellipsoids;
 // it is not a closed native contact surface. Lifecycle needs no contact mesh.
 in.anatomyTriangles.clear();
 Session cloth;auto first=cloth.Update(Style::WhiteJockstrap,in);
 const double restCircumference=first.measuredCircumference;
 const auto inputAnatomy=in.anatomy;
 auto moving=in;
 // Greater than the old 1.5% auto-refit threshold: ordinary pose deformation
 // must not silently redefine the garment's material lengths.
 for(auto& q:moving.waist){q.position[0]*=1.025;q.position[1]*=1.025;}
 auto animated=cloth.Update(Style::WhiteJockstrap,moving);
 REQUIRE(animated.physics.stateReady&&!animated.physics.reset);
 REQUIRE(animated.measuredCircumference==restCircumference);
 REQUIRE(animated.physics.accumulatedSeconds==0);
 REQUIRE(moving.anatomy.size()==inputAnatomy.size());
 for(unsigned i=0;i<inputAnatomy.size();i++)REQUIRE(moving.anatomy[i].position==inputAnatomy[i].position);
 // Only an explicit morphology event changes rest material in the same epoch.
 moving.restRevision++;
 auto refit=cloth.Update(Style::WhiteJockstrap,moving);
 REQUIRE(!refit.physics.reset&&refit.physics.stateReady);
 REQUIRE(std::abs(refit.measuredCircumference/restCircumference-1.025)<1e-12);
 moving.characterEpoch++;
 auto newCharacter=cloth.Update(Style::WhiteJockstrap,moving);
 REQUIRE(newCharacter.physics.reset&&newCharacter.physics.accumulatedSeconds==0);
 auto naked=cloth.Update(Style::Naked,moving);
 REQUIRE(naked.mesh.vertices.empty()&&!naked.physics.stateReady);
 auto dressed=cloth.Update(Style::WhiteJockstrap,moving);
 REQUIRE(dressed.physics.reset&&dressed.physics.stateReady);
 std::cout<<"PASS animated waist retains rest material, explicit morphology refit, epoch and clothing lifecycle; complete input anatomy retained\n";
 return true;
}
int main(){try{return PersistentRest()?0:1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
