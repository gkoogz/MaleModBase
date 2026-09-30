// Exercise the production collision scheduler, not an unlimited plane callback.
static bool BudgetFloorRay(V3 origin,V3 direction,float distance,V3& point,V3& normal){
 if(direction.z>=-1e-7f)return false;
 float t=-origin.z/direction.z;if(t<0||t>distance)return false;
 point=origin+direction*t;normal={0,0,1};return true;
}
static bool CollisionBudgetTest(){
 for(int fps:{15,30,60,120}){
  ResetFluidCollision();tankCameraSceneTick=0;anatomyScene=0;
  fluidPhysicsRayTest=BudgetFloorRay;fluidPhysicsScale=1;fluidCollisionWorld=true;
  volumeFluid::collisionSweep=FluidCollisionSweep;volumeFluid::config={};volumeFluid::config.viscosity=24;
  teaching::Fluid fluid;fluid.SetVariationSeed(0x13579bdf);CHECK(fluid.Begin({0,0,50}));
  volumeFluid::SplatModel splat;splat.project=FluidProjectSplat;splat.resolve=FluidResolveSplat;CHECK(volumeFluid::splatBakes.Open());
  double contacted=0;unsigned contacts=0,maxQueries=0;
  for(int frame=0;frame<22*fps;frame++){
   ++renderFrameSerial;float t=(frame+1.f)/fps,yaw=fluid.MainLateralYaw(t);
   fluid.Advance(1.f/fps,{0,0,50},Unit({cosf(yaw),sinf(yaw),.4f}),{});
   for(const auto& hit:fluid.impacts){contacted+=hit.volume;++contacts;CHECK(fabsf(hit.p.z)<.001f);}
   maxQueries=max(maxQueries,fluidNxQueries);
   DWORD now=1000+(DWORD)(t*1000);CHECK(splat.Add(fluid.impacts,now));splat.Update(now);CHECK(fluidNxQueries<=224);
  }
  printf("Production collision %d FPS: emitted %.6f contacted %.6f (%.2f%%), %u contacts, max %u rays/frame\n",fps,fluid.emittedVolume,contacted,100*contacted/fluid.emittedVolume,contacts,maxQueries);
  CHECK(fabs(contacted-fluid.emittedVolume)<.001);CHECK(maxQueries<=128);
  double retained=0;unsigned visible=0,anchored=0;
  for(const auto& mark:splat.marks){for(float density:mark.density)retained+=density*volumeFluid::splatTexel*volumeFluid::splatTexel;for(const auto& sample:mark.samples)if(sample.field>0){++visible;if(sample.state==1)++anchored;}}
  printf("  Splat density %.6f, anchored visible samples %u/%u\n",retained,anchored,visible);
  CHECK(fabs(retained-contacted)<.01);CHECK(visible>0&&anchored==visible);
 }
 // A full-budget denial must retain the crossing, including for vertical walls.
 ResetFluidCollision();tankCameraSceneTick=0;anatomyScene=0;
 fluidPhysicsRayTest=BudgetFloorRay;fluidPhysicsScale=1;fluidCollisionWorld=true;volumeFluid::collisionSweep=FluidCollisionSweep;
 volumeFluid::CollisionPath path;volumeFluid::FluidImpact hit{};++renderFrameSerial;FluidQueryFrame();fluidNxQueries=128;
 CHECK(!path.Sweep({0,0,1},{0,0,-1},.1f,hit)&&path.pending);
 ++renderFrameSerial;CHECK(path.Sweep({0,0,-1},{0,0,-2},.1f,hit)&&!path.pending&&fabsf(hit.p.z)<.001f);
 fluidPhysicsRayTest=[](V3 p,V3 d,float length,V3& out,V3& n){if(d.x<=0)return false;float t=-p.x/d.x;if(t<0||t>length)return false;out=p+d*t;n={-1,0,0};return true;};
 ++renderFrameSerial;FluidQueryFrame();fluidNxQueries=128;CHECK(!path.Sweep({-1,0,10},{1,0,10},.1f,hit)&&path.pending);
 ++renderFrameSerial;CHECK(path.Sweep({1,0,10},{2,0,10},.1f,hit)&&!path.pending&&fabsf(hit.p.x)<.001f);
 fluidPhysicsRayTest=[](V3,V3,float,V3&,V3&){fluidPhysicsRayAvailable=false;return false;};
 ++renderFrameSerial;CHECK(!path.Sweep({0,0,1},{0,0,-1},.1f,hit)&&path.pending);
 fluidPhysicsRayTest=BudgetFloorRay;++renderFrameSerial;CHECK(path.Sweep({0,0,-1},{0,0,-2},.1f,hit)&&!path.pending);
 // No new splat or query after all detached samples have handed off their mass.
 teaching::ViscousThread burst;burst.live=true;volumeFluid::Settings cfg;cfg.gravity=0;
 for(int i=0;i<512;i++){burst.nodes.push_back({{float(i%32),float(i/32),1},{0,0,-120},0,1});if(i)burst.links.push_back({0,0,0,true});}
 std::vector<volumeFluid::FluidImpact> impacts;double burstVolume=0;
 for(int frame=0;frame<90;frame++){++renderFrameSerial;impacts.clear();burst.Step(1.f/60,frame/60.f,cfg,nullptr,{0,0,50},0,&impacts);for(const auto& impact:impacts)burstVolume+=impact.volume;CHECK(fluidNxQueries<=128);}
 CHECK(fabs(burstVolume-512)<.001);++renderFrameSerial;FluidQueryFrame();impacts.clear();burst.Step(1.f/60,1.51f,cfg,nullptr,{0,0,50},0,&impacts);CHECK(impacts.empty()&&fluidNxQueries==0);
 printf("PASS deferred crossing recovery, 512 simultaneous deposits without lost/duplicate volume, settled samples use zero queries\n");
 ResetFluidCollision();fluidPhysicsRayTest=nullptr;return true;
}
