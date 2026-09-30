#pragma once
static bool SplatContactTest(){
 ResetFluidCollision();fluidCollisionEmitterValid=false;
 fluidCollisionVertices={{-20,-20,0},{20,-20,0},{20,20,0},{-20,20,0}};
 fluidCollisionTriangles={{0,1,2,0,0},{0,2,3,0,1}};fluidTriangleOrder={0,1};FluidBuildBvh(0,2);FluidRefitBvh(0);
 volumeFluid::FluidImpact contact;float best=1.0001f;CHECK(FluidBodySweep({0,0,5},{0,0,-5},0,contact,best,false));
 contact.velocity={50,0,-20};contact.volume=5;contact.sourceId=777;
 volumeFluid::SplatModel model;model.project=FluidProjectSplat;model.resolve=FluidResolveSplat;model.Add({contact},1000);for(DWORD t=1000;t<3000;t+=16)model.Update(t);
 CHECK(model.marks.size()==1);
 volumeFluid::FluidImpact anchor;int anchors=0;for(const auto& sample:model.marks[0].samples)if(sample.state==1){anchor=sample.anchor;++anchors;}CHECK(anchors>5);V3 before,n;CHECK(FluidResolveSplat(anchor,before,n));
 // Rotate the actual receiver through 180 degrees and translate it. Every
 // footprint sample follows its own triangle, including the offset normal.
 for(auto& p:fluidCollisionVertices)p={p.x+10,-p.y+7,-p.z+3};FluidRefitBvh(0);
 V3 after,nn;CHECK(FluidResolveSplat(anchor,after,nn));CHECK(Length(after-V3{before.x+10,-before.y+7,-before.z+3})<.0001f&&nn.z<-.99f);
 // Topology source identifiers survive changes in aggregate triangle ordering.
 std::swap(fluidCollisionTriangles[0],fluidCollisionTriangles[1]);CHECK(FluidResolveSplat(anchor,after,nn));CHECK(Length(after-V3{before.x+10,-before.y+7,-before.z+3})<.0001f);
 // Curved chest-like receiver: portions of the footprint are farther than the
 // old 1.2-unit center-plane projection reach. Neighbor anchors must carry it.
 ResetFluidCollision();fluidCollisionEmitterValid=false;
 for(int y=-16;y<=16;y++)for(int x=-16;x<=16;x++)fluidCollisionVertices.push_back({float(x),float(y),-.04f*(x*x+y*y)});
 for(unsigned y=0;y<32;y++)for(unsigned x=0;x<32;x++){unsigned a=y*33+x,b=a+1,c=a+33,d=c+1,source=(unsigned)fluidCollisionTriangles.size();fluidCollisionTriangles.push_back({a,b,c,0,source});fluidCollisionTriangles.push_back({b,d,c,0,source+1});}
 fluidTriangleOrder.resize(fluidCollisionTriangles.size());for(unsigned i=0;i<fluidTriangleOrder.size();i++)fluidTriangleOrder[i]=i;FluidBuildBvh(0,(unsigned)fluidTriangleOrder.size());FluidRefitBvh(0);
 best=1.0001f;CHECK(FluidBodySweep({0,0,5},{0,0,-5},0,contact,best,false));contact.velocity={85,0,-45};contact.volume=24;
 volumeFluid::SplatModel curved;curved.project=FluidProjectSplat;curved.resolve=FluidResolveSplat;CHECK(curved.Add({contact},1000));
 for(DWORD t=1000;t<4000;t+=16){curved.Update(t);CHECK(curved.projectionBudget>=0);}
 int curvedSamples=0,outsideOldReach=0;std::vector<std::pair<volumeFluid::FluidImpact,V3>> curvedAnchors;
 for(const auto& sample:curved.marks[0].samples)if(sample.state==1&&sample.field>.0001f){V3 p,n;CHECK(FluidResolveSplat(sample.anchor,p,n));if(p.z<-1.3f)outsideOldReach++;curvedSamples++;curvedAnchors.push_back({sample.anchor,p});}
 CHECK(curvedSamples>30&&outsideOldReach>10);auto texture=curved.marks[0].pixels;
 for(auto& p:fluidCollisionVertices)p={p.x+10,-p.y+7,-p.z+3};FluidRefitBvh(0);
 for(const auto& a:curvedAnchors){V3 p,n;CHECK(FluidResolveSplat(a.first,p,n));CHECK(Length(p-V3{a.second.x+10,-a.second.y+7,-a.second.z+3})<.0001f);}
 curved.Update(4500);CHECK(curved.marks[0].pixels==texture);
 printf("PASS curved chest receiver: %d visible anchors, %d beyond old planar reach, deformation-stable position and texture\n",curvedSamples,outsideOldReach);
 ResetFluidCollision();fluidCollisionWorld=true;fluidPhysicsScale=1;anatomyScene=0;tankCameraSceneTick=0;++renderFrameSerial;
 fluidPhysicsRayTest=[](V3 o,V3 d,float range,V3& p,V3& n){if(fabsf(d.x)<1e-6f)return false;float t=(10-o.x)/d.x;if(t<0||t>range)return false;p=o+d*t;n={-1,0,0};return true;};
 volumeFluid::FluidImpact wall;CHECK(FluidCollisionSweep({9,0,10},{11,0,10},.1f,wall));CHECK(wall.kind==volumeFluid::FLUID_IMPACT_WORLD&&fabsf(wall.p.x-10)<.001f);
 volumeFluid::FluidImpact projected;CHECK(FluidProjectSplat(wall,{10,1,11},projected));CHECK(Length(projected.p-V3{10,1,11})<.001f);
 fluidPhysicsRayTest=nullptr;ResetFluidCollision();printf("PASS splat footprint projection, independent body anchors, 180-degree receiver motion, reordered triangles and vertical-wall impact/projection\n");return true;
}
