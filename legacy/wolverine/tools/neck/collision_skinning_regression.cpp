#define NECK_TEST_MAIN
#include "../raphe/raphe_test.cpp"
static void ReferenceAppend(const unsigned char* packed,unsigned vc,const unsigned short* ix,unsigned tc,const unsigned char* bones,const float* palette,unsigned count,unsigned char section,bool world,bool topology){
 unsigned base=(unsigned)fluidCollisionVertices.size();fluidCollisionVertices.reserve(base+vc);
 for(unsigned i=0;i<vc;i++){V3 p=FluidSkinVertex(packed+i*32,bones?bones+i*4:nullptr,palette,count);if(world)p=FluidMatrixPoint(fluidCollisionLocal,p)+fluidCameraWorld;fluidCollisionVertices.push_back(p);}
 if(topology)for(unsigned i=0;i<tc;i++)fluidCollisionTriangles.push_back({base+ix[i*3],base+ix[i*3+1],base+ix[i*3+2],section,i});
}
static bool Run(){
 CHECK(RapheInit(2,85));float palette[5*12];
 for(unsigned b=0;b<5;b++)for(unsigned j=0;j<12;j++)palette[b*12+j]=j%5==0?1.f:float(int((b*19+j*7)%17)-8)*.013f;
 for(unsigned j=0;j<16;j++)fluidCollisionLocal[j]=j%5==0?1.f:float(int(j%7)-3)*.02f;fluidCameraWorld={31,-12,4};
 std::vector<unsigned char> packed(nrPacked,nrPacked+sizeof(nrPacked)),bones(nrCount*4);
 for(unsigned i=0;i<nrCount;i++)for(unsigned j=0;j<4;j++)bones[i*4+j]=(i+j)%7;
 // Include invalid palettes and zero-weight fallback vertices alongside the
 // full authored packed surface, both component and world coordinates.
 for(unsigned i=0;i<nrCount;i+=113)memset(packed.data()+i*32+24,0,4);
 for(bool world:{false,true})for(bool overrideBones:{false,true})for(unsigned count:{0u,3u,5u}){
  const unsigned char* override=overrideBones?bones.data():nullptr;
  fluidCollisionVertices.assign(3,V3{1,2,3});fluidCollisionTriangles.clear();
  ReferenceAppend(packed.data(),nrCount,nrIndices,nrIndexCount/3,override,palette,count,2,world,true);
  auto expected=fluidCollisionVertices;auto triangles=fluidCollisionTriangles;
  fluidCollisionVertices.assign(3,V3{1,2,3});fluidCollisionTriangles.clear();
  FluidAppendSection(packed.data(),nrCount,nrIndices,nrIndexCount/3,override,palette,count,2,world,true);
  CHECK(expected.size()==fluidCollisionVertices.size());CHECK(memcmp(expected.data(),fluidCollisionVertices.data(),expected.size()*sizeof(V3))==0);
  CHECK(triangles.size()==fluidCollisionTriangles.size());for(unsigned i=0;i<triangles.size();i++){const auto&a=triangles[i];const auto&b=fluidCollisionTriangles[i];CHECK(a.a==b.a&&a.b==b.b&&a.c==b.c&&a.section==b.section&&a.source==b.source);}
 }
 // Compact topology is the only anatomy collider. Cached triangle/barycentric
 // anchors must still resolve after deformation and a BVH refit.
 ResetFluidCollision();
 FluidAppendSection(nrPacked,nrCount,nrIndices,nrIndexCount/3,nullptr,nullptr,0,2,false,true);
 CHECK(fluidCollisionTriangles.size()==35000&&nrIndexCount/3==35000);
 fluidTriangleOrder.resize(fluidCollisionTriangles.size());
 for(unsigned i=0;i<fluidTriangleOrder.size();i++)fluidTriangleOrder[i]=i;
 FluidBuildBvh(0,(unsigned)fluidTriangleOrder.size());FluidRefitBvh(0);
 for(unsigned i=0;i<fluidCollisionTriangles.size();i++){
  const auto& t=fluidCollisionTriangles[i];CHECK(t.source==i&&t.section==2);
  CHECK(t.a<nrCount&&t.b<nrCount&&t.c<nrCount);
 }
 for(unsigned frame=0;frame<12;frame++){
  UpdateConstraintSolver(1.f/60.f,.1f,sinf(frame*.3f)*.2f);ApplyShape();
  fluidCollisionVertices.clear();
  FluidAppendSection(nrPacked,nrCount,nrIndices,nrIndexCount/3,nullptr,nullptr,0,2,false,false);
  FluidRefitBvh(0);
  for(unsigned i=0;i<35000;i+=173){
   const auto& t=fluidCollisionTriangles[i];V3 p,n;
   CHECK(FluidResolveBodyAnchor(i,.2f,.3f,p,n));
   V3 a,b,c;memcpy(&a,nrPacked+t.a*32,12);memcpy(&b,nrPacked+t.b*32,12);memcpy(&c,nrPacked+t.c*32,12);
   CHECK(Length(p-(a*.5f+b*.2f+c*.3f))<1e-5f);
   if(Length(Cross(b-a,c-a))>1e-5f){
    volumeFluid::FluidImpact hit{};float best=1.f;
    CHECK(FluidBodySweep(p+n*.02f,p-n*.02f,0,hit,best,false));
    CHECK(hit.triangle<fluidCollisionTriangles.size());
   }
  }
 }
 printf("PASS compact 35,000-triangle collider: valid source IDs, 12 BVH refits, 2,436 moving barycentric anchors and body sweeps\n");
 double serial=0,batched=0;
 for(unsigned i=0;i<120;i++){
  bool first=i%2==0;
  for(unsigned j=0;j<2;j++){
   bool reference=first==(j==0);fluidCollisionVertices.clear();double begin=PerfClock();
   if(reference)ReferenceAppend(nrPacked,nrCount,nrIndices,nrIndexCount/3,nullptr,palette,5,2,true,false);
   else FluidAppendSection(nrPacked,nrCount,nrIndices,nrIndexCount/3,nullptr,palette,5,2,true,false);
   double elapsed=PerfClock()-begin;if(i>=20)(reference?serial:batched)+=elapsed;
  }
 }
 printf("PASS 12 exact section comparisons: palette fallbacks, bone overrides, world/component coordinates, appended indices. Skinning %u vertices serial %.3f batched %.3f ms (100 alternating samples)\n",nrCount,serial/100,batched/100);return true;
}
int main(){setvbuf(stdout,nullptr,_IONBF,0);return Run()?0:1;}
