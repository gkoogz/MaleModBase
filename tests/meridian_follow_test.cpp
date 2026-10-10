#include <malemod/garments/meridian_follow.hpp>
#include <cassert>
#include <cstdio>
using namespace malemod::garments::meridian;
static void Near(Vec a,Vec b){assert(Dot(Sub(a,b),Sub(a,b))<1e-8f);}
int main(){
 std::vector<Vec> raw{{-2,-2,0},{2,-2,0},{2,2,0},{-2,2,0},{-1,-1,2},{1,-1,2},{1,1,2},{-1,1,2},{0,0,4}};
 auto solved=raw;solved[0][2]=.1f;solved[4][2]=2.2f;
 std::vector<FollowFrame> rig{{{0,0,2},{1,0,0},{0,1,0},{0,0,1}}};
 SurfaceFollower follower;follower.Remember(solved,raw,rig,4,9);
 Face displayFaces[]={{0,1,4},{1,5,4},{4,5,8}};
 assert(follower.DisplayWithinBudget(solved,raw,rig,4,displayFaces,3));
 auto current=raw;assert(follower.Move(current,rig,4));for(unsigned i=0;i<9;i++)Near(current[i],solved[i]);
 auto transform=[](Vec p){return Vec{10-p[1],20+p[0],30+p[2]};};
 auto vector=[](Vec p){return Vec{-p[1],p[0],p[2]};};
 current=raw;for(auto& p:current)p=transform(p);auto moved=rig;
 for(auto& f:moved){f.center=transform(f.center);f.x=vector(f.x);f.y=vector(f.y);f.z=vector(f.z);}
 assert(follower.Move(current,moved,4));for(unsigned i=0;i<9;i++)Near(current[i],transform(solved[i]));
 {auto transformedRaw=raw;for(auto& p:transformedRaw)p=transform(p);assert(follower.DisplayWithinBudget(current,transformedRaw,moved,4,displayFaces,3));auto spike=current;spike[4][0]+=10;assert(!follower.DisplayWithinBudget(spike,transformedRaw,moved,4,displayFaces,3));}
 {auto small=rig;small[0].center[0]+=.15f;auto shape=raw;assert(follower.Move(shape,small,4));assert(follower.DisplayWithinBudget(shape,raw,small,4,displayFaces,3));small[0].center[0]+=1;assert(!follower.DisplayWithinBudget(shape,raw,small,4,displayFaces,3));}
 // Follow the existing rig in this frame, without a timestep or history lag.
 current=raw;moved=rig;moved[0].center[0]+=.4f;assert(follower.Move(current,moved,4));
 Near(current[0],solved[0]);Near(current[8],raw[8]);
 assert(std::abs(current[4][0]-(solved[4][0]+.4f*.875f))<1e-6f);
 // Repeated evaluation from the same physics state has no accumulated drift.
 auto first=current;for(unsigned n=0;n<500;n++){current=raw;assert(follower.Move(current,moved,4));for(unsigned i=0;i<9;i++)Near(current[i],first[i]);}
 // Orientation comes from this frame's rig, not just its center translation.
 current=raw;moved=rig;moved[0].x={0,1,0};moved[0].y={-1,0,0};assert(follower.Move(current,moved,4));
 Near(current[4],Add(Mul(solved[4],.125f),Mul(Vec{-solved[4][1],solved[4][0],solved[4][2]},.875f)));
 current=raw;moved[0].x=Mul(moved[0].x,1.5f);assert(!follower.Move(current,moved,4));for(unsigned i=0;i<9;i++)Near(current[i],raw[i]);
 Face face{0,1,2};std::vector<Vec> triangle{{2,-1,0},{2,1,0},{2,0,1}};
 std::vector<Hull> hulls{{{{1,0,0},1},{{-1,0,0},1},{{0,1,0},1},{{0,-1,0},1},{{0,0,1},1},{{0,0,-1},1}}};
 std::vector<unsigned> hints;assert(CertifyFollowedSurface(triangle,&face,1,hulls,hints));
 // Stale index cannot certify a changed collider or miss chord penetration.
 hulls[0][0].offset=3;assert(!CertifyFollowedSurface(triangle,&face,1,hulls,hints));
 hulls[0][0].offset=1;triangle={{-2,0,0},{2,0,0},{0,2,0}};assert(!CertifyFollowedSurface(triangle,&face,1,hulls,hints));
 triangle={{2,0,0},{2,0,0},{2,1,0}};assert(!CertifyFollowedSurface(triangle,&face,1,hulls,hints));
 // Current-pose support bounds reject remote pairs even when the sampled
 // plane list has no useful separating direction. A moving bound is rebuilt.
 Hull box;box.push_back({{0,0,1},1});
 box.support=[](Vec n){return std::abs(n[0])+std::abs(n[1])+std::abs(n[2]);};
 std::vector<Hull> bounded{box};std::vector<Vec> remote{{3,0,0},{3,1,0},{3,0,1}};
 assert(CertifyFollowedSurface(remote,&face,1,bounded,hints));
 auto unchanged=remote;assert(RefitFollowedSurface(remote,0,3,&face,1,bounded,hints));assert(remote==unchanged);
 bounded[0].support=[](Vec n){return 3*n[0]+std::abs(n[0])+std::abs(n[1])+std::abs(n[2]);};
 assert(!CertifyFollowedSurface(remote,&face,1,bounded,hints));
 // A small contact correction is certified, bounded, and does not touch pins.
 current=raw;current[4]={.99f,-.3f,0};current[5]={.99f,.3f,0};current[6]={.99f,0,.3f};Face contactFace{4,5,6};
 auto prior=current;assert(RefitFollowedSurface(current,4,9,&contactFace,1,hulls,hints));
 for(unsigned i=0;i<4;i++)Near(current[i],prior[i]);Near(current[8],prior[8]);
 for(unsigned i=4;i<7;i++){assert(current[i][0]>=1);assert(Dot(Sub(current[i],prior[i]),Sub(current[i],prior[i]))<=.12f*.12f);}
 assert(CertifyFollowedSurface(current,&contactFace,1,hulls,hints));
 current=prior;current[4][0]=.5f;auto failed=current;assert(!RefitFollowedSurface(current,4,9,&contactFace,1,hulls,hints));for(unsigned i=0;i<9;i++)Near(current[i],failed[i]);
 current=prior;current[0]=current[4];contactFace={0,5,6};failed=current;assert(!RefitFollowedSurface(current,4,9,&contactFace,1,hulls,hints));for(unsigned i=0;i<9;i++)Near(current[i],failed[i]);
 // A prescribed seam is reported independently, not silently declared clear
 // and not allowed to disable reuse of a certified movable interior.
 {current=prior;Face topology[]={{0,5,6},{4,5,6},{5,6,8}};std::vector<Face> interior;
  MovableClothFaces(topology,3,4,9,interior);assert(interior.size()==1&&interior[0]==topology[1]);
  auto pinned=current[0],pole=current[8];assert(RefitFollowedSurface(current,4,9,interior.data(),unsigned(interior.size()),hulls,hints));Near(current[0],pinned);Near(current[8],pole);
  current[0]=current[4];assert(!CertifyFollowedSurface(current,topology,3,hulls,hints));
 }
 follower.Reset();assert(!follower.Ready());
 std::puts("meridian anatomy-follow tests passed");
}
