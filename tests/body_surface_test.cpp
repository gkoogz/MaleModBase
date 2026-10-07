#include "malemod/garments/jockstrap.hpp"
#include <iostream>
using namespace malemod::garments;
static void Check(bool value,const char* text){if(!value)throw std::runtime_error(text);}
int main(){try{
 std::vector<Sample> body(4);body[0].position={-2,-2,0};body[1].position={2,-2,0};body[2].position={2,2,0};body[3].position={-2,2,0};std::vector<std::array<std::uint32_t,3>> faces{{0,1,2},{0,2,3}};
 detail::BodyCollider collider;collider.Update(body,faces);Check(std::abs(collider.Closest({0,0,.3}).signedDistance-.3)<1e-12,"Outward surface clearance incorrect");Check(std::abs(collider.Closest({0,0,-.2}).signedDistance+.2)<1e-12,"Near-surface inside sign incorrect");
 // An edge pierces a face while neither edge endpoint lies on the face and
 // no pair of edges intersects. Vertex/edge distances alone miss this case.
 std::array<Point,3> crossing{{{0,0,-1},{.1,0,1},{0,.1,1}}};auto hit=collider.ClosestFace(crossing,.01);Check(hit.distance==0,"Complete cloth-face piercing missed");
 std::array<Point,3> clear{{{0,0,.2},{.1,0,.2},{0,.1,.2}}};auto clearHit=collider.ClosestFace(clear,.01);Check(clearHit.signedDistance==.01&&clearHit.distance==.01&&clearHit.triangle==unsigned(-1),"Disjoint bounded query did not retain its conservative radius/no-hit contract");
 // A contact between the old query radius and the correction threshold must
 // return a real physical triangle, never a miss or a virtual closure donor.
 const double action=.02,radius=cloth_contact::SearchRadius(.01,action);
 detail::BodyCollider::PointNeighborhood guardNeighborhood;
 auto guardedPoint=collider.NearCached({0,0,.015},radius,guardNeighborhood);
 Check(guardedPoint.triangle<faces.size()&&std::abs(guardedPoint.distance-.015)<1e-12,"Application guard lost a physical point contact");
 std::array<Point,3> guardedFace{{{0,0,.015},{.1,0,.015},{0,.1,.015}}};
 auto guarded=collider.ClosestFace(guardedFace,radius);
 Check(guarded.triangle<faces.size()&&std::abs(guarded.distance-.015)<1e-12,"Application guard lost a physical face contact");
 auto miss=collider.ClosestFace(clear,radius);
 Check(miss.triangle==unsigned(-1)&&miss.signedDistance>action,"No-hit result entered the contact application guard");
 auto pointMiss=collider.NearCached({0,0,.2},radius,guardNeighborhood);
 Check(pointMiss.triangle==unsigned(-1)&&pointMiss.signedDistance>action,"Cached no-hit point entered application guard");
 // Refit retained topology, changing the actual measured surface.
 for(auto& vertex:body)vertex.position[2]=.1;collider.Update(body,faces);Check(std::abs(collider.Closest({0,0,.3}).signedDistance-.2)<1e-12,"Body pose refit stale");
 Point origin{1000,-900,800};for(auto& vertex:body)vertex.position=Add(origin,Mul(vertex.position,100));collider.Update(body,faces,origin,100);Check(std::abs(collider.Closest({0,0,.3}).signedDistance-.2)<1e-12,"Unit/world-origin covariance failed");
 detail::BodyCollider::PointNeighborhood neighborhood;
 for(unsigned motion=0;motion<5;motion++){
  for(auto& vertex:body)vertex.position[2]+=.02*100;
  collider.Update(body,faces,origin,100);
  for(unsigned i=0;i<120;i++){
   Point p{double(i%17)/4-2,double(i%13)/4-1.5,double(i%9)/50};
   double radius=.03+double(i%5)*.025;
   auto exact=collider.Near(p,radius),cached=collider.NearCached(p,radius,neighborhood);
   Check(std::abs(exact.distance-cached.distance)<1e-12,"Cached point neighborhood missed a current triangle");
   if(exact.distance<radius)Check(std::abs(exact.signedDistance-cached.signedDistance)<1e-12,"Cached point contact changed its side");
  }
 }
 // A neighborhood must never be reused for another collider's geometry.
 detail::BodyCollider other;auto changedBody=body;for(auto& s:changedBody)s.position[2]-=50;
 other.Update(changedBody,faces,origin,100);
 auto exact=other.Near({0,0,0},1),cached=other.NearCached({0,0,0},1,neighborhood);
 Check(std::abs(exact.distance-cached.distance)<1e-12,"Cached point neighborhood leaked across collider owners");
 bool invalid=false;try{collider.Update(body,{{0,1,99}});}catch(const std::invalid_argument&){invalid=true;}Check(invalid,"Out-of-range native topology accepted");collider.Clear();Check(collider.Empty(),"Body collider reset failed");std::cout<<"PASS measured body signed contacts, actual face piercing, topology refit and unit/world-origin covariance\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
