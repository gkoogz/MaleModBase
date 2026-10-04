#include <malemod/garments/proximity_cache.hpp>
#include <malemod/garments/jockstrap.hpp>
#include <algorithm>
#include <iostream>
#include <limits>
using namespace malemod::garments::proximity;
static void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
template<class F>static void Reject(F f){bool rejected=false;try{f();}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Malformed certificate accepted");}
int main(){try{
 Motion motion;motion.Advance(0,true);auto first=motion.Current();PointCertificate cache;
 cache.Remember({Point{0,0,1}},1,true,first);
 Check(cache.ProvesClear({Point{.1,0,.9}},.1,first),"Conservative outside point was not reused");
 Check(!cache.ProvesClear({Point{0,0,.05}},.1,first),"Point approaching surface falsely certified");
 Check(!cache.ProvesClear({Point{0,0,-1}},.1,first),"Point crossing surface falsely certified");
 motion.Advance(.95);Check(!cache.ProvesClear({Point{0,0,1}},.1,motion.Current()),"Moving body was ignored");
 motion.Advance(0,true);Check(!cache.ProvesClear({Point{0,0,1}},.1,motion.Current()),"Topology reset reused stale certificate");
 cache.Remember({Point{0,0,1}},1,false,motion.Current());Check(!cache.ProvesClear({Point{0,0,1}},.1,motion.Current()),"Unsigned far-inside distance treated as outside");
 // A moving infinite plane supplies an independent exact signed-distance
 // oracle. Adversarial points traverse and cross it; no clear certificate may
 // hide a distance deficit. Motion bounds include every reversal, not net drift.
 unsigned reused=0;Motion plane;plane.Advance(0,true);PointCertificate point;double height=0;
 for(unsigned i=0;i<2000;i++){
  double next=.2*std::sin(i*.19);plane.Advance(std::abs(next-height));height=next;
  Point p{.03*std::sin(i*.7),.02*std::cos(i*.4),.6*std::cos(i*.07)+.5};double exact=p[2]-height;
  if(point.ProvesClear({p},.025,plane.Current())){Check(exact>.025,"Cached point failed independent plane oracle");reused++;}
  else point.Remember({p},std::abs(exact),exact>=0,plane.Current());
 }
 Check(reused>100,"Certificate never accelerates a moving outside surface");
 FaceCertificate face;Motion body;body.Advance(0,true);std::array<Point,3> triangle{{{0,0,.3},{1,0,.3},{0,1,.3}}};
 face.Remember(triangle,.3,true,body.Current());triangle[1][2]=.31;Check(face.ProvesClear(triangle,.1,body.Current()),"Separated face not certified");
 triangle[2][2]=-.1;Check(!face.ProvesClear(triangle,.1,body.Current()),"Edge-face crossing hidden by face certificate");
 PointCertificate tiny;tiny.Remember({Point{0,0,1}},1,true,first);Check(!tiny.ProvesClear({Point{0,0,.1}},.1,first),"Rounding at exact margin falsely certified");
 Reject([&]{motion.Advance(-1);});Reject([&]{point.ProvesClear({Point{0,0,0}},-1,first);});
 Reject([&]{point.Remember({Point{0,0,std::numeric_limits<double>::infinity()}},1,true,first);});
 // Oblique plane-box pruning must retain every within-margin physical pair.
 // Exact unpruned triangle distance is an independent broad-phase oracle.
 for(unsigned i=0;i<3500;i++){
  std::array<Point,3> a,b;Point lo{1e100,1e100,1e100},hi{-1e100,-1e100,-1e100};
  for(unsigned vertex=0;vertex<3;vertex++)for(unsigned axis=0;axis<3;axis++){
   a[vertex][axis]=std::sin((i+1)*.137+(vertex+1)*1.173+(axis+1)*.619);
   b[vertex][axis]=std::cos((i+1)*.091+(vertex+1)*.927+(axis+1)*1.313);
   lo[axis]=(std::min)(lo[axis],b[vertex][axis]);hi[axis]=(std::max)(hi[axis],b[vertex][axis]);
  }
  Point p,q;double distance=malemod::garments::detail::SurfaceTriangleDistance(a,b,p,q),margin=.03+.37*(.5+.5*std::sin(i*.271));
  if(distance<=margin)Check(malemod::garments::detail::FaceBoxesOverlap(a,lo,hi,margin),"Oblique plane-box bound pruned a real physical contact");
  bool original=malemod::garments::detail::FaceBoxesOverlap(a,lo,hi,margin);
  Point offset{1000,-900,800};for(auto& point:a)point=malemod::garments::Add(malemod::garments::Mul(point,100),offset);
  lo=malemod::garments::Add(malemod::garments::Mul(lo,100),offset);hi=malemod::garments::Add(malemod::garments::Mul(hi,100),offset);
  Check(original==malemod::garments::detail::FaceBoxesOverlap(a,lo,hi,margin*100),"Oblique box bound changed under explicit units/translation");
 }
 std::array<Point,3> oblique{{{0,0,0},{1,0,1},{0,1,1}}};
 Check(!malemod::garments::detail::FaceBoxesOverlap(oblique,{0,0,.9},{.1,.1,1},.1),"Oblique plane bound retained its empty coordinate-box volume");
 // Exact current triangle BVH oracle, including surface deformation. This
 // checks the collider's computed movement bound rather than a supplied bound.
 malemod::garments::detail::BodyCollider collider;
 std::vector<malemod::garments::Sample> vertices(8);
 std::array<Point,8> rest{{{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}};
 std::vector<std::array<std::uint32_t,3>> faces{{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
 PointCertificate actual;unsigned actualReused=0,seed=unsigned(-1),faceSeed=unsigned(-1);malemod::garments::detail::BodyCollider::FaceNeighborhood neighborhood;
 for(unsigned i=0;i<1000;i++){
  for(unsigned k=0;k<8;k++){vertices[k].position=rest[k];vertices[k].position[0]+=.1*std::sin(i*.017)+.02*rest[k][2]*std::sin(i*.03);}
  collider.Update(vertices,faces);Point p{1.6+.4*std::sin(i*.012),.1*std::cos(i*.09),.2};
  auto exact=collider.Closest(p);
  auto seeded=collider.Closest(p,seed);Check(std::abs(exact.distance-seeded.distance)<1e-12&&std::abs(exact.signedDistance-seeded.signedDistance)<1e-12,"Seeded BVH changed exact closest contact");seed=seeded.triangle;
  if(i%10==0){
   std::array<Point,3> cloth{{{-.4,-.3,1.1+.2*std::sin(i*.03)},{.5,.1,.9+.1*std::cos(i*.1)},{.1,.4,1.05}}};
   double brute=1e100;for(auto f:faces){std::array<Point,3> skin{vertices[f[0]].position,vertices[f[1]].position,vertices[f[2]].position};Point a,b;double d=malemod::garments::detail::SurfaceTriangleDistance(cloth,skin,a,b);brute=(std::min)(brute,d);}
   auto accelerated=collider.ClosestFace(cloth,.5);Check(std::abs(accelerated.distance-(std::min)(brute,.5))<1e-12,"Cached triangle bounds/SAT changed exact face clearance");
   auto seededFace=collider.ClosestFace(cloth,.5,faceSeed);Check(std::abs(seededFace.distance-accelerated.distance)<1e-12,"Previous physical triangle hid a closer current face");faceSeed=seededFace.triangle;
   auto unrelatedSeed=collider.ClosestFace(cloth,.5,11);Check(std::abs(unrelatedSeed.distance-accelerated.distance)<1e-12,"Unrelated physical triangle changed exact current face clearance");
   auto local=collider.ClosestFaceCached(cloth,.5,neighborhood,faceSeed);Check(std::abs(local.distance-accelerated.distance)<1e-12,"Reused moving face neighborhood omitted a closer current triangle");
  }
  if(actual.ProvesClear({p},.05,collider.MotionStamp())){Check(exact.signedDistance>.05,"Certificate disagrees with exact deformed triangle surface");actualReused++;}
  else actual.Remember({p},exact.distance,exact.signedDistance>=0,collider.MotionStamp());
 }
 Check(actualReused>800,"Exact body motion cache provided insufficient reuse");
 Point pausedProbe{1.3,.1,.2};auto pausedHit=collider.Closest(pausedProbe);auto pausedStamp=collider.MotionStamp();
 for(unsigned repeat=0;repeat<20;repeat++){
  collider.Update(vertices,faces);auto hit=collider.Closest(pausedProbe);
  Check(hit.point==pausedHit.point&&hit.normal==pausedHit.normal&&hit.distance==pausedHit.distance&&hit.triangle==pausedHit.triangle,"Exact unchanged surface update altered current contact");
  Check(collider.MotionStamp().surfaceTravel==pausedStamp.surfaceTravel&&collider.MotionStamp().topology==pausedStamp.topology,"Exact unchanged update invalidated movement certificates");
 }
 auto invalidStatic=vertices;invalidStatic[0].position[0]=std::numeric_limits<double>::infinity();Reject([&]{collider.Update(invalidStatic,faces);});
 // Any real coordinate change, however small, retains the normal update path.
 vertices[1].position[0]+=1e-9;collider.Update(vertices,faces);
 Check(collider.MotionStamp().surfaceTravel>pausedStamp.surfaceTravel,"Small but real surface motion was skipped as unchanged");
 Check(neighborhood.ReuseCount()>50&&neighborhood.RebuildCount()>0,"Physical contact neighborhoods never safely reused");
 malemod::garments::detail::BodyCollider::FaceNeighborhood narrow;bool subset=false;auto changingFaces=faces;
 for(unsigned i=0;i<400;i++){
  for(unsigned k=0;k<8;k++){vertices[k].position=rest[k];vertices[k].position[2]+=.01*std::sin(i*.02)+.006*rest[k][0]*std::cos(i*.07);}
  if(i==200)std::reverse(changingFaces.begin(),changingFaces.end());collider.Update(vertices,changingFaces);
  double lateral=-.3+1.5*double(i)/399;std::array<Point,3> cloth{{{lateral-.05,-.1,1.01},{lateral+.05,-.1,1.025},{lateral,.1,1.015}}};
  auto exact=collider.ClosestFace(cloth,.05),cached=collider.ClosestFaceCached(cloth,.05,narrow);
  Check(std::abs(cached.distance-exact.distance)<1e-12,"Contact entering a narrow cached neighborhood was omitted");subset=subset||narrow.CandidateCount()<changingFaces.size();
 }
 Check(subset&&narrow.ReuseCount()>100&&narrow.RebuildCount()>2,"Narrow moving neighborhood did not exercise candidate exclusion, reuse and topology resets");
 std::array<Point,3> far{{{0,0,4},{.1,0,4},{0,.1,4}}};auto noHit=collider.ClosestFace(far,.02,faceSeed);Check(noHit.triangle==unsigned(-1)&&noHit.distance==.02&&noHit.signedDistance==.02,"No within-radius face hit invented infinite clearance");
 FaceCertificate bounded;bounded.Remember(far,noHit.distance,true,collider.MotionStamp());Check(!bounded.ProvesClear(far,.025,collider.MotionStamp()),"Bounded face query incorrectly certifies a larger clearance");
 auto rebuilt=neighborhood.RebuildCount();auto nearCached=collider.ClosestFaceCached(far,.02,neighborhood);Check(nearCached.distance==.02&&neighborhood.RebuildCount()>rebuilt,"Large cloth motion reused an incomplete candidate set");
 malemod::garments::detail::BodyCollider foreign;foreign.Update(vertices,faces);rebuilt=neighborhood.RebuildCount();foreign.ClosestFaceCached(far,.02,neighborhood);Check(neighborhood.RebuildCount()>rebuilt,"Neighborhood from a different surface owner was reused");
 NeighborhoodBound coverage;coverage.Remember(far,.08,collider.MotionStamp(),1);Check(coverage.Covers(far,.02,collider.MotionStamp(),1),"Expanded candidate bound did not cover original query");auto closeStamp=collider.MotionStamp();closeStamp.surfaceTravel+=.061;Check(!coverage.Covers(far,.02,closeStamp,1),"Moving surface consumed neighborhood padding without rebuilding");Check(!coverage.Covers(far,.02,collider.MotionStamp(),2),"Neighborhood identity check failed");Reject([&]{coverage.Remember(far,0,collider.MotionStamp(),1);});
 Reject([&]{collider.ClosestFace(far,-.02);});Reject([&]{collider.ClosestFace(far,std::numeric_limits<double>::infinity());});auto nonfinite=far;nonfinite[1][0]=std::numeric_limits<double>::quiet_NaN();Reject([&]{collider.ClosestFace(nonfinite,.02);});
 // Many physical triangles inside one query must trigger an exact fallback,
 // never a partial 512-face collision surface. The last source face is closest.
 std::vector<malemod::garments::Sample> denseVertices;
 std::vector<std::array<std::uint32_t,3>> denseFaces;
 for(unsigned i=0;i<600;i++){
  unsigned start=unsigned(denseVertices.size());double z=.03+.00001*(599-i);
  for(Point p:std::array<Point,3>{{{-.1,-.1,z},{.1,-.1,z},{0,.1,z}}}){malemod::garments::Sample sample;sample.position=p;denseVertices.push_back(sample);}
  denseFaces.push_back({start,start+1,start+2});
 }
 malemod::garments::detail::BodyCollider dense;dense.Update(denseVertices,denseFaces);
 malemod::garments::detail::BodyCollider::FaceNeighborhood limited;
 std::array<Point,3> wide{{{-.1,-.1,0},{.1,-.1,0},{0,.1,0}}};
 for(unsigned repeat=0;repeat<3;repeat++){
  auto exhaustive=dense.ClosestFace(wide,.1),limitedHit=dense.ClosestFaceCached(wide,.1,limited,0);
  Check(std::abs(limitedHit.distance-exhaustive.distance)<1e-12&&std::abs(limitedHit.distance-.03)<1e-12,"Bounded candidate fallback omitted a closest physical face");
  Check(limited.CandidateCount()==0&&limited.FallbackCount()==repeat+1&&limited.ReuseCount()==0,"Partial oversized candidate list was retained or reused");
 }
 // A dense padded neighborhood can be complete at a smaller travel radius.
 // Retaining that complete set avoids needless repeated global searches.
 for(unsigned i=0;i<600;i++)for(unsigned k=0;k<3;k++)denseVertices[i*3+k].position[2]=i<510?.05+i*.00001:.25+(i-510)*.0001;
 dense.Update(denseVertices,denseFaces);malemod::garments::detail::BodyCollider::FaceNeighborhood adaptive;
 for(unsigned repeat=0;repeat<4;repeat++){
  auto face=wide;for(auto& p:face)p[2]+=.002*repeat;
  auto exhaustive=dense.ClosestFace(face,.1),cached=dense.ClosestFaceCached(face,.1,adaptive,599);
  Check(std::abs(cached.distance-exhaustive.distance)<1e-12,"Adaptive complete neighborhood omitted a physical contact");
  Check(adaptive.CandidateCount()==510&&adaptive.FallbackCount()==0,"Oversized padding failed to retain a complete smaller neighborhood");
 }
 Check(adaptive.ReuseCount()==3,"Adaptive neighborhood did not retain certified travel padding");
 for(unsigned k=0;k<3;k++)denseVertices[599*3+k].position[2]=.015;
 dense.Update(denseVertices,denseFaces);
 auto exhaustiveAdaptive=dense.ClosestFace(wide,.1),movedAdaptive=dense.ClosestFaceCached(wide,.1,adaptive);
 Check(std::abs(movedAdaptive.distance-exhaustiveAdaptive.distance)<1e-12&&movedAdaptive.triangle==599,"Adaptive padding missed a formerly excluded moving triangle");
 // Large motion in an unrelated source region must not discard an exact
 // nearby neighborhood. Its excluded current BVH bounds remain disjoint.
 auto remoteVertices=denseVertices;auto remoteFaces=denseFaces;
 for(unsigned i=0;i<600;i++)for(unsigned k=0;k<3;k++){auto& p=remoteVertices[i*3+k].position;p=wide[k];p[2]=.03;if(i)p[0]+=100+i;}
 malemod::garments::detail::BodyCollider remote;remote.Update(remoteVertices,remoteFaces);
 malemod::garments::detail::BodyCollider::FaceNeighborhood local;
 auto originalRemote=remote.ClosestFaceCached(wide,.1,local);
 Check(originalRemote.triangle==0&&local.RebuildCount()==1,"Local contact fixture did not establish its first exact neighborhood");
 for(unsigned frame=0;frame<20;frame++){
  for(unsigned i=1;i<600;i++)for(unsigned k=0;k<3;k++)remoteVertices[i*3+k].position[2]=30*std::sin(frame*.4);
  remote.Update(remoteVertices,remoteFaces);auto oracle=remote.ClosestFace(wide,.1),actual=remote.ClosestFaceCached(wide,.1,local);
  Check(actual.distance==oracle.distance&&actual.triangle==oracle.triangle,"Local exclusion witnesses changed a measured contact");
 }
 Check(local.RebuildCount()==1&&local.ReuseCount()==20,"Distant source movement needlessly rebuilt the local complete neighborhood");
 for(unsigned k=0;k<3;k++){remoteVertices[599*3+k].position=wide[k];remoteVertices[599*3+k].position[2]=.01;}
 remote.Update(remoteVertices,remoteFaces);auto remoteOracle=remote.ClosestFace(wide,.1),entered=remote.ClosestFaceCached(wide,.1,local);
 Check(entered.triangle==599&&entered.distance==remoteOracle.distance&&local.RebuildCount()>1,"A formerly excluded BVH region entered without exact neighborhood reconstruction");
 // Finite planar faces give an independent exact contact oracle. A bounded
 // outside query must cover the action threshold, including its guard band,
 // and keep the same physical point under rotation, translation and units.
 for(double units:std::array<double,3>{.001,1.,1000.})for(double angle:std::array<double,3>{0.,.37,1.17}){
  const Point shift{3.7*units,-5.1*units,8.3*units};
  auto transform=[&](Point p){return malemod::garments::Add(malemod::garments::Mul(Point{p[0]*std::cos(angle)+p[2]*std::sin(angle),p[1],-p[0]*std::sin(angle)+p[2]*std::cos(angle)},units),shift);};
  std::vector<malemod::garments::Sample> planeVertices(4);std::array<Point,4> planeRest{{{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}}};
  for(unsigned k=0;k<4;k++)planeVertices[k].position=transform(planeRest[k]);
  malemod::garments::detail::BodyCollider planeCollider;planeCollider.Update(planeVertices,{{0,1,2},{0,2,3}});
  const double margin=.003,action=margin+1e-5,guard=action+1e-8;
  for(double gap:std::array<double,8>{1e-6,margin-1e-6,margin+4e-6,margin+8e-6,action-1e-9,action+5e-9,.02,1.}){
   Point p=transform({.2,.1,gap}),oraclePoint=transform({.2,.1,0});auto exact=planeCollider.Closest(p),near=planeCollider.Near(p,guard*units,1);
   Check(std::abs(exact.distance-gap*units)<2e-13*units,"Finite-plane exact distance failed explicit source-unit oracle");
   Check(std::abs(near.distance-(std::min)(gap,guard)*units)<2e-13*units,"Bounded point query lost a guarded physical contact");
   Check((near.distance<action*units)==(gap<action),"Bounded outside query changed the actual projection action");
   if(gap<guard){Check(malemod::garments::Length(malemod::garments::Sub(near.point,oraclePoint))<2e-13*units,"Bounded point query changed the measured physical contact point");Check(malemod::garments::Length(malemod::garments::Sub(near.point,exact.point))<2e-13*units,"Bounded and exhaustive physical contacts disagree");}
  }
 }
 auto stamp=collider.MotionStamp();collider.Clear();Check(collider.MotionStamp().topology!=stamp.topology,"Cleared collider keeps stale movement epoch");
 std::cout<<"PASS certified point/face separation, moving-surface and exhaustive face oracle, exact seeded queries, crossing, inside, topology and finite-value gates; reused="<<reused<<", actual="<<actualReused<<"\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
