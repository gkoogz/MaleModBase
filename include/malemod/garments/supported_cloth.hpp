#pragma once
#ifdef MALEMOD_SUPPORTED_PROFILE
#include <cstdio>
#endif
namespace malemod::garments {
// Pouch cloth with prescribed, measured trim. The immutable fitted pattern
// supplies material lengths; only explicit morphology revisions rebuild it.
inline void Session::SupportedCloth(const Input& input,const std::vector<Point>& targets,double elapsed){
 using namespace cloth_detail;
 auto began=std::chrono::steady_clock::now();auto& mesh=output_.mesh;auto& t=output_.physics;
 const double C=output_.measuredCircumference,margin=parameters_.clearance*.3;
 const auto origin=input.frame.origin;
 auto local=[&](Point p){return Mul(input.frame.Local(p),1/C);};
 auto localVector=[&](Point p){return Point{Dot(p,input.frame.lateral),Dot(p,input.frame.forward),Dot(p,input.frame.up)};};
 auto worldVector=[&](Point p){return render_contact::RestVector(input.frame,p);};
 const Frame materialFrame{};
 auto world=[&](Point p){return input.frame.World(Mul(p,C));};
 auto& colliderInput=supportedContactInput_;if(colliderInput.bodyTriangles!=input.bodyTriangles)colliderInput.bodyTriangles=input.bodyTriangles;if(colliderInput.anatomyTriangles!=input.anatomyTriangles)colliderInput.anatomyTriangles=input.anatomyTriangles;
 colliderInput.bodySurface.resize(input.bodySurface.size());colliderInput.anatomy.resize(input.anatomy.size());
 for(unsigned i=0;i<input.bodySurface.size();i++)colliderInput.bodySurface[i].position=input.frame.Local(input.bodySurface[i].position);
 for(unsigned i=0;i<input.anatomy.size();i++)colliderInput.anatomy[i].position=input.frame.Local(input.anatomy[i].position);
 bodyCollider_.Update(colliderInput.bodySurface,input.bodyTriangles,{},C);
 anatomyCollider_.Update(colliderInput.anatomy,input.anatomyTriangles,{},C);
 auto physicalUpdated=std::chrono::steady_clock::now();
 ClassifySurfaces(colliderInput,{},C);
 auto classificationUpdated=std::chrono::steady_clock::now();
 std::vector<Point> x(positions_.size()),velocity(x.size()),fixed(x.size());
 for(unsigned i=0;i<x.size();i++){x[i]=local(positions_[i]);velocity[i]=Mul(localVector(velocities_[i]),1/C);fixed[i]=local(targets[i]);}
 std::vector<unsigned> bodySeed(x.size(),UINT32_MAX),anatomySeed(x.size(),UINT32_MAX);
 auto render=[&](unsigned i){return render_contact::Evaluate(renderBindings_[i],x,materialFrame,C);};
 auto seamOffset=[&](Point p){return local(input.frame.World(p));};
 auto updateSewing=[&](){for(auto& s:sewing_){Point rendered=Mul(s.separation,-1/C),linear{};for(auto term:s.vertices)rendered=Add(rendered,Mul(render(term.first),term.second));for(unsigned k=0;k<s.count;k++)linear=Add(linear,Mul(x[s.nodes[k]],s.weights[k]));s.residual=Mul(Sub(rendered,linear),C);}};
 auto sew=[&](){updateSewing();for(const auto& s:sewing_){Point error=seamOffset(s.residual);double weight=0;for(unsigned k=0;k<s.count;k++){error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));weight+=inverseMass_[s.nodes[k]]*s.weights[k]*s.weights[k];}if(weight>1e-20)for(unsigned k=0;k<s.count;k++)if(inverseMass_[s.nodes[k]])x[s.nodes[k]]=Sub(x[s.nodes[k]],Mul(error,inverseMass_[s.nodes[k]]*s.weights[k]/weight));}};
 std::vector<unsigned char> pouchNode(x.size()),bandNode(x.size());
 for(unsigned i=0;i<output_.layout.sheet.start;i++)bandNode[vertexNodes_[i]]=1;
 for(unsigned i=output_.layout.sheet.start;i<output_.layout.sheet.start+(parameters_.pouchRings+1)*(parameters_.pouchSegments+1);i++)pouchNode[vertexNodes_[i]]=1;
 auto pouchEdge=[&](const auto& e){return pouchNode[e.a]&&pouchNode[e.b];};
 auto materialLimit=[&](const auto& e){return pouchEdge(e)?1.15:1.5;};
 auto strainEdges=edges_;for(auto& e:strainEdges)if(!pouchEdge(e))e.rest*=1.4/1.10;
 ReactionCollector reactions(input.anatomy);
 std::vector<Point> contactDelta(x.size()),tissueDelta(x.size());
 // Substep both material and measured obstacles through each captured pose
 // interval. Kinematic motion must advance with the cloth, not teleport ahead
 // of several numerical steps.
 if(!std::isfinite(elapsed)||elapsed<0)throw std::invalid_argument("Invalid supported cloth interval");
 const unsigned steps=elapsed>0?unsigned(std::ceil(elapsed*60)):0;
 const double step=steps?elapsed/steps:1./60;
 accumulator_+=elapsed;
 const unsigned passes=(std::max)(1u,steps);
 const auto start=x,finalFixed=fixed;
 for(auto& memos:pointMemos_)memos.resize(x.size()+mesh.vertices.size());
 for(auto& memos:faceMemos_)memos.resize(mesh.triangles.size());
 auto pointSlot=[&](unsigned vertex){return unsigned(x.size())+vertex;};
 auto pointQuery=[&](bool anatomical,unsigned slot,Point point,double clearance){
  // Search also covers the numerical halo around the correction threshold;
  // only the existing, smaller application guard may apply a force.
  if(clearance>0)clearance=cloth_contact::SearchRadius(clearance,margin);
  auto& physical=anatomical?anatomyCollider_:bodyCollider_;auto& closed=anatomical?closedAnatomy_:closedBody_;auto& classifier=closed.Empty()?physical:closed;auto& memo=pointMemos_[anatomical][slot];auto stamp=classifier.MotionStamp();
  detail::BodyCollider::Hit hit;if(memo.certificate.ProvesClear({point},clearance,stamp)){hit.distance=hit.signedDistance=memo.certificate.LowerBound({point},stamp);return hit;}
  // Exhausting a clearance certificate does not exhaust its separate
  // outside-membership proof. Reuse the positive closed-surface separation
  // to avoid a redundant ray classification for a near-contact point.
  if(memo.certificate.ProvesClear({point},0,stamp)){
   // Membership-only callers need a certified positive lower bound, not the
   // global closest point. Positive-clearance callers still obtain every real
   // contact inside all projection/friction action guards. A no-hit radius is
   // conservative coverage telemetry and must remain outside those guards.
   if(clearance==0){hit.distance=hit.signedDistance=memo.certificate.LowerBound({point},stamp);return hit;}
   const double guard=(std::max)(clearance,margin+1e-5)+1e-8;
   hit=physical.NearCached(point,guard,memo.physicalNeighborhood,memo.physicalSeed);
   if(hit.distance<guard)memo.physicalSeed=hit.triangle;else hit.triangle=UINT32_MAX;
   hit.signedDistance=hit.distance;auto delta=Sub(point,hit.point);
   if(hit.distance<guard&&Length(delta)>1e-14)hit.normal=Unit(delta);return hit;
  }
  auto classified=classifier.Closest(point,memo.closedSeed);memo.closedSeed=classified.triangle;auto side=classifier.Classify(point,!closed.Empty());bool outside=side==detail::BodyCollider::Side::Outside;
  memo.certificate.Remember({point},classified.distance,outside,stamp);
  if(outside&&classified.distance>clearance){classified.signedDistance=classified.distance;return classified;}
  hit=physical.Closest(point,memo.physicalSeed);memo.physicalSeed=hit.triangle;if(!closed.Empty()&&side==detail::BodyCollider::Side::Indeterminate)throw std::invalid_argument("Verified cloth volume membership is indeterminate");if(outside){hit.signedDistance=hit.distance;auto delta=Sub(point,hit.point);if(Length(delta)>1e-14)hit.normal=Unit(delta);}else if(side==detail::BodyCollider::Side::Inside){hit.signedDistance=-hit.distance;auto delta=Sub(hit.point,point);if(Length(delta)>1e-14)hit.normal=Unit(delta);}else if(side==detail::BodyCollider::Side::Boundary)hit.signedDistance=0;return hit;
 };
 auto faceQuery=[&](bool anatomical,unsigned index,const std::array<Point,3>& points,double clearance){
  clearance=cloth_contact::SearchRadius(clearance,margin);
  auto& physical=anatomical?anatomyCollider_:bodyCollider_;auto& closed=anatomical?closedAnatomy_:closedBody_;auto& classifier=closed.Empty()?physical:closed;auto& memo=faceMemos_[anatomical][index];auto stamp=classifier.MotionStamp();detail::BodyCollider::Hit hit;
  if(memo.certificate.ProvesClear(points,clearance,stamp)){hit.distance=hit.signedDistance=memo.certificate.LowerBound(points,stamp);return hit;}
  hit=physical.ClosestFaceCached(points,clearance,memo.physicalNeighborhood,memo.physicalSeed);if(hit.triangle!=UINT32_MAX)memo.physicalSeed=hit.triangle;
  // A positive closed-surface separation still certifies membership even
  // when its clearance reserve is exhausted. Query the physical neighborhood
  // above, but do not reclassify the same nearby face until that proof expires.
  if(memo.certificate.ProvesClear(points,0,stamp)){hit.signedDistance=hit.distance;auto delta=Sub(hit.clothPoint,hit.point);if(hit.distance<clearance&&Length(delta)>1e-14)hit.normal=Unit(delta);return hit;}
  double radius=clearance+.01;auto classification=classifier.ClosestFaceCached(points,radius,memo.closedNeighborhood,memo.closedSeed);if(classification.triangle!=UINT32_MAX)memo.closedSeed=classification.triangle;bool outside=true;
  for(unsigned k=0;k<3;k++)if(pointQuery(anatomical,pointSlot(mesh.triangles[index].vertices[k]),points[k],0).signedDistance<0)outside=false;
  memo.certificate.Remember(points,(std::min)(classification.distance,radius),outside,stamp);if(outside){hit.signedDistance=hit.distance;auto delta=Sub(hit.clothPoint,hit.point);if(hit.distance<clearance&&Length(delta)>1e-14)hit.normal=Unit(delta);}return hit;
 };
 auto residual=[&](Point local){return Mul(Sub(input.frame.World(local),origin),1/C);};
 // Kinematic trim has no cloth degrees of freedom. Build a vertex support
 // mask once, including transported ribbon/director dependencies.
 std::vector<unsigned char> movable(mesh.vertices.size(),1);
 if(parameters_.supportedTrim)for(unsigned i=0;i<mesh.vertices.size();i++){
  const auto& b=renderBindings_[i];bool free=false;
  for(unsigned k=0;k<4;k++)if(b.weights[k]&&!pinned_[b.nodes[k]])free=true;
  if(b.ribbon||b.transported)for(unsigned k=0;k<(b.ribbon?2u:3u);k++)if(!pinned_[b.materialFrame[k]])free=true;
  movable[i]=free;
 }
 auto movableFace=[&](const Triangle& f){return movable[f.vertices[0]]||movable[f.vertices[1]]||movable[f.vertices[2]];};

 auto project=[&](unsigned i,bool anatomy,bool record){
  auto& collider=anatomy?anatomyCollider_:bodyCollider_;if(collider.Empty())return false;
  const double clearance=margin+(bandNode[i]?parameters_.bandThickness*.5:0);
  auto hit=pointQuery(anatomy,i,x[i],clearance+3e-5);
  // A physical oriented surface supplies the contact normal. Closure faces
  // are not physical obstacles and must not push the supporting garment.
  if(hit.signedDistance>=clearance+1e-5)return false;
  double depth=clearance+3e-5-hit.signedDistance;
  auto correction=Mul(hit.normal,depth);x[i]=Add(x[i],correction);contactDelta[i]=Add(contactDelta[i],correction);if(anatomy)tissueDelta[i]=Add(tissueDelta[i],correction);
  if(pinned_[i])fixed[i]=x[i];
  if(anatomy&&record&&inverseMass_[i]>0&&hit.triangle<input.anatomyTriangles.size()){
   // The contact point belongs to this substep's interpolated surface.
   // Final-pose vertices can put its barycentrics outside the actual triangle.
   auto f=input.anatomyTriangles[hit.triangle];auto bary=anatomyCollider_.ContactWeights(hit);
   reactions.Add(hit.triangle,bary,worldVector(Mul(correction,C/(step*inverseMass_[i]))),world(hit.point),hit.signedDistance*C,input.anatomyTriangles);
  }
  output_.projectedContacts++;return true;
 };
 unsigned finalPasses=0,visits=0;
 auto faceBegan=std::chrono::steady_clock::now();
 for(unsigned sub=0;sub<passes;sub++){
  std::fill(contactDelta.begin(),contactDelta.end(),Point{});
  std::fill(tissueDelta.begin(),tissueDelta.end(),Point{});
  const double fraction=double(sub+1)/passes;
  for(unsigned i=0;i<x.size();i++)if(pinned_[i])fixed[i]=Add(Mul(start[i],1-fraction),Mul(finalFixed[i],fraction));
  if(passes>1){
   for(unsigned i=0;i<input.bodySurface.size();i++)colliderInput.bodySurface[i].position=input.frame.Local(Add(Mul(previousBodySurface_[i].position,1-fraction),Mul(input.bodySurface[i].position,fraction)));
   for(unsigned i=0;i<input.anatomy.size();i++)colliderInput.anatomy[i].position=input.frame.Local(Add(Mul(previousAnatomySurface_[i].position,1-fraction),Mul(input.anatomy[i].position,fraction)));
   bodyCollider_.Update(colliderInput.bodySurface,input.bodyTriangles,{},C);anatomyCollider_.Update(colliderInput.anatomy,input.anatomyTriangles,{},C);ClassifySurfaces(colliderInput,{},C);
  }
  auto old=x;
  for(unsigned i=0;i<x.size();i++){
   if(pinned_[i])x[i]=fixed[i];
   else if(steps){const double damping=parameters_.mechanics.dampingRate,decay=std::exp(-damping*step),gravityTime=damping>1e-12?(1-decay)/damping:step;velocity[i]=Add(Mul(velocity[i],decay),Mul(localVector(input.gravity),gravityTime/C));x[i]=Add(x[i],Mul(velocity[i],step));}
  }
  for(auto& e:edges_)e.lambda=0;
  for(unsigned iteration=0;iteration<12;iteration++){
   for(auto& e:edges_){auto d=Sub(x[e.a],x[e.b]);double length=Length(d),wa=inverseMass_[e.a],wb=inverseMass_[e.b],alpha=e.compliance*(pouchEdge(e)?1.:8.)/(step*step);if(length<1e-14||wa+wb==0||(e.tether&&length<=e.rest))continue;double dl=(-(length-e.rest)-alpha*e.lambda)/(wa+wb+alpha);e.lambda+=dl;auto delta=Mul(d,dl/length);if(wa)x[e.a]=Add(x[e.a],Mul(delta,wa));if(wb)x[e.b]=Sub(x[e.b],Mul(delta,wb));}
   sew();
   if(iteration%2==1)for(unsigned i=0;i<x.size();i++){project(i,false,false);project(i,true,steps>0);}
  }
  for(unsigned i=0;i<x.size();i++)if(!pinned_[i]&&steps){velocity[i]=Mul(Sub(x[i],old[i]),1/step);}
 // Retain the finite contact pairs found by a complete surface sweep. Local
 // manifold iterations query those actual triangles, never infinite planes.
 struct Pair {unsigned cloth,physical;bool anatomy;Point normal;};
 std::vector<Pair> pairs;std::set<std::uint64_t> pairKeys;
 auto applyFace=[&](unsigned fi,bool anatomy,const detail::BodyCollider::Hit& hit){
  if(hit.signedDistance>=margin-1e-8)return false;
  const auto& face=mesh.triangles[fi];std::array<Point,3> corners{render(face.vertices[0]),render(face.vertices[1]),render(face.vertices[2])};
  auto bary=Barycentric(hit.clothPoint,corners[0],corners[1],corners[2]);
  render_contact::Gradient gradient;
  for(unsigned k=0;k<3;k++)render_contact::Merge(gradient,render_contact::Derivative(renderBindings_[face.vertices[k]],x,materialFrame,C,hit.normal),bary[k]);
  double denominator=gradient.InverseMass(inverseMass_);bool supported=denominator<1e-20;
  if(supported)for(unsigned k=0;k<gradient.count;k++)denominator+=Dot(gradient.values[k],gradient.values[k]);
  if(denominator<1e-20)return false;
  const double depth=margin+5e-5-hit.signedDistance;
  auto application=render_contact::Application(gradient,x,inverseMass_);
  if(anatomy&&!supported&&steps&&input.anatomyMass>0){
   const auto tri=input.anatomyTriangles[hit.triangle];auto weights=anatomyCollider_.ContactWeights(hit);
   auto impulse=worldVector(Mul(hit.normal,depth*C/(step*denominator)));auto particleImpulse=worldVector(Mul(application.impulse,depth*C/(step*denominator)));
   auto moment=Add(worldVector(Mul(application.moment,depth*C*C/(step*denominator))),Cross(origin,particleImpulse));
   reactions.Add(hit.triangle,weights,impulse,world(hit.point),hit.signedDistance*C,input.anatomyTriangles,particleImpulse,moment);
  }
  for(unsigned k=0;k<gradient.count;k++){unsigned i=gradient.nodes[k];double w=supported?1:inverseMass_[i];if(w){auto delta=Mul(gradient.values[k],depth*w/denominator);x[i]=Add(x[i],delta);contactDelta[i]=Add(contactDelta[i],delta);if(anatomy)tissueDelta[i]=Add(tissueDelta[i],delta);if(pinned_[i])fixed[i]=x[i];}}
  return true;
 };
 auto discover=[&](){bool moved=false;
  for(unsigned fi=0;fi<mesh.triangles.size();fi++)for(bool anatomy:{false,true}){
   const auto& face=mesh.triangles[fi];std::array<Point,3> corners{render(face.vertices[0]),render(face.vertices[1]),render(face.vertices[2])};
   auto hit=faceQuery(anatomy,fi,corners,margin+6e-5);if(hit.triangle==UINT32_MAX||hit.signedDistance>=margin+6e-5)continue;
   auto key=(std::uint64_t(anatomy)<<63)|(std::uint64_t(fi)<<32)|hit.triangle;
   if(pairKeys.insert(key).second)pairs.push_back({fi,hit.triangle,anatomy,hit.normal});
   moved=applyFace(fi,anatomy,hit)||moved;
  }return moved;
 };
 auto localContact=[&](){bool moved=false;for(const auto& pair:pairs){
  const auto& f=mesh.triangles[pair.cloth];const auto& surface=pair.anatomy?colliderInput.anatomy:colliderInput.bodySurface;const auto tri=(pair.anatomy?input.anatomyTriangles:input.bodyTriangles)[pair.physical];
  std::array<Point,3> cloth{render(f.vertices[0]),render(f.vertices[1]),render(f.vertices[2])},body{Mul(surface[tri[0]].position,1/C),Mul(surface[tri[1]].position,1/C),Mul(surface[tri[2]].position,1/C)};
  Point a,b;double distance=detail::SurfaceTriangleDistance(cloth,body,a,b);if(distance>margin+6e-5)continue;
  detail::BodyCollider::Hit hit{b,pair.normal,distance,Dot(Sub(a,b),pair.normal),pair.physical,a};moved=applyFace(pair.cloth,pair.anatomy,hit)||moved;
 }return moved;};
 auto beforeFinal=x;

 for(unsigned discovery=0;discovery<8;discovery++){
  bool moved=discover();
  for(unsigned pass=0;pass<24;pass++){finalPasses++;bool stretched=false;for(const auto& edge:edges_)if(!edge.bend&&!edge.tether&&Length(Sub(x[edge.a],x[edge.b]))>edge.rest*materialLimit(edge)){stretched=true;break;}
   if(stretched&&!preparingPlacement_){updateSewing();auto strain=cloth_stretch::ProjectCoupled(x,inverseMass_,strainEdges,stretchAdjacency_,sewing_,seamOffset,1.10);visits+=strain.visits;}else sew();
   bool pointsMoved=false;for(unsigned i=0;i<x.size();i++){pointsMoved=project(i,false,false)||pointsMoved;pointsMoved=project(i,true,steps>0)||pointsMoved;}if(!localContact()&&!pointsMoved)break;
  }
  if(!moved)break;
 }
 // Contact and sewing corrections participate in reconstructed velocity.
 if(steps)for(unsigned i=0;i<x.size();i++)if(!pinned_[i])velocity[i]=Add(velocity[i],Mul(Sub(x[i],beforeFinal[i]),1/step));
 if(steps){
  auto obstacleVelocity=[&](const detail::BodyCollider::Hit& hit,bool anatomy){
   const auto& faces=anatomy?input.anatomyTriangles:input.bodyTriangles;const auto& now=anatomy?input.anatomy:input.bodySurface;const auto& previous=anatomy?previousAnatomySurface_:previousBodySurface_;
   if(previous.size()!=now.size()||hit.triangle>=faces.size())return Point{};
   auto f=faces[hit.triangle];auto bary=(anatomy?anatomyCollider_:bodyCollider_).ContactWeights(hit);Point v{};
   for(unsigned k=0;k<3;k++)v=Add(v,Mul(localVector(Sub(now[f[k]].position,previous[f[k]].position)),bary[k]/(C*steps*step)));return v;
  };
  for(unsigned i=0;i<x.size();i++)if(!pinned_[i])for(bool anatomy:{false,true}){
   auto correction=anatomy?tissueDelta[i]:Sub(contactDelta[i],tissueDelta[i]);if(Length(correction)<1e-14)continue;
   auto hit=pointQuery(anatomy,i,x[i],margin+8e-5);const auto& faces=anatomy?input.anatomyTriangles:input.bodyTriangles;
   if(hit.triangle>=faces.size()||hit.signedDistance>margin+8e-5)continue;
   auto response=cloth_contact::Solve(velocity[i],obstacleVelocity(hit,anatomy),hit.normal,(std::max)(0.,Dot(correction,hit.normal)),inverseMass_[i],parameters_.mechanics.friction,step);
   if(anatomy&&input.anatomyMass>0){auto f=faces[hit.triangle];auto bary=anatomyCollider_.ContactWeights(hit);reactions.Add(hit.triangle,bary,worldVector(Mul(Sub(response.velocity,velocity[i]),C/inverseMass_[i])),world(hit.point),hit.signedDistance*C,input.anatomyTriangles);}
   velocity[i]=response.velocity;
  }
 }
 }
 previousBodySurface_=input.bodySurface;previousAnatomySurface_=input.anatomy;
 accumulator_-=steps*step;clock_+=steps*step;
 t.substeps=steps;t.advancedSeconds=steps*step;t.accumulatedSeconds=clock_;
 t.maxStretchRatio=0;t.maxSeamGap=0;updateSewing();
 bool extensionAccepted=true;
 for(const auto& e:edges_)if(!e.bend&&!e.tether){double ratio=Length(Sub(x[e.a],x[e.b]))/e.rest;if(ratio>t.maxStretchRatio){t.maxStretchRatio=ratio;t.worstStretchA=e.a;t.worstStretchB=e.b;}if(ratio>materialLimit(e))extensionAccepted=false;}
 for(const auto& s:sewing_){auto error=seamOffset(s.residual);for(unsigned k=0;k<s.count;k++)error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));t.maxSeamGap=(std::max)(t.maxSeamGap,Length(error)*C);}

 for(unsigned i=0;i<x.size();i++){positions_[i]=world(x[i]);velocities_[i]=worldVector(Mul(velocity[i],C));t.maxSpeed=(std::max)(t.maxSpeed,Length(velocities_[i]));}
 for(unsigned i=0;i<mesh.vertices.size();i++)mesh.vertices[i].position=world(render(i));
 // Complete rendered-surface contact is independently checked. A fast solve
 // does not turn missed triangle-interior contact into an accepted result.
 auto validationBegan=std::chrono::steady_clock::now();
 output_.coverageMargin=1e100;output_.contactBudgetSatisfied=true;
 double worstGap=0;unsigned worstVertex=UINT32_MAX,worstFace=UINT32_MAX;bool worstAnatomy=false;
 for(unsigned fi=0;fi<mesh.triangles.size();fi++){const auto& face=mesh.triangles[fi];std::array<Point,3> corners;for(unsigned k=0;k<3;k++)corners[k]=local(mesh.vertices[face.vertices[k]].position);for(bool anatomical:{false,true}){auto hit=faceQuery(anatomical,fi,corners,margin);if(hit.signedDistance<margin-1e-8){output_.contactBudgetSatisfied=false;if(hit.signedDistance-margin<worstGap){worstGap=hit.signedDistance-margin;worstFace=fi;worstVertex=UINT32_MAX;worstAnatomy=anatomical;}}if(anatomical)output_.coverageMargin=(std::min)(output_.coverageMargin,(hit.signedDistance-margin)*C);}}
 for(unsigned i=0;i<mesh.vertices.size();i++)for(bool anatomical:{false,true}){auto hit=pointQuery(anatomical,pointSlot(i),local(mesh.vertices[i].position),margin);if(hit.signedDistance<margin-1e-8){output_.contactBudgetSatisfied=false;if(hit.signedDistance-margin<worstGap){worstGap=hit.signedDistance-margin;worstVertex=i;worstFace=UINT32_MAX;worstAnatomy=anatomical;}}}

 if(output_.layout.revision==2)for(unsigned fi=output_.layout.sheetFaces.start;fi<output_.layout.sheetFaces.start+output_.layout.sheetFaces.count;fi++){const auto& f=mesh.triangles[fi];for(unsigned k=0;k<3;k++){unsigned a=f.vertices[k],b=f.vertices[(k+1)%3];double rest=Length(Sub(restMesh_.vertices[a].position,restMesh_.vertices[b].position));if(rest>C*1e-12)t.maxRenderStretchRatio=(std::max)(t.maxRenderStretchRatio,Length(Sub(mesh.vertices[a].position,mesh.vertices[b].position))/rest);}}
 t.materialBudgetSatisfied=t.maxRenderStretchRatio<=1.15&&extensionAccepted&&t.maxSeamGap<=C*parameters_.bandThickness*.25;
 // Complete one-time material authoring only after the placed garment is
 // outside both measured surfaces and its actual rendered stitches agree.
 // This never runs during ordinary motion; only PlacePrepared enables it.
 if(preparingPlacement_&&output_.contactBudgetSatisfied&&t.maxSeamGap<=C*parameters_.bandThickness*.25){
  for(auto& e:edges_){double length=Length(Sub(x[e.a],x[e.b]));e.rest=e.tether?(std::max)(e.rest,length*1.02):length;e.lambda=0;}
  for(auto& s:sewing_){Point separation{};for(auto term:s.vertices)separation=Add(separation,Mul(render(term.first),term.second));s.separation=Mul(separation,C);}
  updateSewing();restMesh_=mesh;std::fill(velocities_.begin(),velocities_.end(),Point{});clock_=accumulator_=0;preparingPlacement_=false;
  t.maxStretchRatio=t.maxRenderStretchRatio=1;t.maxSeamGap=t.maxSpeed=0;t.advancedSeconds=t.accumulatedSeconds=0;t.substeps=0;t.materialBudgetSatisfied=true;
  output_.reactions.clear();output_.support.clear();output_.reaction={};
 }else if(input.anatomyMass>0)reactions.Publish(output_,steps*step,clothMass_);
 detail::Shading(mesh);
#ifdef MALEMOD_SUPPORTED_PROFILE
 if(!t.materialBudgetSatisfied){for(auto node:{t.worstStretchA,t.worstStretchB}){unsigned v=0;while(v<vertexNodes_.size()&&vertexNodes_[v]!=node)v++;std::fprintf(stderr,"MATERIAL node %u vertex %u pinned %u pouch %u\n",node,v,unsigned(pinned_[node]),unsigned(pouchNode[node]));}}
 if(!output_.contactBudgetSatisfied)std::fprintf(stderr,"CONTACT FAIL gap %.9f vertex %u face %u anatomy %u\n",worstGap*C,worstVertex,worstFace,unsigned(worstAnatomy));
 if(true)std::fprintf(stderr,"PROFILE clock %.4f step %u physical %.3f classification %.3f pre %.3f final %.3f verify %.3f passes %u visits %u\n",clock_,steps,std::chrono::duration<double,std::milli>(physicalUpdated-began).count(),std::chrono::duration<double,std::milli>(classificationUpdated-physicalUpdated).count(),std::chrono::duration<double,std::milli>(faceBegan-classificationUpdated).count(),std::chrono::duration<double,std::milli>(validationBegan-faceBegan).count(),std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-validationBegan).count(),finalPasses,visits);
#endif
 t.solverMilliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
}
}
