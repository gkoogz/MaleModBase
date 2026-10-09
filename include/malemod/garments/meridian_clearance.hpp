#pragma once
#include "meridian_runtime.hpp"
#include <algorithm>
#include <functional>
#include <limits>
#include <string>
namespace malemod::garments::meridian {
// A plane bounds a convex solid: dot(normal, point) <= offset is inside.
struct Plane { Vec normal; float offset; };
inline float Signed(const Plane& p,Vec v){return Dot(p.normal,v)-p.offset;}
struct Hull:std::vector<Plane> {
 using std::vector<Plane>::vector;
 // Exact support of the current convex primitive, in the same coordinate
 // frame as the cloth. Plane samples alone miss valid separating directions.
 std::function<float(Vec)> support;
};
inline bool TriangleSupportPlane(const Hull& hull,Vec a,Vec b,Vec c,Plane& plane){
 if(!hull.support)return false;
 Vec n=Cross(Sub(b,a),Sub(c,a));float squared=Dot(n,n);
 if(!std::isfinite(squared)||squared<1e-14f)return false;
 n=Mul(n,1/std::sqrt(squared));float best=-std::numeric_limits<float>::infinity();
 for(float sign:{1.f,-1.f}){
  Vec direction=Mul(n,sign);float d=hull.support(direction);
  if(!std::isfinite(d))throw std::runtime_error("Nonfinite exact collision support");
  float gap=(std::min)({Dot(direction,a),Dot(direction,b),Dot(direction,c)})-d;
  if(gap>best){best=gap;plane={direction,d};}
 }return true;
}
inline bool ExactTriangleSeparated(const Hull& hull,Vec a,Vec b,Vec c,float tolerance=1e-5f){
 Plane plane{};return TriangleSupportPlane(hull,a,b,c,plane)&&
  (std::min)({Signed(plane,a),Signed(plane,b),Signed(plane,c)})>=-tolerance;
}
// Taut fabric bridges between neighboring solids; separate radial exits can
// select incompatible sides of the two lobes. A common convex support envelope
// supplies coherent contact directions across all longitudes.
inline Hull ConvexCover(const std::vector<Hull>& hulls,Vec axis,float padding=.04f){
 if(hulls.empty()||padding<0)throw std::runtime_error("Invalid convex cloth cover");
 std::vector<std::function<float(Vec)>> supports;
 for(const auto& hull:hulls){if(!hull.support)throw std::runtime_error("Convex cover requires exact primitive supports");supports.push_back(hull.support);}
 Hull cover;cover.support=[supports,padding](Vec n){float d=-std::numeric_limits<float>::infinity();for(const auto& support:supports)d=(std::max)(d,support(n));return d+padding;};
 for(unsigned k=0;k<130;k++){
  Vec n;
  if(k<2)n=Mul(Unit(axis),k?-1.f:1.f);
  else{float z=1-2*((k-2)+.5f)/128,a=(k-2)*2.39996323f,r=std::sqrt((std::max)(0.f,1-z*z));n={r*std::cos(a),r*std::sin(a),z};}
  cover.push_back({n,cover.support(n)});
 }return cover;
}
struct WrapReceipt { unsigned iterations=0; float maximumDisplacement=0, minimumSeparation=0; };
inline Vec AttachmentCentroid(const std::vector<Vec>& points,unsigned columns){
 if(columns<3||columns>points.size())throw std::runtime_error("Invalid attachment outline");
 Vec center{};float total=0;
 for(unsigned i=0;i<columns;i++){Vec a=points[i],b=points[(i+1)%columns],d=Sub(b,a);float length=std::sqrt(Dot(d,d));center=Add(center,Mul(Add(a,b),length*.5f));total+=length;}
 if(!std::isfinite(total)||total<1e-7f)throw std::runtime_error("Collapsed attachment outline");return Mul(center,1/total);
}
inline Vec PreparePole(std::vector<Vec>& points,unsigned poleIndex,unsigned collisionFirst,unsigned collisionEnd,Vec axisAnchor,float thickness=.12f){
 if(poleIndex>=points.size()||collisionFirst>=collisionEnd||collisionEnd>points.size()||!std::isfinite(thickness)||thickness<0)throw std::runtime_error("Invalid pole support inputs");
 Vec axis=Unit(Sub(points[poleIndex],axisAnchor));float advance=0;
 for(unsigned i=collisionFirst;i<collisionEnd;i++)advance=(std::max)(advance,Dot(Sub(points[i],points[poleIndex]),axis));
 points[poleIndex]=Add(points[poleIndex],Mul(axis,advance+thickness));return axis;
}
// A deeply folded tip cannot define a stable polar chart for a sewn outline.
// Use the current boundary's area normal, oriented toward the contents, then
// put only the cloth pole beyond the current complete collision support.
inline Vec PrepareEnvelopePole(std::vector<Vec>& points,unsigned columns,unsigned poleIndex,unsigned collisionFirst,unsigned collisionEnd,float thickness=.12f){
 if(columns<3||columns>points.size()||poleIndex>=points.size()||collisionFirst>=collisionEnd||collisionEnd>points.size()||thickness<0)throw std::runtime_error("Invalid envelope pole inputs");
 Vec center=AttachmentCentroid(points,columns),area{};
 for(unsigned i=0;i<columns;i++)area=Add(area,Cross(Sub(points[i],center),Sub(points[(i+1)%columns],center)));
 Vec axis=Unit(area);float orientation=0,reach=0;
 for(unsigned i=collisionFirst;i<collisionEnd;i++)orientation+=Dot(Sub(points[i],center),axis);
 if(orientation<0)axis=Mul(axis,-1);
 for(unsigned i=collisionFirst;i<collisionEnd;i++)reach=(std::max)(reach,Dot(Sub(points[i],center),axis));
 points[poleIndex]=Add(center,Mul(axis,reach+thickness));return axis;
}
// An invertible shear preserves convexity and separating planes while keeping
// the polar chart centered over the sewn outline. The render pole retains the
// current tip's transverse coordinates instead of flaring at boundary height.
struct EnvelopeChart {
 Vec center{},axis{},shift{};float height=1;
 Vec Forward(Vec p)const{return Sub(p,Mul(shift,Dot(Sub(p,center),axis)/height));}
 Vec Inverse(Vec p)const{return Add(p,Mul(shift,Dot(Sub(p,center),axis)/height));}
 Hull Transform(const Hull& original)const{
  Hull hull;Vec translation=Mul(shift,Dot(center,axis)/height);
  for(auto plane:original){Vec n=Add(plane.normal,Mul(axis,Dot(shift,plane.normal)/height));float length=std::sqrt(Dot(n,n));n=Mul(n,1/length);hull.push_back({n,plane.offset/length+Dot(n,translation)});}
  if(original.support){auto support=original.support;auto a=axis,s=shift;auto h=height;
   hull.support=[support,a,s,h,translation](Vec n){Vec q=Sub(n,Mul(a,Dot(s,n)/h));float length=std::sqrt(Dot(q,q));return length*support(Mul(q,1/length))+Dot(n,translation);};
  }return hull;
 }
};
inline EnvelopeChart PrepareAnchoredEnvelope(std::vector<Vec>& points,unsigned columns,unsigned poleIndex,unsigned collisionFirst,unsigned collisionEnd,Vec tip,float thickness=.12f){
 Vec axis=PrepareEnvelopePole(points,columns,poleIndex,collisionFirst,collisionEnd,thickness),center=AttachmentCentroid(points,columns);
 Vec offset=Sub(tip,center),shift=Sub(offset,Mul(axis,Dot(offset,axis)));float height=Dot(Sub(points[poleIndex],center),axis);
 if(!std::isfinite(height)||height<1e-6f)throw std::runtime_error("Collapsed envelope chart height");
 for(float value:shift)if(!std::isfinite(value))throw std::runtime_error("Nonfinite envelope chart shift");
 points[poleIndex]=Add(points[poleIndex],shift);return {center,axis,shift,height};
}
inline bool WithinMeridianSampling(const std::vector<Vec>& solved,const std::vector<Vec>& taut,unsigned columns,unsigned rows,float margin=.04f){
 unsigned count=columns*rows+1;if(columns<3||rows<2||count>solved.size()||count>taut.size())throw std::runtime_error("Invalid sampling budget");
 for(unsigned col=0;col<columns;col++){
  float average=0;for(unsigned row=1;row<=rows;row++){unsigned i=row==rows?count-1:row*columns+col;Vec d=Sub(taut[i],taut[(row-1)*columns+col]);average+=std::sqrt(Dot(d,d))/rows;}
  for(unsigned row=1;row<rows;row++){unsigned i=row*columns+col,j=row+1==rows?count-1:i+columns;Vec a=Sub(taut[i],taut[i-columns]),b=Sub(taut[i],taut[j]);float limit=(std::max)({2*margin,.75f*average,.5f*std::sqrt((std::min)(Dot(a,a),Dot(b,b)))});Vec d=Sub(solved[i],taut[i]);if(!std::isfinite(Dot(d,d))||Dot(d,d)>limit*limit)return false;}
 }return true;
}
inline void SeedMeridians(std::vector<Vec>& points,unsigned columns,unsigned rows,const float* heightFractions,Vec axis){
 if(!heightFractions||columns<3||rows<2||std::size_t(columns)*rows>=points.size())throw std::runtime_error("Invalid live meridian seed");
 Vec pole=points[columns*rows];axis=Unit(axis);
 for(unsigned col=0;col<columns;col++){
  Vec q=Sub(points[col],pole);float height=Dot(q,axis);Vec radial=Sub(q,Mul(axis,height));
  for(unsigned row=1;row<rows;row++){float fraction=(std::max)(.0001f,heightFractions[row*columns+col]);points[row*columns+col]=Add(pole,Mul(Add(Mul(axis,height),radial),fraction));}
 }
}
// Remove isolated reversals left by independent triangle depenetration. Each
// accepted move shortens its longitude and retains a separating plane for
// every incident triangle and solid. Never smooth through a collision guide.
inline unsigned FairMeridianReversals(std::vector<Vec>& points,unsigned columns,unsigned rows,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,unsigned sweeps=8){
 if(columns<3||rows<3||points.size()<=std::size_t(columns)*rows||(!faces&&faceCount))throw std::runtime_error("Invalid meridian fairing inputs");
 if(!faceCount)return 0; // Without triangles there is no collision certificate.
 const unsigned pole=columns*rows;std::vector<std::vector<unsigned>> incident(pole+1);
 for(unsigned f=0;f<faceCount;f++)for(unsigned id:faces[f]){if(id>pole)throw std::runtime_error("Invalid fairing face");incident[id].push_back(f);}
 unsigned moves=0;
 for(unsigned sweep=0;sweep<sweeps;sweep++){
  unsigned changed=0;
  for(unsigned row=1;row<rows;row++)for(unsigned col=0;col<columns;col++)for(unsigned transverse=0;transverse<2;transverse++){
   unsigned id=row*columns+col,next=row+1==rows?pole:id+columns;
   unsigned previous=id-columns;
   if(transverse){previous=row*columns+(col+columns-1)%columns;next=row*columns+(col+1)%columns;}
   Vec a=Sub(points[id],points[previous]),b=Sub(points[id],points[next]);
   // An ordinary rounded contact has opposing edges. An acute return along
   // a path is a spike, regardless of source-unit scale.
   if(Dot(a,b)<=0)continue;
   Vec original=points[id],target=Mul(Add(points[previous],points[next]),.5f);
   if(transverse){Vec axis=Unit(Sub(points[pole],AttachmentCentroid(points,columns))),q=Sub(points[id],points[pole]);
    Vec radial=Sub(q,Mul(axis,Dot(q,axis)));float length=std::sqrt(Dot(radial,radial));
    if(length<1e-7f)continue;radial=Mul(radial,1/length);q=Sub(target,points[pole]);
    target=Add(points[pole],Add(Mul(axis,Dot(q,axis)),Mul(radial,Dot(q,radial))));
   }
   for(float fraction:{1.f,.5f,.25f,.125f,.0625f}){
    Vec candidate=Add(original,Mul(Sub(target,original),fraction));bool clear=true;
    for(unsigned f:incident[id]){
     auto ids=faces[f];Vec tri[3],old[3];for(unsigned k=0;k<3;k++){old[k]=points[ids[k]];tri[k]=ids[k]==id?candidate:old[k];}
     Vec normal=Cross(Sub(tri[1],tri[0]),Sub(tri[2],tri[0]));
     if(Dot(normal,normal)<1e-14f||Dot(normal,Cross(Sub(old[1],old[0]),Sub(old[2],old[0])))<=0){clear=false;break;}
     for(const auto& hull:hulls){bool separated=false;for(const auto& plane:hull)if(Signed(plane,tri[0])>=-1e-5f&&Signed(plane,tri[1])>=-1e-5f&&Signed(plane,tri[2])>=-1e-5f){separated=true;break;}if(!separated)separated=ExactTriangleSeparated(hull,tri[0],tri[1],tri[2]);if(!separated){clear=false;break;}}
     if(!clear)break;
    }
    if(clear){points[id]=candidate;++changed;break;}
   }
  }
  moves+=changed;if(!changed)break;
 }
 return moves;
}
inline void TautenCorrectedMeridians(std::vector<Vec>& points,unsigned columns,unsigned rows,Vec axis){
 const Vec pole=points[columns*rows];
 struct Station {float h,r;unsigned id;};
 for(unsigned col=0;col<columns;col++){
  Vec anchor=Sub(points[col],pole);float end=Dot(anchor,axis);if(std::abs(end)<1e-6f)continue;
  Vec radial=Unit(Sub(anchor,Mul(axis,end)));std::vector<Station> nodes;nodes.reserve(rows+1);
  for(unsigned row=0;row<=rows;row++){unsigned id=row==rows?columns*rows:row*columns+col;Vec d=Sub(points[id],pole);nodes.push_back({Dot(d,axis)/end,Dot(d,radial),id});}
  bool monotone=true;for(unsigned row=1;row<=rows;row++)if(nodes[row].h>=nodes[row-1].h)monotone=false;if(!monotone)continue;
  std::vector<Station> hull;
  for(int row=int(rows);row>=0;row--){auto n=nodes[row];while(hull.size()>1){auto a=hull[hull.size()-2],b=hull.back();if((b.h-a.h)*(n.r-a.r)-(b.r-a.r)*(n.h-a.h)<0)break;hull.pop_back();}hull.push_back(n);}
  for(unsigned row=1;row<rows;row++){
   auto n=nodes[row];unsigned k=1;while(k+1<hull.size()&&hull[k].h<n.h)++k;auto a=hull[k-1],b=hull[k];
   float radius=a.r+(b.r-a.r)*(n.h-a.h)/(b.h-a.h);
   if(radius>n.r)points[n.id]=Add(points[n.id],Mul(radial,radius-n.r));
  }
 }
}
// Kinematic trim fitting, separate from the sewn cloth solve. The adapter must
// apply these same deltas to the hem ribbon, keeping cloth and trim coincident.
inline std::vector<Vec> FitSeam(std::vector<Vec>& points,unsigned columns,Vec pole,Vec axis,const std::vector<Hull>& hulls,float clearance=.12f,float limit=2.f){
 if(columns<3||columns>points.size()||!std::isfinite(clearance)||!std::isfinite(limit)||clearance<0||limit<=0)throw std::runtime_error("Invalid sewn trim inputs");
 axis=Unit(axis);std::vector<Vec> deltas(columns);const auto originalPoints=points;std::vector<Vec> directions(columns);
 // Directions and support normals are invariant during this fit. Lazily
 // compute each vertex/plane route once, even when twelve edge passes need it.
 // This cache is local to this pose and preserves the original arithmetic.
 struct Route {Vec move{};float slope=0,root=0;bool ready=false;};
 std::vector<unsigned> offsets;unsigned planes=0;
 for(const auto& hull:hulls){offsets.push_back(planes);planes+=unsigned(hull.size());}
 std::vector<Route> routes(std::size_t(columns)*planes);
 auto routeFor=[&](unsigned id,const Hull& hull,const Plane& plane)->Route&{
  auto& route=routes[std::size_t(id)*planes+offsets[&hull-hulls.data()]+unsigned(&plane-hull.data())];
  if(!route.ready){route.move=Add(Mul(directions[id],(std::max)(0.f,Dot(plane.normal,directions[id]))),Mul(axis,Dot(plane.normal,axis)));route.slope=Dot(plane.normal,route.move);route.root=std::sqrt(route.slope);route.ready=true;}
  return route;
 };
 for(unsigned i=0;i<columns;i++){
  const Vec original=points[i];Vec q=Sub(original,pole);Vec direction=Unit(Sub(q,Mul(axis,Dot(q,axis))));directions[i]=direction;
  for(const Hull& hull:hulls){
   bool outside=false;float best=std::numeric_limits<float>::infinity();Vec correction{};
   for(const Plane& plane:hull){
    float value=Signed(plane,points[i]);if(value>=clearance){outside=true;break;}
    const auto& route=routeFor(i,hull,plane);Vec move=route.move;
    float slope=route.slope;if(slope<1e-8f)continue;
    Vec delta=Mul(move,(clearance-value)/slope);float cost=Dot(delta,delta);
    if(cost<best){best=cost;correction=delta;}
   }
   if(!outside){if(!std::isfinite(best))throw std::runtime_error("No local seam exit");points[i]=Add(points[i],correction);}
  }

 }
 // The sewn edge itself must clear each solid; individually clear endpoints
 // can still make a chord through the root collider.
 for(unsigned pass=0;pass<12;pass++){
  std::vector<Vec> corrections(columns);std::vector<unsigned> counts(columns,0);bool clear=true;
  for(unsigned i=0;i<columns;i++)for(const Hull& hull:hulls){
   unsigned j=(i+1)%columns;float best=std::numeric_limits<float>::infinity();const Plane* chosen=nullptr;bool separated=false;
   for(const Plane& plane:hull){float x=Signed(plane,points[i]),y=Signed(plane,points[j]);
    if(x>=clearance*.5f&&y>=clearance*.5f){separated=true;break;}
    float a=routeFor(i,hull,plane).root,b=routeFor(j,hull,plane).root;
    if((x<clearance&&a<=1e-5f)||(y<clearance&&b<=1e-5f))continue;
    float cost=(std::max)(x<clearance?(clearance-x)/a:0.f,y<clearance?(clearance-y)/b:0.f);
    if(cost<best){best=cost;chosen=&plane;}
   }
   if(separated)continue;if(!chosen)throw std::runtime_error("No outward route for sewn edge");clear=false;
   for(unsigned id:{i,j}){
    float value=Signed(*chosen,points[id]);if(value<clearance){
     const auto& cached=routeFor(id,hull,*chosen);Vec route=cached.move;
     // Accumulate complete vector corrections; scalar radial-only corrections
     // can drag a sewn edge far outward when a nearby cap exit is available.
     float slope=cached.slope;Vec delta=Mul(route,(clearance-value)/slope);
     corrections[id]=Add(corrections[id],delta);counts[id]++;
    }
   }
  }
  if(clear){for(unsigned i=0;i<columns;i++){deltas[i]=Sub(points[i],originalPoints[i]);if(Dot(deltas[i],deltas[i])>limit*limit)throw std::runtime_error("Seam fitting exceeds trim allowance: vertex="+std::to_string(i)+" distance="+std::to_string(std::sqrt(Dot(deltas[i],deltas[i]))));}return deltas;}
  for(unsigned i=0;i<columns;i++)if(counts[i])points[i]=Add(points[i],Mul(corrections[i],1.f/counts[i]));
 }
 throw std::runtime_error("Sewn edge clearance did not converge");
}
// Conservative support polytope. The caller supplies current, coherent-space
// collision samples and outward normals; every sample is enclosed by every plane.
inline Hull SupportHull(const Vec* points,unsigned count,const Vec* normals,unsigned planeCount,float padding){
 if(!count||planeCount<4||padding<0)throw std::runtime_error("Invalid collision hull");
 Hull result;result.reserve(planeCount);
 for(unsigned k=0;k<planeCount;k++){
  Vec n=Unit(normals[k]);float d=-std::numeric_limits<float>::infinity();
  for(unsigned i=0;i<count;i++){float t=Dot(n,points[i]);if(!std::isfinite(t))throw std::runtime_error("Nonfinite collider");d=(std::max)(d,t);}
  result.push_back({n,d+padding});
 }return result;
}
// Keep the sewn boundary and pole exact. Interior motion is restricted to each
// ordered longitude half-plane. An entire triangle must share a separating
// support plane for EVERY solid; testing vertices alone misses edge penetration.
// Work is transactional and bounded. A failed pose must not hide the anatomy.
inline WrapReceipt ClearMeridians(std::vector<Vec>& points,unsigned columns,unsigned rows,
 const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,Vec axis,float margin=.04f,unsigned budget=12,std::vector<unsigned>* certificates=nullptr,unsigned detailedHullFirst=0,bool retainTautSeed=false){
 const unsigned count=columns*rows+1,poleIndex=count-1;
 if(columns<3||rows<2||points.size()<count||margin<0||detailedHullFirst>hulls.size())throw std::runtime_error("Invalid meridian grid");
 axis=Unit(axis);const Vec pole=points[poleIndex];auto work=points;
 std::vector<Vec> directions(count);std::vector<float> increments(count);
 std::vector<unsigned char> fixed(count);for(unsigned i=0;i<columns;i++)fixed[i]=1;fixed[poleIndex]=1;
 std::vector<float> angles(columns);Vec first{},second{};
 for(unsigned col=0;col<columns;col++){
  Vec q=Sub(work[col],pole);Vec d=Unit(Sub(q,Mul(axis,Dot(q,axis))));
  if(!col){first=d;second=Cross(axis,first);}
  float angle=std::atan2(Dot(d,second),Dot(d,first));
  if(col){while(angle-angles[col-1]<=-3.14159265f)angle+=6.28318531f;while(angle-angles[col-1]>3.14159265f)angle-=6.28318531f;}angles[col]=angle;
 }
 float orientation=angles.back()>angles.front()?1.f:-1.f;
 struct Block {unsigned first,last;float sum;};std::vector<Block> blocks;
 for(unsigned col=0;col<columns;col++){
  blocks.push_back({col,col+1,angles[col]*orientation-col*.001f});
  while(blocks.size()>1){auto r=blocks.back(),l=blocks[blocks.size()-2];if(l.sum/(l.last-l.first)<=r.sum/(r.last-r.first))break;blocks.pop_back();blocks.back()={l.first,r.last,l.sum+r.sum};}
 }
 auto repaired=angles;for(auto block:blocks)for(unsigned col=block.first;col<block.last;col++)repaired[col]=(block.sum/(block.last-block.first)+col*.001f)*orientation;
 // Longitude is periodic. A tiny reversal across the closing seam must be
 // repaired like an interior reversal, rather than mistaken for two turns.
 for(unsigned col=0;col<columns;col++)repaired[col]=orientation*(std::max)(col*.001f,(std::min)(6.28318531f-(columns-col)*.001f,repaired[col]*orientation));
 for(unsigned col=0;col<columns;col++){
  if(std::abs(repaired[col]-angles[col])>.25f)throw std::runtime_error("Live seam order exceeds local repair allowance");
  Vec direction=Add(Mul(first,std::cos(repaired[col])),Mul(second,std::sin(repaired[col])));
  for(unsigned row=0;row<rows;row++){
   unsigned i=row*columns+col;directions[i]=direction;if(!row)continue;
   Vec q=Sub(work[i],pole);float h=Dot(q,axis);Vec radial=Sub(q,Mul(axis,h));float radius=std::sqrt(Dot(radial,radial));
   work[i]=Add(pole,Add(Mul(axis,h),Mul(direction,radius)));
  }
 }
 directions[poleIndex]=axis;
 // A caller may already have walked a common support envelope. Correct its
 // tessellated triangles without redistributing that established surface.
 if(!retainTautSeed){
 // Exit the complete intersection interval, including an obstacle on the far
 // side of the polar axis. A parallel miss is not an infinite exit distance.
 struct RayPlane {float intercept,slope,constant,axial;int sign;};
 std::vector<RayPlane> rayPlanes;
 // Virtual walk points refine the collision silhouette without adding render
 // vertices. They stay in each ordered longitude plane and are rebuilt from
 // the current pose, including both moving lobes.
 struct Node {float h,r;unsigned id;};
 constexpr unsigned walkSteps=48;
 std::vector<std::vector<Node>> walkNodes(columns);
 for(unsigned col=0;col<columns;col++){
  Vec q=Sub(work[col],pole),direction=directions[columns+col];
  float height=Dot(q,axis),radius=Dot(q,direction);
  for(unsigned step=1;step<walkSteps;step++){
   float f=.5f+.5f*std::cos(3.14159265f*step/walkSteps);
   walkNodes[col].push_back({height*f,radius*f,rows+1});
  }
 }
 for(unsigned col=0;col<columns;col++)for(const Hull& hull:hulls){
  rayPlanes.clear();rayPlanes.reserve(hull.size());Vec direction=directions[columns+col];
  for(const Plane& plane:hull){float rate=Dot(plane.normal,direction),axial=Dot(plane.normal,axis),constant=Signed(plane,pole);
   if(std::abs(rate)<1e-7f)rayPlanes.push_back({0,0,constant,axial,0});
   else rayPlanes.push_back({-constant/rate,-axial/rate,0,0,rate>0?1:-1});
  }
  for(unsigned row=1;row<rows;row++){
   unsigned i=row*columns+col;Vec q=Sub(work[i],pole);float height=Dot(q,axis),radius=Dot(q,direction);
   float low=-std::numeric_limits<float>::infinity(),high=std::numeric_limits<float>::infinity();bool hit=true;
   for(const RayPlane& plane:rayPlanes){
    if(!plane.sign){if(plane.constant+plane.axial*height>0){hit=false;break;}}
    else {float distance=plane.intercept+plane.slope*height;if(plane.sign>0)high=(std::min)(high,distance);else low=(std::max)(low,distance);}
   }
   if(hit&&low<=high&&high>radius&&std::isfinite(high))work[i]=Add(work[i],Mul(direction,high-radius+margin));
  }
  // Adapters identify the curved solids needing refinement. All solids still
  // participate in the original walk and the final whole-triangle certificate.
  if(unsigned(&hull-hulls.data())>=detailedHullFirst)for(auto& node:walkNodes[col]){
   float low=-std::numeric_limits<float>::infinity(),high=std::numeric_limits<float>::infinity();bool hit=true;
   for(const RayPlane& plane:rayPlanes){
    if(!plane.sign){if(plane.constant+plane.axial*node.h>0){hit=false;break;}}
    else{float distance=plane.intercept+plane.slope*node.h;if(plane.sign>0)high=(std::min)(high,distance);else low=(std::max)(low,distance);}
   }
   if(hit&&low<=high&&high>node.r&&std::isfinite(high))node.r=high+margin;
  }
 }
 // Pull each provisional meridian taut on its outward convex envelope.
 // Full triangle clearance below is still authoritative after resampling.
 std::vector<std::vector<Node>> tautPaths(columns);
 for(unsigned col=0;col<columns;col++){
  std::vector<Node> nodes=std::move(walkNodes[col]);Vec direction=directions[columns+col];
  // The true attachment is the start; virtual nodes have no anchor identity.
  Vec anchor=Sub(work[col],pole);Node start{Dot(anchor,axis),Dot(anchor,direction),0};
  for(unsigned row=0;row<rows;row++){Vec d=Sub(work[row*columns+col],pole);nodes.push_back({Dot(d,axis),Dot(d,direction),row});}
  nodes.push_back({0,0,rows});
  auto turn=[](Node a,Node b,Node c){return (b.h-a.h)*(c.r-a.r)-(b.r-a.r)*(c.h-a.h);};
  std::sort(nodes.begin(),nodes.end(),[](Node a,Node b){return a.h<b.h||(a.h==b.h&&a.r<b.r);});
  std::vector<Node> hull;
  for(auto n:nodes){while(hull.size()>1&&turn(hull[hull.size()-2],hull.back(),n)<=0)hull.pop_back();hull.push_back(n);}
  auto lower=hull.size();for(int i=int(nodes.size())-2;i>=0;i--){auto n=nodes[i];while(hull.size()>lower&&turn(hull[hull.size()-2],hull.back(),n)<=0)hull.pop_back();hull.push_back(n);}hull.pop_back();
  int firstIndex=-1,lastIndex=-1;for(unsigned i=0;i<hull.size();i++){if(hull[i].id==0)firstIndex=int(i);if(hull[i].id==rows)lastIndex=int(i);}
  // A deeply folded anchor needs a union visibility solve; leave its safe
  // provisional path for the certificate step rather than moving the seam.
  if(firstIndex<0||lastIndex<0)continue;
  std::vector<Node> paths[2];float scores[2]{};
  for(unsigned route=0;route<2;route++){
   int index=firstIndex,step=route? -1:1;
   for(unsigned i=0;i<=hull.size();i++){Node n=hull[index];paths[route].push_back(n);scores[route]+=(-start.h)*(n.r-start.r)+start.r*(n.h-start.h);if(index==lastIndex)break;index=(index+step+int(hull.size()))%int(hull.size());}
  }
  auto& path=paths[scores[1]>scores[0]?1:0];tautPaths[col]=path;std::vector<float> lengths(path.size(),0);
  for(unsigned i=1;i<path.size();i++){float dh=path[i].h-path[i-1].h,dr=path[i].r-path[i-1].r;lengths[i]=lengths[i-1]+std::sqrt(dh*dh+dr*dr);}
  // Spend more of the existing surface budget around bends. Spread each
  // turn's weight across its neighboring segments for continuous placement;
  // straight spans retain a positive arc-length density. No origin moves.
  std::vector<float> weights=lengths;
  for(unsigned i=1;i+1<path.size();i++){
   float ah=path[i].h-path[i-1].h,ar=path[i].r-path[i-1].r,bh=path[i+1].h-path[i].h,br=path[i+1].r-path[i].r;
   float turnAngle=std::atan2(std::abs(ah*br-ar*bh),ah*bh+ar*br);
   float extra=lengths.back()*.12f*turnAngle;
   for(unsigned j=i;j<weights.size();j++)weights[j]+=extra*.5f;
   for(unsigned j=i+1;j<weights.size();j++)weights[j]+=extra*.5f;
  }
  for(unsigned row=1;row<rows;row++){
   float distance=(.5f-.5f*std::cos(3.14159265f*row/rows))*weights.back();unsigned i=1;while(i+1<path.size()&&weights[i]<distance)++i;
   float t=(distance-weights[i-1])/(std::max)(1e-9f,weights[i]-weights[i-1]);float h=path[i-1].h+(path[i].h-path[i-1].h)*t,r=path[i-1].r+(path[i].r-path[i-1].r)*t;
   work[row*columns+col]=Add(pole,Add(Mul(axis,h),Mul(direction,r)));
  }
 }
 // Independent arc-length sampling puts neighboring rows on opposite sides
 // of a contact bend. Share the curvature-derived axial fractions across
 // longitudes, then evaluate each original taut envelope at that station.
 // Origins, longitude planes, anchors and polygon count stay unchanged.
 std::vector<float> stations(rows,0.f);unsigned stationCount=0;
 for(unsigned col=0;col<columns;col++){
  const auto& path=tautPaths[col];if(path.empty())continue;if(std::abs(path.front().h)<1e-6f){tautPaths[col].clear();continue;}
  bool monotone=true;for(unsigned i=1;i<path.size();i++)if((path[i].h-path[i-1].h)*path.front().h>1e-6f)monotone=false;
  if(!monotone){tautPaths[col].clear();continue;}
  ++stationCount;for(unsigned row=1;row<rows;row++)stations[row]+=Dot(Sub(work[row*columns+col],pole),axis)/path.front().h;
 }
 if(stationCount)for(unsigned row=1;row<rows;row++)stations[row]/=stationCount;
 for(unsigned col=0;col<columns;col++){
  const auto& path=tautPaths[col];if(path.empty())continue;Vec direction=directions[columns+col];
  for(unsigned row=1;row<rows;row++){
   float h=path.front().h*stations[row];unsigned i=1;while(i+1<path.size()&&(h-path[i].h)*path.front().h<0)++i;
   float t=(h-path[i-1].h)/(path[i].h-path[i-1].h),r=path[i-1].r+(path[i].r-path[i-1].r)*t;
   work[row*columns+col]=Add(pole,Add(Mul(axis,h),Mul(direction,r)));
  }
 }
 // A contact certificate may refine a taut path, but must not reshape it into
 // a distant spike. Bound correction by its local sampling spacing, measured
 // before correction. Failed solves remain transactional for the caller.
 }
 const auto taut=work;std::vector<float> correctionLimits(count,0.f);
 std::vector<float> averageSpacing(columns,0.f);
 for(unsigned col=0;col<columns;col++)for(unsigned row=1;row<=rows;row++){
  unsigned id=row==rows?poleIndex:row*columns+col,previous=(row-1)*columns+col;
  Vec d=Sub(taut[id],taut[previous]);averageSpacing[col]+=std::sqrt(Dot(d,d))/rows;
 }
 for(unsigned row=1;row<rows;row++)for(unsigned col=0;col<columns;col++){
  unsigned id=row*columns+col,next=row+1==rows?poleIndex:id+columns;
  Vec a=Sub(taut[id],taut[id-columns]),b=Sub(taut[id],taut[next]);
  correctionLimits[id]=(std::max)({margin*2.f,.75f*averageSpacing[col],.5f*std::sqrt((std::min)(Dot(a,a),Dot(b,b)))});
 }
 // Near the sewn edge and terminal dome, allow axial relief within the same
 // longitude plane. Purely radial corrections cannot clear a cap plane, even
 // when a valid taut route exists. The radial component always stays outward.
 for(unsigned row=1;row<rows;row++){
  float t=float(row)/(rows-1),tilt=t<.15f? -(1-t/.15f):(t>.85f?(t-.85f)/.15f:0.f);
  for(unsigned col=0;col<columns;col++){unsigned i=row*columns+col;directions[i]=Unit(Add(directions[i],Mul(axis,tilt)));}
 }
 WrapReceipt receipt;
 if(certificates&&certificates->size()!=faceCount*hulls.size())certificates->assign(faceCount*hulls.size(),0);
 for(unsigned iteration=0;iteration<budget;iteration++){
  std::fill(increments.begin(),increments.end(),0.f);float minimum=std::numeric_limits<float>::infinity();
  for(unsigned f=0;f<faceCount;f++){
   Face ids=faces[f];for(unsigned id:ids)if(id>=count)throw std::runtime_error("Noncloth face in wrap");
   for(const Hull& hull:hulls){
    const unsigned certificate=f*unsigned(hulls.size())+unsigned(&hull-hulls.data());
    if(certificates&&(*certificates)[certificate]<hull.size()){
     const Plane& plane=hull[(*certificates)[certificate]];float score=std::numeric_limits<float>::infinity();
     for(unsigned id:ids)score=(std::min)(score,Signed(plane,work[id]));
     if(score>=0){minimum=(std::min)(minimum,score);continue;}
    }
    float bestCost=std::numeric_limits<float>::infinity(),bestScore=-std::numeric_limits<float>::infinity();const Plane* chosen=nullptr;
    for(const Plane& plane:hull){float cost=0,score=std::numeric_limits<float>::infinity();bool allowed=true;
     // The cost pass reads the same immutable iteration positions. Reuse the
     // three exact distances instead of evaluating each plane dot product twice.
     float distances[3];for(unsigned j=0;j<3;j++){distances[j]=Signed(plane,work[ids[j]]);score=(std::min)(score,distances[j]);}
     // An already separated face needs no motion, even if its outward ray
     // points away from this particular plane. Margin is a repair target,
     // not a reason to reject a valid support certificate.
     if(score>=0){bestCost=0;bestScore=score;chosen=&plane;break;}
     for(unsigned j=0;j<3;j++){unsigned id=ids[j];float value=distances[j];score=(std::min)(score,value);
      if(fixed[id]){if(value< -1e-5f){allowed=false;break;}}
      else if(value<margin){float slope=Dot(plane.normal,directions[id]);if(slope<=1e-5f){allowed=false;break;}cost=(std::max)(cost,(margin-value)/slope);}
     }
     if(allowed&&cost<bestCost){bestCost=cost;bestScore=score;chosen=&plane;if(score>=0)break;}
    }
    Plane exact{};
    if(bestScore<0&&TriangleSupportPlane(hull,work[ids[0]],work[ids[1]],work[ids[2]],exact)){
     float score=std::numeric_limits<float>::infinity(),cost=0;bool allowed=true;
     for(unsigned id:ids){float gap=Signed(exact,work[id]);score=(std::min)(score,gap);if(fixed[id]){if(gap< -1e-5f)allowed=false;}
      else if(gap<margin){float slope=Dot(exact.normal,directions[id]);if(slope<=1e-5f)allowed=false;else cost=(std::max)(cost,(margin-gap)/slope);}}
     if(score>=-1e-5f||(allowed&&cost<bestCost)){chosen=&exact;bestScore=score;bestCost=cost;}
    }
    if(!chosen)throw std::runtime_error("Live collision intersects fixed seam or pole: face="+std::to_string(f)+" hull="+std::to_string(&hull-hulls.data())+" pass="+std::to_string(iteration));
    if(certificates)(*certificates)[certificate]=chosen==&exact?unsigned(hull.size()):unsigned(chosen-hull.data());
    minimum=(std::min)(minimum,bestScore);
    if(bestScore< -1e-5f)for(unsigned id:ids)if(!fixed[id]){float value=Signed(*chosen,work[id]);if(value<margin)increments[id]=(std::max)(increments[id],(margin-value)/Dot(chosen->normal,directions[id]));}
   }
  }
  receipt.iterations=iteration+1;receipt.minimumSeparation=minimum;
  if(minimum>=-1e-5f){
   for(unsigned i=0;i<count;i++){Vec d=Sub(work[i],points[i]);receipt.maximumDisplacement=(std::max)(receipt.maximumDisplacement,std::sqrt(Dot(d,d)));}
   for(unsigned f=0;f<faceCount;f++){auto ids=faces[f];Vec n=Cross(Sub(work[ids[1]],work[ids[0]]),Sub(work[ids[2]],work[ids[0]]));if(Dot(n,n)<1e-14f)throw std::runtime_error("Collapsed live cloth face="+std::to_string(f)+" vertices="+std::to_string(ids[0])+","+std::to_string(ids[1])+","+std::to_string(ids[2])+" edge2="+std::to_string(Dot(Sub(work[ids[1]],work[ids[0]]),Sub(work[ids[1]],work[ids[0]])))+","+std::to_string(Dot(Sub(work[ids[2]],work[ids[0]]),Sub(work[ids[2]],work[ids[0]]))));}
   if(!retainTautSeed&&FairMeridianReversals(work,columns,rows,faces,faceCount,hulls))receipt.minimumSeparation=(std::min)(receipt.minimumSeparation,-1e-5f);
   // Fairing can change the location of the largest displacement. The
   // separation above is now a conservative bound, not the pre-fair value.
   receipt.maximumDisplacement=0;for(unsigned i=0;i<count;i++){Vec d=Sub(work[i],points[i]);receipt.maximumDisplacement=(std::max)(receipt.maximumDisplacement,std::sqrt(Dot(d,d)));}
   points.swap(work);return receipt;
  }
  // A separating triangle can demand a large move at just one vertex. Share
  // that lift with adjacent longitudes/stations before applying it, retaining
  // every demanded correction. The next pass rechecks all affected faces.
  for(unsigned pass=0;pass<3;pass++){
   const auto original=increments;
   for(unsigned row=1;row<rows;row++)for(unsigned col=0;col<columns;col++){
    unsigned i=row*columns+col;float neighbor=(std::max)(original[row*columns+(col+columns-1)%columns],original[row*columns+(col+1)%columns]);
    if(row>1)neighbor=(std::max)(neighbor,original[i-columns]);if(row+1<rows)neighbor=(std::max)(neighbor,original[i+columns]);
    increments[i]=(std::max)(original[i],neighbor*.65f);
   }
  }
  for(unsigned i=columns;i<poleIndex;i++){
   if(!std::isfinite(increments[i]))throw std::runtime_error("Nonfinite live wrap correction");
   Vec candidate=Add(work[i],Mul(directions[i],increments[i])),d=Sub(candidate,taut[i]);
   if(Dot(d,d)>correctionLimits[i]*correctionLimits[i])throw std::runtime_error("Contact correction exceeds taut sampling spacing");
   work[i]=candidate;
  }
  // Collision correction must not leave local dents between raised vertices.
  // Re-tauten before the next authoritative full-triangle certificate pass.
  TautenCorrectedMeridians(work,columns,rows,axis);
  for(unsigned i=columns;i<poleIndex;i++){Vec d=Sub(work[i],taut[i]);if(Dot(d,d)>correctionLimits[i]*correctionLimits[i])throw std::runtime_error("Taut repair exceeds local sampling spacing");}
 }
 throw std::runtime_error("Live wrap exhausted correction budget");
}
// A fresh, current-pose display envelope. It has no triangle-clearance claim;
// callers certify separately. Never reuse partially corrected failed geometry.
inline WrapReceipt WalkMeridians(std::vector<Vec>& points,unsigned columns,unsigned rows,const float* heights,const std::vector<Hull>& hulls,Vec axis,float padding=.04f,unsigned detailedHullFirst=0){
 auto work=points;SeedMeridians(work,columns,rows,heights,axis);
 auto receipt=ClearMeridians(work,columns,rows,nullptr,0,hulls,axis,padding,1,nullptr,detailedHullFirst);
 points.swap(work);return receipt;
}
}
