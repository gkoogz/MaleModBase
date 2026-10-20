#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
// Exact current-pose support bounds. A face beyond one axis bound cannot
// contact that primitive; no bound or certificate is reused after motion.
struct TautBounds {Vec low{},high{};bool valid=false;};
inline std::vector<TautBounds> BuildTautBounds(const std::vector<Hull>& hulls){
 std::vector<TautBounds> result(hulls.size());
 for(unsigned h=0;h<hulls.size();h++)if(hulls[h].support){
  auto& b=result[h];b.valid=true;
  for(unsigned k=0;k<3;k++){Vec axis{};axis[k]=1;b.high[k]=hulls[h].support(axis);b.low[k]=-hulls[h].support(Mul(axis,-1));}
 }return result;
}
inline bool TautOutside(Vec a,Vec b,Vec c,const TautBounds& bounds,float margin){
 if(!bounds.valid)return false;
 for(unsigned k=0;k<3;k++)if((std::min)({a[k],b[k],c[k]})>bounds.high[k]+margin||
                            (std::max)({a[k],b[k],c[k]})<bounds.low[k]-margin)return true;
 return false;
}
// Refine an existing envelope without a second walk. Each correction uses a
// current whole-triangle support plane and the vertex's longitude plane. Axial
// relief follows that contact normal instead of a prescribed row-dependent
// tilt, which can be nearly tangent to the required separating plane.
inline WrapReceipt RefineTautContacts(std::vector<Vec>& points,unsigned columns,unsigned rows,
 const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,Vec axis,float margin=.04f,unsigned budget=24){
 const unsigned count=columns*rows+1,pole=count-1;
 if(columns<3||rows<2||points.size()<count||!faces||!faceCount||margin<0||!budget)throw std::runtime_error("Invalid taut contact surface");
 axis=Unit(axis);const auto original=points;auto work=points;
 const auto bounds=BuildTautBounds(hulls);
 std::vector<Vec> radial(count);std::vector<float> limits(count,0.f);
 for(unsigned col=0;col<columns;col++){
  Vec q=Sub(work[col],work[pole]);Vec direction=Unit(Sub(q,Mul(axis,Dot(q,axis))));float average=0;
  for(unsigned row=1;row<=rows;row++){unsigned i=row==rows?pole:row*columns+col;auto d=Sub(work[i],work[(row-1)*columns+col]);average+=std::sqrt(Dot(d,d))/rows;}
  for(unsigned row=1;row<rows;row++){unsigned i=row*columns+col;radial[i]=direction;limits[i]=(std::max)(2*margin,.75f*average);}
 }
 auto fixed=[&](unsigned i){return i<columns||i==pole;};
 auto route=[&](unsigned i,Vec n){return Add(Mul(radial[i],(std::max)(0.f,Dot(n,radial[i]))),Mul(axis,Dot(n,axis)));};
 // A longitude direction and a sampled support plane do not change during
 // this solve. Reuse the exact route/square root across rows, faces and passes.
 // Triangle-normal supports are dynamic and deliberately bypass this cache.
 struct Route {Vec move{};float slope=0,root=0;bool ready=false;};
 std::vector<unsigned> offsets;unsigned planeCount=0;
 for(const auto& hull:hulls){offsets.push_back(planeCount);planeCount+=unsigned(hull.size());}
 std::vector<Route> routes(std::size_t(columns)*planeCount);
 auto getRoute=[&](unsigned i,Vec n,unsigned planeIndex){
  auto calculate=[&](){Route r;r.move=route(i,n);r.slope=Dot(r.move,n);r.root=r.slope>=1e-8f?std::sqrt(r.slope):0;r.ready=true;return r;};
  if(planeIndex>=planeCount)return calculate();
  auto& cached=routes[std::size_t(i%columns)*planeCount+planeIndex];
  if(!cached.ready)cached=calculate();return cached;
 };
 WrapReceipt receipt;
 for(unsigned pass=0;pass<budget;pass++){
  float minimum=std::numeric_limits<float>::infinity();bool clear=true;
  for(unsigned f=0;f<faceCount;f++){
   auto ids=faces[f];for(auto i:ids)if(i>=count)throw std::runtime_error("Invalid taut contact face");
   for(unsigned h=0;h<hulls.size();h++){
    if(TautOutside(work[ids[0]],work[ids[1]],work[ids[2]],bounds[h],margin)){
     // The true separation is greater than margin. Keep a conservative
     // minimum while skipping the full plane and contact-route scan.
     minimum=(std::min)(minimum,margin);continue;
    }
    const auto& hull=hulls[h];
    Plane chosen{};unsigned chosenIndex=planeCount;float best=std::numeric_limits<float>::infinity(),chosenGap=0;bool found=false,separated=false;
    auto consider=[&](Plane plane,unsigned planeIndex){
     float gap=std::numeric_limits<float>::infinity(),cost=0;
     for(auto i:ids){float value=Signed(plane,work[i]);gap=(std::min)(gap,value);
      if(fixed(i)){if(value< -1e-5f)return;}
      else if(value<margin){auto r=getRoute(i,plane.normal,planeIndex);if(r.slope<1e-8f)return;cost=(std::max)(cost,(margin-value)/r.root);}
     }
     if(gap>=-1e-5f){separated=true;chosenGap=gap;return;}
     if(cost<best){best=cost;chosen=plane;chosenIndex=planeIndex;chosenGap=gap;found=true;}
    };
    for(unsigned k=0;k<hull.size();k++){consider(hull[k],offsets[h]+k);if(separated)break;}
    if(!separated){Plane exact{};if(TriangleSupportPlane(hull,work[ids[0]],work[ids[1]],work[ids[2]],exact))consider(exact,planeCount);}
    minimum=(std::min)(minimum,chosenGap);
    if(separated)continue;
    if(!found)throw std::runtime_error("Taut contact has no fixed-edge route");clear=false;
    for(auto i:ids)if(!fixed(i)){
     float gap=Signed(chosen,work[i]);if(gap>=margin)continue;auto r=getRoute(i,chosen.normal,chosenIndex);Vec move=r.move;float slope=r.slope;
     Vec candidate=Add(work[i],Mul(move,(margin-gap)/slope)),d=Sub(candidate,original[i]);
     if(!std::isfinite(Dot(d,d))||Dot(d,d)>limits[i]*limits[i])throw std::runtime_error("Taut normal correction exceeds sampling spacing vertex="+std::to_string(i)+" face="+std::to_string(f)+" pass="+std::to_string(pass)+" displacement="+std::to_string(std::sqrt(Dot(d,d)))+" limit="+std::to_string(limits[i]));
     work[i]=candidate;
    }
   }
  }
  receipt.iterations=pass+1;receipt.minimumSeparation=minimum;
  if(!clear)continue;
  for(unsigned f=0;f<faceCount;f++){
   auto ids=faces[f];Vec before=Cross(Sub(original[ids[1]],original[ids[0]]),Sub(original[ids[2]],original[ids[0]]));
   Vec after=Cross(Sub(work[ids[1]],work[ids[0]]),Sub(work[ids[2]],work[ids[0]]));
   if(Dot(after,after)<1e-14f||Dot(before,after)<=0)throw std::runtime_error("Taut contact inverted a face");
  }
  for(unsigned i=0;i<count;i++){auto d=Sub(work[i],original[i]);receipt.maximumDisplacement=(std::max)(receipt.maximumDisplacement,std::sqrt(Dot(d,d)));}
  points.swap(work);return receipt;
 }
 throw std::runtime_error("Taut contact correction did not converge");
}
inline float TautTriangleClearance(const std::vector<Vec>& p,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls){
 float minimum=std::numeric_limits<float>::infinity();
 const auto bounds=BuildTautBounds(hulls);
 for(unsigned f=0;f<faceCount;f++)for(unsigned h=0;h<hulls.size();h++){
  auto ids=faces[f];float best=-std::numeric_limits<float>::infinity();
  if(TautOutside(p[ids[0]],p[ids[1]],p[ids[2]],bounds[h],0)){
   // Only the sign matters to the caller when deciding whether to enlarge
   // the cover. Zero is a conservative lower bound for this distant pair.
   minimum=(std::min)(minimum,0.f);continue;
  }
  const auto& hull=hulls[h];
  for(const auto& plane:hull)best=(std::max)(best,(std::min)({Signed(plane,p[ids[0]]),Signed(plane,p[ids[1]]),Signed(plane,p[ids[2]])}));
  Plane exact{};if(TriangleSupportPlane(hull,p[ids[0]],p[ids[1]],p[ids[2]],exact))best=(std::max)(best,(std::min)({Signed(exact,p[ids[0]]),Signed(exact,p[ids[1]]),Signed(exact,p[ids[2]])}));
  minimum=(std::min)(minimum,best);
 }return minimum;
}
// The final acceptance gate needs a threshold, not the best separation over
// every direction. Keep the full minimum query for failed-seed padding only.
// Inputs are validated before early exits so a distant/clear face cannot hide
// malformed geometry. Support functions must describe finite convex solids.
inline bool TautTrianglesSeparated(const std::vector<Vec>& p,const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,float tolerance=1e-5f){
 if((faceCount&&!faces)||!std::isfinite(tolerance)||tolerance<0)throw std::runtime_error("Invalid taut certificate inputs");
 for(unsigned f=0;f<faceCount;f++)for(auto id:faces[f]){
  if(id>=p.size())throw std::runtime_error("Invalid taut certificate face");
  for(float v:p[id])if(!std::isfinite(v))throw std::runtime_error("Nonfinite taut certificate vertex");
 }
 for(const auto& hull:hulls)for(const auto& plane:hull){
  if(!std::isfinite(plane.offset))throw std::runtime_error("Nonfinite taut certificate plane");
  for(float v:plane.normal)if(!std::isfinite(v))throw std::runtime_error("Nonfinite taut certificate plane");
 }
 const auto bounds=BuildTautBounds(hulls);
 for(const auto& b:bounds)if(b.valid)for(unsigned k=0;k<3;k++)
  if(!std::isfinite(b.low[k])||!std::isfinite(b.high[k])||b.low[k]>b.high[k])throw std::runtime_error("Invalid taut support bounds");
 for(unsigned f=0;f<faceCount;f++)for(unsigned h=0;h<hulls.size();h++){
  auto ids=faces[f];const Vec a=p[ids[0]],b=p[ids[1]],c=p[ids[2]];
  if(TautOutside(a,b,c,bounds[h],0))continue;
  bool clear=false;
  for(const auto& plane:hulls[h])if((std::min)({Signed(plane,a),Signed(plane,b),Signed(plane,c)})>=-tolerance){clear=true;break;}
  if(!clear&&!ExactTriangleSeparated(hulls[h],a,b,c,tolerance))return false;
 }
 return true;
}
// Account for chord error when a finite grid samples a rounded convex cover.
// Increase construction support only after measuring failed triangle contact;
// final primitive certificates, fixed anchors and correction bounds remain.
inline WrapReceipt WalkCertifiedTautEnvelope(std::vector<Vec>& points,unsigned columns,unsigned rows,const float* heights,
 const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,Vec axis,float margin=.04f,float* usedPadding=nullptr,std::vector<Vec>* acceptedSeed=nullptr){
 const auto original=points;float padding=margin;
 for(unsigned attempt=0;attempt<6;attempt++){
  auto seed=original;std::vector<Hull> cover{ConvexCover(hulls,axis,padding)};
  WalkMeridians(seed,columns,rows,heights,cover,axis,margin);
  float spacing=0;for(unsigned col=0;col<columns;col++)for(unsigned row=1;row<=rows;row++){
   unsigned i=row==rows?columns*rows:row*columns+col;auto d=Sub(seed[i],seed[(row-1)*columns+col]);spacing+=std::sqrt(Dot(d,d))/(columns*rows);
  }
  for(unsigned col=0;col<columns;col++)for(unsigned row=0;row<rows;row++){
   auto d=Sub(seed[row*columns+(col+1)%columns],seed[row*columns+col]);spacing+=std::sqrt(Dot(d,d))/(columns*rows);
  }
  if(padding>(std::max)(2*margin,1.5f*spacing))throw std::runtime_error("Cover discretization exceeds surface spacing");
  auto work=seed;
  try{
   auto receipt=RefineTautContacts(work,columns,rows,faces,faceCount,hulls,axis,margin);
   if(!TautTrianglesSeparated(work,faces,faceCount,hulls))throw std::runtime_error("Refined cover lacks triangle separation");
   if(!WithinMeridianSampling(work,seed,columns,rows,margin))throw std::runtime_error("Refined cover exceeds physical spacing");
   if(usedPadding)*usedPadding=padding;if(acceptedSeed)*acceptedSeed=seed;points.swap(work);return receipt;
  }catch(const std::exception&){if(attempt==5)throw;}
  // Only a retry consumes the seed's minimum gap. Successful attempts and
  // terminal failures avoid this exhaustive scan; seed remains unchanged.
  float gap=TautTriangleClearance(seed,faces,faceCount,hulls);
  padding+=(std::max)(margin,-gap*1.25f);
 }
 throw std::runtime_error("Certified cover construction did not converge");
}
}
