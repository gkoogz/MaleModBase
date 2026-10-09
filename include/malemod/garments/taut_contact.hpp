#pragma once
#include "meridian_clearance.hpp"
namespace malemod::garments::meridian {
// Refine an existing envelope without a second walk. Each correction uses a
// current whole-triangle support plane and the vertex's longitude plane. Axial
// relief follows that contact normal instead of a prescribed row-dependent
// tilt, which can be nearly tangent to the required separating plane.
inline WrapReceipt RefineTautContacts(std::vector<Vec>& points,unsigned columns,unsigned rows,
 const Face* faces,unsigned faceCount,const std::vector<Hull>& hulls,Vec axis,float margin=.04f,unsigned budget=24){
 const unsigned count=columns*rows+1,pole=count-1;
 if(columns<3||rows<2||points.size()<count||!faces||!faceCount||margin<0||!budget)throw std::runtime_error("Invalid taut contact surface");
 axis=Unit(axis);const auto original=points;auto work=points;
 std::vector<Vec> radial(count);std::vector<float> limits(count,0.f);
 for(unsigned col=0;col<columns;col++){
  Vec q=Sub(work[col],work[pole]);Vec direction=Unit(Sub(q,Mul(axis,Dot(q,axis))));float average=0;
  for(unsigned row=1;row<=rows;row++){unsigned i=row==rows?pole:row*columns+col;auto d=Sub(work[i],work[(row-1)*columns+col]);average+=std::sqrt(Dot(d,d))/rows;}
  for(unsigned row=1;row<rows;row++){unsigned i=row*columns+col;radial[i]=direction;limits[i]=(std::max)(2*margin,.75f*average);}
 }
 auto fixed=[&](unsigned i){return i<columns||i==pole;};
 auto route=[&](unsigned i,Vec n){return Add(Mul(radial[i],(std::max)(0.f,Dot(n,radial[i]))),Mul(axis,Dot(n,axis)));};
 WrapReceipt receipt;
 for(unsigned pass=0;pass<budget;pass++){
  float minimum=std::numeric_limits<float>::infinity();bool clear=true;
  for(unsigned f=0;f<faceCount;f++){
   auto ids=faces[f];for(auto i:ids)if(i>=count)throw std::runtime_error("Invalid taut contact face");
   for(const auto& hull:hulls){
    Plane chosen{};float best=std::numeric_limits<float>::infinity(),chosenGap=0;bool found=false,separated=false;
    auto consider=[&](Plane plane){
     float gap=std::numeric_limits<float>::infinity(),cost=0;
     for(auto i:ids){float value=Signed(plane,work[i]);gap=(std::min)(gap,value);
      if(fixed(i)){if(value< -1e-5f)return;}
      else if(value<margin){Vec move=route(i,plane.normal);float slope=Dot(move,plane.normal);if(slope<1e-8f)return;cost=(std::max)(cost,(margin-value)/std::sqrt(slope));}
     }
     if(gap>=-1e-5f){separated=true;chosenGap=gap;return;}
     if(cost<best){best=cost;chosen=plane;chosenGap=gap;found=true;}
    };
    for(const auto& plane:hull){consider(plane);if(separated)break;}
    if(!separated){Plane exact{};if(TriangleSupportPlane(hull,work[ids[0]],work[ids[1]],work[ids[2]],exact))consider(exact);}
    minimum=(std::min)(minimum,chosenGap);
    if(separated)continue;
    if(!found)throw std::runtime_error("Taut contact has no fixed-edge route");clear=false;
    for(auto i:ids)if(!fixed(i)){
     float gap=Signed(chosen,work[i]);if(gap>=margin)continue;Vec move=route(i,chosen.normal);float slope=Dot(move,chosen.normal);
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
 for(unsigned f=0;f<faceCount;f++)for(const auto& hull:hulls){
  auto ids=faces[f];float best=-std::numeric_limits<float>::infinity();
  for(const auto& plane:hull)best=(std::max)(best,(std::min)({Signed(plane,p[ids[0]]),Signed(plane,p[ids[1]]),Signed(plane,p[ids[2]])}));
  Plane exact{};if(TriangleSupportPlane(hull,p[ids[0]],p[ids[1]],p[ids[2]],exact))best=(std::max)(best,(std::min)({Signed(exact,p[ids[0]]),Signed(exact,p[ids[1]]),Signed(exact,p[ids[2]])}));
  minimum=(std::min)(minimum,best);
 }return minimum;
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
  float gap=TautTriangleClearance(seed,faces,faceCount,hulls);auto work=seed;
  try{
   auto receipt=RefineTautContacts(work,columns,rows,faces,faceCount,hulls,axis,margin);
   if(TautTriangleClearance(work,faces,faceCount,hulls)<-1e-5f)throw std::runtime_error("Refined cover lacks triangle separation");
   if(!WithinMeridianSampling(work,seed,columns,rows,margin))throw std::runtime_error("Refined cover exceeds physical spacing");
   if(usedPadding)*usedPadding=padding;if(acceptedSeed)*acceptedSeed=seed;points.swap(work);return receipt;
  }catch(const std::exception&){if(attempt==5)throw;}
  padding+=(std::max)(margin,-gap*1.25f);
 }
 throw std::runtime_error("Certified cover construction did not converge");
}
}
