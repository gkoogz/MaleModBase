#pragma once
#include "compound_graft.hpp"
#include "suspension_profile.hpp"
#include "convex_surface_cut.hpp"
#include <memory>
#include <numeric>
#include <iterator>

namespace malemod::surface {
// The engine supplies a finished body/root/shaft support and the independently
// posed suspended surface. Neither the numerical solver nor a game SDK is owned
// here. Geometry ownership is separate from per-corner material ownership.
struct CompoundContactReference {
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 std::vector<Vec> support,source,patch,firstTangents,lastTangents;
 std::vector<std::array<unsigned,3>> supportFaces,sourceFaces;
 std::vector<UV> chart,footprint;
 std::vector<bool> eligible;
 std::vector<bool> protectedSupport;
 // Optional derived boundary constraints when a chart is reparameterized.
 // These do not alter the serialized authoring contract.
 std::vector<bool> protectedKnots;
 std::vector<unsigned> sourceLoop;
 std::vector<double> curve,parameters;
};
struct CompoundContactCorner {
 unsigned vertex=0,domain=0,parent=0;
 std::array<double,3> donor{};
 // Patch attributes use an authored ring and continuous source-curve position.
 double curve=0;unsigned row=0;
};
struct CompoundContactCache {
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 struct Sample {unsigned vertex=0,parent=0;Vec donor{};};
 struct Differential {unsigned parent=0;Vec point{},du{},dv{};};
 std::vector<Vec> rest;
 std::vector<std::array<CompoundContactCorner,3>> faces;
 std::vector<Sample> supportSamples,sourceSamples;
 std::vector<Differential> differentials;
 struct SourceDifferential {unsigned parent,segment;Vec point;double t;};
 std::vector<SourceDifferential> sourceDifferentials;std::vector<unsigned> sourceLoop;
 std::vector<double> sourceReferenceSideLength;
 std::vector<double> curveKnots,frameCoordinates,parameters;double framePeriod=0;
 struct FrameFace {unsigned parent;Vec du,dv;double area;};
 std::vector<FrameFace> frameFaces;std::vector<double> frameWeights;
 std::vector<std::array<unsigned,3>> supportFaces,sourceFaces;
 std::vector<std::vector<unsigned>> rings;
 std::unique_ptr<CompoundGraftPlan> graft;
 unsigned supportCount=0;
 double suspensionHang=0;
 Vec suspensionForward{};
 unsigned collapsedEdges=0,flippedEdges=0,dissolvedFans=0;double maximumRemovedEdge=0;
 void Clean(std::vector<bool> protectedVertices){
  if(protectedVertices.size()!=rest.size())throw std::invalid_argument("Compound cleanup protection differs");
  Vec lo=rest.front(),hi=lo;for(auto p:rest)for(unsigned k=0;k<3;k++){lo[k]=(std::min)(lo[k],p[k]);hi[k]=(std::max)(hi[k],p[k]);}
  // A face just above the rest degeneracy threshold can drop below it under
  // ordinary suspension compression. Condition generated subdivisions with
  // an eightfold area reserve; do not move protected original samples.
  const auto extent=Sub(hi,lo);double threshold=Dot(extent,extent)*8e-12,distanceLimit=std::sqrt(Dot(extent,extent))*1e-5;
  const double minimumEdge2=Dot(extent,extent)*4e-18;
  auto normal=[&](const std::array<CompoundContactCorner,3>& f){return Cross(Sub(rest[f[1].vertex],rest[f[0].vertex]),Sub(rest[f[2].vertex],rest[f[0].vertex]));};
  auto needsCleanup=[&](const std::array<CompoundContactCorner,3>& f,double area){
   bool original=true;for(auto c:f)original&=protectedVertices[c.vertex];
   if(area<(f[0].domain==0||original?threshold/8:threshold))return true;
   for(unsigned k=0;k<3;k++){auto d=Sub(rest[f[k].vertex],rest[f[(k+1)%3].vertex]);if(Dot(d,d)<minimumEdge2)return true;}return false;
  };
  std::vector<bool> active(faces.size(),true);std::vector<double> areas(faces.size());std::vector<std::set<unsigned>> incident(rest.size());
  std::multimap<double,unsigned> bad;
  for(unsigned i=0;i<faces.size();i++){auto n=normal(faces[i]);areas[i]=std::sqrt(Dot(n,n));if(needsCleanup(faces[i],areas[i]))bad.emplace(areas[i],i);for(auto c:faces[i])incident[c.vertex].insert(i);}
  auto neighbors=[&](unsigned vertex,unsigned other){std::set<unsigned> result;for(auto fi:incident[vertex])for(auto c:faces[fi])if(c.vertex!=vertex&&c.vertex!=other)result.insert(c.vertex);return result;};
  // Clipped slivers are precisely where the Gram determinant aa*bb-ab*ab
  // loses all significant digits. Cross-product coordinates retain their
  // signed area and preserve the face's own attribute chart during cleanup.
  auto bary=[](Vec p,const std::array<Vec,3>& q){auto a=Sub(q[1],q[0]),b=Sub(q[2],q[0]),v=Sub(p,q[0]),n=Cross(a,b);double den=Dot(n,n);if(den<=1e-30)throw std::runtime_error("Collapsed cleanup attribute triangle");double u=Dot(Cross(v,b),n)/den,w=Dot(Cross(a,v),n)/den;return Vec{1-u-w,u,w};};
  unsigned attempts=0,passCollapses=0;std::vector<std::pair<double,unsigned>> deferred;std::string rejection;
  auto nextPass=[&]{
   if(!bad.empty()||deferred.empty())return;
   const unsigned progress=collapsedEdges+flippedEdges+dissolvedFans;
   if(progress==passCollapses){
    throw std::runtime_error("Compound cleanup cannot safely remove "+std::to_string(deferred.size())+" faces; first="+std::to_string(deferred.front().second)+rejection);
   }
   passCollapses=progress;for(auto item:deferred)if(active[item.second]&&needsCleanup(faces[item.second],areas[item.second]))bad.emplace(areas[item.second],item.second);deferred.clear();
  };
  while(!bad.empty()){
   if(++attempts>faces.size()*4)throw std::runtime_error("Compound cleanup exceeded its work bound");
   auto first=bad.begin();auto fi=first->second;auto value=first->first;bad.erase(first);if(!active[fi]||areas[fi]!=value){nextPass();continue;}
   std::vector<std::pair<double,std::pair<unsigned,unsigned>>> edges;
   for(unsigned k=0;k<3;k++){auto a=faces[fi][k].vertex,b=faces[fi][(k+1)%3].vertex;auto d=Sub(rest[a],rest[b]);edges.push_back({std::sqrt(Dot(d,d)),{a,b}});}
   // A tiny immutable rim segment cannot be merged across the support's
   // original crease. Coarsen the adjacent meridian instead: its next-row
   // neighbor can remove an unnecessary interior sample while preserving
   // the full rim. The same surface-error and link-condition checks apply.
   if(faces[fi][0].domain==2)for(auto corner:faces[fi])if(!protectedVertices[corner.vertex])
    for(auto neighbor:incident[corner.vertex])for(auto other:faces[neighbor])if(other.domain==2&&other.vertex!=corner.vertex){
     auto edge=std::minmax(corner.vertex,other.vertex);auto d=Sub(rest[edge.first],rest[edge.second]);edges.push_back({std::sqrt(Dot(d,d)),edge});
    }
   std::sort(edges.begin(),edges.end());edges.erase(std::unique(edges.begin(),edges.end()),edges.end());
   bool accepted=false;rejection=" face="+std::to_string(fi);
   for(auto edge:edges){if(edge.first>distanceLimit*100){rejection+=" long="+std::to_string(edge.first);continue;}
    for(unsigned direction=0;direction<2;direction++){
     unsigned remove=direction?edge.second.second:edge.second.first,keep=direction?edge.second.first:edge.second.second;rejection+=" edge="+std::to_string(remove)+"->"+std::to_string(keep);if(protectedVertices[remove]){rejection+=" protected";continue;}
     std::vector<unsigned> both;std::set_intersection(incident[remove].begin(),incident[remove].end(),incident[keep].begin(),incident[keep].end(),std::back_inserter(both));if(both.size()!=2)continue;
     auto na=neighbors(remove,keep),nb=neighbors(keep,remove);std::set<unsigned> common,opposite;std::set_intersection(na.begin(),na.end(),nb.begin(),nb.end(),std::inserter(common,common.end()));for(auto b:both)for(auto c:faces[b])if(c.vertex!=remove&&c.vertex!=keep)opposite.insert(c.vertex);if(common!=opposite)continue;
     bool generatedFan=true;for(auto f:incident[remove])for(auto c:faces[f])generatedFan&=c.domain==2;
     std::vector<std::pair<unsigned,std::array<CompoundContactCorner,3>>> revised;bool valid=true;
     for(auto f:incident[remove])if(f!=both[0]&&f!=both[1]){
      auto next=faces[f];for(auto& c:next)if(c.vertex==remove)c.vertex=keep;auto a=normal(faces[f]),b=normal(next);double aa=std::sqrt(Dot(a,a)),bb=std::sqrt(Dot(b,b));
      // A nearly collinear strip can have a long edge yet negligible surface
      // error. Permit its contraction only when the retained point stays on
      // every affected face plane within one thousandth of the short-edge
      // budget. Protected source/boundary samples still cannot be removed.
      // A generated meridian can be long along the surface yet have a tiny
      // normal displacement. Its full closed fan may be retriangulated within
      // a tenth of the EXISTING positional budget. This exception never
      // crosses a retained material rim or removes a protected source sample.
      // Source/support fans retain their original stricter plane budget.
      if(edge.first>distanceLimit&&std::abs(Dot(Sub(rest[keep],rest[remove]),a))>distanceLimit*(generatedFan?.1:.001)*aa){valid=false;break;}
      // A below-tolerance sliver has an ill-conditioned normal. Preserve its
      // winding, but constrain angular change using the resolved neighbors.
      // The cleanup threshold contains an eightfold motion reserve. Faces
      // only just above it still have unreliable angular normals; their
      // signed orientation and bounded displacement are the useful guards.
      const double resolvedArea=threshold*(generatedFan?8:1);
      if(bb<(std::min)(aa,threshold)*.999||Dot(a,b)<(aa>=resolvedArea?.995:0.)*aa*bb){valid=false;break;}
      // Extend this corner's own material chart to the retained sample.
      const std::array<Vec,3> q{rest[faces[f][0].vertex],rest[faces[f][1].vertex],rest[faces[f][2].vertex]};
      auto weights=bary(rest[keep],q);
      for(unsigned k=0;k<3;k++)if(faces[f][k].vertex==remove){
       if(next[k].domain<2){Vec donor{};for(unsigned j=0;j<3;j++)donor=Add(donor,Scale(faces[f][j].donor,weights[j]));next[k].donor=donor;}
       // Generated patch UV fields are sampled independently by row/curve.
       // Keep their existing chart at this sub-tolerance replacement.
      }
      revised.push_back({f,next});
     }
     if(!valid){rejection+=" geometry";continue;}
     for(auto f:both){active[f]=false;for(auto c:faces[f])incident[c.vertex].erase(f);}
     for(auto item:revised){auto f=item.first;incident[remove].erase(f);incident[keep].insert(f);faces[f]=item.second;auto n=normal(faces[f]);areas[f]=std::sqrt(Dot(n,n));if(needsCleanup(faces[f],areas[f]))bad.emplace(areas[f],f);}
     collapsedEdges++;maximumRemovedEdge=(std::max)(maximumRemovedEdge,edge.first);accepted=true;break;
    }if(accepted)break;
   }
   // A long, almost collinear ear can also occur at the generated strip's
   // finely sampled rim. Flip an internal diagonal within the SAME material
   // domain/parent, keeping all positions and per-corner charts. Nonplanar
   // flips still have to satisfy the strict orientation checks below.
   if(!accepted)for(unsigned k=0;k<3&&!accepted;k++){
    const auto a=faces[fi][k],b=faces[fi][(k+1)%3],c=faces[fi][(k+2)%3];
    std::vector<unsigned> both;std::set_intersection(incident[a.vertex].begin(),incident[a.vertex].end(),incident[b.vertex].begin(),incident[b.vertex].end(),std::back_inserter(both));if(both.size()!=2)continue;
    const unsigned other=both[0]==fi?both[1]:both[0];auto d=faces[other][0];bool compatible=true;
    for(auto corner:faces[other]){compatible&=corner.domain==a.domain&&corner.parent==a.parent;if(corner.vertex!=a.vertex&&corner.vertex!=b.vertex)d=corner;}
    if(!compatible){rejection+=" flip-parent="+std::to_string(a.parent)+":"+std::to_string(d.parent);continue;}
    bool connected=false;for(auto f:incident[c.vertex])for(auto corner:faces[f])connected|=corner.vertex==d.vertex;if(connected)continue;
    std::array<CompoundContactCorner,3> x{c,d,b},y{d,c,a};auto nx=normal(x),ny=normal(y),before=Add(normal(faces[fi]),normal(faces[other]));
    double ax=std::sqrt(Dot(nx,nx)),ay=std::sqrt(Dot(ny,ny)),ab=std::sqrt(Dot(before,before));
    // A chain of skinny triangles may need several improving flips. Requiring
    // both triangles to clear the final threshold in one step strands valid
    // planar subdivisions. Strictly increase their minimum area to prevent
    // oscillation, and requeue any still-small result.
    bool orientation=Dot(nx,before)>=.999999*ax*ab&&Dot(ny,before)>=.999999*ay*ab;
    if(!orientation&&a.domain==0){
     // A cut ear may be far thinner than its parent. Its world-space cross
     // product is then an unreliable plane reference. Use the untouched
     // parent's plane and the face-specific affine donors instead. This is
     // bounded by the same surface-error budget as edge contraction.
     const auto owner=supportFaces.at(a.parent);const auto origin=rest[owner[0]];
     const auto plane=Cross(Sub(rest[owner[1]],origin),Sub(rest[owner[2]],origin));
     const double length=std::sqrt(Dot(plane,plane));
     auto signedDonor=[](const auto& f){auto u=Sub(f[1].donor,f[0].donor),v=Sub(f[2].donor,f[0].donor);return u[1]*v[2]-u[2]*v[1];};
     orientation=length>0&&signedDonor(x)>0&&signedDonor(y)>0&&Dot(nx,plane)>0&&Dot(ny,plane)>0&&Dot(nx,before)>0&&Dot(ny,before)>0;
     // Short accepted contractions can leave this parent slightly nonplanar.
     // Reuse their existing displacement budget rather than treating the
     // nearly zero-area ear's noisy normal as an exact plane constraint.
     for(auto corner:{a,b,c,d})orientation&=std::abs(Dot(Sub(rest[corner.vertex],origin),plane))<=distanceLimit*length;
    }
    if((std::min)(ax,ay)<=(std::min)(areas[fi],areas[other])*(1+1e-8)||!orientation){rejection+=" flip-quality="+std::to_string(ax/threshold)+","+std::to_string(ay/threshold)+","+std::to_string(Dot(nx,before)/(ax*ab))+","+std::to_string(Dot(ny,before)/(ay*ab))+" gain="+std::to_string((std::min)(ax,ay)/(std::min)(areas[fi],areas[other]));continue;}
    for(auto f:{fi,other})for(auto corner:faces[f])incident[corner.vertex].erase(f);
    faces[fi]=x;faces[other]=y;areas[fi]=ax;areas[other]=ay;
    for(auto f:{fi,other})for(auto corner:faces[f])incident[corner.vertex].insert(f);
    for(auto f:{fi,other})if(needsCleanup(faces[f],areas[f]))bad.emplace(areas[f],f);
    flippedEdges++;accepted=true;
   }
   if(!accepted&&faces[fi][0].domain<2)for(auto candidate:faces[fi]){
    unsigned center=candidate.vertex;if(protectedVertices[center]||incident[center].size()!=3)continue;
    auto neighborhood=incident[center];std::map<unsigned,CompoundContactCorner> corners;std::map<unsigned,unsigned> next;bool compatible=true;Vec summed{};
    for(auto f:neighborhood){summed=Add(summed,normal(faces[f]));for(unsigned k=0;k<3;k++){
     auto a=faces[f][k],b=faces[f][(k+1)%3];compatible&=a.domain==candidate.domain&&a.parent==candidate.parent;
     if(a.vertex!=center){corners[a.vertex]=a;if(b.vertex!=center)next[a.vertex]=b.vertex;}
    }}
    if(!compatible||corners.size()!=3||next.size()!=3)continue;
    unsigned a=next.begin()->first,b=next.at(a),c=next.at(b);if(c==a||next.at(c)!=a)continue;
    std::array<CompoundContactCorner,3> replacement{corners.at(a),corners.at(b),corners.at(c)};auto n=normal(replacement);double area=std::sqrt(Dot(n,n)),sumArea=std::sqrt(Dot(summed,summed));
    if(needsCleanup(replacement,area)||Dot(n,summed)<.999999*area*sumArea||std::abs(area-sumArea)>threshold)continue;
    for(auto f:neighborhood){for(auto corner:faces[f])incident[corner.vertex].erase(f);active[f]=false;}
    faces[fi]=replacement;active[fi]=true;areas[fi]=area;for(auto corner:replacement)incident[corner.vertex].insert(fi);
    dissolvedFans++;accepted=true;break;
   }
   if(!accepted)deferred.push_back({areas[fi],fi});
   nextPass();
  }
  std::vector<std::array<CompoundContactCorner,3>> retained;for(unsigned i=0;i<faces.size();i++)if(active[i])retained.push_back(faces[i]);faces=std::move(retained);
 }
 static Vec SamplePoint(const std::vector<Vec>& points,std::array<unsigned,3> face,Vec weights){
  Vec p{};for(unsigned k=0;k<3;k++)p=Add(p,Scale(points.at(face[k]),weights[k]));return p;
 }
 void PrepareFrames(const std::vector<Vec>& points,const std::vector<UV>& chart,const std::vector<bool>& eligible){
  frameWeights.assign(points.size(),0);frameFaces.clear();
  for(unsigned i=0;i<supportFaces.size();i++)if(eligible[i]){
   auto f=supportFaces[i];auto a=chart[f[0]],b=chart[f[1]],c=chart[f[2]];double u=b[0]-a[0],v=b[1]-a[1],x=c[0]-a[0],y=c[1]-a[1],det=u*y-v*x;
   if(std::abs(det)<1e-18)continue;
   auto n=Cross(Sub(points[f[1]],points[f[0]]),Sub(points[f[2]],points[f[0]]));double area=std::sqrt(Dot(n,n));
   frameFaces.push_back({i,{(-y+v)/det,y/det,-v/det},{(x-u)/det,-x/det,u/det},area});for(auto id:f)frameWeights[id]+=area;
  }
 }
 std::vector<std::array<Vec,2>> Frames(const std::vector<Vec>& points)const{
  std::vector<std::array<Vec,2>> result(points.size());
  for(const auto& item:frameFaces){auto f=supportFaces[item.parent];auto a=Sub(points[f[1]],points[f[0]]),b=Sub(points[f[2]],points[f[0]]);
   auto du=Scale(Add(Scale(a,item.du[1]),Scale(b,item.du[2])),item.area),dv=Scale(Add(Scale(a,item.dv[1]),Scale(b,item.dv[2])),item.area);
   for(auto id:f){result[id][0]=Add(result[id][0],du);result[id][1]=Add(result[id][1],dv);}
  }
  for(unsigned i=0;i<result.size();i++)if(frameWeights[i]>0)for(auto& v:result[i])v=Scale(v,1/frameWeights[i]);return result;
 }
 std::vector<Vec> DifferentialPoints(const std::vector<Vec>& support)const{
  const auto frames=Frames(support);
  std::vector<Vec> values;values.reserve(differentials.size()*3);
  for(const auto& d:differentials){auto face=supportFaces.at(d.parent);auto p=SamplePoint(support,face,d.point);
   Vec du{},dv{};for(unsigned k=0;k<3;k++){du=Add(du,Scale(frames[face[k]][0],d.point[k]));dv=Add(dv,Scale(frames[face[k]][1],d.point[k]));}
   values.push_back(p);values.push_back(Add(p,du));values.push_back(Add(p,dv));
  }return StabilizeFrames(values);
 }
 std::vector<Vec> StabilizeFrames(const std::vector<Vec>& field)const{
  // Intersection knots are tessellation samples, not independent deformation
  // controls. Use 128 uniform reference arc-length stations as the stable
  // frame lattice so nearly coincident cut knots cannot acquire different
  // handles. Reference and live differentials use this same versioned lattice.
  const unsigned count=unsigned(curveKnots.size()),period=128;
  std::vector<std::array<Vec,2>> anchors(period);
  for(unsigned j=0;j<period;j++){
   double u=framePeriod*j/period;auto upper=std::upper_bound(frameCoordinates.begin(),frameCoordinates.end(),u);unsigned next=unsigned(upper-frameCoordinates.begin())%count,previous=(next+count-1)%count;
   double a=frameCoordinates[previous],b=frameCoordinates[next];if(b<=a)b+=framePeriod;if(u<a)u+=framePeriod;double t=(u-a)/(b-a);
   for(unsigned k=0;k<2;k++)anchors[j][k]=Add(Scale(Sub(field[3*previous+k+1],field[3*previous]),1-t),Scale(Sub(field[3*next+k+1],field[3*next]),t));
  }
  auto out=field;for(unsigned j=0;j<count;j++){double u=frameCoordinates[j]/framePeriod*period;unsigned a=unsigned(std::floor(u))%period;double t=u-std::floor(u);for(unsigned k=0;k<2;k++)out[3*j+k+1]=Add(field[3*j],Add(Scale(anchors[a][k],1-t),Scale(anchors[(a+1)%period][k],t)));}return out;
 }
 std::vector<Vec> SourceDifferentialPoints(const std::vector<Vec>& source)const{
  std::vector<Vec> normals(source.size());for(auto f:sourceFaces){auto n=Cross(Sub(source[f[1]],source[f[0]]),Sub(source[f[2]],source[f[0]]));for(auto id:f)normals[id]=Add(normals[id],n);}
  // Remove reference tessellation density before interpolating the frame.
  // The divisor stays fixed during motion: live strain and uniform scaling
  // remain part of the differential, unlike normalizing every live vector.
  if(sourceReferenceSideLength.size()!=sourceLoop.size())throw std::invalid_argument("Compound source frame gauge differs");
  std::vector<Vec> side(sourceLoop.size());for(unsigned j=0;j<sourceLoop.size();j++)
   side[j]=Scale(Sub(source[sourceLoop[(j+1)%sourceLoop.size()]],source[sourceLoop[(j+sourceLoop.size()-1)%sourceLoop.size()]]),.5/sourceReferenceSideLength[j]);
  std::vector<Vec> result;result.reserve(sourceDifferentials.size()*3);
  for(auto d:sourceDifferentials){auto f=sourceFaces.at(d.parent);auto p=SamplePoint(source,f,d.point);Vec normal{};for(unsigned k=0;k<3;k++)normal=Add(normal,Scale(normals[f[k]],d.point[k]));
   auto tangent=Add(Scale(side[d.segment],1-d.t),Scale(side[(d.segment+1)%side.size()],d.t));
   if(Dot(tangent,tangent)<1e-20)throw std::runtime_error("Compound source tangent collapsed");
   normal=Sub(normal,Scale(tangent,Dot(normal,tangent)/Dot(tangent,tangent)));
   double n=std::sqrt(Dot(normal,normal));if(n<1e-14||Dot(tangent,tangent)<1e-20)throw std::runtime_error("Compound source differential collapsed");
   auto meridian=Cross(Scale(normal,1/n),tangent);result.push_back(p);result.push_back(Add(p,tangent));result.push_back(Add(p,meridian));
  }return StabilizeFrames(result);
 }
 std::vector<Vec> Evaluate(const std::vector<Vec>& source,const std::vector<Vec>& support,double* profileFraction=nullptr)const{
  if(support.size()!=supportCount)throw std::invalid_argument("Compound support topology changed without rebuilding");
  auto points=rest;std::copy(support.begin(),support.end(),points.begin());
  for(auto s:supportSamples)points[s.vertex]=SamplePoint(support,supportFaces.at(s.parent),s.donor);
  for(auto s:sourceSamples)points[s.vertex]=SamplePoint(source,sourceFaces.at(s.parent),s.donor);
  auto sourceFrames=SourceDifferentialPoints(source);auto patch=graft->Evaluate(sourceFrames,DifferentialPoints(support));
  std::vector<Vec> entryNormals;for(unsigned j=0;j<rings.front().size();j++)entryNormals.push_back(Cross(Sub(sourceFrames[3*j+1],sourceFrames[3*j]),Sub(sourceFrames[3*j+2],sourceFrames[3*j])));
  graft->ConformSourceEntry(patch,parameters,entryNormals);
  // Fit the source tangent before the reversible Hang profile. A conformity
  // pass afterwards would silently alter the profile's authored fraction.
  const double applied=suspensionHang>0?ShapeSuspensionStrip(patch,parameters,unsigned(rings.front().size()),1,suspensionHang,1,suspensionForward):1;
  if(profileFraction)*profileFraction=applied;
  for(unsigned row=0;row<rings.size();row++)for(unsigned j=0;j<rings[row].size();j++)points[rings[row][j]]=patch[row*rings[row].size()+j];
  return points;
 }
};

class CompoundContactPlan {
 using Vec=PresentationPoint;using UV=std::array<double,2>;
 CompoundContactReference reference_;
 static double Cross2(UV a,UV b){return a[0]*b[1]-a[1]*b[0];}
 static UV Minus(UV a,UV b){return {a[0]-b[0],a[1]-b[1]};}
 static Vec Bary(UV p,const std::array<UV,3>& t){auto a=Minus(t[1],t[0]),b=Minus(t[2],t[0]),q=Minus(p,t[0]);double den=Cross2(a,b);if(std::abs(den)<1e-16)throw std::invalid_argument("Collapsed compound chart triangle");double u=Cross2(q,b)/den,v=Cross2(a,q)/den;return {1-u-v,u,v};}
 static CompoundContactCache::Differential Differential(unsigned parent,Vec point,const std::array<UV,3>& t){
  const auto a=Minus(t[1],t[0]),b=Minus(t[2],t[0]);double den=Cross2(a,b);
  if(std::abs(den)<1e-16)throw std::invalid_argument("Collapsed compound differential");
  Vec du{(-b[1]+a[1])/den,b[1]/den,-a[1]/den};
  Vec dv{(b[0]-a[0])/den,-b[0]/den,a[0]/den};return {parent,point,du,dv};
 }
 std::pair<unsigned,Vec> ReferenceAt(UV q)const{
  for(unsigned i=0;i<reference_.supportFaces.size();i++)if(reference_.eligible.at(i)){
   auto f=reference_.supportFaces[i];std::array<UV,3> t{reference_.chart[f[0]],reference_.chart[f[1]],reference_.chart[f[2]]};
   bool outside=false;for(unsigned k=0;k<2;k++)outside|=q[k]<(std::min)({t[0][k],t[1][k],t[2][k]})-1e-9||q[k]>(std::max)({t[0][k],t[1][k],t[2][k]})+1e-9;
   if(outside)continue;auto b=Bary(q,t);if(*std::min_element(b.begin(),b.end())>=-1e-8)return {i,b};
  }throw std::runtime_error("Compound rim leaves its authored support chart");
 }
 double Parameter(UV q)const{
  double best=1e300,value=0;
  for(unsigned j=0;j<reference_.footprint.size();j++){
   auto a=reference_.footprint[j],edge=Minus(reference_.footprint[(j+1)%reference_.footprint.size()],a),delta=Minus(q,a);
   double t=std::clamp((delta[0]*edge[0]+delta[1]*edge[1])/(edge[0]*edge[0]+edge[1]*edge[1]),0.,1.);
   double error=std::hypot(delta[0]-t*edge[0],delta[1]-t*edge[1]);if(error<best){best=error;value=j+t;}
  }
  if(best>1e-6)throw std::runtime_error("Compound cut boundary left its footprint: chart="+std::to_string(q[0])+","+std::to_string(q[1])+" distance="+std::to_string(best));
  if(value>=reference_.footprint.size()-1e-9)value=0;return value;
 }
 Vec Field(const std::vector<Vec>& values,unsigned row,double u)const{
  auto b=std::upper_bound(reference_.curve.begin(),reference_.curve.end(),u);
  unsigned hi=unsigned(b-reference_.curve.begin()),lo=(hi+unsigned(reference_.curve.size())-1)%unsigned(reference_.curve.size());
  double a=reference_.curve[lo],z=hi<reference_.curve.size()?reference_.curve[hi]:reference_.curve.front()+reference_.footprint.size();
  if(hi==0){a-=reference_.footprint.size();}double t=(u-a)/(z-a);hi%=unsigned(reference_.curve.size());
  return Add(Scale(values.at(row*reference_.curve.size()+lo),1-t),Scale(values.at(row*reference_.curve.size()+hi),t));
 }
public:
 explicit CompoundContactPlan(CompoundContactReference reference):reference_(std::move(reference)){
  auto& r=reference_;if(r.chart.size()!=r.support.size()||r.eligible.size()!=r.supportFaces.size()||r.curve.empty()||r.parameters.size()<3||r.patch.size()!=r.curve.size()*r.parameters.size()||r.sourceLoop.size()!=r.footprint.size()||r.firstTangents.size()!=r.curve.size()||r.lastTangents.size()!=r.curve.size())throw std::invalid_argument("Compound reference fields disagree");
  if(r.support.empty()||r.source.empty()||r.sourceLoop.size()<3||(!r.protectedSupport.empty()&&r.protectedSupport.size()!=r.support.size()))throw std::invalid_argument("Invalid compound reference topology");
  for(const auto* field:{&r.support,&r.source,&r.patch,&r.firstTangents,&r.lastTangents})for(auto p:*field)for(auto x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite compound reference field");
  for(const auto* field:{&r.chart,&r.footprint})for(auto p:*field)for(auto x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite compound chart field");
  auto triangles=[](const auto& faces,std::size_t count){for(auto f:faces)if(f[0]>=count||f[1]>=count||f[2]>=count||f[0]==f[1]||f[1]==f[2]||f[2]==f[0])throw std::invalid_argument("Invalid compound reference triangle");};
  triangles(r.supportFaces,r.support.size());triangles(r.sourceFaces,r.source.size());
  std::set<unsigned> loop;for(auto id:r.sourceLoop)if(id>=r.source.size()||!loop.insert(id).second)throw std::invalid_argument("Invalid compound source loop");
  auto ordered=[](const std::vector<double>& field,double lo,double hi){double previous=-1;for(auto v:field){if(!std::isfinite(v)||v<lo||v>hi||v<=previous)throw std::invalid_argument("Invalid compound ordered field");previous=v;}};
  ordered(r.curve,0,double(r.footprint.size()));ordered(r.parameters,0,1);
  if(r.curve.back()>=r.footprint.size()||r.parameters.front()!=0||r.parameters.back()!=1)throw std::invalid_argument("Invalid compound periodic or endpoint parameter");
 }
 CompoundContactCache Build(const std::vector<Vec>& source,const std::vector<Vec>& support,
     const std::vector<std::array<unsigned,3>>& faces,const std::vector<UV>& chart,const std::vector<bool>& eligible,
     const std::vector<bool>& protectedSupport={},double suspensionHang=0,Vec suspensionForward={})const{
  const auto& r=reference_;if(source.size()!=r.source.size())throw std::invalid_argument("Compound source topology differs");
  if(!r.protectedKnots.empty()&&r.protectedKnots.size()!=r.footprint.size())throw std::invalid_argument("Compound boundary protection differs");
  if(!protectedSupport.empty()&&protectedSupport.size()!=support.size())throw std::invalid_argument("Compound support protection differs");
  auto cut=ConvexSurfaceCut(support,faces,eligible,chart,1e-8).CutSimple(r.footprint);
  CompoundContactCache out;out.rest=cut.points;out.supportCount=unsigned(support.size());out.supportFaces=faces;out.sourceFaces=r.sourceFaces;out.sourceLoop=r.sourceLoop;out.parameters=r.parameters;
  for(unsigned j=0;j<r.sourceLoop.size();j++){
   auto side=Scale(Sub(r.source[r.sourceLoop[(j+1)%r.sourceLoop.size()]],r.source[r.sourceLoop[(j+r.sourceLoop.size()-1)%r.sourceLoop.size()]]),.5);
   const double square=Dot(side,side);if(square<=1e-20)throw std::invalid_argument("Collapsed compound reference source side");
   out.sourceReferenceSideLength.push_back(std::sqrt(square));
  }
  out.suspensionHang=suspensionHang;
  out.suspensionForward=suspensionForward;
  out.PrepareFrames(support,chart,eligible);
  CompoundContactCache referenceFrames;referenceFrames.supportFaces=r.supportFaces;referenceFrames.PrepareFrames(r.support,r.chart,r.eligible);const auto smoothReference=referenceFrames.Frames(r.support);
  std::vector<bool> seen(cut.points.size());std::vector<CompoundContactCache::Sample> owners(cut.points.size());
  for(auto f:cut.faces){std::array<CompoundContactCorner,3> face;
   for(unsigned k=0;k<3;k++){auto c=f.corners[k];face[k]={c.vertex,0,f.parent,c.donor};if(!seen[c.vertex]){seen[c.vertex]=true;owners[c.vertex]={c.vertex,f.parent,c.donor};if(c.vertex>=support.size())out.supportSamples.push_back(owners[c.vertex]);}}
   out.faces.push_back(face);
  }
  struct Knot {double u;unsigned end,parent;Vec bary;};std::vector<Knot> knots;
  for(auto id:cut.opening){auto owner=owners[id];auto f=faces[owner.parent];UV q{};for(unsigned k=0;k<3;k++)for(unsigned d=0;d<2;d++)q[d]+=chart[f[k]][d]*owner.donor[k];knots.push_back({Parameter(q),id,owner.parent,owner.donor});}
  std::sort(knots.begin(),knots.end(),[](const Knot& a,const Knot& b){return a.u<b.u;});
  for(auto knot:knots)out.curveKnots.push_back(knot.u);
  std::vector<double> sourceArc{0};for(unsigned j=0;j<r.sourceLoop.size();j++){auto d=Sub(r.source[r.sourceLoop[(j+1)%r.sourceLoop.size()]],r.source[r.sourceLoop[j]]);sourceArc.push_back(sourceArc.back()+std::sqrt(Dot(d,d)));}out.framePeriod=sourceArc.back();
  for(auto knot:knots){unsigned j=unsigned(knot.u);double t=knot.u-j;out.frameCoordinates.push_back(sourceArc[j]*(1-t)+sourceArc[j+1]*t);}
  for(unsigned j=1;j<knots.size();j++)if(knots[j].u-knots[j-1].u<1e-9)throw std::runtime_error("Duplicate compound rim parameter: "+std::to_string(knots[j-1].end)+","+std::to_string(knots[j].end)+" at "+std::to_string(knots[j].u)+" parents "+std::to_string(knots[j-1].parent)+","+std::to_string(knots[j].parent));
  // Source rim refinement uses exact original edges. Retained source vertices
  // and face-specific corner donors survive; no spatial nearest-neighbor bind.
  const unsigned sourceOffset=unsigned(out.rest.size());out.rest.insert(out.rest.end(),source.begin(),source.end());
  std::vector<unsigned> sourceParent(source.size(),~0u);std::vector<Vec> sourceWeight(source.size());
  for(unsigned fi=0;fi<r.sourceFaces.size();fi++)for(unsigned k=0;k<3;k++){auto v=r.sourceFaces[fi][k];sourceParent[v]=fi;sourceWeight[v]={};sourceWeight[v][k]=1;}
  for(unsigned v=0;v<source.size();v++)if(sourceParent[v]!=~0u)out.sourceSamples.push_back({sourceOffset+v,sourceParent[v],sourceWeight[v]});
  using Edge=std::pair<unsigned,unsigned>;std::map<Edge,std::vector<std::pair<double,unsigned>>> splits;
  std::vector<CompoundSurfaceDonor> starts,ends;std::vector<Vec> refSupport,first,last,patch(r.parameters.size()*knots.size());
  out.rings.resize(r.parameters.size(),std::vector<unsigned>(knots.size()));
  for(unsigned j=0;j<knots.size();j++){
   auto knot=knots[j];unsigned segment=unsigned(std::floor(knot.u));double t=knot.u-segment;
   auto a=r.sourceLoop[segment],b=r.sourceLoop[(segment+1)%r.sourceLoop.size()];
   unsigned parent=~0u;Vec w{};for(unsigned fi=0;fi<r.sourceFaces.size();fi++){auto f=r.sourceFaces[fi];bool aa=false,bb=false;for(auto v:f){aa|=v==a;bb|=v==b;}if(aa&&bb){parent=fi;for(unsigned k=0;k<3;k++)w[k]=f[k]==a?1-t:f[k]==b?t:0;break;}}
   if(parent==~0u)throw std::runtime_error("Compound source edge has no retained owner");
   unsigned id;
   if(t<1e-9)id=sourceOffset+a;
   else if(t>1-1e-9)id=sourceOffset+b;
   else{id=unsigned(out.rest.size());out.rest.push_back(CompoundContactCache::SamplePoint(source,r.sourceFaces[parent],w));out.sourceSamples.push_back({id,parent,w});splits[std::minmax(a,b)].push_back({a<b?t:1-t,id});}
   out.rings.front()[j]=id;out.rings.back()[j]=knot.end;
   out.sourceDifferentials.push_back({parent,segment,w,t});starts.push_back({{3*j,3*j+1,3*j+2},{1,0,0}});ends.push_back({{3*j,3*j+1,3*j+2},{1,0,0}});
   UV q{};auto liveFace=faces[knot.parent];for(unsigned k=0;k<3;k++)for(unsigned d=0;d<2;d++)q[d]+=chart[liveFace[k]][d]*knot.bary[k];
   auto ref=ReferenceAt(q);auto rf=r.supportFaces[ref.first];std::array<UV,3> rt{r.chart[rf[0]],r.chart[rf[1]],r.chart[rf[2]]};
   auto d=Differential(ref.first,ref.second,rt);auto p=CompoundContactCache::SamplePoint(r.support,rf,d.point);
   Vec du{},dv{};for(unsigned k=0;k<3;k++){du=Add(du,Scale(smoothReference[rf[k]][0],ref.second[k]));dv=Add(dv,Scale(smoothReference[rf[k]][1],ref.second[k]));}
   refSupport.push_back(p);refSupport.push_back(Add(p,du));refSupport.push_back(Add(p,dv));
   out.differentials.push_back(Differential(knot.parent,knot.bary,{chart[liveFace[0]],chart[liveFace[1]],chart[liveFace[2]]}));
   first.push_back(Field(r.firstTangents,0,knot.u));last.push_back(Field(r.lastTangents,0,knot.u));
   const auto start=CompoundContactCache::SamplePoint(r.source,r.sourceFaces[parent],w);
   auto da=Sub(start,Field(r.patch,0,knot.u)),db=Sub(p,Field(r.patch,unsigned(r.parameters.size()-1),knot.u));
   for(unsigned row=0;row<r.parameters.size();row++){double t=r.parameters[row],weight=t*t*t*(10+t*(-15+6*t));patch[row*knots.size()+j]=Add(Field(r.patch,row,knot.u),Add(Scale(da,1-weight),Scale(db,weight)));}
  }
  for(auto& item:splits)std::sort(item.second.begin(),item.second.end());
  for(unsigned fi=0;fi<r.sourceFaces.size();fi++){
   auto f=r.sourceFaces[fi];std::vector<CompoundContactCorner> boundary;
   for(unsigned k=0;k<3;k++){unsigned a=f[k],b=f[(k+1)%3];Vec w{};w[k]=1;boundary.push_back({sourceOffset+a,1,fi,w});auto found=splits.find(std::minmax(a,b));if(found==splits.end())continue;
    auto entries=found->second;if(a>b)std::reverse(entries.begin(),entries.end());
    for(auto entry:entries){double t=a<b?entry.first:1-entry.first;Vec w{};w[k]=1-t;w[(k+1)%3]=t;boundary.push_back({entry.second,1,fi,w});}
   }
   if(boundary.size()==3){out.faces.push_back({boundary[0],boundary[1],boundary[2]});continue;}
   Vec w{};for(auto c:boundary)w=Add(w,c.donor);w=Scale(w,1./boundary.size());unsigned id=unsigned(out.rest.size());out.rest.push_back(CompoundContactCache::SamplePoint(source,f,w));out.sourceSamples.push_back({id,fi,w});
   for(unsigned k=0;k<boundary.size();k++)out.faces.push_back({CompoundContactCorner{id,1,fi,w},boundary[k],boundary[(k+1)%boundary.size()]});
  }
  refSupport=out.StabilizeFrames(refSupport);
  out.graft=std::make_unique<CompoundGraftPlan>(out.SourceDifferentialPoints(r.source),refSupport,starts,ends,first,last,r.parameters,patch);
  // Freeze the valid control-sized material before applying transient motion.
  // Repeatedly transporting the canonical authoring patch would reintroduce
  // its large morphology strain on top of the settled connector correction.
  auto sourceFrame=out.SourceDifferentialPoints(source),supportFrame=out.DifferentialPoints(support);
  out.graft=std::make_unique<CompoundGraftPlan>(out.graft->Rebased(sourceFrame,supportFrame));
  auto livePatch=out.graft->Evaluate(sourceFrame,supportFrame);
  std::vector<Vec> entryNormals;for(unsigned j=0;j<knots.size();j++)entryNormals.push_back(Cross(Sub(sourceFrame[3*j+1],sourceFrame[3*j]),Sub(sourceFrame[3*j+2],sourceFrame[3*j])));
  out.graft->ConformSourceEntry(livePatch,r.parameters,entryNormals);
  if(suspensionHang>0)ShapeSuspensionStrip(livePatch,r.parameters,unsigned(knots.size()),1,suspensionHang,1,suspensionForward);
  for(unsigned row=1;row+1<out.rings.size();row++)for(unsigned j=0;j<knots.size();j++){out.rings[row][j]=unsigned(out.rest.size());out.rest.push_back(livePatch[row*knots.size()+j]);}
  // Match the directed source rim: its retained surface runs in sourceLoop
  // order, so the connecting strip traverses that seam in the opposite order.
  for(unsigned row=0;row+1<out.rings.size();row++)for(unsigned j=0;j<knots.size();j++){
   unsigned k=(j+1)%unsigned(knots.size());auto c=[&](unsigned y,unsigned x){return CompoundContactCorner{out.rings[y][x],2,0,{},knots[x].u,y};};
   out.faces.push_back({c(row,k),c(row,j),c(row+1,j)});out.faces.push_back({c(row,k),c(row+1,j),c(row+1,k)});
  }
  std::vector<bool> protect(out.rest.size());
  for(unsigned i=0;i<support.size();i++)protect[i]=protectedSupport.empty()?true:protectedSupport.at(i);
  for(unsigned i=0;i<source.size();i++)protect[sourceOffset+i]=true;
  double minStation=1e300,maxStation=-1e300;for(auto p:r.footprint){minStation=(std::min)(minStation,p[0]);maxStation=(std::max)(maxStation,p[0]);}
  for(auto knot:knots)if(std::abs(knot.u-std::round(knot.u))<1e-8){auto j=unsigned(std::round(knot.u))%r.footprint.size();if(r.protectedKnots.empty()?r.footprint[j][0]>=.5*(minStation+maxStation):r.protectedKnots.at(j))protect[knot.end]=true;}
  out.Clean(std::move(protect));
  return out;
 }
};
}
