#pragma once
#if defined(MALEMOD_GARMENT_DIAGNOSTIC) || defined(MALEMOD_GARMENT_CONTACT_TRACE)
#include <cstdio>
#endif
// Persistent world-space fabric. The measured fitter supplies rest material,
// never a replacement for the advanced particle positions.
namespace malemod::garments {
inline const Output& Session::InitializeDraped(const Input& reference,const Input& current){
 if(reference.characterEpoch!=current.characterEpoch||reference.topologyRevision!=current.topologyRevision||reference.restRevision!=current.restRevision||reference.bodyTriangles!=current.bodyTriangles||reference.anatomyTriangles!=current.anatomyTriangles)
  throw std::invalid_argument("Dressing reference and live pose have different material identities");
 auto place=[&](const Sample& source){
  auto result=source;result.position=current.frame.World(reference.frame.Local(source.position));
  auto n=reference.frame.Local(Add(reference.frame.origin,source.normal));
  result.normal=Unit(Add(Add(Mul(current.frame.lateral,n[0]),Mul(current.frame.forward,n[1])),Mul(current.frame.up,n[2])));
  return result;
 };
 auto aligned=reference;aligned.frame=current.frame;aligned.gravity={};
 for(auto* group:{&aligned.waist,&aligned.opening,&aligned.anatomy,&aligned.bodySurface,&aligned.rearStraps[0],&aligned.rearStraps[1]})for(auto& sample:*group)sample=place(sample);
 // Explicitly reject unimplemented capsule transforms instead of using
 // reference-space obstacles in an unrelated live coordinate frame.
 if(reference.bodySurface.empty()||current.bodySurface.empty())throw std::invalid_argument("Dressing requires measured body surfaces");
 aligned.bodyContacts.clear(); // measured triangles supersede legacy capsules
 try{
  Initialize(reference,aligned,place);
  constexpr unsigned dressingSteps=90;
  for(unsigned step=1;step<=dressingSteps;step++){
   auto pose=current;pose.gravity={};pose.deltaTime=1./120;pose.bodyContacts.clear();
   const double fraction=double(step)/dressingSteps;
   auto blend=[&](std::vector<Sample>& target,const std::vector<Sample>& source){
    if(target.size()!=source.size())throw std::invalid_argument("Dressing surface topology changed");
    for(unsigned i=0;i<target.size();i++){
     target[i].position=Add(Mul(source[i].position,1-fraction),Mul(target[i].position,fraction));
     auto normal=Add(Mul(source[i].normal,1-fraction),Mul(target[i].normal,fraction));
     target[i].normal=Length(normal)>1e-12?Unit(normal):target[i].normal;
    }
   };
   blend(pose.waist,aligned.waist);blend(pose.opening,aligned.opening);
   blend(pose.anatomy,aligned.anatomy);blend(pose.bodySurface,aligned.bodySurface);
   for(unsigned side=0;side<2;side++)blend(pose.rearStraps[side],aligned.rearStraps[side]);
   Update(Style::WhiteJockstrap,pose,TimeContinuity::Continuous);
  }
  if(!output_.contactBudgetSatisfied||!output_.physics.materialBudgetSatisfied)throw std::invalid_argument("Dressing did not reach contact/material equilibrium");
  // Dressing is preparation, not elapsed gameplay or a kick to the character.
  std::fill(velocities_.begin(),velocities_.end(),Point{});clock_=accumulator_=0;
  output_.reactions.clear();output_.support.clear();output_.reaction={};
  output_.physics.advancedSeconds=output_.physics.accumulatedSeconds=0;
  output_.physics.substeps=0;return output_;
 }catch(...){Reset();throw;}
}
inline const Output& Session::Initialize(const Input& reference,const Input& current,const std::function<Sample(const Sample&)>& place){
 if(!parameters_.simulate||!place)throw std::invalid_argument("Cloth initialization requires persistent simulation and placement");
 if(reference.characterEpoch!=current.characterEpoch||reference.topologyRevision!=current.topologyRevision||reference.restRevision!=current.restRevision||reference.bodyTriangles!=current.bodyTriangles||reference.anatomyTriangles!=current.anatomyTriangles)
  throw std::invalid_argument("Cloth reference and current pose have different material identities");
 for(unsigned family=0;family<5;family++){
  const auto& a=family==0?reference.waist:family==1?reference.opening:family==2?reference.anatomy:reference.rearStraps[family-3];
  const auto& b=family==0?current.waist:family==1?current.opening:family==2?current.anatomy:current.rearStraps[family-3];
  if(a.size()!=b.size())throw std::invalid_argument("Cloth reference and current pose have different surface families");
 }
 if(reference.bodySurface.size()!=current.bodySurface.size())throw std::invalid_argument("Cloth reference and current body have different topology");
 Reset();
 try{
  auto rest=reference;rest.deltaTime=0;rest.gravity={};Update(Style::WhiteJockstrap,rest);
  // A finite-element render director and its sewn joints can require a short
  // equilibrium solve after the fitted sheet becomes physical material.
  // Keep its rest metric fixed and run the real constraints/contact oracle;
  // never publish an unverified initial penetration or change rest lengths.
  for(unsigned relax=0;relax<32&&(!output_.contactBudgetSatisfied||!output_.physics.materialBudgetSatisfied);relax++){rest.deltaTime=1./120;Update(Style::WhiteJockstrap,rest,TimeContinuity::Continuous);}
  if(!output_.contactBudgetSatisfied||!output_.physics.materialBudgetSatisfied)throw std::invalid_argument("Cloth reference material did not reach contact/material equilibrium");
  if(referenceNodes_.size()!=positions_.size())throw std::logic_error("Cloth reference particle ownership missing");
  for(unsigned i=0;i<positions_.size();i++){
   auto source=referenceNodes_[i];source.position=positions_[i];
   auto posed=place(source);
   if(!Finite(posed.position)||!Finite(posed.normal)||Length(posed.normal)<1e-12)throw std::invalid_argument("Cloth placement produced a nonfinite or degenerate sample");
   for(unsigned k=0;k<posed.lineage.donors.size();k++){const auto& a=posed.lineage.donors[k];const auto& b=referenceNodes_[i].lineage.donors[k];if(a.surface!=b.surface||a.vertex!=b.vertex||a.weight!=b.weight)throw std::invalid_argument("Cloth placement changed material lineage");}
   positions_[i]=posed.position;velocities_[i]={};
   auto target=referenceNodes_[i];target.position=previousTargets_[i];previousTargets_[i]=place(target).position;
   if(!Finite(previousTargets_[i]))throw std::invalid_argument("Cloth placement produced a nonfinite anchor");
  }
  restInput_=current;previousBodySurface_=current.bodySurface;previousAnatomySurface_=current.anatomy;previousBodies_=current.bodyContacts;
  for(auto& memos:pointMemos_)for(auto& memo:memos)memo=PointMemo{};
  for(auto& memos:faceMemos_)for(auto& memo:memos)memo=FaceMemo{};
  ClassifySurfaces(current,current.frame.origin,output_.measuredCircumference,true);
  clock_=accumulator_=0;auto initial=current;initial.deltaTime=0;
  return Update(Style::WhiteJockstrap,initial,TimeContinuity::Continuous);
 }catch(...){Reset();throw;}
}
namespace cloth_detail {
inline Point Barycentric(Point p,Point a,Point b,Point c){
 auto v=Sub(b,a),w=Sub(c,a),r=Sub(p,a);double vv=Dot(v,v),vw=Dot(v,w),ww=Dot(w,w),rv=Dot(r,v),rw=Dot(r,w),d=vv*ww-vw*vw;
 if(std::abs(d)<1e-24)return {1,0,0};double y=(ww*rv-vw*rw)/d,z=(vv*rw-vw*rv)/d;return {1-y-z,y,z};
}
inline const Sample& SampleAt(const Input& in,unsigned family,unsigned index){
 return family==0?in.waist.at(index):family==1?in.opening.at(index):family==2?in.anatomy.at(index):in.rearStraps.at(family-3).at(index);
}
struct Envelope {
 Frame frame;Point center{},radius{};unsigned rings,segments;double gap,C;
 std::vector<double> extent;
 Envelope(const Input& in,double circumference,const Parameters& p):frame(in.frame),rings(p.pouchRings),segments(p.pouchSegments),gap(p.clearance*circumference),C(circumference),extent((rings+1)*segments){
  Point lo{1e100,1e100,1e100},hi{-1e100,-1e100,-1e100};for(auto s:in.anatomy){auto q=frame.Local(s.position);for(unsigned k=0;k<3;k++){lo[k]=(std::min)(lo[k],q[k]);hi[k]=(std::max)(hi[k],q[k]);}}
  double y=0;for(auto s:in.opening)y+=frame.Local(s.position)[1];y=(std::min)(y/in.opening.size(),lo[1]-gap);
  center={(lo[0]+hi[0])*.5,y,(lo[2]+hi[2])*.5};radius={(std::max)((hi[0]-lo[0])*.5+gap,gap*2),(std::max)(hi[1]-y+gap,gap*2),(std::max)((hi[2]-lo[2])*.5+gap,gap*2)};
  for(auto s:in.anatomy){auto q=Coordinates(frame.Local(s.position));unsigned row=(std::min)(rings-1,unsigned(q[0])),col=unsigned(q[1])%segments;for(unsigned j=row;j<=row+1;j++)for(unsigned k=0;k<2;k++)extent[j*segments+(col+k)%segments]=(std::max)(extent[j*segments+(col+k)%segments],q[2]);}
  // Conservative radial obstacle, not a force pulling fabric onto every valley.
  for(unsigned pass=0;pass<rings+segments/2;pass++){auto next=extent;bool changed=false;for(unsigned y=0;y<=rings;y++)for(unsigned x=0;x<segments;x++){unsigned at=y*segments+x;if(extent[at]>0)continue;double total=0;unsigned count=0;for(auto k:{y*segments+(x+segments-1)%segments,y*segments+(x+1)%segments,(y?y-1:y)*segments+x,(std::min)(y+1,rings)*segments+x})if(extent[k]>0){total+=extent[k];count++;}if(count){next[at]=total/count;changed=true;}}extent.swap(next);if(!changed)break;}
  detail::TensionEnvelope(extent,rings,segments);
 }
 Point Coordinates(Point p)const{p=Sub(p,center);Point q{p[0]/radius[0],p[1]/radius[1],p[2]/radius[2]};double d=Length(q),theta=std::atan2(q[2],q[0]);if(theta<0)theta+=2*detail::pi;return {std::acos(std::clamp(q[1]/(std::max)(d,1e-20),0.,1.))/(detail::pi*.5)*rings,theta/(2*detail::pi)*segments,d};}
 double Minimum(Point p)const{auto c=Coordinates(p);unsigned y=(std::min)(rings-1,unsigned(c[0])),x=unsigned(c[1])%segments;double fy=c[0]-y,fx=c[1]-std::floor(c[1]);double e=(1-fy)*((1-fx)*extent[y*segments+x]+fx*extent[y*segments+(x+1)%segments])+fy*((1-fx)*extent[(y+1)*segments+x]+fx*extent[(y+1)*segments+(x+1)%segments]);double chord=1/std::cos(std::sqrt(std::pow(detail::pi/(2*rings),2)+std::pow(2*detail::pi/segments,2)));return e*chord+gap*c[2]/(std::max)(Length(Sub(p,center)),gap);}
 std::pair<double,Point> ValueGradient(Point point)const{
  auto p=Sub(point,center),c=Coordinates(point);double D=(std::max)(c[2],1e-14),L=(std::max)(Length(p),gap),chord=1/std::cos(std::sqrt(std::pow(detail::pi/(2*rings),2)+std::pow(2*detail::pi/segments,2)));unsigned y=(std::min)(rings-1,unsigned(c[0])),x=unsigned(c[1])%segments;double fy=c[0]-y,fx=c[1]-std::floor(c[1]),a=extent[y*segments+x],b=extent[y*segments+(x+1)%segments],d=extent[(y+1)*segments+x],e=extent[(y+1)*segments+(x+1)%segments];double value=(1-fy)*((1-fx)*a+fx*b)+fy*((1-fx)*d+fx*e),rowDerivative=(1-fx)*(d-a)+fx*(e-b),colDerivative=(1-fy)*(b-a)+fy*(e-d);
  Point gradient{},gradD{};for(unsigned k=0;k<3;k++)gradD[k]=p[k]/(radius[k]*radius[k]*D);double u=p[1]/radius[1]/D,sine=std::sqrt((std::max)(1e-16,1-u*u)),qx=p[0]/radius[0],qz=p[2]/radius[2],azimuth=(std::max)(1e-16,qx*qx+qz*qz);
  for(unsigned k=0;k<3;k++){double phi=u>=0&&u<1?-((k==1?1/radius[1]:0)/D-u/D*gradD[k])/sine:0;double theta=k==0?-qz/(azimuth*radius[0]):k==2?qx/(azimuth*radius[2]):0;gradient[k]=gradD[k]*(1-gap/L)+D*gap*p[k]/(L*L*L)-chord*(rowDerivative*phi*(2*rings/detail::pi)+colDerivative*theta*(segments/(2*detail::pi)));}
  return {D*(1-gap/L)-value*chord,gradient};
 }
 bool Project(Point& world)const{auto p=frame.Local(world);auto evaluation=ValueGradient(p);const double guard=C*1e-4;if(evaluation.first>=guard*Length(evaluation.second))return false;for(unsigned iteration=0;iteration<3;iteration++){double norm=Dot(evaluation.second,evaluation.second),f=evaluation.first-guard*std::sqrt(norm);if(f>=0)break;if(norm<1e-20)break;p=Add(p,Mul(evaluation.second,(-f+1e-10)/norm));evaluation=ValueGradient(p);}auto q=Sub(p,center),c=Coordinates(p);double minimum=Minimum(p)+guard*c[2]/(std::max)(Length(q),gap);if(c[2]<minimum&&c[2]>1e-14)p=Add(center,Mul(q,minimum/c[2]));world=frame.World(p);return true;}
};
}
inline const Output& Session::Update(Style style,const Input& input,TimeContinuity time){
 if(!parameters_.simulate)return Fit(style,input);
 if(style==Style::Naked){Reset();output_.characterEpoch=input.characterEpoch;output_.topologyRevision=input.topologyRevision;return output_;}
 if(style!=Style::WhiteJockstrap)throw std::invalid_argument("Unknown garment style");
 input.frame.Validate();detail::ValidateSamples(input.waist,8,128);detail::ValidateSamples(input.opening,8,128);detail::ValidateSamples(input.anatomy,4,131072);for(const auto& path:input.rearStraps)detail::ValidateSamples(path,3,128);
 if(input.bodyContacts.size()>64||!Finite(input.gravity)||!std::isfinite(input.deltaTime)||input.deltaTime<0||!std::isfinite(input.anatomyMass)||input.anatomyMass<0)throw std::invalid_argument("Invalid cloth contact/timing/mass input");
 for(auto c:input.bodyContacts)if(!Finite(c.a)||!Finite(c.b)||!std::isfinite(c.radius)||c.radius<0)throw std::invalid_argument("Invalid cloth body volume");
 for(auto f:input.anatomyTriangles)for(auto id:f)if(id>=input.anatomy.size())throw std::invalid_argument("Cloth anatomy index outside surface");
 const bool discontinuity=input.deltaTime>1&&time!=TimeContinuity::Continuous;
 bool reset=positions_.empty()||input.characterEpoch!=restInput_.characterEpoch||input.topologyRevision!=restInput_.topologyRevision||discontinuity;
 if(!reset&&(input.waist.size()!=restInput_.waist.size()||input.opening.size()!=restInput_.opening.size()||input.anatomy.size()!=restInput_.anatomy.size()||input.rearStraps[0].size()!=restInput_.rearStraps[0].size()||input.rearStraps[1].size()!=restInput_.rearStraps[1].size()))reset=true;
 if(!reset&&output_.measuredCircumference>0&&Length(Sub(input.waist.front().position,previousTargets_.front()))>2*output_.measuredCircumference)reset=true;
 if(reset)Reset();
 bool rebuild=restMesh_.vertices.empty()||input.restRevision!=restInput_.restRevision;
 double C=0;for(unsigned i=0;i<input.waist.size();i++)C+=Length(Sub(input.waist[i].position,input.waist[(i+1)%input.waist.size()].position));
 if(!std::isfinite(C)||C<1e-12)throw std::invalid_argument("Degenerate cloth circumference");
 // Rest changes are explicit morphology revisions; animated skin contours
 // must not recreate the material or replace its persistent rest lengths.
 if(rebuild){auto fittedInput=input;fittedInput.deltaTime=0;Fit(style,fittedInput);
  // The accepted fitted material is the rest configuration. Reparameterizing
  // it after the fitter would invalidate its tested contacts and sewn aliases.
  restMesh_=output_.mesh;restInput_=input;}
 output_.physics={};output_.physics.reset=reset;output_.physics.resetCount=resetCount_;output_.style=style;output_.characterEpoch=input.characterEpoch;output_.topologyRevision=input.topologyRevision;output_.support.clear();output_.reactions.clear();output_.reaction={};output_.projectedContacts=0;output_.contactBudgetSatisfied=true;output_.mesh=restMesh_;
 Cloth(input,rebuild,discontinuity?0:input.deltaTime);
 // Material refits preserve displacement and velocity where topology remains
 // compatible; this branch is handled inside Cloth before integration.
 return output_;
}
inline void Session::Cloth(const Input& input,bool rebuild,double elapsed){
 using namespace cloth_detail;
 auto begin=std::chrono::steady_clock::now();auto& mesh=output_.mesh;double C=output_.measuredCircumference;const bool sheetMesh=output_.layout.revision==2;const unsigned waist=sheetMesh?output_.layout.band.columns:output_.band.topAttachments?output_.band.topAttachments:unsigned(input.waist.size()),band=14*(waist+1),rings=parameters_.pouchRings,segments=parameters_.pouchSegments,pouchEnd=sheetMesh?output_.layout.sheet.start+(rings+1)*(segments+1):band+1+rings*(segments+1),hemEnd=sheetMesh?pouchEnd:pouchEnd+4*(segments+1),joinEnd=sheetMesh?pouchEnd:hemEnd+2*(segments+1),strap=sheetMesh?output_.layout.straps[0].start:joinEnd+9*25;
 if(rebuild){
  auto oldPositions=positions_,oldVelocities=velocities_,oldTargets=previousTargets_;auto oldNodes=vertexNodes_;double oldC=restCircumference_;
  vertexNodes_.assign(mesh.vertices.size(),unsigned(-1));renderBindings_.assign(mesh.vertices.size(),Binding{});solverTriangles_.clear();positions_.clear();referenceNodes_.clear();velocities_.clear();anchors_.clear();pinned_.clear();edges_.clear();sewing_.clear();previousTargets_.clear();
  // Physical aliases share a particle, independent of material/UV duplication.
  std::map<std::array<long long,3>,unsigned> aliases;
  std::map<unsigned,unsigned> anatomyDonors;for(unsigned i=0;i<input.anatomy.size();i++)for(auto d:input.anatomy[i].lineage.donors)if(d.weight>.99999&&d.surface==Surface::Anatomy)anatomyDonors[d.vertex]=i;
  std::map<unsigned,unsigned> bodyDonors;for(unsigned i=0;i<input.bodySurface.size();i++)for(auto donor:input.bodySurface[i].lineage.donors)if(donor.weight>.99999&&donor.surface==Surface::Body)bodyDonors[donor.vertex]=i;
  auto pinVertex=[&](unsigned i){if(!sheetMesh&&i>=joinEnd&&i<joinEnd+25)return false;if(i>=band)return false;if(input.bodySurface.empty())return true;unsigned row=(i/(waist+1))%7;auto p=input.frame.Local(mesh.vertices[i].position);double front=p[1];return row==6||front<=0;};
  for(unsigned i=0;i<mesh.vertices.size();i++){
   if(!sheetMesh&&i>=pouchEnd&&i<hemEnd)continue; // reinforcement is sewn to the simulated rim
   if(!sheetMesh&&i>=hemEnd+segments+1&&i<joinEnd)continue; // opening's front row is the same sewn rim
   if(!sheetMesh&&i>=joinEnd+8*25&&i<strap)continue; // panel ends share the continuous pouch stitch
   if(i>=band/2&&i<band)continue; // inner and outer layers share the band midsurface
   if(sheetMesh&&i>=output_.layout.sideHems[0].start)continue;
   if(i>=strap&&(i-strap)%4)continue; // elastic ribbons have centerline particles
   if(sheetMesh&&i>=band&&i<pouchEnd){unsigned row=(i-band)/(segments+1),col=(i-band)%(segments+1);if(!cloth_detail::SheetParticle(row,col,rings))continue;}
   if(!sheetMesh&&i>band&&i<pouchEnd){unsigned row=(i-band-1)/(segments+1)+1,col=(i-band-1)%(segments+1);if(row<rings&&(row%2||col%2))continue;}
   Point world=mesh.vertices[i].position;if(i<band/2)world=band_material::Midsurface(world,mesh.vertices[i+band/2].position);if(i>=strap){world={};for(unsigned k=0;k<4;k++)world=Add(world,mesh.vertices[i+k].position);world=Mul(world,.25);}auto p=input.frame.Local(world);std::array<long long,3> key{std::llround(p[0]/C*1e9),std::llround(p[1]/C*1e9),std::llround(p[2]/C*1e9)};auto found=aliases.find(key);
   if(found!=aliases.end()){vertexNodes_[i]=found->second;pinned_[found->second]=pinned_[found->second]||pinVertex(i);continue;}
   unsigned id=unsigned(positions_.size());aliases[key]=id;vertexNodes_[i]=id;positions_.push_back(world);referenceNodes_.push_back({world,mesh.vertices[i].normal,mesh.vertices[i].lineage});velocities_.push_back({});pinned_.push_back(pinVertex(i));
   Anchor anchor;double best=1e100;bool anatomical=mesh.vertices[i].lineage.donors[0].surface==Surface::Anatomy;
   if(anatomical){auto dominant=mesh.vertices[i].lineage.donors[0];auto donor=anatomyDonors.find(dominant.vertex);if(donor!=anatomyDonors.end()){anchor.family=2;anchor.index=donor->second;best=0;}}
   if(best>0){for(unsigned family=0;family<5;family++){if(family==2)continue;const auto& samples=family==0?input.waist:family==1?input.opening:input.rearStraps[family-3];for(unsigned k=0;k<samples.size();k++){double d=Length(Sub(mesh.vertices[i].position,samples[k].position));if(d<best){best=d;anchor.family=family;anchor.index=k;}}}}
   if(i<band&&!input.bodySurface.empty()){Point attached{};double sum=0;for(unsigned k=0;k<16;k++){auto donor=mesh.vertices[i].lineage.donors[k];if(donor.weight==0)continue;const auto& map=donor.surface==Surface::Body?bodyDonors:anatomyDonors;auto found=map.find(donor.vertex);if(found==map.end())throw std::invalid_argument("Band measured body/anatomy attachment missing");anchor.bodyIndices[k]=found->second;anchor.bodyWeights[k]=donor.weight;anchor.attachmentSurfaces[k]=donor.surface;const auto& surface=donor.surface==Surface::Body?input.bodySurface:input.anatomy;attached=Add(attached,Mul(surface[found->second].position,donor.weight));sum+=donor.weight;}if(std::abs(sum-1)>1e-5)throw std::invalid_argument("Band attachment weights invalid");Point normal{};for(unsigned k=0;k<16;k++)if(anchor.bodyWeights[k]){const auto& surface=anchor.attachmentSurfaces[k]==Surface::Body?input.bodySurface:input.anatomy;normal=Add(normal,Mul(surface[anchor.bodyIndices[k]].normal,anchor.bodyWeights[k]));}anchor.referenceNormal=Unit(input.frame.Local(Add(input.frame.origin,normal)));anchor.bodyAttachment=true;anchor.residual=Sub(p,input.frame.Local(attached));}else anchor.residual=Sub(p,input.frame.Local(SampleAt(input,anchor.family,anchor.index).position));anchors_.push_back(anchor);
  }
  auto sheetMaterialFaces=sheetMesh?cloth_detail::SheetTriangles(band,rings,segments):std::vector<std::array<unsigned,3>>{};
  for(unsigned i=0;i<mesh.vertices.size();i++){
   auto& binding=renderBindings_[i];if(i>=band/2&&i<band)vertexNodes_[i]=vertexNodes_[i-band/2];if(vertexNodes_[i]!=unsigned(-1)){binding.nodes[0]=vertexNodes_[i];binding.weights[0]=1;binding.residual=Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(positions_[binding.nodes[0]]));continue;}
   if(!sheetMesh&&i>=pouchEnd&&i<hemEnd){unsigned rim=band+1+(rings-1)*(segments+1)+(i-pouchEnd)/4;binding=renderBindings_[rim];binding.residual=Add(binding.residual,Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(mesh.vertices[rim].position)));vertexNodes_[i]=vertexNodes_[rim];continue;}
   if(!sheetMesh&&i>=hemEnd+segments+1&&i<joinEnd){unsigned rim=band+1+(rings-1)*(segments+1)+i-(hemEnd+segments+1);binding=renderBindings_[rim];vertexNodes_[i]=vertexNodes_[rim];continue;}
   if(!sheetMesh&&i>=joinEnd+8*25&&i<strap){unsigned col=i-(joinEnd+8*25);double at=(1./6+double(col)/24/6)*segments;unsigned a=unsigned(at)%segments;double t=at-std::floor(at);unsigned rim=band+1+(rings-1)*(segments+1)+a;for(unsigned end=0;end<2;end++){const auto& donor=renderBindings_[rim+end];double fraction=end?t:1-t;binding.residual=Add(binding.residual,Mul(donor.residual,fraction));for(unsigned k=0;k<4;k++){double w=donor.weights[k]*fraction;if(std::abs(w)<1e-14)continue;unsigned slot=0;while(slot<4&&binding.weights[slot]!=0&&binding.nodes[slot]!=donor.nodes[k])slot++;if(slot==4)throw std::logic_error("Panel sewing exceeds bounded material donors");binding.nodes[slot]=donor.nodes[k];binding.weights[slot]+=w;}}unsigned dominant=0;for(unsigned k=1;k<4;k++)if(binding.weights[k]>binding.weights[dominant])dominant=k;vertexNodes_[i]=binding.nodes[dominant];continue;}
   if(sheetMesh&&i>=output_.layout.sideHems[0].start){unsigned side=i>=output_.layout.sideHems[1].start?1:0;const auto& hem=output_.layout.sideHems[side];unsigned section=(i-hem.start)/4,edge=output_.layout.sideBoundary[side][section];binding=renderBindings_[edge];binding.residual=Add(binding.residual,Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(mesh.vertices[edge].position)));vertexNodes_[i]=vertexNodes_[edge];continue;}
   if(i>=strap){unsigned first=i-(i-strap)%4;binding=renderBindings_[first];binding.residual=Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(positions_[binding.nodes[0]]));vertexNodes_[i]=vertexNodes_[first];continue;}
   if(sheetMesh){Point coordinate{double((i-band)%(segments+1)),double((i-band)/(segments+1)),0};bool found=false;for(auto f:sheetMaterialFaces){std::array<Point,3> uv;for(unsigned k=0;k<3;k++)uv[k]={double((f[k]-band)%(segments+1)),double((f[k]-band)/(segments+1)),0};auto w=Barycentric(coordinate,uv[0],uv[1],uv[2]);if(w[0]<-1e-10||w[1]<-1e-10||w[2]<-1e-10)continue;Point interpolated{};unsigned dominant=0;for(unsigned k=0;k<3;k++){binding.nodes[k]=vertexNodes_[f[k]];binding.weights[k]=w[k];interpolated=Add(interpolated,Mul(positions_.at(binding.nodes[k]),w[k]));if(w[k]>w[dominant])dominant=k;}binding.residual=Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(interpolated));vertexNodes_[i]=binding.nodes[dominant];found=true;break;}if(!found)throw std::logic_error("Material sheet point has no physical finite element");continue;}
   unsigned row=sheetMesh?(i-band)/(segments+1):(i-band-1)/(segments+1)+1,col=sheetMesh?(i-band)%(segments+1):(i-band-1)%(segments+1),lo=row/2*2,hi=(std::min)(rings,lo+2),left=col/2*2,right=sheetMesh?(std::min)(segments,left+2):(left+2)%segments;double y=hi>lo?double(row-lo)/(hi-lo):0,x=double(col-left)/2;
   auto control=[&](unsigned r,unsigned c){return vertexNodes_[sheetMesh?band+r*(segments+1)+c:r?band+1+(r-1)*(segments+1)+c:band];};
   binding.nodes={control(lo,left),control(lo,right),control(hi,left),control(hi,right)};binding.weights={(1-y)*(1-x),(1-y)*x,y*(1-x),y*x};for(unsigned a=0;a<4;a++)for(unsigned b=a+1;b<4;b++)if(binding.nodes[a]==binding.nodes[b]){binding.weights[a]+=binding.weights[b];binding.weights[b]=0;}Point interpolated{};unsigned dominant=0;for(unsigned k=0;k<4;k++){interpolated=Add(interpolated,Mul(positions_.at(binding.nodes[k]),binding.weights[k]));if(binding.weights[k]>binding.weights[dominant])dominant=k;}binding.residual=Sub(input.frame.Local(mesh.vertices[i].position),input.frame.Local(interpolated));vertexNodes_[i]=binding.nodes[dominant];
  }
  // Fine pouch curvature follows its coarse material frame. A fixed pelvis
  // offset would change edge cancellation when the cloth bends, manufacturing
  // apparent fine-surface strain even when the material cell rotates rigidly.
  for(unsigned i=band+1;i<pouchEnd;i++){auto& binding=renderBindings_[i];std::vector<unsigned> nodes;for(unsigned k=0;k<(sheetMesh?3u:4u);k++)if((sheetMesh||binding.weights[k])&&std::find(nodes.begin(),nodes.end(),binding.nodes[k])==nodes.end())nodes.push_back(binding.nodes[k]);if(nodes.size()<3||Length(binding.residual)<1e-15)continue;auto a=positions_[nodes[0]],u=Sub(positions_[nodes[1]],a),second=Sub(positions_[nodes[2]],a),n=Cross(u,second);if(Length(u)<C*1e-12||Length(n)<C*C*1e-12)continue;auto world=Sub(input.frame.World(binding.residual),input.frame.origin);double uu=Dot(u,u),uv=Dot(u,second),vv=Dot(second,second),det=uu*vv-uv*uv;if(det<=C*C*C*C*1e-24)continue;double ru=Dot(world,u),rv=Dot(world,second);n=Unit(n);binding.materialFrame={nodes[0],nodes[1],nodes[2]};binding.materialResidual={(vv*ru-uv*rv)/det,(uu*rv-uv*ru)/det,Dot(world,n)};binding.transported=true;}
  for(unsigned i=0;i<band;i++)band_material::Director(renderBindings_[i],i,waist,vertexNodes_,positions_,input.frame,C);
  for(unsigned offset=strap,route=0;route<2;route++){unsigned count=(std::max)(33u,unsigned(input.rearStraps[route].size())*4+1);for(unsigned k=0;k<count;k++){unsigned previous=offset+4*(k?k-1:k),next=offset+4*(std::min)(count-1,k+1);auto tangent=Sub(input.frame.Local(positions_[vertexNodes_[next]]),input.frame.Local(positions_[vertexNodes_[previous]]));for(unsigned corner=0;corner<4;corner++){auto& b=renderBindings_[offset+4*k+corner];b.ribbon=true;b.materialFrame={vertexNodes_[previous],vertexNodes_[next],0};b.restTangent=Unit(tangent);}}offset+=4*count;}
  if(sheetMesh)for(unsigned side=0;side<2;side++){const auto& hem=output_.layout.sideHems[side];const auto& boundary=output_.layout.sideBoundary[side];for(unsigned section=0;section<boundary.size();section++){unsigned before=boundary[section?section-1:section],after=boundary[(std::min)(unsigned(boundary.size()-1),section+1)];if(vertexNodes_[before]==vertexNodes_[after]){before=boundary[section>1?section-2:0];after=boundary[(std::min)(unsigned(boundary.size()-1),section+2)];}auto tangent=Sub(input.frame.Local(mesh.vertices[after].position),input.frame.Local(mesh.vertices[before].position));for(unsigned k=0;k<4;k++){auto& b=renderBindings_[hem.start+section*4+k];b.ribbon=true;b.materialFrame={vertexNodes_[before],vertexNodes_[after],0};b.restTangent=Unit(tangent);}}}
  previousTargets_=positions_;
  std::map<std::pair<unsigned,unsigned>,std::vector<unsigned>> adjacency;
  std::set<std::array<unsigned,3>> uniqueFaces;
  auto addMaterialTriangle=[&](std::array<unsigned,3> ids){auto sorted=ids;std::sort(sorted.begin(),sorted.end());if(sorted[0]==sorted[1]||sorted[1]==sorted[2]||!uniqueFaces.insert(sorted).second)return;solverTriangles_.push_back(ids);for(unsigned k=0;k<3;k++){unsigned a=ids[k],b=ids[(k+1)%3],c=ids[(k+2)%3];if(a>b)std::swap(a,b);adjacency[{a,b}].push_back(c);}};
  for(unsigned fi=0;fi<mesh.triangles.size();fi++){if(sheetMesh&&fi>=output_.layout.sheetFaces.start&&fi<output_.layout.sheetFaces.start+output_.layout.sheetFaces.count)continue;auto face=mesh.triangles[fi];addMaterialTriangle({vertexNodes_[face.vertices[0]],vertexNodes_[face.vertices[1]],vertexNodes_[face.vertices[2]]});}
  for(auto f:sheetMaterialFaces)addMaterialTriangle({vertexNodes_[f[0]],vertexNodes_[f[1]],vertexNodes_[f[2]]});
  inverseMass_.assign(positions_.size(),0);double totalArea=0;unsigned freeNodes=0;for(auto face:mesh.triangles){double area=Length(Cross(Sub(mesh.vertices[face.vertices[1]].position,mesh.vertices[face.vertices[0]].position),Sub(mesh.vertices[face.vertices[2]].position,mesh.vertices[face.vertices[0]].position)))/(6*C*C);for(auto vertex:face.vertices){const auto& binding=renderBindings_[vertex];for(unsigned k=0;k<4;k++)inverseMass_[binding.nodes[k]]+=area*binding.weights[k];}}for(unsigned i=0;i<inverseMass_.size();i++)if(!pinned_[i]){totalArea+=inverseMass_[i];freeNodes++;}double meanArea=totalArea/(std::max)(1u,freeNodes);
  clothMass_=input.anatomyMass>0?input.anatomyMass*parameters_.mechanics.massFraction:double(freeNodes);
  const double materialMassScale=double(freeNodes)/(std::max)(clothMass_,1e-12);
  for(unsigned i=0;i<inverseMass_.size();i++)inverseMass_[i]=pinned_[i]?0:materialMassScale*meanArea/(std::max)(inverseMass_[i],1e-12);
  std::set<std::pair<unsigned,unsigned>> elasticSides;
  if(sheetMesh)for(const auto& side:output_.layout.sideBoundary)for(unsigned k=1;k<side.size();k++){unsigned a=vertexNodes_[side[k-1]],b=vertexNodes_[side[k]];if(a>b)std::swap(a,b);elasticSides.insert({a,b});}
  std::set<std::pair<unsigned,unsigned>> bendPairs;
  for(const auto& row:adjacency){auto a=row.first.first,b=row.first.second;double distance=Length(Sub(positions_[a],positions_[b]))/C;if(distance>1e-9&&(!pinned_[a]||!pinned_[b]))edges_.push_back({a,b,distance,parameters_.mechanics.stretchCompliance*distance*distance*(elasticSides.count(row.first)?parameters_.mechanics.hemComplianceMultiplier:1.),0,false});if(row.second.size()==2){a=row.second[0];b=row.second[1];if(a>b)std::swap(a,b);if(a!=b&&(!pinned_[a]||!pinned_[b])&&bendPairs.insert({a,b}).second){distance=Length(Sub(positions_[a],positions_[b]))/C;if(distance>1e-9)edges_.push_back({a,b,distance,parameters_.mechanics.bendCompliance*distance*distance,0,true});}}}
  if(sheetMesh)for(const auto& side:output_.layout.sideBoundary)for(unsigned k=2;k<side.size();k++){unsigned a=vertexNodes_[side[k-2]],b=vertexNodes_[side[k]];if(a>b)std::swap(a,b);if(a!=b&&(!pinned_[a]||!pinned_[b])&&bendPairs.insert({a,b}).second){double distance=Length(Sub(positions_[a],positions_[b]))/C;if(distance>1e-9)edges_.push_back({a,b,distance,parameters_.mechanics.bendCompliance*parameters_.mechanics.hemComplianceMultiplier*distance*distance,0,true});}}
  for(unsigned offset=strap,route=0;route<2;route++){unsigned count=(std::max)(33u,unsigned(input.rearStraps[route].size())*4+1);for(unsigned k=1;k<count;k++){unsigned a=vertexNodes_[offset+4*(k-1)],b=vertexNodes_[offset+4*k];double d=Length(Sub(positions_[a],positions_[b]))/C;edges_.push_back({a,b,d,parameters_.mechanics.stretchCompliance*d*d,0,false});if(k>1){a=vertexNodes_[offset+4*(k-2)];d=Length(Sub(positions_[a],positions_[b]))/C;edges_.push_back({a,b,d,parameters_.mechanics.bendCompliance*d*d,0,true});}}offset+=4*count;}
  // Long-range attachment constraints bound material extension without forcing
  // free fabric onto a skin-shaped target. They still permit tangential motion.
  std::vector<std::vector<std::pair<unsigned,double>>> materialGraph(positions_.size());for(auto e:edges_)if(!e.bend){materialGraph[e.a].push_back({e.b,e.rest});materialGraph[e.b].push_back({e.a,e.rest});}std::vector<double> geodesic(positions_.size(),1e100);std::vector<unsigned> anchor(positions_.size());using Route=std::pair<double,unsigned>;std::priority_queue<Route,std::vector<Route>,std::greater<Route>> queue;for(unsigned i=0;i<positions_.size();i++)if(pinned_[i]){geodesic[i]=0;anchor[i]=i;queue.push({0,i});}while(!queue.empty()){auto current=queue.top();queue.pop();if(current.first>geodesic[current.second])continue;for(auto link:materialGraph[current.second]){double distance=current.first+link.second;if(distance<geodesic[link.first]){geodesic[link.first]=distance;anchor[link.first]=anchor[current.second];queue.push({distance,link.first});}}}for(unsigned i=0;i<positions_.size();i++)if(!pinned_[i]&&geodesic[i]<1e100)edges_.push_back({i,anchor[i],geodesic[i]*1.02,0,0,false,true});
  auto term=[&](Sew& s,unsigned vertex,double coefficient){const auto& binding=renderBindings_[vertex];s.residual=Add(s.residual,Mul(binding.residual,coefficient));for(unsigned j=0;j<4;j++){double weight=coefficient*binding.weights[j];if(std::abs(weight)<1e-14)continue;unsigned node=binding.nodes[j];bool present=false;for(unsigned k=0;k<s.count;k++)if(s.nodes[k]==node){s.weights[k]+=weight;present=true;}if(!present){s.nodes[s.count]=node;s.weights[s.count++]=weight;}}};
  auto appendSew=[&](Sew s){
   // Preserve the accepted separation between finite-thickness material layers.
   // Forcing closest sewn points to coincide would pull cloth into the skin.
   Point initial=s.residual;for(unsigned k=0;k<s.count;k++)initial=Add(initial,Mul(input.frame.Local(positions_[s.nodes[k]]),s.weights[k]));s.residual=Sub(s.residual,initial);sewing_.push_back(s);
  };
  auto sew=[&](std::initializer_list<std::pair<unsigned,double>> terms){Sew s;for(auto t:terms)term(s,t.first,t.second);appendSew(s);};
  // Hem cross sections are sewn at their material centers to the pouch rim.
  if(!sheetMesh)for(unsigned k=0;k<=segments;k++)sew({{pouchEnd+4*k,.25},{pouchEnd+4*k+1,.25},{pouchEnd+4*k+2,.25},{pouchEnd+4*k+3,.25},{band+1+(rings-1)*(segments+1)+k,-1}});
  // The broad upper panel is physically sewn to the current elastic band,
  // not independently prescribed skin points or an unattached fitted shell.
  for(unsigned col=0;col<(sheetMesh?output_.layout.topSeam.size():25u);col++){unsigned vertex=sheetMesh?output_.layout.topSeam[col]:joinEnd+col;auto p=mesh.vertices[vertex].position;double best=1e100;unsigned triangle=0;Point closest{};for(unsigned j=0;j<28*waist;j++){auto f=mesh.triangles[j];auto q=detail::ClosestTriangle(p,mesh.vertices[f.vertices[0]].position,mesh.vertices[f.vertices[1]].position,mesh.vertices[f.vertices[2]].position);double d=Dot(Sub(p,q),Sub(p,q));if(d<best){best=d;triangle=j;closest=q;}}auto face=mesh.triangles[triangle];auto weights=Barycentric(closest,mesh.vertices[face.vertices[0]].position,mesh.vertices[face.vertices[1]].position,mesh.vertices[face.vertices[2]].position);Sew s;term(s,vertex,1);for(unsigned k=0;k<3;k++)term(s,face.vertices[k],-weights[k]);appendSew(s);}
  // Panel bottom and ring have the same continuous material stitch.
  if(!sheetMesh)for(unsigned col=0;col<=24;col++){double at=(1./6+double(col)/24/6)*segments;unsigned a=unsigned(at)%segments;double t=at-std::floor(at);sew({{joinEnd+8*25+col,1},{band+1+(rings-1)*(segments+1)+a,-(1-t)},{band+1+(rings-1)*(segments+1)+a+1,-t}});}
  unsigned offset=strap;
  for(unsigned route=0;route<2;route++){unsigned count=(std::max)(33u,unsigned(input.rearStraps[route].size())*4+1);for(unsigned end=0;end<2;end++){
   if(sheetMesh&&end&&output_.layout.jointRevision){const auto name="strap-"+std::to_string(route)+"-bottom";auto joint=std::find_if(output_.layout.authoredJoints.begin(),output_.layout.authoredJoints.end(),[&](const auto& j){return j.name==name;});if(joint==output_.layout.authoredJoints.end())throw std::invalid_argument("Measured strap lacks its authored underside stitch");Sew s;for(unsigned k=0;k<joint->a.vertices.size();k++)term(s,joint->a.vertices[k],joint->a.weights.at(k));for(unsigned k=0;k<joint->b.vertices.size();k++)term(s,joint->b.vertices[k],-joint->b.weights.at(k));s.residual=Sub(s.residual,input.frame.Local(Add(input.frame.origin,joint->restOffset)));sewing_.push_back(s);continue;}
   unsigned cap=offset+(end?count-1:0)*4;
   double best=1e100;std::array<unsigned,3> source{},target{};Point weightsA{},weightsB{};
   unsigned targetStart=end?(sheetMesh?output_.layout.sheetFaces.start+(rings-1)*segments*2:28*waist+segments+(rings-1)*segments*2):0,targetFinish=end?(sheetMesh?output_.layout.sheetFaces.start+output_.layout.sheetFaces.count:targetStart+segments*8):28*waist;
   for(unsigned j=targetStart;j<targetFinish;j++)for(auto corners:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{0,2,3}}){std::array<Point,3> a,b;for(unsigned k=0;k<3;k++){a[k]=mesh.vertices[cap+corners[k]].position;b[k]=mesh.vertices[mesh.triangles[j].vertices[k]].position;}Point pa,pb;double d=detail::ClosestTriangles(a,b,pa,pb);if(d<best){best=d;source={cap+corners[0],cap+corners[1],cap+corners[2]};target=mesh.triangles[j].vertices;weightsA=Barycentric(pa,a[0],a[1],a[2]);weightsB=Barycentric(pb,b[0],b[1],b[2]);}}
   Sew s;for(unsigned k=0;k<3;k++){term(s,source[k],weightsA[k]);term(s,target[k],-weightsB[k]);}appendSew(s);
  }offset+=count*4;}
  // Weak bending modes are solved before structural modes. Within each
  // family, solve longer material spans before shorter, stiffer features;
  // otherwise the final long edge can undo the tiny-edge extension bound.
  std::stable_sort(edges_.begin(),edges_.end(),[](const Edge&a,const Edge&b){if(a.bend!=b.bend)return a.bend>b.bend;if(a.tether!=b.tether)return a.tether<b.tether;return a.rest>b.rest;});
  stretchAdjacency_=cloth_stretch::Adjacency(unsigned(positions_.size()),edges_);
  // Rest lengths above belong to freshly fitted material, not old dynamic strain.
  if(oldNodes.size()==vertexNodes_.size()&&oldC>0){std::vector<bool> copied(positions_.size());for(unsigned i=0;i<vertexNodes_.size();i++){unsigned next=vertexNodes_[i],old=oldNodes[i];if(copied[next]||old>=oldPositions.size())continue;copied[next]=true;if(!pinned_[next]){positions_[next]=Add(positions_[next],Mul(Sub(oldPositions[old],oldTargets[old]),C/oldC));velocities_[next]=Mul(oldVelocities[old],C/oldC);}}}
  // All numerical material masses/compliances share this explicit mass unit.
  // Preserve selected softness while adopting an actual source-mass scale.
  for(auto& edge:edges_)edge.compliance*=materialMassScale;
  restCircumference_=C;
 }
 if(input.anatomyMass>0){const double desired=input.anatomyMass*parameters_.mechanics.massFraction;if(clothMass_>0&&desired!=clothMass_){const double scale=clothMass_/desired;for(auto& mass:inverseMass_)mass*=scale;for(auto& edge:edges_)edge.compliance*=scale;clothMass_=desired;}}
 std::vector<Point> targets(positions_.size());
 for(unsigned i=0;i<targets.size();i++){
  const auto& a=anchors_[i];Point sample{},normal{},offset=a.residual;
  if(a.bodyAttachment){
   for(unsigned k=0;k<16;k++)if(a.bodyWeights[k]){
    const auto& surface=a.attachmentSurfaces[k]==Surface::Body?input.bodySurface:input.anatomy;
    const auto& donor=surface.at(a.bodyIndices[k]);
    sample=Add(sample,Mul(donor.position,a.bodyWeights[k]));
    normal=Add(normal,Mul(donor.normal,a.bodyWeights[k]));
   }
   normal=input.frame.Local(Add(input.frame.origin,normal));
   // Small finite-thickness reserve for faces spanning curved skin between
   // measured anchors. Full face contact validation remains authoritative.
   offset=anchor_transport::Offset(offset,a.referenceNormal,normal,C*parameters_.bandThickness/12);
  }else sample=SampleAt(input,a.family,a.index).position;
  targets[i]=input.frame.World(Add(input.frame.Local(sample),offset));
 }
 if(input.deltaTime==0){for(unsigned i=0;i<positions_.size();i++)if(pinned_[i])positions_[i]=targets[i];}
 auto oldTargets=previousTargets_;previousTargets_=targets;
 auto& telemetry=output_.physics;telemetry.active=true;telemetry.stateReady=true;telemetry.nodes=unsigned(positions_.size());telemetry.constraints=unsigned(edges_.size()+sewing_.size());
 Envelope envelope(input,C,parameters_);
 const bool measuredAnatomy=!input.anatomyTriangles.empty();
 const Point origin=input.frame.origin;
 std::vector<Point> contactImpulse(positions_.size()),tissueCorrection(positions_.size());
 ReactionCollector reactions(input.anatomy);double reactionStep=0;
 const std::vector<Sample>* reactionSurface=&input.anatomy;
 std::vector<Point> x(positions_.size()),v(positions_.size());for(unsigned i=0;i<x.size();i++){x[i]=Mul(Sub(positions_[i],origin),1/C);v[i]=Mul(velocities_[i],1/C);targets[i]=Mul(Sub(targets[i],origin),1/C);oldTargets[i]=Mul(Sub(oldTargets[i],origin),1/C);}
 std::vector<bool> covered(x.size());for(unsigned i=band;i<pouchEnd;i++)covered[vertexNodes_[i]]=true;
 std::vector<Capsule> volumes=input.bodySurface.empty()?input.bodyContacts:std::vector<Capsule>{},oldVolumes=previousBodies_;previousBodies_=volumes;if(oldVolumes.size()!=volumes.size())oldVolumes=volumes;for(auto* list:{&volumes,&oldVolumes})for(auto& c:*list){c.a=Mul(Sub(c.a,origin),1/C);c.b=Mul(Sub(c.b,origin),1/C);c.radius/=C;}auto finalVolumes=volumes;
 auto previousBody=previousBodySurface_;previousBodySurface_=input.bodySurface;if(previousBody.size()!=input.bodySurface.size())previousBody=input.bodySurface;bodyCollider_.Update(input.bodySurface,input.bodyTriangles,origin,C);
 auto previousAnatomy=previousAnatomySurface_;previousAnatomySurface_=input.anatomy;if(previousAnatomy.size()!=input.anatomy.size())previousAnatomy=input.anatomy;if(measuredAnatomy)anatomyCollider_.Update(input.anatomy,input.anatomyTriangles,origin,C);else anatomyCollider_.Clear();ClassifySurfaces(input,origin,C);
 double margin=parameters_.clearance*.3;
 if(rebuild)for(unsigned family=0;family<2;family++){pointMemos_[family].assign(x.size()+mesh.vertices.size(),PointMemo{});faceMemos_[family].assign(mesh.triangles.size(),FaceMemo{});}
 // Exactly coincident physical/render aliases share a conservative point
 // certificate. Offsets and blended elements retain independent query slots.
 auto pointSlot=[&](unsigned vertex){const auto& binding=renderBindings_[vertex];
  if(!binding.ribbon&&!binding.transported&&binding.residual==Point{}&&binding.weights[0]==1&&binding.weights[1]==0&&binding.weights[2]==0&&binding.weights[3]==0)return binding.nodes[0];
  return unsigned(x.size())+vertex;
 };
 // Certificates cover the closed classification surface, including its measured
 // root cap. The physical clearance oracle remains the uncapped native surface.
 auto pointQuery=[&](bool anatomical,unsigned slot,Point point,double clearance){
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
  auto& physical=anatomical?anatomyCollider_:bodyCollider_;auto& closed=anatomical?closedAnatomy_:closedBody_;auto& classifier=closed.Empty()?physical:closed;auto& memo=faceMemos_[anatomical][index];auto stamp=classifier.MotionStamp();detail::BodyCollider::Hit hit;
  if(memo.certificate.ProvesClear(points,clearance,stamp)){hit.distance=hit.signedDistance=memo.certificate.LowerBound(points,stamp);return hit;}
  hit=physical.ClosestFaceCached(points,clearance,memo.physicalNeighborhood,memo.physicalSeed);if(hit.triangle!=UINT32_MAX)memo.physicalSeed=hit.triangle;double radius=clearance+.01;auto classification=classifier.ClosestFaceCached(points,radius,memo.closedNeighborhood,memo.closedSeed);if(classification.triangle!=UINT32_MAX)memo.closedSeed=classification.triangle;bool outside=true;
  for(unsigned k=0;k<3;k++)if(pointQuery(anatomical,pointSlot(mesh.triangles[index].vertices[k]),points[k],0).signedDistance<0)outside=false;
  memo.certificate.Remember(points,(std::min)(classification.distance,radius),outside,stamp);if(outside){hit.signedDistance=hit.distance;auto delta=Sub(hit.clothPoint,hit.point);if(hit.distance<clearance&&Length(delta)>1e-14)hit.normal=Unit(delta);}return hit;
 };
 auto residual=[&](Point local){return Mul(Sub(input.frame.World(local),origin),1/C);};
 // Render donors are shared by many contact faces. Reuse their exact evaluated
 // positions until a constraint changes particles; never reuse across a solve.
 std::vector<Point> renderPositions(mesh.vertices.size());
 std::vector<std::uint64_t> renderStamps(mesh.vertices.size());std::uint64_t renderRevision=1;
 auto invalidateRendered=[&](){if(++renderRevision==0){std::fill(renderStamps.begin(),renderStamps.end(),0);renderRevision=1;}};
 auto rendered=[&](unsigned index){if(renderStamps[index]!=renderRevision){renderPositions[index]=render_contact::Evaluate(renderBindings_[index],x,input.frame,C);renderStamps[index]=renderRevision;}return renderPositions[index];};
 struct ParticleApplication {Point point{};double weight=0;bool gradient=false;Point impulseCoefficient{},momentCoefficient{};};
 struct ContactProjection {double inverseMass=0;ParticleApplication particles;};
 auto projectContact=[&](const render_contact::Gradient& gradient,double distance,bool tissue){
  ContactProjection result;result.inverseMass=gradient.InverseMass(inverseMass_);
  if(result.inverseMass>1e-20){invalidateRendered();auto applied=render_contact::Application(gradient,x,inverseMass_);result.particles.gradient=true;result.particles.impulseCoefficient=applied.impulse;result.particles.momentCoefficient=applied.moment;
   for(unsigned k=0;k<gradient.count;k++)if(inverseMass_[gradient.nodes[k]]){auto change=Mul(gradient.values[k],distance*inverseMass_[gradient.nodes[k]]/result.inverseMass);auto node=gradient.nodes[k];x[node]=Add(x[node],change);contactImpulse[node]=Add(contactImpulse[node],change);if(tissue)tissueCorrection[node]=Add(tissueCorrection[node],change);}
  }return result;
 };
 auto projectBinding=[&](unsigned index,Point correction,bool tissue=false){double distance=Length(correction);if(distance<1e-20)return ContactProjection{};return projectContact(render_contact::Derivative(renderBindings_[index],x,input.frame,C,Mul(correction,1/distance)),distance,tissue);};
 auto faceGradient=[&](const Triangle& face,Point bary,Point direction){render_contact::Gradient result;for(unsigned k=0;k<3;k++)if(std::abs(bary[k])>1e-14)render_contact::Merge(result,render_contact::Derivative(renderBindings_[face.vertices[k]],x,input.frame,C,direction),bary[k]);return result;};
 auto recordReaction=[&](const detail::BodyCollider::Hit& hit,Point correction,double inverseEffectiveMass,ParticleApplication particles){
  if(reactionStep<=0||input.anatomyMass<=0||inverseEffectiveMass<=1e-20||Length(correction)<=1e-15)return;
  if(hit.triangle>=input.anatomyTriangles.size())throw std::invalid_argument("Reaction lacks physical anatomy triangle");
  auto face=input.anatomyTriangles[hit.triangle];std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=Mul(Sub(reactionSurface->at(face[k]).position,origin),1/C);
  const auto bary=Barycentric(hit.point,points[0],points[1],points[2]);
  const auto pointImpulse=Mul(correction,C/(inverseEffectiveMass*reactionStep));
  const auto particleImpulse=particles.gradient?Mul(particles.impulseCoefficient,Length(correction)*C/(inverseEffectiveMass*reactionStep)):Mul(correction,C*particles.weight/(inverseEffectiveMass*reactionStep));
  const auto relativeParticleMoment=particles.gradient?Mul(particles.momentCoefficient,Length(correction)*C*C/(inverseEffectiveMass*reactionStep)):Mul(Cross(particles.point,correction),C*C/(inverseEffectiveMass*reactionStep));
  reactions.Add(hit.triangle,bary,pointImpulse,Add(origin,Mul(hit.point,C)),hit.signedDistance*C,input.anatomyTriangles,particleImpulse,Add(relativeParticleMoment,Cross(origin,particleImpulse)));
 };
 auto solveSewing=[&](){for(const auto& s:sewing_){Point error=residual(s.residual);double sum=0;for(unsigned k=0;k<s.count;k++){error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));sum+=inverseMass_[s.nodes[k]]*s.weights[k]*s.weights[k];}if(sum>1e-20)for(unsigned k=0;k<s.count;k++)if(inverseMass_[s.nodes[k]])x[s.nodes[k]]=Sub(x[s.nodes[k]],Mul(error,inverseMass_[s.nodes[k]]*s.weights[k]/sum));}};
 auto compatibleDirection=[&](Point point,Point direction,bool front){if(!front||measuredAnatomy)return direction;auto local=input.frame.Local(Add(origin,Mul(point,C)));auto gradient=envelope.ValueGradient(local).second;auto n=Add(Add(Mul(input.frame.lateral,gradient[0]),Mul(input.frame.forward,gradient[1])),Mul(input.frame.up,gradient[2]));double magnitude=Dot(n,n),against=Dot(direction,n);if(magnitude>1e-20&&against<0){auto tangent=Sub(direction,Mul(n,against/magnitude));double cosine=Dot(tangent,direction);if(cosine>1e-4)return Mul(tangent,1/cosine);}return direction;};
 auto projectVolume=[&](Point& point,const Capsule& volume,bool front){auto projected=point;if(!detail::Project(projected,volume,margin+1e-5,input.frame.forward))return false;auto correction=Sub(projected,point);double d=Length(correction);point=Add(point,Mul(compatibleDirection(point,Mul(correction,1/d),front),d));return true;};
 struct ActiveContact {unsigned clothFace,physicalFace;bool anatomy;Point normal;};
 std::vector<ActiveContact> activeContacts;std::unordered_map<std::uint64_t,unsigned> activeIndex;const Input* contactGeometry=&input;
 auto rememberContact=[&](bool anatomical,unsigned clothFace,const detail::BodyCollider::Hit& hit){
  if(hit.triangle==UINT32_MAX)return;
  const auto key=(std::uint64_t(anatomical)<<63)|(std::uint64_t(clothFace)<<32)|hit.triangle;
  auto found=activeIndex.find(key);
  if(found!=activeIndex.end()){auto& c=activeContacts[found->second];if(Finite(hit.normal)&&Length(hit.normal)>1e-14&&Dot(hit.normal,c.normal)>=0)c.normal=hit.normal;return;}
  activeIndex.emplace(key,unsigned(activeContacts.size()));
  activeContacts.push_back({clothFace,hit.triangle,anatomical,hit.normal});
 };
 auto solveActiveContacts=[&](){
  invalidateRendered();
  double maximumAppliedDepth=0;
  for(const auto& contact:activeContacts){const auto& face=mesh.triangles[contact.clothFace];const auto& physical=contact.anatomy?anatomyCollider_:bodyCollider_;
   const auto& source=contact.anatomy?contactGeometry->anatomy:contactGeometry->bodySurface;const auto& physicalFaces=contact.anatomy?input.anatomyTriangles:input.bodyTriangles;
   // Exact pair query uses the current interpolated physical triangle.
   // It never substitutes an infinite plane for a finite source face.
   auto tri=physicalFaces[contact.physicalFace];std::array<Point,3> cloth{rendered(face.vertices[0]),rendered(face.vertices[1]),rendered(face.vertices[2])},body;
   for(unsigned k=0;k<3;k++)body[k]=Mul(Sub(source[tri[k]].position,origin),1/C);
   Point p,q;double d=detail::ClosestTriangles(cloth,body,p,q);if(d>=margin+5e-5)continue;
   detail::BodyCollider::Hit hit{q,contact.normal,d,d,contact.physicalFace,p};
   Point delta=Sub(hit.clothPoint,hit.point);double distance=Dot(delta,contact.normal);if(distance>=margin+1e-5)continue;hit.normal=contact.normal;hit.signedDistance=distance;
   auto bary=Barycentric(hit.clothPoint,cloth[0],cloth[1],cloth[2]);const double depth=margin-distance+1e-5;
   auto applied=projectContact(faceGradient(face,bary,contact.normal),depth,contact.anatomy);
   if(applied.inverseMass>1e-20)maximumAppliedDepth=(std::max)(maximumAppliedDepth,depth);
   if(contact.anatomy)recordReaction(hit,Mul(contact.normal,depth),applied.inverseMass,applied.particles);
  }
  // Maximum scalar gap corrected during this pass, in normalized source
  // coordinates. Later pairs may disturb earlier ones: only a subsequent
  // zero-correction pass demonstrates local convergence. This is not the
  // exhaustive publication contact/material certificate.
  return maximumAppliedDepth;
 };
 auto collision=[&](bool triangles){
  invalidateRendered();
  for(unsigned i=0;i<x.size();i++)if(!pinned_[i]){auto before=x[i];if(covered[i]&&!measuredAnatomy){auto world=Add(origin,Mul(x[i],C));if(envelope.Project(world)){x[i]=Mul(Sub(world,origin),1/C);output_.projectedContacts++;}}for(auto volume:volumes)if(projectVolume(x[i],volume,covered[i]))output_.projectedContacts++;if(measuredAnatomy){auto hit=pointQuery(true,i,x[i],margin+5e-6);if(hit.signedDistance<margin+1e-5){auto correction=Mul(hit.normal,margin-hit.signedDistance+1e-5);x[i]=Add(x[i],correction);tissueCorrection[i]=Add(tissueCorrection[i],correction);recordReaction(hit,correction,inverseMass_[i],{x[i],1});output_.projectedContacts++;}}if(!bodyCollider_.Empty()){auto hit=pointQuery(false,i,x[i],margin+5e-6);if(hit.signedDistance<margin+1e-5){x[i]=Add(x[i],Mul(compatibleDirection(x[i],hit.normal,covered[i]),margin-hit.signedDistance+1e-5));output_.projectedContacts++;}}contactImpulse[i]=Add(contactImpulse[i],Sub(x[i],before));}
  invalidateRendered(); // point/capsule projections above changed particles
  if(triangles&&!measuredAnatomy)for(unsigned pass=0;pass<2;pass++)for(unsigned i=band;i<pouchEnd;i++){auto point=rendered(i),world=Add(origin,Mul(point,C));if(envelope.Project(world)){projectBinding(i,Sub(Mul(Sub(world,origin),1/C),point));output_.projectedContacts++;}}
  if(triangles){
   for(unsigned i=band;i<mesh.vertices.size();i++)for(auto volume:volumes){auto point=rendered(i),projected=point;if(projectVolume(projected,volume,i>=band&&i<hemEnd)){projectBinding(i,Sub(projected,point));output_.projectedContacts++;}}
   for(auto* collider:{&bodyCollider_,&anatomyCollider_})if(!collider->Empty()){
    const bool openAnatomy=collider==&anatomyCollider_;
    for(unsigned i=0;i<mesh.vertices.size();i++){auto point=rendered(i);auto hit=pointQuery(openAnatomy,pointSlot(i),point,margin+5e-6);if(hit.signedDistance<margin+1e-5){auto correction=Mul(compatibleDirection(point,hit.normal,i>=band&&i<hemEnd),margin-hit.signedDistance+1e-5);auto applied=projectBinding(i,correction,openAnatomy);if(openAnatomy)recordReaction(hit,correction,applied.inverseMass,applied.particles);output_.projectedContacts++;}}
    for(unsigned faceIndex=0;faceIndex<mesh.triangles.size();faceIndex++){
     const auto& face=mesh.triangles[faceIndex];std::array<Point,3> corners{rendered(face.vertices[0]),rendered(face.vertices[1]),rendered(face.vertices[2])};auto hit=faceQuery(openAnatomy,faceIndex,corners,margin+5e-6);if(hit.signedDistance>=margin+1e-5)continue;rememberContact(openAnatomy,faceIndex,hit);
     auto bary=Barycentric(hit.clothPoint,corners[0],corners[1],corners[2]);auto direction=compatibleDirection(hit.clothPoint,hit.normal,face.vertices[0]>=band&&face.vertices[0]<hemEnd);double length=Length(direction);if(length<1e-20)continue;const double depth=(margin-hit.signedDistance+1e-5)*length;direction=Mul(direction,1/length);
     auto applied=projectContact(faceGradient(face,bary,direction),depth,openAnatomy);if(openAnatomy)recordReaction(hit,Mul(direction,depth),applied.inverseMass,applied.particles);if(applied.inverseMass>1e-20)output_.projectedContacts++;
    }
   }
   for(auto face:mesh.triangles)for(auto volume:volumes){
    std::array<Point,3> corners{rendered(face.vertices[0]),rendered(face.vertices[1]),rendered(face.vertices[2])};Point p,q;double distance=detail::TriangleCapsule(corners[0],corners[1],corners[2],volume,p,q,margin);if(distance>=margin-1e-10)continue;auto bary=Barycentric(p,corners[0],corners[1],corners[2]);auto direction=Sub(p,q);direction=Length(direction)>1e-12?Unit(direction):input.frame.forward;direction=compatibleDirection(p,direction,face.vertices[0]>=band&&face.vertices[0]<hemEnd);double length=Length(direction);if(length>1e-20)projectContact(faceGradient(face,bary,Mul(direction,1/length)),(margin-distance+1e-5)*length,false);
   }
  }
 };
 // Recheck the whole rendered physical surface after shared-donor corrections.
 // Exact query certificates may prove distant faces clear, but an exhausted
 // certificate must rediscover its actual closest physical face. Newly active
 // pairs join the same unilateral manifold; no surface is omitted from this
 // convergence check and no new material rest state is generated.
 auto contactResidual=[&](){double maximum=0;
  invalidateRendered();
  for(bool anatomical:{false,true}){auto& collider=anatomical?anatomyCollider_:bodyCollider_;if(collider.Empty())continue;
   for(unsigned i=0;i<mesh.vertices.size();i++){auto hit=pointQuery(anatomical,pointSlot(i),rendered(i),margin);maximum=(std::max)(maximum,margin-hit.signedDistance);}
   for(unsigned i=0;i<mesh.triangles.size();i++){const auto& face=mesh.triangles[i];std::array<Point,3> points{rendered(face.vertices[0]),rendered(face.vertices[1]),rendered(face.vertices[2])};auto hit=faceQuery(anatomical,i,points,margin);maximum=(std::max)(maximum,margin-hit.signedDistance);if(hit.signedDistance<margin-1e-8)rememberContact(anatomical,i,hit);}
  }
  for(const auto& face:mesh.triangles)for(const auto& volume:volumes){Point p,q;double distance=detail::TriangleCapsule(rendered(face.vertices[0]),rendered(face.vertices[1]),rendered(face.vertices[2]),volume,p,q,margin);maximum=(std::max)(maximum,margin-distance);}
  return maximum;
 };
 auto seamResidual=[&](){double maximum=0;for(const auto& s:sewing_){auto error=residual(s.residual);for(unsigned k=0;k<s.count;k++)error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));maximum=(std::max)(maximum,Length(error));}return maximum;};
 constexpr double step=1./120;accumulator_+=elapsed;if(!std::isfinite(accumulator_))throw std::invalid_argument("Cloth active time accumulator overflow");unsigned steps=unsigned((std::min)(120.,std::floor((accumulator_+1e-12)/step)));telemetry.substeps=steps;
 // Preserve all geometry/donors but clone interpolation storage only once per
 // call. A single final-pose substep can read the supplied immutable input.
 std::optional<Input> interpolation;if(steps>1)interpolation=input;
 for(unsigned substep=0;substep<steps;substep++){
  reactionStep=step;
  std::fill(contactImpulse.begin(),contactImpulse.end(),Point{});std::fill(tissueCorrection.begin(),tissueCorrection.end(),Point{});activeContacts.clear();activeIndex.clear();
  double fraction=double(substep+1)/steps;std::vector<Point> subTargets(targets.size());for(unsigned i=0;i<targets.size();i++)subTargets[i]=Add(Mul(oldTargets[i],1-fraction),Mul(targets[i],fraction));for(unsigned k=0;k<volumes.size();k++){volumes[k].a=Add(Mul(oldVolumes[k].a,1-fraction),Mul(finalVolumes[k].a,fraction));volumes[k].b=Add(Mul(oldVolumes[k].b,1-fraction),Mul(finalVolumes[k].b,fraction));volumes[k].radius=oldVolumes[k].radius*(1-fraction)+finalVolumes[k].radius*fraction;}
  if(interpolation){for(unsigned i=0;i<interpolation->bodySurface.size();i++)interpolation->bodySurface[i].position=Add(Mul(previousBody[i].position,1-fraction),Mul(input.bodySurface[i].position,fraction));for(unsigned i=0;i<interpolation->anatomy.size();i++)interpolation->anatomy[i].position=Add(Mul(previousAnatomy[i].position,1-fraction),Mul(input.anatomy[i].position,fraction));}
  const auto& contactInput=interpolation?*interpolation:input;
  reactionSurface=&contactInput.anatomy;
  contactGeometry=&contactInput;bodyCollider_.Update(contactInput.bodySurface,input.bodyTriangles,origin,C);if(measuredAnatomy)anatomyCollider_.Update(contactInput.anatomy,input.anatomyTriangles,origin,C);ClassifySurfaces(contactInput,origin,C);
  auto old=x;double damp=std::exp(-parameters_.mechanics.dampingRate*step);for(unsigned i=0;i<x.size();i++){if(pinned_[i])x[i]=subTargets[i];else{v[i]=Add(Mul(v[i],damp),Mul(input.gravity,step/C));x[i]=Add(x[i],Mul(v[i],step));}}
  if(measuredAnatomy)for(unsigned i=0;i<x.size();i++)if(!pinned_[i]){auto hit=anatomyCollider_.Sweep(old[i],x[i]);if(hit.distance<1e99){auto before=x[i];x[i]=Add(hit.point,Mul(hit.normal,margin+1e-5));recordReaction(hit,Sub(x[i],before),inverseMass_[i],{x[i],1});tissueCorrection[i]=Add(tissueCorrection[i],Sub(x[i],before));contactImpulse[i]=Add(contactImpulse[i],Sub(x[i],old[i]));output_.projectedContacts++;}}
  for(auto& e:edges_)e.lambda=0;
  // XPBD lambda is local to a substep. Triangle diagonals provide shear;
  // opposite-triangle distance constraints provide material bending resistance.
  const unsigned solveIterations=12u;

#ifdef MALEMOD_GARMENT_DIAGNOSTIC
  auto traceMaterial=[&](const char* label,unsigned iteration){if(clock_<.08||clock_>.13||x.size()<=769)return;for(auto e:edges_)if(!e.bend&&!e.tether&&(e.a==577&&e.b==642||e.a==642&&e.b==577)){auto bodyA=bodyCollider_.Closest(x[e.a]),bodyB=bodyCollider_.Closest(x[e.b]);std::fprintf(stderr,"TRACE %s %u ratio %.9g rest %.9g pinned %d,%d A %.9g %.9g %.9g B %.9g %.9g %.9g body %.9g %.9g invMass %.9g %.9g bodyTri %u %u bodyNormalA %.9g %.9g %.9g bodyNormalB %.9g %.9g %.9g\n",label,iteration,Length(Sub(x[e.a],x[e.b]))/e.rest,e.rest*C,int(pinned_[e.a]),int(pinned_[e.b]),x[e.a][0]*C,x[e.a][1]*C,x[e.a][2]*C,x[e.b][0]*C,x[e.b][1]*C,x[e.b][2]*C,bodyA.distance*C,bodyB.distance*C,inverseMass_[e.a],inverseMass_[e.b],bodyA.triangle,bodyB.triangle,bodyA.normal[0],bodyA.normal[1],bodyA.normal[2],bodyB.normal[0],bodyB.normal[1],bodyB.normal[2]);}};
#endif
  for(unsigned iteration=0;iteration<solveIterations;iteration++){
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   traceMaterial("start",iteration);
#endif

   for(unsigned k=0;k<edges_.size();k++){auto& e=edges_[k];auto delta=Sub(x[e.a],x[e.b]);double distance=Length(delta),wa=inverseMass_[e.a],wb=inverseMass_[e.b],alpha=e.compliance/(step*step);if(distance<1e-14||wa+wb==0||e.tether&&distance<=e.rest)continue;double dl=(-(distance-e.rest)-alpha*e.lambda)/(wa+wb+alpha);e.lambda+=dl;auto correction=Mul(delta,dl/distance);if(wa)x[e.a]=Add(x[e.a],Mul(correction,wa));if(wb)x[e.b]=Sub(x[e.b],Mul(correction,wb));}

#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   if(clock_==0){double d=0;unsigned id=0;for(unsigned k=0;k<x.size();k++)if(Length(Sub(x[k],old[k]))>d){d=Length(Sub(x[k],old[k]));id=k;}std::fprintf(stderr,"iter %u preSew max %g node %u\n",iteration,d*C,id);}
#endif
   // Elastic extension limit is an explicit inequality constraint, independent
   // of the stricter published material/contact diagnostic. It does not alter
   // rest lengths or hide conflicts with collision/sewing.
   // The coupled projection follows guides so stitched endpoints and adjacent
   // fabric obey their constraints together rather than undoing each other.

#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   traceMaterial("material",iteration);
#endif
   // Free waistband rows obey elastic material, stitches and skin contact.
   // A near-rigid guide on every row conflicts with those constraints during
   // waist flexion; only prescribed attachment rows follow their skin targets.
   if(iteration+1==solveIterations){auto projected=cloth_stretch::ProjectCoupled(x,inverseMass_,edges_,stretchAdjacency_,sewing_,residual,parameters_.mechanics.extensionLimit);
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
    std::fprintf(stderr,"coupled projection clock %.9g visits %u exhausted %d\n",clock_,projected.visits,int(projected.exhausted));
#endif
   }else solveSewing();
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   traceMaterial("sewn",iteration);
#endif
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   if(clock_==0&&x.size()>2601&&iteration+1==solveIterations)std::fprintf(stderr,"material precontact edge2600 dist %g restpos %g xA %g %g %g xB %g %g %g\n",Length(Sub(x[2600],x[2601]))*C,Length(Sub(old[2600],old[2601]))*C,x[2600][0]*C,x[2600][1]*C,x[2600][2]*C,x[2601][0]*C,x[2601][1]*C,x[2601][2]*C);
#endif
   if(iteration==0||(iteration+1)%(solveIterations/3)==0)collision(iteration==0||iteration+1==solveIterations);
   // A complete face sweep can move a shared material donor after an earlier
   // face was cleared. Revisit the retained physical manifold before velocity
   // reconstruction; these are integrated unilateral projections, not a new fit.
   for(unsigned pass=0;pass<(iteration+1==solveIterations?64u:1u);pass++)if(solveActiveContacts()<1e-10)break;
   if(iteration+1==solveIterations)for(unsigned discovery=0;discovery<8;discovery++){if(contactResidual()<=1e-8&&seamResidual()<=1e-6)break;solveSewing();collision(true);for(unsigned pass=0;pass<64;pass++)if(solveActiveContacts()<1e-10)break;}
   for(unsigned i=0;i<x.size();i++)if(pinned_[i])x[i]=subTargets[i];
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
   traceMaterial("contact",iteration);
#endif
  }
  auto obstacleVelocity=[&](const detail::BodyCollider::Hit& hit,bool anatomy){
   const auto& faces=anatomy?input.anatomyTriangles:input.bodyTriangles;const auto& now=anatomy?input.anatomy:input.bodySurface;const auto& previous=anatomy?previousAnatomy:previousBody;
   if(hit.triangle>=faces.size())throw std::invalid_argument("Contact velocity lacks exact physical triangle");
   auto tri=faces[hit.triangle];std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=Mul(Sub((anatomy?contactInput.anatomy:contactInput.bodySurface)[tri[k]].position,origin),1/C);
   auto bary=Barycentric(hit.point,points[0],points[1],points[2]);Point velocity{};
   for(unsigned k=0;k<3;k++)velocity=Add(velocity,Mul(Sub(now[tri[k]].position,previous[tri[k]].position),bary[k]/(C*steps*step)));
   return velocity;
  };
  for(unsigned i=0;i<x.size();i++){
   v[i]=Mul(Sub(x[i],old[i]),1/step);if(pinned_[i]||Length(contactImpulse[i])<=1e-14)continue;
   if(measuredAnatomy&&Length(tissueCorrection[i])>1e-14){
    auto hit=pointQuery(true,i,x[i],margin+5e-5);
    if(hit.triangle<input.anatomyTriangles.size()&&hit.signedDistance<=margin+5e-5){
     auto response=cloth_contact::Solve(v[i],obstacleVelocity(hit,true),hit.normal,Dot(tissueCorrection[i],hit.normal),inverseMass_[i],parameters_.mechanics.friction,step);
     auto delta=Sub(response.velocity,v[i]);v[i]=response.velocity;recordReaction(hit,Mul(delta,step),inverseMass_[i],{x[i],1});
    }
   }
   auto staticCorrection=Sub(contactImpulse[i],tissueCorrection[i]);
   if(Length(staticCorrection)>1e-14){
    bool exact=false;if(!bodyCollider_.Empty()){auto hit=pointQuery(false,i,x[i],margin+5e-5);if(hit.triangle<input.bodyTriangles.size()&&hit.signedDistance<=margin+5e-5){v[i]=cloth_contact::Solve(v[i],obstacleVelocity(hit,false),hit.normal,Dot(staticCorrection,hit.normal),inverseMass_[i],parameters_.mechanics.friction,step).velocity;exact=true;}}
    if(!exact&&bodyCollider_.Empty())v[i]=cloth_contact::Solve(v[i],{},Unit(staticCorrection),Length(staticCorrection),inverseMass_[i],parameters_.mechanics.friction,step).velocity;
   }
  }
 }
 contactGeometry=&input;volumes=finalVolumes;bodyCollider_.Update(input.bodySurface,input.bodyTriangles,origin,C);if(measuredAnatomy)anatomyCollider_.Update(input.anatomy,input.anatomyTriangles,origin,C);else anatomyCollider_.Clear();ClassifySurfaces(input,origin,C);
 // Final alternating sewing/contact projection does not overwrite particles
 // with fitted target geometry. Any unresolved physical conflict is reported.
 reactionStep=0;reactionSurface=&input.anatomy;
 if(!steps)for(unsigned iteration=0;iteration<4;iteration++){solveSewing();collision(true);}
 accumulator_-=steps*step;clock_+=steps*step;telemetry.advancedSeconds=steps*step;telemetry.accumulatedSeconds=clock_;
 double speed2=0;for(unsigned i=0;i<x.size();i++){positions_[i]=Add(origin,Mul(x[i],C));velocities_[i]=Mul(v[i],C);double speed=Length(velocities_[i]);speed2+=speed*speed;telemetry.maxSpeed=(std::max)(telemetry.maxSpeed,speed);telemetry.maxDisplacement=(std::max)(telemetry.maxDisplacement,Length(Sub(positions_[i],Add(origin,Mul(targets[i],C)))));}
 telemetry.rmsSpeed=std::sqrt(speed2/x.size());
 for(auto e:edges_){double length=Length(Sub(x[e.a],x[e.b]));if(e.bend)telemetry.maxBendError=(std::max)(telemetry.maxBendError,std::abs(length-e.rest)*C);else if(length/e.rest>telemetry.maxStretchRatio){telemetry.maxStretchRatio=length/e.rest;telemetry.worstStretchA=e.a;telemetry.worstStretchB=e.b;telemetry.worstStretchRestLength=e.rest*C;telemetry.worstStretchCurrentLength=length*C;}}
 for(const auto& s:sewing_){Point error=residual(s.residual);for(unsigned k=0;k<s.count;k++)error=Add(error,Mul(x[s.nodes[k]],s.weights[k]));telemetry.maxSeamGap=(std::max)(telemetry.maxSeamGap,Length(error)*C);}
 #ifdef MALEMOD_GARMENT_DIAGNOSTIC
 if(telemetry.maxStretchRatio>1.15){for(unsigned i=0;i<vertexNodes_.size();i++)if(vertexNodes_[i]==telemetry.worstStretchA||vertexNodes_[i]==telemetry.worstStretchB)std::fprintf(stderr,"Worst edge donor node %u vertex %u rest %g current %g\n",vertexNodes_[i],i,telemetry.worstStretchRestLength,telemetry.worstStretchCurrentLength);}
#endif
 telemetry.materialBudgetSatisfied=telemetry.maxStretchRatio<=1.15&&telemetry.maxSeamGap<=C*parameters_.bandThickness*.25;
 invalidateRendered();
 for(unsigned i=0;i<mesh.vertices.size();i++)mesh.vertices[i].position=Add(origin,Mul(rendered(i),C));
 if(sheetMesh)for(unsigned fi=output_.layout.sheetFaces.start;fi<output_.layout.sheetFaces.start+output_.layout.sheetFaces.count;fi++){const auto& f=mesh.triangles[fi];for(unsigned k=0;k<3;k++){unsigned a=f.vertices[k],b=f.vertices[(k+1)%3];double rest=Length(Sub(restMesh_.vertices[a].position,restMesh_.vertices[b].position));if(rest<C*1e-12)continue;double ratio=Length(Sub(mesh.vertices[a].position,mesh.vertices[b].position))/rest;if(ratio>telemetry.maxRenderStretchRatio){telemetry.maxRenderStretchRatio=ratio;telemetry.worstRenderA=a;telemetry.worstRenderB=b;}}}telemetry.materialBudgetSatisfied=telemetry.materialBudgetSatisfied&&telemetry.maxRenderStretchRatio<=1.15;
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
 if(telemetry.maxRenderStretchRatio>1.15){unsigned a=telemetry.worstRenderA,b=telemetry.worstRenderB;std::fprintf(stderr,"fineFail vertices %u,%u nodes %u,%u weights %g,%g,%g,%g / %g,%g,%g,%g\n",a,b,vertexNodes_[a],vertexNodes_[b],renderBindings_[a].weights[0],renderBindings_[a].weights[1],renderBindings_[a].weights[2],renderBindings_[a].weights[3],renderBindings_[b].weights[0],renderBindings_[b].weights[1],renderBindings_[b].weights[2],renderBindings_[b].weights[3]);for(auto e:edges_)if((e.a==vertexNodes_[a]&&e.b==vertexNodes_[b])||(e.b==vertexNodes_[a]&&e.a==vertexNodes_[b]))std::fprintf(stderr,"finePhysical edge bend %d tether %d ratio %g rest %g current %g\n",int(e.bend),int(e.tether),Length(Sub(x[e.a],x[e.b]))/e.rest,e.rest*C,Length(Sub(x[e.a],x[e.b]))*C);}
#endif


 for(auto face:mesh.triangles)for(auto volume:volumes){Point p,q;volume.a=Add(origin,Mul(volume.a,C));volume.b=Add(origin,Mul(volume.b,C));volume.radius*=C;if(detail::TriangleCapsule(mesh.vertices[face.vertices[0]].position,mesh.vertices[face.vertices[1]].position,mesh.vertices[face.vertices[2]].position,volume,p,q,margin*C)<(margin-1e-8)*C)output_.contactBudgetSatisfied=false;}
 if(!bodyCollider_.Empty()){
  output_.band.minimumInnerClearance=1e100;output_.band.maximumInnerClearance=-1e100;
  // Band metadata is an exact measured clearance; publication contact reuse is
  // separately conservative and never relabelled as an exact minimum.
  for(unsigned i=band/2;i<band;i++){auto& seed=pointMemos_[0][pointSlot(i)].physicalSeed;auto hit=bodyCollider_.Closest(Mul(Sub(mesh.vertices[i].position,origin),1/C),seed);seed=hit.triangle;double d=hit.signedDistance*C;output_.band.minimumInnerClearance=(std::min)(output_.band.minimumInnerClearance,d);output_.band.maximumInnerClearance=(std::max)(output_.band.maximumInnerClearance,d);}
  for(unsigned i=0;i<mesh.vertices.size();i++){auto p=Mul(Sub(mesh.vertices[i].position,origin),1/C);auto hit=pointQuery(false,pointSlot(i),p,margin);if(hit.signedDistance<margin-1e-8){output_.contactBudgetSatisfied=false;
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
   std::fprintf(stderr,"final-body point clock %.9g vertex %u triangle %u deficit %.9g band %d\n",clock_,i,hit.triangle,(hit.signedDistance-margin)*C,int(i<band));
#endif
  }}
  for(unsigned i=0;i<mesh.triangles.size();i++){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=Mul(Sub(mesh.vertices[mesh.triangles[i].vertices[k]].position,origin),1/C);auto hit=faceQuery(false,i,points,margin);if(hit.signedDistance<margin-1e-8){output_.contactBudgetSatisfied=false;
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
   std::fprintf(stderr,"final-body face clock %.9g face %u triangle %u deficit %.9g vertices %u,%u,%u\n",clock_,i,hit.triangle,(hit.signedDistance-margin)*C,mesh.triangles[i].vertices[0],mesh.triangles[i].vertices[1],mesh.triangles[i].vertices[2]);
#endif
  }}
 }
 output_.coverageMargin=1e100;
 if(measuredAnatomy){
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
  double worstPoint=0,worstFace=0;unsigned worstPointId=0,worstFaceId=0,pointTriangle=0,faceTriangle=0;
#endif
  // This is a certified lower bound on actual sheet-to-tissue clearance. Exact
  // nearby queries replace the certificates whenever their bound is exhausted.
  for(unsigned i=0;i<mesh.vertices.size();i++){auto p=Mul(Sub(mesh.vertices[i].position,origin),1/C);auto hit=pointQuery(true,pointSlot(i),p,margin);output_.coverageMargin=(std::min)(output_.coverageMargin,(hit.distance-margin)*C);
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
   if(hit.distance-margin<worstPoint){worstPoint=hit.distance-margin;worstPointId=i;pointTriangle=hit.triangle;}
#endif
  }
  for(unsigned i=0;i<mesh.triangles.size();i++){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=Mul(Sub(mesh.vertices[mesh.triangles[i].vertices[k]].position,origin),1/C);auto hit=faceQuery(true,i,points,margin);output_.coverageMargin=(std::min)(output_.coverageMargin,(hit.distance-margin)*C);
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
   if(hit.distance-margin<worstFace){worstFace=hit.distance-margin;worstFaceId=i;faceTriangle=hit.triangle;}
#endif
  }
#ifdef MALEMOD_GARMENT_CONTACT_TRACE
  if(worstPoint<-1e-7||worstFace<-1e-7)std::fprintf(stderr,"final-contact clock %.9g point %u triangle %u deficit %.9g face %u triangle %u deficit %.9g C %.9g\n",clock_,worstPointId,pointTriangle,worstPoint*C,worstFaceId,faceTriangle,worstFace*C,C);
#endif
 }else for(unsigned i=band;i<pouchEnd;i++){auto p=input.frame.Local(mesh.vertices[i].position),q=envelope.Coordinates(p);output_.coverageMargin=(std::min)(output_.coverageMargin,(q[2]-envelope.Minimum(p))*Length(Sub(p,envelope.center))/(std::max)(q[2],1e-14));}
 if(output_.coverageMargin<-1e-7*C)output_.contactBudgetSatisfied=false;
 for(const auto& q:mesh.vertices)if(!Finite(q.position))throw std::invalid_argument("Nonfinite advanced cloth position");try{detail::Shading(mesh);}catch(const std::invalid_argument&){throw std::invalid_argument("Cloth advanced shading direction degenerate");}
 // Only actual integrated contact projections produce anatomical feedback.
 // Zero-time fit/repair projections do not manufacture physical impulses.
 if(input.anatomyMass>0)reactions.Publish(output_,steps*step,clothMass_);
 telemetry.solverMilliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
}
}
