#pragma once
#include "graft_runtime.hpp"
#include "root_motion.hpp"
#include <unordered_map>
#include <string>
#include <deque>
#include <functional>
namespace malemod::surface {
 // A rejected candidate is diagnostic evidence only, never an accepted surface.
 struct RecruitmentConstraintFailure : std::runtime_error {
  std::vector<PrecisePoint> candidate;
  RecruitmentConstraintFailure(std::string message,std::vector<PrecisePoint> points)
   :std::runtime_error(std::move(message)),candidate(std::move(points)){}
 };
struct SurfaceSupportPlane {unsigned vertex;PrecisePoint normal;double offset;};
// Local inequality projection in independent donor space. Unlike positional
// smoothing, this only changes violated faces; fixed tissue remains exact.
class RecruitmentConstraints {
 using P=PrecisePoint;using F=std::array<std::uint32_t,3>;
 std::vector<P> rest_;std::vector<F> faces_;
 std::vector<std::array<unsigned,2>> pairs_;
 std::vector<std::vector<std::pair<unsigned,double>>> rows_;
 std::vector<bool> locked_,body_;
 public:
 explicit RecruitmentConstraints(const GraftDomain& domain):rest_(domain.points),faces_(domain.triangles),rows_(domain.points.size()),locked_(domain.points.size(),false),body_(domain.triangles.size(),false){
  for(unsigned i=0;i<rows_.size();i++)rows_[i]={{i,1}};
  for(auto i:domain.protectedVertices)locked_.at(i)=true;
  for(auto i:domain.prescribedVertices)locked_.at(i)=true;
  for(auto e:domain.seams){rows_.at(e.slave)={{e.a,1-e.weight},{e.b,e.weight}};locked_.at(e.slave)=true;}
  // A lower-detail edge can itself contain a donor introduced by the shared
  // boundary. Resolve all such rows to independent masters before projection.
  std::vector<unsigned char> visited(rows_.size());
  std::function<void(unsigned)> flatten=[&](unsigned i){
   if(i>=rows_.size())throw std::invalid_argument("Seam donor outside surface");
   if(visited[i]==2)return;
   if(visited[i]==1)throw std::invalid_argument("Cyclic seam donors");
   visited[i]=1;auto original=rows_[i];
   if(!(original.size()==1&&original[0].first==i&&original[0].second==1)){
    std::vector<std::pair<unsigned,double>> flat;
    for(auto [parent,weight]:original){if(!weight)continue;flatten(parent);for(auto [master,w]:rows_[parent]){
     auto found=std::find_if(flat.begin(),flat.end(),[&](const auto& row){return row.first==master;});
     if(found==flat.end())flat.push_back({master,weight*w});else found->second+=weight*w;
    }}rows_[i]=std::move(flat);
   }visited[i]=2;
  };
  for(unsigned i=0;i<rows_.size();i++)flatten(i);
  for(auto i:domain.orientationTriangles)body_.at(i)=true;
  std::unordered_map<std::uint64_t,unsigned> edge;
  for(unsigned i=0;i<faces_.size();i++)for(unsigned j=0;j<3;j++){auto a=std::min(faces_[i][j],faces_[i][(j+1)%3]),b=std::max(faces_[i][j],faces_[i][(j+1)%3]);auto key=(std::uint64_t(a)<<32)|b;auto it=edge.find(key);if(it==edge.end())edge.emplace(key,i);else pairs_.push_back({it->second,i});}
 }
 std::vector<P> ProjectSweeps(const std::vector<P>& reference,const std::vector<P>& target)const{
  using namespace root_motion;
  if(reference.size()!=rest_.size()||target.size()!=rest_.size())throw std::invalid_argument("Recruitment constraint topology differs");
  auto master=reference;
  std::vector<P> increment(target.size());for(unsigned i=0;i<target.size();i++)increment[i]=root_motion::Scale(root_motion::Add(target[i],root_motion::Scale(reference[i],-1)),1./32);
  auto position=[&](unsigned i){P p{};for(auto [m,w]:rows_[i])p=Add(p,Scale(master[m],w));return p;};
  auto normal=[&](F f){return Cross(Add(position(f[1]),Scale(position(f[0]),-1)),Add(position(f[2]),Scale(position(f[0]),-1)));};
  auto faceNormal=[](const std::vector<P>& p,F f){return Cross(Add(p[f[1]],Scale(p[f[0]],-1)),Add(p[f[2]],Scale(p[f[0]],-1)));};
  auto gradients=[&](F f,P q){auto a=Cross(Add(position(f[2]),Scale(position(f[0]),-1)),q),b=Cross(q,Add(position(f[1]),Scale(position(f[0]),-1)));return std::array<P,3>{Scale(Add(a,b),-1),a,b};};
  auto step=[&](const std::vector<std::pair<unsigned,P>>& vertex,double error){
   std::unordered_map<unsigned,P> g;
   for(auto [i,v]:vertex)for(auto [m,w]:rows_[i])if(!locked_[m])g[m]=Add(g[m],Scale(v,w));
   double denominator=0;for(auto [m,v]:g)denominator+=Dot(v,v);
   if(denominator<1e-20)return;
   double diameter2=0,longest2=0;
   for(auto [i,v]:vertex)for(auto [j,q]:vertex){const auto d=Add(position(i),Scale(position(j),-1));diameter2=std::max(diameter2,Dot(d,d));}
   for(auto [m,v]:g)longest2=std::max(longest2,Dot(v,v));
   const double amount=std::min(.65*error/denominator,.10*std::sqrt(diameter2/std::max(longest2,1e-30)));
   for(auto [m,v]:g)master[m]=Add(master[m],Scale(v,amount));
  };
  std::vector<double> area(faces_.size());std::vector<P> restNormal(faces_.size());
  for(unsigned i=0;i<faces_.size();i++){auto n=faceNormal(reference,faces_[i]);area[i]=std::sqrt(Dot(n,n));auto r=faceNormal(rest_,faces_[i]);restNormal[i]=r;}
  std::vector<bool> protect(pairs_.size());
  for(unsigned i=0;i<pairs_.size();i++){auto pair=pairs_[i];auto a=faceNormal(reference,faces_[pair[0]]),b=faceNormal(reference,faces_[pair[1]]);protect[i]=Dot(a,b)>.5*area[pair[0]]*area[pair[1]];}
  unsigned remaining=0;
  for(unsigned phase=0;phase<32;phase++){
  for(unsigned i=0;i<master.size();i++)master[i]=Add(master[i],increment[i]);
  for(unsigned iteration=0;iteration<(phase==31?1024u:128u);iteration++){
   remaining=0;
   for(unsigned i=0;i<faces_.size();i++){
    auto f=faces_[i];auto n=normal(f);const double length=std::sqrt(Dot(n,n));
    if(length<.021*area[i]&&length>1e-20){remaining++;auto g=gradients(f,Scale(n,1/length));step({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},.025*area[i]-length);}
    if(body_[i]){const double expected=Dot(restNormal[i],restNormal[i]),value=Dot(n,restNormal[i]);if(value<.002*expected){remaining++;auto g=gradients(f,restNormal[i]);step({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},.005*expected-value);}}
   }
   for(unsigned i=0;i<pairs_.size();i++)if(protect[i]){
    auto pair=pairs_[i];auto a=faces_[pair[0]],b=faces_[pair[1]];auto na=normal(a),nb=normal(b);const double aa=Dot(na,na),bb=Dot(nb,nb),length=std::sqrt(aa*bb);if(length<1e-25)continue;
    const double cosine=Dot(na,nb)/length;if(cosine>=.10)continue;remaining++;
    auto ga=gradients(a,Add(Scale(nb,1/length),Scale(na,-cosine/aa))),gb=gradients(b,Add(Scale(na,1/length),Scale(nb,-cosine/bb)));
    step({{a[0],ga[0]},{a[1],ga[1]},{a[2],ga[2]},{b[0],gb[0]},{b[1],gb[1]},{b[2],gb[2]}},.15-cosine);
   }
   if(!remaining)break;
  }
  }
  // The iteration uses headroom. Verify the actual invariants independently
  // after the final full sweep, rather than its intermediate violation count.
  remaining=0;
  for(unsigned i=0;i<faces_.size();i++){auto n=normal(faces_[i]);if(std::sqrt(Dot(n,n))<.020001*area[i])remaining++;if(body_[i]&&Dot(n,restNormal[i])<.001001*Dot(restNormal[i],restNormal[i]))remaining++;}
  for(unsigned i=0;i<pairs_.size();i++)if(protect[i]){auto pair=pairs_[i];auto a=normal(faces_[pair[0]]),b=normal(faces_[pair[1]]);if(Dot(a,b)<.0001*std::sqrt(Dot(a,a)*Dot(b,b)))remaining++;}
  if(remaining){std::string detail;for(unsigned i=0;i<faces_.size();i++){auto n=normal(faces_[i]);if(std::sqrt(Dot(n,n))<.020001*area[i])detail+=" area "+std::to_string(i);if(body_[i]&&Dot(n,restNormal[i])<.001001*Dot(restNormal[i],restNormal[i]))detail+=" body "+std::to_string(i);}for(unsigned i=0;i<pairs_.size();i++)if(protect[i]){auto pair=pairs_[i];auto a=normal(faces_[pair[0]]),b=normal(faces_[pair[1]]);if(Dot(a,b)<.0001*std::sqrt(Dot(a,a)*Dot(b,b)))detail+=" pair "+std::to_string(pair[0])+","+std::to_string(pair[1])+" cosine "+std::to_string(Dot(a,b)/std::sqrt(Dot(a,a)*Dot(b,b)));}throw std::runtime_error("Recruitment surface inequalities did not converge: "+std::to_string(remaining)+detail);}
  std::vector<P> out(rest_.size());for(unsigned i=0;i<out.size();i++)out[i]=position(i);return out;
 }
 // Revisit only constraints touched by a projected master. A global sweep
 // revisits tens of thousands of unchanged faces for each local correction.
 std::vector<P> Project(const std::vector<P>& reference,const std::vector<P>& target,const std::vector<SurfaceSupportPlane>& support={},unsigned continuationSteps=32)const{
  using namespace root_motion;
  if(!continuationSteps||continuationSteps>128)throw std::invalid_argument("Invalid recruitment continuation count");
  const unsigned nf=unsigned(faces_.size()),planeBegin=nf+unsigned(pairs_.size()),nc=planeBegin+unsigned(support.size()),nv=unsigned(rest_.size());
  if(reference.size()!=nv||target.size()!=nv)throw std::invalid_argument("Recruitment constraint topology differs");
  std::vector<std::vector<unsigned>> masterFaces(nv),facePairs(nf),masterPlanes(nv);
  std::vector<double> initialSupport;double phaseFraction=0;
  for(unsigned j=0;j<support.size();j++){const auto& s=support[j];if(s.vertex>=nv||!std::isfinite(s.offset)||!std::isfinite(Dot(s.normal,s.normal))||std::abs(Dot(s.normal,s.normal)-1)>1e-5)throw std::invalid_argument("Invalid surface support plane");initialSupport.push_back(std::min(s.offset,Dot(reference[s.vertex],s.normal)));for(auto [m,w]:rows_[s.vertex])if(w)masterPlanes[m].push_back(planeBegin+j);}
    // Opposed halfspaces describe an exact section coordinate. Eliminate that
    // degree of freedom instead of alternating opposing projections, which can
    // fight triangle repair and let a constrained collar drift in radius.
    std::vector<std::vector<P>> equalityBasis(nv);
    std::vector<std::vector<double>> equalityTarget(nv);
    for(unsigned m=0;m<nv;m++)for(auto aid:masterPlanes[m]){
     unsigned a=aid-planeBegin;const auto& s=support[a];
     if(s.vertex!=m||rows_[m].size()!=1||rows_[m][0].first!=m)continue;
     bool opposed=false;
     for(auto bid:masterPlanes[m]){unsigned b=bid-planeBegin;if(b==a||support[b].vertex!=m)continue;
      if(Dot(s.normal,support[b].normal)<-1+1e-12&&std::abs(s.offset+support[b].offset)<1e-10){opposed=true;initialSupport[a]=Dot(reference[m],s.normal);break;}}
     if(!opposed)continue;
     auto n=s.normal;double value=s.offset;
     for(unsigned j=0;j<equalityBasis[m].size();j++){double component=Dot(n,equalityBasis[m][j]);n=Add(n,Scale(equalityBasis[m][j],-component));value-=component*equalityTarget[m][j];}
     double length=std::sqrt(Dot(n,n));if(length<1e-10)continue;
     equalityBasis[m].push_back(Scale(n,1/length));equalityTarget[m].push_back(value/length);
    }
  for(unsigned i=0;i<nf;i++)for(auto v:faces_[i])for(auto [m,w]:rows_[v])if(w)masterFaces[m].push_back(i);
  for(unsigned i=0;i<pairs_.size();i++)for(auto f:pairs_[i])facePairs[f].push_back(i);
  for(auto& list:masterFaces){std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());}
  auto master=reference;
  auto position=[&](unsigned i){P p{};for(auto [m,w]:rows_[i])p=Add(p,Scale(master[m],w));return p;};
  auto normalOf=[](P a,P b,P c){return Cross(Add(b,Scale(a,-1)),Add(c,Scale(a,-1)));};
  auto faceNormal=[&](const std::vector<P>& p,F f){return normalOf(p[f[0]],p[f[1]],p[f[2]]);};
  std::vector<P> normals(nf),restNormals(nf),increment(nv);std::vector<double> area(nf);std::vector<bool> protect(pairs_.size());
  for(unsigned i=0;i<nv;i++)increment[i]=Scale(Add(target[i],Scale(reference[i],-1)),1./continuationSteps);
  for(unsigned i=0;i<nf;i++){auto n=faceNormal(reference,faces_[i]);area[i]=std::sqrt(Dot(n,n));restNormals[i]=faceNormal(rest_,faces_[i]);}
  for(unsigned i=0;i<pairs_.size();i++){auto pair=pairs_[i];protect[i]=Dot(faceNormal(reference,faces_[pair[0]]),faceNormal(reference,faces_[pair[1]]))>.5*area[pair[0]]*area[pair[1]];}
  auto update=[&](unsigned i){auto f=faces_[i];normals[i]=normalOf(position(f[0]),position(f[1]),position(f[2]));};
  auto gradients=[&](F f,P q){auto gb=Cross(Add(position(f[2]),Scale(position(f[0]),-1)),q),gc=Cross(q,Add(position(f[1]),Scale(position(f[0]),-1)));return std::array<P,3>{Scale(Add(gb,gc),-1),gb,gc};};
  std::deque<unsigned> queue;std::vector<bool> queued(nc),dirty(nf);
  auto enqueue=[&](unsigned i){if(!queued[i]){queue.push_back(i);queued[i]=true;}};
  auto violated=[&](unsigned id){
   if(id>=planeBegin){const auto& s=support[id-planeBegin];return Dot(position(s.vertex),s.normal)<initialSupport[id-planeBegin]*(1-phaseFraction)+s.offset*phaseFraction-1e-9;}
   if(id<nf){auto n=normals[id];if(body_[id]){double restArea=std::sqrt(Dot(restNormals[id],restNormals[id]));return Dot(n,restNormals[id])<std::max(.021*area[id],.002*restArea)*restArea;}return std::sqrt(Dot(n,n))<.021*area[id];}
   id-=nf;if(!protect[id])return false;auto pair=pairs_[id];auto a=normals[pair[0]],b=normals[pair[1]];double length=std::sqrt(Dot(a,a)*Dot(b,b));return length>1e-25&&Dot(a,b)<.10*length;
  };
  auto step=[&](const std::vector<std::pair<unsigned,P>>& vertex,double error){
   std::vector<std::pair<unsigned,P>> g;
   for(auto [i,v]:vertex)for(auto [m,w]:rows_[i])if(!locked_[m]){auto it=std::find_if(g.begin(),g.end(),[&](const auto& row){return row.first==m;});if(it==g.end())g.push_back({m,Scale(v,w)});else it->second=Add(it->second,Scale(v,w));}
   for(auto& row:g)for(auto n:equalityBasis[row.first])row.second=Add(row.second,Scale(n,-Dot(row.second,n)));
   double denominator=0,diameter2=0,longest2=0;
   for(auto [m,v]:g){denominator+=Dot(v,v);longest2=std::max(longest2,Dot(v,v));}if(denominator<1e-20)return false;
   for(auto [i,v]:vertex)for(auto [j,q]:vertex){auto d=Add(position(i),Scale(position(j),-1));diameter2=std::max(diameter2,Dot(d,d));}
   if(vertex.size()==1)diameter2=error*error;
   double amount=std::min(.65*error/denominator,.10*std::sqrt(diameter2/std::max(longest2,1e-30)));
   std::vector<unsigned> affected;
   for(auto [m,v]:g){master[m]=Add(master[m],Scale(v,amount));for(auto f:masterFaces[m])if(!dirty[f]){dirty[f]=true;affected.push_back(f);}}
   for(auto f:affected)update(f);
   for(auto f:affected){dirty[f]=false;if(violated(f))enqueue(f);for(auto pair:facePairs[f])if(violated(nf+pair))enqueue(nf+pair);}
   for(auto [m,v]:g)for(auto plane:masterPlanes[m])if(violated(plane))enqueue(plane);
   return true;
  };
  for(unsigned phase=0;phase<continuationSteps;phase++){
   phaseFraction=double(phase+1)/continuationSteps;
   for(unsigned i=0;i<nv;i++)master[i]=(continuationSteps==1||(phase+1==continuationSteps&&locked_[i]))?target[i]:Add(master[i],increment[i]);
   for(unsigned i=0;i<nv;i++)for(unsigned j=0;j<equalityBasis[i].size();j++){
    auto n=equalityBasis[i][j];double value=(1-phaseFraction)*Dot(reference[i],n)+phaseFraction*equalityTarget[i][j];
    if(locked_[i]){if(std::abs(Dot(master[i],n)-value)>1e-7)throw std::runtime_error("Fixed tissue conflicts with exact section coordinate");}
    else master[i]=Add(master[i],Scale(n,value-Dot(master[i],n)));
   }
   for(unsigned i=0;i<nf;i++)update(i);
   for(unsigned i=0;i<nc;i++)if(violated(i))enqueue(i);
   const std::size_t budget=std::size_t(nc)*(phase+1==continuationSteps?64:8);std::size_t work=0;
   while(!queue.empty()&&work++<budget){
    auto id=queue.front();queue.pop_front();queued[id]=false;if(!violated(id))continue;
    if(id>=planeBegin){const auto& s=support[id-planeBegin];double wanted=initialSupport[id-planeBegin]*(1-phaseFraction)+s.offset*phaseFraction;step({{s.vertex,s.normal}},wanted-Dot(position(s.vertex),s.normal));}
    else if(id<nf){auto f=faces_[id];auto n=normals[id];double length=std::sqrt(Dot(n,n));
     if(body_[id]){
      // A body face has a prescribed outward winding. Inflating its unsigned
      // area and then correcting its signed area can pull it in conflicting
      // directions near collapse. One projected-area bound enforces both.
      const double restArea=std::sqrt(Dot(restNormals[id],restNormals[id]));
      const double value=Dot(n,restNormals[id])/restArea,minimum=std::max(.021*area[id],.002*restArea);
      if(value<minimum){auto g=gradients(f,Scale(restNormals[id],1/restArea));step({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},std::max(.025*area[id],.005*restArea)-value);}
     }else if(length<.021*area[id]&&length>1e-20){auto g=gradients(f,Scale(n,1/length));step({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},.025*area[id]-length);}
    }else{auto pair=pairs_[id-nf];auto a=faces_[pair[0]],b=faces_[pair[1]];auto na=normals[pair[0]],nb=normals[pair[1]];double aa=Dot(na,na),bb=Dot(nb,nb),length=std::sqrt(aa*bb);if(length<1e-25)continue;double cosine=Dot(na,nb)/length;
     auto ga=gradients(a,Add(Scale(nb,1/length),Scale(na,-cosine/aa))),gb=gradients(b,Add(Scale(na,1/length),Scale(nb,-cosine/bb)));
     step({{a[0],ga[0]},{a[1],ga[1]},{a[2],ga[2]},{b[0],gb[0]},{b[1],gb[1]},{b[2],gb[2]}},.15-cosine);
    }
   }
   queue.clear();std::fill(queued.begin(),queued.end(),false);
  }
  unsigned remaining=0;std::string detail;
  for(unsigned i=0;i<nf;i++){auto n=normals[i];if(std::sqrt(Dot(n,n))<.020001*area[i]||(body_[i]&&Dot(n,restNormals[i])<.001001*Dot(restNormals[i],restNormals[i]))){remaining++;detail+=" face "+std::to_string(i);}}
  for(unsigned i=0;i<pairs_.size();i++)if(protect[i]){auto pair=pairs_[i];auto a=normals[pair[0]],b=normals[pair[1]];if(Dot(a,b)<.0001*std::sqrt(Dot(a,a)*Dot(b,b))){remaining++;detail+=" pair "+std::to_string(pair[0])+","+std::to_string(pair[1]);}}
  for(const auto& s:support)if(Dot(position(s.vertex),s.normal)<s.offset-1e-7){remaining++;detail+=" support "+std::to_string(s.vertex);}
  if(remaining){for(unsigned i=0;i<nf;i++)if(std::sqrt(Dot(normals[i],normals[i]))<.020001*area[i]){detail+=" ratio "+std::to_string(std::sqrt(Dot(normals[i],normals[i]))/area[i]);for(auto v:faces_[i])detail+=" v "+std::to_string(v)+" lock "+std::to_string(locked_[v])+" move "+std::to_string(std::sqrt(Dot(Add(position(v),Scale(reference[v],-1)),Add(position(v),Scale(reference[v],-1)))));}std::vector<P> rejected(nv);for(unsigned i=0;i<nv;i++)rejected[i]=position(i);throw RecruitmentConstraintFailure("Recruitment surface inequalities did not converge: "+std::to_string(remaining)+detail,std::move(rejected));}
  std::vector<P> out(nv);for(unsigned i=0;i<nv;i++)out[i]=position(i);return out;
 }
};
}
