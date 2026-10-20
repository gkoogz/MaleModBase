#pragma once
#include "attachment_corrective.hpp"
#include <functional>
#include <map>
#include <string>

namespace malemod::surface::corrective {
struct EdgeDonor {unsigned slave,a,b;double weight;};
// Local endpoint constraints in independent donor coordinates. This does not
// smooth the mesh, certify intersections, or certify the trajectory to it.
class EndpointConstraints {
 using Row=std::vector<std::pair<unsigned,double>>;
 std::vector<Face> faces_;
 std::vector<unsigned> checked_,ids_;
 std::vector<std::array<unsigned,2>> pairs_;
 std::vector<Row> donors_,dependents_;
 std::vector<unsigned> oriented_;
 std::vector<Point> restNormal_;
 std::vector<double> restArea_;
 double quality_;
 mutable std::string report_;
 using GradientRow=std::vector<std::pair<unsigned,Point>>;
 GradientRow Bind(const GradientRow& input)const{
  GradientRow result;result.reserve(16);
  for(auto [v,g]:input)for(auto [master,w]:donors_[v]){
   auto found=std::find_if(result.begin(),result.end(),[&](auto& x){return x.first==master;});
   if(found==result.end())result.emplace_back(master,Scale(g,w));else found->second=Add(found->second,Scale(g,w));
  }return result;
 }
 static Point Normal(const std::vector<Point>& p,Face f){return Cross(Sub(p[f[1]],p[f[0]]),Sub(p[f[2]],p[f[0]]));}
 static std::array<Point,3> Gradient(const std::vector<Point>& p,Face f,Point q){
  auto a=Cross(Sub(p[f[2]],p[f[0]]),q),b=Cross(q,Sub(p[f[1]],p[f[0]]));return {Scale(Add(a,b),-1),a,b};
 }
 void Step(std::vector<Point>& p,const std::vector<std::pair<unsigned,Point>>& input,double error,double trust)const{
  auto gradient=Bind(input);
  double denominator=0,largest=0;for(auto [v,g]:gradient){auto norm=Dot(g,g);denominator+=norm;largest=std::max(largest,norm);}if(denominator<1e-24)return;
  const double amount=std::min(.8*error/denominator,trust/std::sqrt(largest));
  for(auto [master,g]:gradient)for(auto [v,w]:dependents_[master])p[v]=Add(p[v],Scale(g,amount*w));
 }
 // Coupled linearized correction for a small remaining contact set. Solving
 // these rows together avoids cycling when adjacent seam constraints oppose
 // one another. Every step still has the same displacement budget and gate.
 bool JointStep(std::vector<Point>& p,const std::vector<double>& area,const std::vector<bool>& protect,const std::vector<double>& floor,const std::vector<double>& signedFloor,double trust)const{
  std::vector<GradientRow> rows;std::vector<double> residual;
  auto append=[&](GradientRow input,double error){
   auto row=Bind(input);double norm=0;for(auto [v,g]:row)norm+=Dot(g,g);if(norm<1e-24)return;
   norm=std::sqrt(norm);for(auto& item:row)item.second=Scale(item.second,1/norm);
   rows.push_back(std::move(row));residual.push_back(error/norm);
  };
  for(auto i:checked_){auto f=faces_[i];auto n=Normal(p,f);double length=std::sqrt(Dot(n,n));if(length<.08*area[i]&&length>1e-18){auto g=Gradient(p,f,Scale(n,1/length));append({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},.08*area[i]-length);}}
  for(auto i:oriented_){auto f=faces_[i];double value=Dot(Normal(p,f),restNormal_[i]);if(value<1.05*signedFloor[i]){auto g=Gradient(p,f,restNormal_[i]);append({{f[0],g[0]},{f[1],g[1]},{f[2],g[2]}},1.05*signedFloor[i]-value);}}
  for(unsigned i=0;i<pairs_.size();i++)if(protect[i]){
   auto pair=pairs_[i];auto a=faces_[pair[0]],b=faces_[pair[1]];auto na=Normal(p,a),nb=Normal(p,b);double aa=Dot(na,na),bb=Dot(nb,nb),ab=std::sqrt(aa*bb);if(ab<1e-30)continue;
   double cosine=Dot(na,nb)/ab;if(cosine>=floor[i]+.05)continue;
   auto ga=Gradient(p,a,Sub(Scale(nb,1/ab),Scale(na,cosine/aa))),gb=Gradient(p,b,Sub(Scale(na,1/ab),Scale(nb,cosine/bb)));
   GradientRow input;for(unsigned j=0;j<3;j++){input.emplace_back(a[j],ga[j]);input.emplace_back(b[j],gb[j]);}append(std::move(input),floor[i]+.05-cosine);
  }
  const auto n=rows.size();if(!n||n>128)return false;
  std::vector<double> matrix(n*n),solution=residual;
  for(unsigned i=0;i<n;i++)for(unsigned j=0;j<=i;j++){
   double value=0;for(auto [a,ga]:rows[i])for(auto [b,gb]:rows[j])if(a==b)value+=Dot(ga,gb);
   if(i==j)value+=1e-5;matrix[i*n+j]=matrix[j*n+i]=value;
  }
  for(unsigned i=0;i<n;i++)for(unsigned j=0;j<=i;j++){
   double value=matrix[i*n+j];for(unsigned k=0;k<j;k++)value-=matrix[i*n+k]*matrix[j*n+k];
   if(i==j){if(value<=0)return false;matrix[i*n+j]=std::sqrt(value);}else matrix[i*n+j]=value/matrix[j*n+j];
  }
  for(unsigned i=0;i<n;i++){for(unsigned j=0;j<i;j++)solution[i]-=matrix[i*n+j]*solution[j];solution[i]/=matrix[i*n+i];}
  for(unsigned i=unsigned(n);i-->0;){for(unsigned j=i+1;j<n;j++)solution[i]-=matrix[j*n+i]*solution[j];solution[i]/=matrix[i*n+i];}
  std::map<unsigned,Point> delta;for(unsigned i=0;i<n;i++)for(auto [v,g]:rows[i])delta[v]=Add(delta[v],Scale(g,solution[i]));
  double largest=0;for(auto [v,d]:delta)largest=std::max(largest,std::sqrt(Dot(d,d)));
  double amount=std::min(.5,trust/std::max(largest,1e-20));
  for(auto [master,d]:delta)for(auto [v,w]:dependents_[master])p[v]=Add(p[v],Scale(d,amount*w));
  return true;
 }
 public:
 const std::string& Report()const{return report_;}
 EndpointConstraints(std::size_t count,const std::vector<Face>& faces,const std::vector<unsigned>& ids,const std::vector<EdgeDonor>& seams,
  const std::vector<Point>& rest={},const std::vector<unsigned>& orientation={},double quality=.01):faces_(faces),ids_(ids),donors_(count),dependents_(count),quality_(quality){
  if(!std::isfinite(quality)||quality<.01||quality>.85)throw std::invalid_argument("Invalid endpoint curvature floor");
  std::vector<bool> editable(count),affected(faces.size());for(auto id:ids){if(id>=count||editable[id])throw std::invalid_argument("Invalid endpoint row");editable[id]=true;}
  std::map<unsigned,EdgeDonor> edges;
  for(auto e:seams){if(e.slave>=count||e.a>=count||e.b>=count||!std::isfinite(e.weight)||e.weight<0||e.weight>1||!edges.emplace(e.slave,e).second)throw std::invalid_argument("Invalid endpoint donor");}
  std::vector<unsigned char> state(count);
  std::function<void(unsigned)> bind=[&](unsigned i){
   if(state[i]==2)return;if(state[i]==1)throw std::invalid_argument("Cyclic endpoint donors");state[i]=1;
   auto edge=edges.find(i);
   if(edge==edges.end()){if(editable[i])donors_[i].push_back({i,1});}
   else{auto e=edge->second;bind(e.a);bind(e.b);std::map<unsigned,double> row;
    for(auto [v,w]:donors_[e.a])row[v]+=(1-e.weight)*w;for(auto [v,w]:donors_[e.b])row[v]+=e.weight*w;
    for(auto [v,w]:row)if(w)donors_[i].push_back({v,w});
    if(!editable[i]&&!donors_[i].empty())throw std::invalid_argument("Endpoint donor moves a fixed slave");
   }state[i]=2;
  };
  for(unsigned i=0;i<count;i++){bind(i);for(auto [master,w]:donors_[i])dependents_[master].push_back({i,w});}
  std::map<std::pair<unsigned,unsigned>,std::vector<unsigned>> adjacency;
  for(unsigned i=0;i<faces.size();i++){auto f=faces[i];for(auto v:f){if(v>=count)throw std::invalid_argument("Invalid endpoint face");affected[i]=affected[i]||editable[v];}
   for(unsigned j=0;j<3;j++)adjacency[std::minmax(f[j],f[(j+1)%3])].push_back(i);
  }
  auto needed=affected;
  for(auto& item:adjacency){auto& p=item.second;if(p.size()>2)throw std::invalid_argument("Nonmanifold endpoint edge");if(p.size()==2&&(affected[p[0]]||affected[p[1]])){pairs_.push_back({p[0],p[1]});needed[p[0]]=needed[p[1]]=true;}}
  for(unsigned i=0;i<needed.size();i++)if(needed[i])checked_.push_back(i);
  if(!orientation.empty()){
   if(rest.size()!=count)throw std::invalid_argument("Missing endpoint orientation reference");restNormal_.resize(faces.size());restArea_.resize(faces.size());
   for(auto i:orientation){if(i>=faces.size())throw std::invalid_argument("Invalid oriented endpoint triangle");if(!affected[i])continue;
    auto normal=Normal(rest,faces[i]);double area=std::sqrt(Dot(normal,normal));if(area<=0||!std::isfinite(area))throw std::invalid_argument("Degenerate oriented endpoint triangle");
    restNormal_[i]=Scale(normal,1/area);restArea_[i]=area;oriented_.push_back(i);
   }
  }
 }
 bool Project(const std::vector<Point>& original,std::vector<Point>& target,unsigned iterations=160)const{
  const auto desired=target;
  if(ProjectOnce(original,target,std::min(iterations,128u),true))return true;
  auto first=report_;target=desired;
  if(ProjectOnce(original,target,iterations,false)){report_="coupled retry "+report_;return true;}
  report_=first+"; coupled retry "+report_;target=desired;return false;
 }
 // Reuse only the previous correction, never the previous pose. The cold
 // target remains the displacement-budget origin and every constraint is
 // checked against the current reference. Fixed rows and donor interpolation
 // are reconstructed from independent coordinates, including for stale seeds.
 bool ProjectWarm(const std::vector<Point>& original,std::vector<Point>& target,const std::vector<Point>& correction)const{
  if(correction.size()!=target.size()||target.size()!=donors_.size())return false;
  const auto desired=target;
  for(unsigned i=0;i<target.size();i++)for(auto [master,w]:donors_[i]){
   if(!Finite(correction[master])){target=desired;return false;}
   target[i]=Add(target[i],Scale(correction[master],w));
  }
  if(ProjectOnce(original,target,24,true,&desired)){report_="warm "+report_;return true;}
  target=desired;return false;
 }
 private:
 bool ProjectOnce(const std::vector<Point>& original,std::vector<Point>& target,unsigned iterations,bool alternate,const std::vector<Point>* cold=nullptr)const{
  report_="iterations";
  if(original.size()!=donors_.size()||target.size()!=original.size())throw std::invalid_argument("Invalid endpoint positions");
  const auto desired=cold?*cold:target;std::vector<Point> normals(faces_.size());std::vector<double> area(faces_.size());std::vector<bool> protectedPair(pairs_.size());
  double budget=0;for(auto i:ids_){auto delta=Sub(desired[i],original[i]);budget=std::max(budget,std::sqrt(Dot(delta,delta)));}budget=std::max(1e-8,budget*.25);
  for(auto i:checked_){normals[i]=Normal(original,faces_[i]);area[i]=std::sqrt(Dot(normals[i],normals[i]));if(!std::isfinite(area[i])||area[i]<=0)return false;}
  std::vector<double> signedFloor(faces_.size());for(auto i:oriented_)signedFloor[i]=std::max(.001*restArea_[i],std::min(.08*restArea_[i],.98*Dot(normals[i],restNormal_[i])));
  std::vector<double> floor(pairs_.size());
  for(unsigned i=0;i<pairs_.size();i++){auto p=pairs_[i];double cosine=Dot(normals[p[0]],normals[p[1]])/(area[p[0]]*area[p[1]]);protectedPair[i]=cosine>.5;floor[i]=std::min(quality_,.9*cosine);}
  std::vector<std::pair<unsigned,Point>> gradient;gradient.reserve(6);
  for(unsigned iteration=0;iteration<iterations;iteration++){
   const bool joint=iteration>=24&&(!alternate||iteration%2==0)&&JointStep(target,area,protectedPair,floor,signedFloor,.1*budget);
   unsigned violations=0;
   for(auto i:oriented_){auto f=faces_[i];double value=Dot(Normal(target,f),restNormal_[i]);if(value<signedFloor[i]){
    violations++;if(!joint){auto g=Gradient(target,f,restNormal_[i]);gradient.clear();for(unsigned j=0;j<3;j++)gradient.emplace_back(f[j],g[j]);Step(target,gradient,1.05*signedFloor[i]-value,.1*budget);}
   }}
   for(auto i:checked_){auto f=faces_[i];auto n=Normal(target,f);double a=std::sqrt(Dot(n,n));if(!std::isfinite(a))return false;
    if(a<.04*area[i]){violations++;if(!joint){auto direction=a>1e-18?Scale(n,1/a):Scale(normals[i],1/area[i]);auto g=Gradient(target,f,direction);gradient.clear();for(unsigned j=0;j<3;j++)gradient.emplace_back(f[j],g[j]);Step(target,gradient,.05*area[i]-a,.1*budget);}}
   }
   for(unsigned i=0;i<pairs_.size();i++)if(protectedPair[i]){
    auto pair=pairs_[i];auto a=faces_[pair[0]],b=faces_[pair[1]];auto na=Normal(target,a),nb=Normal(target,b);double aa=Dot(na,na),bb=Dot(nb,nb),ab=std::sqrt(aa*bb);if(ab<1e-30)continue;
    double cosine=Dot(na,nb)/ab;if(cosine>=floor[i])continue;violations++;if(joint)continue;
    auto ga=Gradient(target,a,Sub(Scale(nb,1/ab),Scale(na,cosine/aa))),gb=Gradient(target,b,Sub(Scale(na,1/ab),Scale(nb,cosine/bb)));
    gradient.clear();for(unsigned j=0;j<3;j++){gradient.emplace_back(a[j],ga[j]);gradient.emplace_back(b[j],gb[j]);}Step(target,gradient,floor[i]+.02-cosine,.1*budget);
   }
   for(auto i:ids_){auto d=Sub(target[i],desired[i]);if(!Finite(target[i])||Dot(d,d)>budget*budget){report_="budget iteration "+std::to_string(iteration)+" row "+std::to_string(i)+" shift "+std::to_string(std::sqrt(Dot(d,d)))+" limit "+std::to_string(budget);return false;}}
   report_="iterations "+std::to_string(iteration)+" violations "+std::to_string(violations);
   if(!violations){report_="complete "+std::to_string(iteration);return true;}
  }
  for(unsigned i=0;i<pairs_.size();i++)if(protectedPair[i]){auto pair=pairs_[i];auto a=Normal(target,faces_[pair[0]]),b=Normal(target,faces_[pair[1]]);double c=Dot(a,b)/std::sqrt(Dot(a,a)*Dot(b,b));if(c<floor[i])report_+=" pair "+std::to_string(pair[0])+"/"+std::to_string(pair[1])+" cosine "+std::to_string(c);}
  return false;
 }
};
}
