#pragma once
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <functional>

namespace malemod::surface::corrective {
using Point=std::array<double,3>;
using Face=std::array<std::uint32_t,3>;
inline Point Add(Point a,Point b){for(unsigned i=0;i<3;i++)a[i]+=b[i];return a;}
inline Point Sub(Point a,Point b){for(unsigned i=0;i<3;i++)a[i]-=b[i];return a;}
inline Point Scale(Point a,double b){for(auto& v:a)v*=b;return a;}
inline double Dot(Point a,Point b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Point Cross(Point a,Point b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline bool Finite(Point p){return std::all_of(p.begin(),p.end(),[](double x){return std::isfinite(x);});}
inline Point Unit(Point p){double n=std::sqrt(Dot(p,p));if(!Finite(p)||n<1e-12)throw std::invalid_argument("Invalid corrective frame");return Scale(p,1/n);}
inline double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
struct Frame {
 Point forward,side,up;
 static Frame FromAxis(Point axis,Point sideHint={0,1,0}){
  axis=Unit(axis);auto side=Unit(Sub(sideHint,Scale(axis,Dot(axis,sideHint))));
  return {axis,side,Cross(axis,side)};
 }
 Point ToWorld(Point p)const{return Add(Add(Scale(forward,p[0]),Scale(side,p[1])),Scale(up,p[2]));}
 bool Valid()const{
  return Finite(forward)&&Finite(side)&&Finite(up)&&std::abs(Dot(forward,forward)-1)<1e-6&&
   std::abs(Dot(side,side)-1)<1e-6&&std::abs(Dot(up,up)-1)<1e-6&&
   std::abs(Dot(forward,side))<1e-6&&std::abs(Dot(forward,up))<1e-6&&std::abs(Dot(side,up))<1e-6;
 }
};
struct Knot {double weight;std::vector<Point> localDisplacement;};
struct Result {std::vector<Point> positions;double appliedWeight=0;};

namespace detail {
using Polynomial=std::array<double,5>;
using NormalPolynomial=std::array<Point,3>;
inline Polynomial Product(const NormalPolynomial& a,const NormalPolynomial& b){
 Polynomial p{};for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)p[i+j]+=Dot(a[i],b[j]);return p;
}
inline bool BernsteinPositive(const Polynomial& b,unsigned depth,double tolerance){
 if(std::all_of(b.begin(),b.end(),[&](double v){return v>=-tolerance;}))return true;
 if(b[0]<-tolerance||b[4]<-tolerance||!depth)return false;
 auto work=b;Polynomial left{},right{};left[0]=b[0];right[4]=b[4];
 for(unsigned level=1;level<=4;level++){
  for(unsigned i=0;i<=4-level;i++)work[i]=(work[i]+work[i+1])*.5;
  left[level]=work[0];right[4-level]=work[4-level];
 }
 return BernsteinPositive(left,depth-1,tolerance)&&BernsteinPositive(right,depth-1,tolerance);
}
// A Bernstein lower bound checks the whole interpolation interval, including
// a possible collapse between two individually valid endpoints.
inline bool Nonnegative(const Polynomial& p,double end){
 constexpr double choose[5][5]={{1,0,0,0,0},{1,1,0,0,0},{1,2,1,0,0},{1,3,3,1,0},{1,4,6,4,1}};
 Polynomial b{};double power[5]={1,end,end*end,end*end*end,end*end*end*end};
 for(unsigned k=0;k<5;k++)for(unsigned i=0;i<=k;i++)b[k]+=p[i]*power[i]*choose[k][i]/choose[4][i];
 double magnitude=0;for(auto v:b){if(!std::isfinite(v))return false;magnitude=std::max(magnitude,std::abs(v));}
 return BernsteinPositive(b,14,128*std::numeric_limits<double>::epsilon()*magnitude);
}
inline NormalPolynomial Normals(const std::vector<Point>& p,const std::vector<Point>& delta,Face f){
 const auto a=Sub(p[f[1]],p[f[0]]),b=Sub(p[f[2]],p[f[0]]);
 const auto u=Sub(delta[f[1]],delta[f[0]]),v=Sub(delta[f[2]],delta[f[0]]);
 return {Cross(a,b),Add(Cross(u,b),Cross(a,v)),Cross(u,v)};
}
}

// Sparse, material-bound pose correction. Geometry, reference scale, the
// motion frame and excluded anatomy are supplied by a versioned adapter.
// Original-edge slaves and their fixed donors must be excluded together.
// The guard detects intrinsic collapse and newly opposing neighbouring faces;
// it is not a global intersection certificate or an automatic seam weld.
class Plan {
 std::size_t count_;
 std::vector<Face> faces_;
 std::vector<std::uint32_t> ids_,checked_;
 std::vector<std::array<unsigned,2>> pairs_;
 std::vector<Knot> knots_;
 std::vector<double> frameWeights_;
 double areaFloor_;
 double Fraction(const std::vector<Point>& from,const std::vector<Point>& to,
                 const std::vector<double>& areaSquared,const std::vector<bool>& protectPair)const{
  std::vector<Point> delta(count_);for(auto i:ids_)delta[i]=Sub(to[i],from[i]);
  std::vector<detail::NormalPolynomial> normals(faces_.size());std::vector<detail::Polynomial> constraints;constraints.reserve(checked_.size()+pairs_.size());
  for(auto f:checked_){normals[f]=detail::Normals(from,delta,faces_[f]);auto p=detail::Product(normals[f],normals[f]);p[0]-=areaFloor_*areaFloor_*areaSquared[f];constraints.push_back(p);}
  for(unsigned i=0;i<pairs_.size();i++)if(protectPair[i])constraints.push_back(detail::Product(normals[pairs_[i][0]],normals[pairs_[i][1]]));
  auto safe=[&](double a){for(const auto& p:constraints)if(!detail::Nonnegative(p,a))return false;return true;};
  if(safe(1))return 1;
  double lo=0,hi=1;for(unsigned i=0;i<18;i++){double mid=(lo+hi)*.5;if(safe(mid))lo=mid;else hi=mid;}return lo*.999;
 }
 // Correct a strained target locally before falling back to global damping.
 // This is a constraint projection, not a blur of the surrounding surface.
 void Relax(std::vector<Point>& p,const std::vector<double>& areaSquared,
            const std::vector<bool>& protectPair)const{
  std::vector<bool> free(count_,false);for(auto i:ids_)free[i]=true;
  auto gradients=[](const std::vector<Point>& x,Face f,Point q){
   auto a=Cross(Sub(x[f[2]],x[f[0]]),q),b=Cross(q,Sub(x[f[1]],x[f[0]]));
   return std::array<Point,3>{Scale(Add(a,b),-1),a,b};
  };
  for(unsigned iteration=0;iteration<96;iteration++){
   bool changed=false;
   for(auto index:checked_){
    auto f=faces_[index];auto n=Cross(Sub(p[f[1]],p[f[0]]),Sub(p[f[2]],p[f[0]]));double length=std::sqrt(Dot(n,n));
    const double minimum=1.05*areaFloor_*std::sqrt(areaSquared[index]);
    if(length>=minimum||length<1e-15)continue;
    auto g=gradients(p,f,Scale(n,1/length));double denominator=0;
    for(unsigned j=0;j<3;j++)if(free[f[j]])denominator+=Dot(g[j],g[j]);
    if(denominator<1e-20)continue;
    const double step=(minimum-length)/denominator;
    for(unsigned j=0;j<3;j++)if(free[f[j]])p[f[j]]=Add(p[f[j]],Scale(g[j],step));changed=true;
   }
   for(unsigned index=0;index<pairs_.size();index++)if(protectPair[index]){
    auto pair=pairs_[index];auto a=faces_[pair[0]],b=faces_[pair[1]];
    auto na=Cross(Sub(p[a[1]],p[a[0]]),Sub(p[a[2]],p[a[0]])),nb=Cross(Sub(p[b[1]],p[b[0]]),Sub(p[b[2]],p[b[0]]));
    double aa=Dot(na,na),bb=Dot(nb,nb),length=std::sqrt(aa*bb);if(length<1e-20)continue;
    double cosine=Dot(na,nb)/length;if(cosine>=.15)continue;
    auto ga=gradients(p,a,Sub(Scale(nb,1/length),Scale(na,cosine/aa)));
    auto gb=gradients(p,b,Sub(Scale(na,1/length),Scale(nb,cosine/bb)));
    std::array<unsigned,6> vertices{a[0],a[1],a[2],b[0],b[1],b[2]};
    std::array<Point,6> gradient{ga[0],ga[1],ga[2],gb[0],gb[1],gb[2]};
    for(unsigned j=0;j<6;j++)for(unsigned k=0;k<j;k++)if(vertices[j]==vertices[k]){gradient[k]=Add(gradient[k],gradient[j]);gradient[j]={};break;}
    double denominator=0;for(unsigned j=0;j<6;j++)if(free[vertices[j]])denominator+=Dot(gradient[j],gradient[j]);
    if(denominator<1e-20)continue;
    double step=(.15-cosine)/denominator;
    for(unsigned j=0;j<6;j++)if(free[vertices[j]])p[vertices[j]]=Add(p[vertices[j]],Scale(gradient[j],step));changed=true;
   }
   if(!changed)break;
  }
 }
 public:
 Plan(std::size_t count,std::vector<Face> faces,std::vector<std::uint32_t> fixed,
      std::vector<std::uint32_t> ids,std::vector<Knot> knots,double areaFloor=.02,std::vector<double> frameWeights={}):
  count_(count),faces_(std::move(faces)),ids_(std::move(ids)),knots_(std::move(knots)),frameWeights_(std::move(frameWeights)),areaFloor_(areaFloor){
  if(!count_||count_>1000000||faces_.empty()||knots_.empty()||knots_.size()>128||
     !std::isfinite(areaFloor_)||areaFloor_<=0||areaFloor_>1)throw std::invalid_argument("Invalid corrective domain");
  std::vector<bool> locked(count_,false),editable(count_,false),affected(faces_.size(),false);
  for(auto i:fixed){if(i>=count_)throw std::invalid_argument("Invalid fixed corrective row");locked[i]=true;}
  for(auto i:ids_){if(i>=count_||editable[i]||locked[i])throw std::invalid_argument("Corrective touches fixed or duplicate row");editable[i]=true;}
  if(!frameWeights_.empty()&&frameWeights_.size()!=ids_.size())throw std::invalid_argument("Corrective frame bindings differ");
  for(auto w:frameWeights_)if(!std::isfinite(w)||w<0||w>1)throw std::invalid_argument("Invalid corrective frame influence");
  double previous=0;for(const auto& k:knots_){
   if(!std::isfinite(k.weight)||k.weight<=previous||k.weight>1||k.localDisplacement.size()!=ids_.size())throw std::invalid_argument("Invalid corrective knots");
   for(auto p:k.localDisplacement)if(!Finite(p))throw std::invalid_argument("Nonfinite corrective field");previous=k.weight;
  }
  if(previous!=1)throw std::invalid_argument("Corrective must have a complete final knot");
  std::unordered_map<std::uint64_t,std::array<unsigned,2>> edges;
  for(unsigned i=0;i<faces_.size();i++){
   auto f=faces_[i];for(auto v:f){if(v>=count_)throw std::invalid_argument("Invalid corrective triangle");affected[i]=affected[i]||editable[v];}
   if(f[0]==f[1]||f[1]==f[2]||f[0]==f[2])throw std::invalid_argument("Degenerate corrective triangle");
   for(unsigned j=0;j<3;j++){
    auto a=std::min(f[j],f[(j+1)%3]),b=std::max(f[j],f[(j+1)%3]);auto key=(std::uint64_t(a)<<32)|b;
    auto it=edges.find(key);if(it==edges.end())edges.emplace(key,std::array<unsigned,2>{i,UINT32_MAX});
    else if(it->second[1]==UINT32_MAX)it->second[1]=i;
    else throw std::invalid_argument("Nonmanifold corrective edge");
   }
  }
  auto needed=affected;for(const auto& edge:edges){auto pair=edge.second;if(pair[1]!=UINT32_MAX&&(affected[pair[0]]||affected[pair[1]])){pairs_.push_back(pair);needed[pair[0]]=needed[pair[1]]=true;}}
  for(unsigned i=0;i<needed.size();i++)if(needed[i])checked_.push_back(i);
 }
 // Reuse the topology/adjacency preparation for a different authored pose.
 // A caller must retain the same material rows and exact seam constraints.
 void UpdateKnots(std::vector<Knot> knots){
  if(knots.empty()||knots.size()>128)throw std::invalid_argument("Invalid corrective path length");
  double previous=0;
  for(const auto& k:knots){
   if(!std::isfinite(k.weight)||k.weight<=previous||k.weight>1||k.localDisplacement.size()!=ids_.size())throw std::invalid_argument("Invalid corrective knots");
   for(auto p:k.localDisplacement)if(!Finite(p))throw std::invalid_argument("Nonfinite corrective field");previous=k.weight;
  }
  if(previous!=1)throw std::invalid_argument("Corrective must have a complete final knot");
  knots_.swap(knots);
 }
 // Immediate vertex-buffer evaluation. This checks the actual output mesh;
 // it does not claim safety for an animation through activation weights.
 // Call Apply instead when the caller displays that activation trajectory.
 Result EvaluateEndpoint(const std::vector<Point>& original,const Frame& frame,Point scale,double requested,
   const std::function<bool(const std::vector<Point>&,std::vector<Point>&)>& repair={})const{
  if(original.size()!=count_||!std::isfinite(requested)||requested<0||requested>1||!frame.Valid()||!Finite(scale)||std::any_of(scale.begin(),scale.end(),[](double x){return x<0;}))throw std::invalid_argument("Invalid corrective pose");
  Result result{original,0};if(!requested||ids_.empty())return result;
  const Knot* left=nullptr;const Knot* right=&knots_.back();
  for(const auto& k:knots_){right=&k;if(k.weight>=requested)break;left=&k;}
  const double previous=left?left->weight:0,t=Smooth((requested-previous)/(right->weight-previous));
  for(unsigned i=0;i<ids_.size();i++){
   auto delta=Add(Scale(left?left->localDisplacement[i]:Point{},1-t),Scale(right->localDisplacement[i],t));
   for(unsigned a=0;a<3;a++)delta[a]*=scale[a];result.positions[ids_[i]]=Add(original[ids_[i]],frame.ToWorld(delta));
  }
  std::vector<Point> before(faces_.size()),after(faces_.size());std::vector<double> aa(faces_.size()),bb(faces_.size());
  auto valid=[&](){
   for(auto i:checked_){auto f=faces_[i];before[i]=Cross(Sub(original[f[1]],original[f[0]]),Sub(original[f[2]],original[f[0]]));after[i]=Cross(Sub(result.positions[f[1]],result.positions[f[0]]),Sub(result.positions[f[2]],result.positions[f[0]]));aa[i]=Dot(before[i],before[i]);bb[i]=Dot(after[i],after[i]);if(!std::isfinite(aa[i]+bb[i])||aa[i]<=0||bb[i]<areaFloor_*areaFloor_*aa[i])return false;}
   for(auto pair:pairs_)if(Dot(before[pair[0]],before[pair[1]])>.5*std::sqrt(aa[pair[0]]*aa[pair[1]])&&Dot(after[pair[0]],after[pair[1]])<0)return false;
   return true;
  };
  if(!valid()&&(!repair||!repair(original,result.positions)||!valid()))return {original,0};
  result.appliedWeight=requested;return result;
 }
 Result Apply(const std::vector<Point>& original,const Frame& frame,Point scale,double requested,bool relax=false,const Frame* anchor=nullptr)const{
  if(original.size()!=count_||!std::isfinite(requested)||requested<0||requested>1||
     !frame.Valid()||(anchor&&!anchor->Valid())||!Finite(scale)||std::any_of(scale.begin(),scale.end(),[](double x){return x<0;}))throw std::invalid_argument("Invalid corrective pose");
  Result out{original,0};if(!requested||ids_.empty())return out;
  std::vector<Point> zero(count_);std::vector<detail::NormalPolynomial> reference(faces_.size());
  std::vector<double> areaSquared(faces_.size());std::vector<bool> protectPair(pairs_.size());
  for(auto f:checked_){reference[f]=detail::Normals(original,zero,faces_[f]);areaSquared[f]=Dot(reference[f][0],reference[f][0]);if(!std::isfinite(areaSquared[f])||areaSquared[f]<=0)return out;}
  for(unsigned i=0;i<pairs_.size();i++){auto pair=pairs_[i];protectPair[i]=Dot(reference[pair[0]][0],reference[pair[1]][0])>.5*std::sqrt(areaSquared[pair[0]]*areaSquared[pair[1]]);}
  std::vector<Point> last(ids_.size());double previous=0,acceptedGoal=0;
  for(const auto& k:knots_){
   const double finalGoal=std::min(requested,k.weight);const unsigned steps=relax?8:1;
   for(unsigned step=1;step<=steps;step++){
   const double goal=previous+(finalGoal-previous)*step/steps,t=Smooth((goal-previous)/(k.weight-previous));
   auto target=original;
   for(unsigned i=0;i<ids_.size();i++){
    auto p=Add(Scale(last[i],1-t),Scale(k.localDisplacement[i],t));for(unsigned a=0;a<3;a++)p[a]*=scale[a];
    auto world=frame.ToWorld(p);
    if(anchor){const double w=frameWeights_.empty()?0:frameWeights_[i];world=Add(Scale(anchor->ToWorld(p),1-w),Scale(world,w));}
    target[ids_[i]]=Add(original[ids_[i]],world);
   }
   double preparedFraction=Fraction(out.positions,target,areaSquared,protectPair);
   // Preserve an already valid authored shape exactly. Local constraint work
   // is needed only when the intended chord actually reaches a fold boundary.
   if(relax&&preparedFraction<1){Relax(target,areaSquared,protectPair);preparedFraction=-1;}
   bool reached=false;
   for(unsigned retry=0;retry<(relax?5u:1u);retry++){
    const double alpha=retry==0&&preparedFraction>=0?preparedFraction:Fraction(out.positions,target,areaSquared,protectPair);
    for(auto i:ids_)out.positions[i]=Add(out.positions[i],Scale(Sub(target[i],out.positions[i]),alpha));
    out.appliedWeight=acceptedGoal+(goal-acceptedGoal)*alpha;acceptedGoal=out.appliedWeight;
    if(alpha==1){reached=true;break;}
    if(!relax)return out;
    // A straight chord can hit the fold boundary even with valid endpoints.
    // Move away along the local constraint gradient, then approach the same
    // target again. Every escape chord gets the same continuous safety check.
    auto escaped=out.positions;Relax(escaped,areaSquared,protectPair);
    const double escape=Fraction(out.positions,escaped,areaSquared,protectPair);double movement=0;
    for(auto i:ids_){auto d=Scale(Sub(escaped[i],out.positions[i]),escape);movement=std::max(movement,Dot(d,d));out.positions[i]=Add(out.positions[i],d);}
    if(movement<1e-20)return out;
   }
   if(!reached)return out;acceptedGoal=goal;
   }
   if(finalGoal==requested)return out;
   previous=k.weight;last=k.localDisplacement;
  }
  return out;
 }
};
}
