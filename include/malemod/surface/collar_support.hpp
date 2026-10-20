#pragma once
#include "recruitment_constraints.hpp"
#include <memory>

namespace malemod::surface {
// Material-bound support of the recruited collar. The adapter supplies its
// measured body membership and immutable anatomy coordinates. The fitted
// endpoint is the current, unchanged healthy shaft, not a nominal cylinder.
class CollarSupport {
 using P=PrecisePoint;
 GraftDomain domain_;
 std::vector<bool> body_;
 std::vector<double> participation_,angle_,station_,distance_;
 std::vector<unsigned> opening_,healthy_;
 std::vector<std::pair<double,unsigned>> orderedOpening_;
 double end_;
 std::unique_ptr<RecruitmentConstraints> constraints_;
 std::unique_ptr<GraftPlan> fair_;
 double fairReach_=0,neutral_=0;
 GraftFrame cachedFrame_{};
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 static std::array<double,7> Terms(double a){return {1,std::cos(a),std::sin(a),std::cos(2*a),std::sin(2*a),std::cos(3*a),std::sin(3*a)};}
 static double Dot(P a,P b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
 static bool Same(const GraftFrame& a,const GraftFrame& b){return a.root==b.root&&a.axis==b.axis&&a.up==b.up&&a.radius==b.radius&&a.length==b.length&&a.sourceLengthScale==b.sourceLengthScale&&a.seamSupportRadius==b.seamSupportRadius;}
 std::array<double,14> Fit(const std::vector<P>& p,P up)const{
  double matrix[14][15]{};
  for(auto i:healthy_){auto basis=Terms(angle_[i]);std::array<double,14> row{};for(unsigned j=0;j<7;j++){row[j]=basis[j];row[j+7]=basis[j]*(station_[i]-end_);}for(unsigned a=0;a<14;a++){for(unsigned b=0;b<14;b++)matrix[a][b]+=row[a]*row[b];matrix[a][14]+=row[a]*Dot(p[i],up);}}
  double ridge=matrix[0][0]*1e-11;for(unsigned i=0;i<14;i++)matrix[i][i]+=ridge;
  for(unsigned i=0;i<14;i++){unsigned pivot=i;for(unsigned j=i+1;j<14;j++)if(std::abs(matrix[j][i])>std::abs(matrix[pivot][i]))pivot=j;if(std::abs(matrix[pivot][i])<1e-20)throw std::runtime_error("Collar healthy section is underdetermined");if(pivot!=i)for(unsigned j=0;j<15;j++)std::swap(matrix[i][j],matrix[pivot][j]);double d=matrix[i][i];for(unsigned j=i;j<15;j++)matrix[i][j]/=d;for(unsigned j=0;j<14;j++)if(j!=i){double w=matrix[j][i];for(unsigned k=i;k<15;k++)matrix[j][k]-=w*matrix[i][k];}}
  std::array<double,14> result{};for(unsigned i=0;i<14;i++)result[i]=matrix[i][14];return result;
 }
 double OpeningHeight(double a,const std::vector<P>& p,P up)const{
  constexpr double tau=6.28318530717958647692;
  auto next=std::upper_bound(orderedOpening_.begin(),orderedOpening_.end(),a,[](double a,const auto& entry){return a<entry.first;});
  auto hi=next==orderedOpening_.end()?orderedOpening_.front():*next;
  auto lo=next==orderedOpening_.begin()?orderedOpening_.back():*(next-1);
  if(next==orderedOpening_.end())hi.first+=tau;
  if(next==orderedOpening_.begin())lo.first-=tau;
  double span=hi.first-lo.first,w=span>1e-12?(a-lo.first)/span:0;
  return (1-w)*Dot(p[lo.second],up)+w*Dot(p[hi.second],up);
 }
 public:
 CollarSupport(GraftDomain domain,std::vector<bool> body,std::vector<double> participation,std::vector<double> angle,std::vector<double> station,double end=.36):domain_(std::move(domain)),body_(std::move(body)),participation_(std::move(participation)),angle_(std::move(angle)),station_(std::move(station)),end_(end){
  const auto n=domain_.points.size();if(body_.size()!=n||participation_.size()!=n||angle_.size()!=n||station_.size()!=n||!std::isfinite(end_)||end_<=.06)throw std::invalid_argument("Invalid collar material domain");
  std::vector<bool> border(n),slave(n),locked(n);for(auto e:domain_.seams)slave.at(e.slave)=true;for(auto i:domain_.protectedVertices)locked.at(i)=true;
  auto prescribed=std::move(domain_.prescribedVertices);domain_.prescribedVertices.clear();for(auto i:prescribed)if(!locked.at(i)&&!slave.at(i)){domain_.prescribedVertices.push_back(i);locked[i]=true;}
  for(auto f:domain_.triangles){bool mixed=body_.at(f[0])!=body_.at(f[1])||body_.at(f[1])!=body_.at(f[2]);if(mixed)for(auto i:f)if(body_[i])border[i]=true;}
  distance_.resize(n,1e100);
  for(unsigned i=0;i<n;i++){
   if(!std::isfinite(participation_[i])||participation_[i]<0||participation_[i]>1||!std::isfinite(angle_[i])||!std::isfinite(station_[i]))throw std::invalid_argument("Nonfinite collar material binding");
   if(border[i]){opening_.push_back(i);orderedOpening_.push_back({angle_[i],i});}
   if(!body_[i]&&participation_[i]>.99&&station_[i]>end_-.06&&station_[i]<end_+.06)healthy_.push_back(i);
   if((body_[i]||participation_[i]==0||station_[i]>=end_)&&!locked[i]&&!slave[i]){domain_.prescribedVertices.push_back(i);locked[i]=true;}
  }
  if(opening_.size()<3||healthy_.size()<30)throw std::invalid_argument("Measured collar boundary/healthy section missing");
  std::sort(orderedOpening_.begin(),orderedOpening_.end());
  for(unsigned i=0;i<n;i++)for(auto j:opening_){double ds=0;for(unsigned a=0;a<3;a++)ds+=(domain_.points[i][a]-domain_.points[j][a])*(domain_.points[i][a]-domain_.points[j][a]);distance_[i]=std::min(distance_[i],std::sqrt(ds));}
  constraints_=std::make_unique<RecruitmentConstraints>(domain_);
 }
 std::vector<P> Apply(const std::vector<P>& reference,const std::vector<P>& expanded,const GraftFrame& frame,double neutral,P up={0,0,1},double response=1){
  if(reference.size()!=domain_.points.size()||expanded.size()!=reference.size()||!std::isfinite(neutral)||neutral<=0||!std::isfinite(frame.radius)||frame.radius<=0||!std::isfinite(response)||response<0||response>1||!std::isfinite(Dot(up,up))||std::abs(Dot(up,up)-1)>1e-6)throw std::invalid_argument("Invalid collar support pose");
  for(const auto* list:{&reference,&expanded})for(auto p:*list)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite collar support positions");
  response*=Smooth((frame.radius-neutral)/(.75*neutral));
  if(response==0)return expanded;
  auto coefficient=Fit(reference,up);auto goal=expanded;std::vector<SurfaceSupportPlane> support;
  for(unsigned i=0;i<goal.size();i++){
   if(body_[i]||participation_[i]<=.99||station_[i]<=0||station_[i]>=end_||std::sin(angle_[i])<=.05)continue;
   auto basis=Terms(angle_[i]);double finish=0;for(unsigned j=0;j<7;j++)finish+=basis[j]*coefficient[j];
   const double t=Smooth(station_[i]/end_),start=OpeningHeight(angle_[i],expanded,up),floor=(1-t)*start+t*finish;
   const double weight=response*participation_[i]*Smooth(std::sin(angle_[i])/.65)*Smooth(distance_[i]/neutral)*(1-Smooth((station_[i]-(end_-.09))/.09));
   const double increment=weight*std::max(0.,floor-Dot(goal[i],up));
   if(increment<=.0034*neutral)continue;
   for(unsigned a=0;a<3;a++)goal[i][a]+=up[a]*increment;
   support.push_back({i,up,Dot(goal[i],up)});
  }
  if(support.empty())return expanded;
  auto out=constraints_->Project(reference,goal,support);
  // Cover the complete supported material patch. A fixed seven-unit window
  // left the 9.7-11.8-unit crown rows outside the surface curvature energy.
  double reach=0;for(auto plane:support)reach=std::max(reach,distance_[plane.vertex]);reach+=neutral;
  auto f=frame;f.seamSupportRadius=reach;
  if(!fair_||reach!=fairReach_||neutral!=neutral_){auto d=domain_;d.coupledCurvature=true;d.preserveTargetDifferential=false;d.surfaceFairingDistance=reach;d.surfaceFairingStrength=64;
   // All body masters are fixed during this solve. Its preparation headroom
   // must not reject an already guarded body. The independent projection
   // below still checks signed body area and all surface inequalities.
   d.orientationTriangles.clear();fair_=std::make_unique<GraftPlan>(d,f);fairReach_=reach;neutral_=neutral;cachedFrame_=f;}
  else if(!Same(f,cachedFrame_)){fair_->UpdateFrame(f);cachedFrame_=f;}
  for(unsigned pass=0;pass<4;pass++){
   auto delta=out;for(unsigned i=0;i<delta.size();i++)for(unsigned a=0;a<3;a++)delta[i][a]-=domain_.points[i][a];
   auto solved=fair_->SolveDisplacement(delta),candidate=domain_.points;
   for(unsigned i=0;i<delta.size();i++)for(unsigned a=0;a<3;a++)candidate[i][a]+=solved[i][a];
   for(auto i:domain_.prescribedVertices)candidate[i]=expanded[i];
   for(auto i:domain_.protectedVertices)candidate[i]=expanded[i];
   out=constraints_->Project(out,candidate,support);
  }
  return out;
 }
};
}
