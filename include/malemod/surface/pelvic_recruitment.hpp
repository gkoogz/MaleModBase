#pragma once
#include "graft_runtime.hpp"
#include "root_motion.hpp"

namespace malemod::surface {
// An opt-in material field: the measured opening expands and follows the
// attachment; the support annulus grows with dilation. Adapters supply actual
// coordinates, motion and material participation. No engine/resource guesses.
class PelvicRecruitment {
 using P=PrecisePoint;
 std::vector<PrecisePoint> rest_;
 std::vector<double> radius_,distance_;
 PrecisePoint root_{},forward_,up_,side_;
 std::vector<unsigned> openingIDs_;std::vector<double> openingAngles_;
 double openingForwardMax_=0;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(t*(t*6-15)+10);}
 struct Sample {PrecisePoint delta{};double fraction=0;};
 Sample Field(unsigned i,const GraftFrame& frame,double neutral)const{
  using namespace root_motion;
  if(i>=rest_.size()||!std::isfinite(neutral)||neutral<=0||!std::isfinite(frame.radius)||frame.radius<=0||!std::isfinite(frame.sourceLengthScale)||frame.sourceLengthScale<=0)throw std::invalid_argument("Invalid pelvic recruitment dimensions");
  Finite(frame.root);const auto q=Add(rest_[i],Scale(root_,-1));
  const double s=Dot(q,forward_),z=Dot(q,up_),y=Dot(q,side_),rho=std::hypot(y,z),r0=radius_[i];
  const double growth=Growth(frame.radius,neutral);
  if(!growth||rho<1e-10*frame.sourceLengthScale)return {};
  const double inferior=std::pow(std::max(0.,-z/rho),2),lateral=y*y/(rho*rho);
  const double raised=Smooth(Dot(frame.axis,up_)/.2);
  const double cosine=Dot(frame.axis,forward_);
  const double desiredRadius=frame.radius*(1+.22*inferior+.08*lateral);
  const double dilation=std::max(0.,desiredRadius-r0)*growth;
  if(!dilation)return {};
  // The radial derivative stays positive: width > 1.875*dilation, the
  // maximum derivative of the quintic. There is no compressed contact ring.
  const double width=.81*r0+2.18*dilation,t=std::max(0.,rho-r0)/width;
  if(t>=1)return {};
  const double bell=1-Smooth(t);
  const double anterior=Smooth((s+3*neutral)/(1.5*neutral));
  const double depth=1-Smooth(std::max(0.,s)/(3*frame.radius));
  const double weight=anterior*depth;
  if(!weight)return {};
  const auto radial=Add(Scale(up_,z/rho),Scale(side_,y/rho));
  Sample out;out.fraction=growth*bell*weight*raised;
  // The growing socket covers the proximal pinned support. Flatten its
  // measured axial irregularity as it becomes a shaft-sized section, then
  // turn that section with the live attachment. The original exterior fades
  // to its exact measured shape, rather than becoming an enlarged flat wall.
  auto material=Add(q,Scale(radial,dilation*bell*weight));
  material=Add(material,Scale(forward_,-s*out.fraction));
  Rotation rotation(forward_,frame.axis,up_);
  out.delta=Add(rotation.Apply(material,out.fraction),Scale(q,-1));
  // Recruit the original pelvis towards the observed attachment centre.
  // A broad annular forward ramp adds depth without changing the mouth size.
  const double socketAdvance=.95*frame.radius*raised;
  const double compensation=raised*std::max(0.,frame.radius*std::max(0.,Dot(frame.axis,up_))-socketAdvance*cosine-Dot(Add(frame.root,Scale(root_,-1)),forward_)+openingForwardMax_);
  const auto anchor=Add(Add(frame.root,Scale(frame.axis,socketAdvance)),Scale(forward_,compensation));
  out.delta=Add(out.delta,Scale(Add(anchor,Scale(root_,-1)),bell*weight*growth));
  out.delta=Add(out.delta,Scale(forward_,.7*dilation*4*bell*(1-bell)*weight));
  return out;
 }
 public:
 static double Growth(double radius,double neutral){
  if(!std::isfinite(radius)||radius<=0||!std::isfinite(neutral)||neutral<=0)throw std::invalid_argument("Invalid pelvic growth");
  return Smooth((radius-neutral)/(.75*neutral));
 }
 PelvicRecruitment(std::vector<PrecisePoint> rest,const std::vector<PrecisePoint>& opening,PrecisePoint forward={1,0,0},PrecisePoint up={0,0,1}):rest_(std::move(rest)),forward_(root_motion::Unit(forward)),up_(root_motion::Unit(up)){
  using namespace root_motion;
  if(rest_.empty()||opening.size()<3||std::abs(Dot(forward_,up_))>1e-6)throw std::invalid_argument("Incomplete pelvic recruitment frame");
  side_=Cross(up_,forward_);
  for(auto p:opening){Finite(p);root_=Add(root_,Scale(p,1./opening.size()));}
  std::vector<PrecisePoint> section;
  for(auto p:opening){const auto q=Add(p,Scale(root_,-1));const double z=Dot(q,up_),y=Dot(q,side_),r=std::hypot(z,y);if(r>1e-10)section.push_back({z/r,y/r,r});}
  if(section.size()<3)throw std::invalid_argument("Degenerate pelvic recruitment opening");
  for(auto p:rest_){Finite(p);const auto q=Add(p,Scale(root_,-1));const double z=Dot(q,up_),y=Dot(q,side_),r=std::hypot(z,y);const double uz=r>1e-10?z/r:1,uy=r>1e-10?y/r:0;double total=0,sum=0,best=INFINITY;
   for(auto v:section){const double w=std::exp(24*(uz*v[0]+uy*v[1]-1));total+=w;sum+=w*v[2];}
   for(auto v:opening){const auto d=Add(p,Scale(v,-1));best=std::min(best,Dot(d,d));}
   radius_.push_back(sum/total);distance_.push_back(std::sqrt(best));
  }
  for(auto p:opening){double best=INFINITY;unsigned id=0;for(unsigned i=0;i<rest_.size();i++){auto q=Add(rest_[i],Scale(p,-1));const auto d=Dot(q,q);if(d<best){best=d;id=i;}}openingIDs_.push_back(id);const auto q=Add(p,Scale(root_,-1));openingAngles_.push_back(std::atan2(Dot(q,up_),Dot(q,side_)));openingForwardMax_=std::max(openingForwardMax_,Dot(q,forward_));}
 }
 PrecisePoint Root()const{return root_;}
 PrecisePoint BodyDisplacement(unsigned i,const GraftFrame& frame,double neutral)const{return Field(i,frame,neutral).delta;}
 std::vector<PrecisePoint> Loft(const std::vector<PrecisePoint>& moving,const GraftFrame& frame,double neutral,const std::vector<double>& participation,const std::vector<double>& angle,const std::vector<double>& station,const std::vector<PrecisePoint>& guide)const{
  using namespace root_motion;
  if(moving.size()!=rest_.size()||participation.size()!=moving.size()||angle.size()!=moving.size()||station.size()!=moving.size()||guide.size()<3)throw std::invalid_argument("Incomplete pelvic loft material chart");
  auto out=moving;std::vector<P> opening;
  for(auto i:openingIDs_)opening.push_back(Add(rest_[i],Field(i,frame,neutral).delta));
  auto section=[&](double t,double phi){const double u=std::clamp(t,0.,1.)*(guide.size()-1);const unsigned i=std::min(unsigned(u),unsigned(guide.size()-2));const double f=u-i;const auto center=Add(Scale(guide[i],1-f),Scale(guide[i+1],f));const auto axis=Unit(Add(guide[i+1],Scale(guide[i],-1)));const auto side=Unit(Add(side_,Scale(axis,-Dot(side_,axis))));const auto up=Cross(axis,side);return Add(center,Scale(Add(Scale(side,std::cos(phi)),Scale(up,std::sin(phi))),frame.radius));};
  constexpr double end=.30;
  auto basis=[](double phi,double s){std::array<double,14> v{};v[0]=1;for(unsigned k=1;k<=3;k++){v[2*k-1]=std::cos(k*phi);v[2*k]=std::sin(k*phi);}for(unsigned k=0;k<7;k++)v[k+7]=v[k]*s;return v;};
  double fit[14][17]{};unsigned samples=0;
  for(unsigned i=0;i<moving.size();i++)if(participation[i]>.8&&station[i]>.24&&station[i]<.36){
   const auto v=basis(angle[i],station[i]-end);const auto p=Add(rest_[i],moving[i]);samples++;
   for(unsigned a=0;a<14;a++){for(unsigned b=0;b<14;b++)fit[a][b]+=v[a]*v[b];for(unsigned b=0;b<3;b++)fit[a][14+b]+=v[a]*p[b];}
  }
  bool fitted=samples>=30;
  if(fitted){double ridge=fit[0][0]*1e-9;for(unsigned i=0;i<14;i++)fit[i][i]+=ridge;for(unsigned i=0;i<14;i++){unsigned pivot=i;for(unsigned j=i+1;j<14;j++)if(std::abs(fit[j][i])>std::abs(fit[pivot][i]))pivot=j;if(std::abs(fit[pivot][i])<1e-14){fitted=false;break;}if(pivot!=i)for(unsigned j=0;j<17;j++)std::swap(fit[i][j],fit[pivot][j]);const double d=fit[i][i];for(unsigned j=i;j<17;j++)fit[i][j]/=d;for(unsigned j=0;j<14;j++)if(j!=i){const double w=fit[j][i];for(unsigned k=i;k<17;k++)fit[j][k]-=w*fit[i][k];}}}
  auto fittedSection=[&](double phi,bool derivative){P p{};const auto v=basis(phi,0);for(unsigned k=0;k<7;k++)for(unsigned a=0;a<3;a++)p[a]+=v[k]*fit[k+(derivative?7:0)][14+a];return p;};
  for(unsigned i=0;i<out.size();i++){
   if(!participation[i]||station[i]<=0||station[i]>=end)continue;
   if(!std::isfinite(angle[i]+station[i]+participation[i]))throw std::invalid_argument("Nonfinite pelvic loft chart");
   P start{};double sum=0;for(unsigned j=0;j<opening.size();j++){const double w=std::exp(48*(std::cos(angle[i]-openingAngles_[j])-1));start=Add(start,Scale(opening[j],w));sum+=w;}start=Scale(start,1/sum);
   P target;
   if(station[i]<end){
    const auto finish=fitted?fittedSection(angle[i],false):section(end,angle[i]);auto tangent=fitted?Scale(fittedSection(angle[i],true),end):Scale(Add(section(end+.0001,angle[i]),Scale(section(end-.0001,angle[i]),-1)),end/.0002);
    auto departure=Scale(Add(finish,Scale(start,-1)),.6);
    for(unsigned a=0;a<3;a++){const double change=finish[a]-start[a];if(change==0)departure[a]=tangent[a]=0;else{const double sign=change>0?1:-1;tangent[a]=sign*std::clamp(sign*tangent[a],0.,3*std::abs(change)-std::abs(departure[a]));}}
    // Dorsal departure is monotone in height: an interior dip is not an
    // allowed interpolation of two healthy boundary sections.
    const double t=station[i]/end,t2=t*t,t3=t2*t;
    target=Add(Add(Scale(start,2*t3-3*t2+1),Scale(departure,t3-2*t2+t)),Add(Scale(finish,-2*t3+3*t2),Scale(tangent,t3-t2)));
   }else target=section(station[i],angle[i]);
   const double weight=participation[i]*Smooth(std::sin(angle[i])/.65)*Smooth(frame.axis[2]/.2)*(1-Smooth((station[i]-.20)/.10));
   const auto posed=Add(rest_[i],moving[i]);auto change=Scale(Add(target,Scale(posed,-1)),weight);change[1]=0;out[i]=Add(moving[i],change);
  }
  return out;
 }
 std::vector<PrecisePoint> Evaluate(const std::vector<PrecisePoint>& moving,const GraftFrame& frame,double neutral,const std::vector<double>& participation)const{
  using namespace root_motion;
  if(moving.size()!=rest_.size()||participation.size()!=moving.size())throw std::invalid_argument("Pelvic material topology differs");
  auto out=moving;Rotation rotation(forward_,frame.axis,up_);
  const double reach=std::max(2*neutral,1.5*frame.radius);
  for(unsigned i=0;i<out.size();i++){
   if(!std::isfinite(participation[i])||participation[i]<0||participation[i]>1)throw std::invalid_argument("Invalid pelvic material participation");
   Finite(out[i]);if(participation[i]==0||distance_[i]>=reach)continue;
   const auto fixed=Field(i,frame,neutral);const double t=Smooth(distance_[i]/reach);
   const auto q=Add(rest_[i],Scale(root_,-1));
   const auto a=rotation.Apply(Add(q,fixed.delta),-fixed.fraction),b=rotation.Apply(Add(q,moving[i]),-1);
   const auto material=Add(Scale(a,1-t),Scale(b,t));
   const auto delta=Add(rotation.Apply(material,fixed.fraction+(1-fixed.fraction)*t),Scale(q,-1));
   out[i]=Add(Scale(out[i],1-participation[i]),Scale(delta,participation[i]));
  }
  return out;
 }
};
}
