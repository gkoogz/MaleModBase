#pragma once
#include "presentation.hpp"
#include <queue>
#include <limits>
#include <map>

namespace malemod::surface {
// Material-space skin relief. This changes display geometry only: mechanical
// owners, volumes, socket positions and simulation cadence remain independent.
class SkinRelief {
 using Vec=PresentationPoint;
 struct Sample {double x=0,z=0,front=0,fade=0,metric=0,detail=0,web=0;};
 std::vector<Sample> samples_;
 std::vector<std::array<unsigned,3>> faces_;
 std::vector<std::array<unsigned,2>> edges_;
 struct Stencil {std::array<unsigned,8> nodes{};std::array<double,8> weights{};};
 std::vector<Stencil> stencils_;std::vector<double> gridMetric_;
 double radius_=1;
 static double Smooth(double t){t=std::clamp(t,0.,1.);return t*t*t*(10+t*(-15+6*t));}
 static std::array<double,2> Pattern(const Sample& s){
   const double x=s.x,z=s.z;
   const double neck=Smooth((z-.25)/1.25),bottom=1-Smooth((z+.8)/1.5);
   const double phase=22*z+5*x+2.1*std::sin(4.1*x-2.3*z)+1.4*std::sin(7.3*x+3.7*z);
   const double patches=.35+.65*std::pow(std::sin(4.7*x+2.9*z+.8*std::sin(3*z)),2);
   const double wrinkles=(.004+.0015*bottom)*patches*(std::sin(phase)+.3*std::sin(37*z-12*x+std::sin(5*x))+ .15*std::sin(49*z+21*x));
   const double neckFolds=.012*neck*std::sin(10*x+1.2*std::sin(2.1*z)+.6*x*z);
   // A shallow longitudinal web beside the raphe, fading into the sac and
   // the existing attachment. Its varying width avoids a second cord/seam.
   const double lateral=.22+.065*Smooth((z-.1)/2),width=.17+.055*(1-neck);
   const double ridge=std::exp(-std::pow((std::abs(x)-lateral)/width,2));
   const double central=Smooth(std::abs(x)/.12);
   // The longitudinal web belongs to the connecting skin, not the front of
   // the lobes. Carrying it down their medial faces made a projecting seam.
   const double web=.045*ridge*central*Smooth((z-.45)/.8)*s.front;
   // A shallow, compact medial envelope replaces the forward bridge with
   // skin suspended between the two supports. C2 falloffs leave the lobe
   // crowns, back surface and mounted rim unchanged. Coordinates are fixed
   // to the material, so this cannot cancel swing or move mechanical bodies.
   const double medial=std::pow((std::max)(0.,1-std::pow(x/.45,2)),3);
   const double bridge=.12*medial*Smooth((z+1.05)/.35)*(1-Smooth((z-.05)/.6))*s.front;
  return {(wrinkles+neckFolds)*(.55+.45*s.front),web-bridge};
 }
 public:
 SkinRelief()=default;
 SkinRelief(const std::vector<Vec>& rest,const std::vector<std::array<unsigned,3>>& faces,
  const std::vector<bool>& active,Vec center,PresentationBasis chart,double radius):radius_(radius){
  if(rest.size()!=active.size()||!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid skin relief material");
  samples_.resize(rest.size());std::vector<Vec> normals(rest.size());
  std::vector<std::vector<std::pair<unsigned,double>>> adjacency(rest.size());
  std::vector<std::array<unsigned,2>> edges;
  for(auto f:faces){
   for(auto i:f)if(i>=rest.size())throw std::invalid_argument("Skin relief face outside material");
   auto n=Cross(Sub(rest[f[1]],rest[f[0]]),Sub(rest[f[2]],rest[f[0]]));
   for(auto i:f)normals[i]=Add(normals[i],n);
   if(active[f[0]]||active[f[1]]||active[f[2]]){
    faces_.push_back(f);
    for(unsigned k=0;k<3;k++){auto a=f[k],b=f[(k+1)%3];edges.push_back({(std::min)(a,b),(std::max)(a,b)});}
   }
  }
  std::sort(edges.begin(),edges.end());edges.erase(std::unique(edges.begin(),edges.end()),edges.end());edges_=std::move(edges);
  for(auto e:edges_){double length=std::sqrt(Dot(Sub(rest[e[0]],rest[e[1]]),Sub(rest[e[0]],rest[e[1]])));
   for(unsigned k=0;k<2;k++){adjacency[e[k]].push_back({e[1-k],length});samples_[e[k]].metric+=length;}}
  // Geodesic boundary fade has zero first/second derivatives. The complete
  // shaft attachment remains exact; folds continue across the lower skin join.
  std::vector<double> distance(rest.size(),std::numeric_limits<double>::infinity());
  using Node=std::pair<double,unsigned>;std::priority_queue<Node,std::vector<Node>,std::greater<Node>> queue;
  for(unsigned i=0;i<rest.size();i++)if(!active[i]){distance[i]=0;queue.push({0,i});}
  while(!queue.empty()){auto [d,i]=queue.top();queue.pop();if(d!=distance[i])continue;
   for(auto [j,length]:adjacency[i])if(d+length<distance[j]){distance[j]=d+length;queue.push({distance[j],j});}}
  for(unsigned i=0;i<rest.size();i++)if(active[i]){
   if(Dot(normals[i],normals[i])<1e-24)throw std::invalid_argument("Skin relief has undefined normal");
   auto delta=Scale(Sub(rest[i],center),1/radius);auto& s=samples_[i];
   s.x=Dot(delta,chart[1]);s.z=Dot(delta,chart[2]);
   s.front=Smooth((Dot(Unit(normals[i]),chart[0])+.15)/.8);
   s.fade=Smooth(distance[i]/(.35*radius));
  }
  // The cut/loft contains needle triangles with nearly coincident donors.
  // Their raw per-vertex normal/strain estimates depend on valence. Deposit
  // these estimates on a material-space grid, then interpolate one continuous
  // field at every original vertex. No topology or base surface is smoothed.
  using Cell=std::array<long long,3>;std::map<Cell,unsigned> nodes;
  stencils_.resize(rest.size());const double cellSize=.3*radius;
  for(unsigned i=0;i<rest.size();i++){
   Cell cell;Vec fraction;
   for(unsigned k=0;k<3;k++){double v=(rest[i][k]-center[k])/cellSize;cell[k]=static_cast<long long>(std::floor(v));fraction[k]=v-cell[k];}
   unsigned j=0;for(unsigned x=0;x<2;x++)for(unsigned y=0;y<2;y++)for(unsigned z=0;z<2;z++){
    Cell key{cell[0]+x,cell[1]+y,cell[2]+z};auto entry=nodes.emplace(key,unsigned(nodes.size()));
    stencils_[i].nodes[j]=entry.first->second;
    stencils_[i].weights[j]=(x?fraction[0]:1-fraction[0])*(y?fraction[1]:1-fraction[1])*(z?fraction[2]:1-fraction[2]);++j;
   }
  }
  gridMetric_.resize(nodes.size());std::vector<Vec> gridNormals(nodes.size());
  for(unsigned i=0;i<rest.size();i++)for(unsigned k=0;k<8;k++){auto node=stencils_[i].nodes[k];auto w=stencils_[i].weights[k];gridMetric_[node]+=samples_[i].metric*w;gridNormals[node]=Add(gridNormals[node],Scale(normals[i],w));}
  for(unsigned i=0;i<rest.size();i++)if(active[i]){Vec normal{};for(unsigned k=0;k<8;k++){auto node=stencils_[i].nodes[k];if(Dot(gridNormals[node],gridNormals[node])>1e-24)normal=Add(normal,Scale(Unit(gridNormals[node]),stencils_[i].weights[k]));}samples_[i].front=Smooth((Dot(Unit(normal),chart[0])+.15)/.8);auto pattern=Pattern(samples_[i]);samples_[i].detail=pattern[0];samples_[i].web=pattern[1];}

 }
 bool Active(unsigned i)const{return i<samples_.size()&&samples_[i].fade>0;}
 struct Result {double amplitude=1,maximumDisplacement=0,attemptedMaximum=0;unsigned changed=0;std::array<unsigned,3> limitingFace{};};
 Result Apply(std::vector<Vec>& points,double scale=1,std::vector<Vec>* normalDelta=nullptr)const{
  if(samples_.empty())return {};
  if(points.size()!=samples_.size()||!std::isfinite(scale)||scale<=0)throw std::invalid_argument("Invalid posed skin relief");
  std::vector<Vec> normals(points.size()),delta(points.size());std::vector<double> metric(points.size()),detail(points.size()),webHeight(points.size());
  for(auto f:faces_){auto n=Cross(Sub(points[f[1]],points[f[0]]),Sub(points[f[2]],points[f[0]]));for(auto i:f)normals[i]=Add(normals[i],n);}
  for(auto e:edges_){auto d=Sub(points[e[0]],points[e[1]]);double length=std::sqrt(Dot(d,d));for(auto i:e)metric[i]+=length;}
  std::vector<Vec> gridNormals(gridMetric_.size());std::vector<double> gridMetric(gridMetric_.size());
  for(unsigned i=0;i<points.size();i++)for(unsigned k=0;k<8;k++){auto node=stencils_[i].nodes[k];auto w=stencils_[i].weights[k];gridNormals[node]=Add(gridNormals[node],Scale(normals[i],w));gridMetric[node]+=metric[i]*w;}
  for(auto& n:gridNormals)if(Dot(n,n)>1e-24)n=Unit(n);
  for(unsigned i=0;i<points.size();i++)if(Active(i)){Vec normal{};double strain=0;
   for(unsigned k=0;k<8;k++){auto node=stencils_[i].nodes[k];auto w=stencils_[i].weights[k];if(Dot(gridNormals[node],gridNormals[node])>1e-24)normal=Add(normal,Scale(gridNormals[node],w));if(gridMetric_[node]>0)strain+=w*gridMetric[node]/gridMetric_[node];}
   normals[i]=normal;metric[i]=strain*samples_[i].metric;
  }
  for(unsigned i=0;i<points.size();i++)if(Active(i)&&Dot(normals[i],normals[i])>1e-24){
   const auto& s=samples_[i];const double stretch=metric[i]/(s.metric*scale);
   // No time-dependent noise: a feature is fixed to the same skin material.
   // Compression gathers folds; extension relaxes them without erasing them.
   const double gather=std::clamp(1+1.6*(1-stretch),.18,1.7);
   detail[i]=radius_*scale*s.fade*gather*s.detail;
   const double height=radius_*scale*s.fade*s.web;
   webHeight[i]=height;delta[i]=Scale(Unit(normals[i]),height);
  }
  // Geometry carries the broad web. Smaller wrinkles use a bounded normal
  // field, avoiding corrugation of needle-thin cut triangles or a false rigid
  // physics shape. This fine field remains present when geometry is limited.
  // Continuous common amplitude: bound the full oriented-area polynomial,
  // not a discrete halving ladder that could make relief pop during motion.
  double constraint=0;Result result;
  for(auto f:faces_){auto e=Sub(points[f[1]],points[f[0]]),v=Sub(points[f[2]],points[f[0]]),n=Cross(e,v);double area2=Dot(n,n);if(area2<1e-24)continue;
   auto de=Sub(delta[f[1]],delta[f[0]]),dv=Sub(delta[f[2]],delta[f[0]]);
   double linear=Dot(n,Add(Cross(de,v),Cross(e,dv)))/area2,quadratic=Dot(n,Cross(de,dv))/area2;
   double bound=((std::max)(0.,-linear)+std::abs(quadratic))/.8;
   if(bound>constraint){constraint=bound;result.limitingFace=f;}
  }
  const double amplitude=1/(1+constraint);
  result.amplitude=amplitude;
  // Carry the unrepresented broad relief in the normal field when strained
  // geometry needs protection. Its appearance therefore does not disappear.
  for(unsigned i=0;i<detail.size();i++)detail[i]+=(1-amplitude)*webHeight[i];
  if(normalDelta){
   normalDelta->assign(points.size(),{});std::vector<Vec> gradients(points.size()),geometric(points.size());std::vector<double> areas(points.size());
   for(auto f:faces_){auto e=Sub(points[f[1]],points[f[0]]),v=Sub(points[f[2]],points[f[0]]),n=Cross(e,v);double det=Dot(n,n),area=std::sqrt(det);if(det<1e-24)continue;
    double h=detail[f[1]]-detail[f[0]],j=detail[f[2]]-detail[f[0]],a=Dot(e,e),b=Dot(e,v),c=Dot(v,v);
    auto gradient=Scale(Add(Scale(e,h*c-j*b),Scale(v,j*a-h*b)),1/det);double length=std::sqrt(Dot(gradient,gradient));if(length>.65)gradient=Scale(gradient,.65/length);
    for(auto i:f){gradients[i]=Add(gradients[i],Scale(gradient,area));geometric[i]=Add(geometric[i],n);areas[i]+=area;}
   }
   for(unsigned i=0;i<points.size();i++)if(Active(i)&&areas[i]>0&&Dot(geometric[i],geometric[i])>1e-24){auto n=Unit(geometric[i]),g=Scale(gradients[i],1/areas[i]);g=Sub(g,Scale(n,Dot(g,n)));(*normalDelta)[i]=Sub(Unit(Sub(n,g)),n);}
  }

  for(unsigned i=0;i<points.size();i++)if(Active(i)){
   result.attemptedMaximum=(std::max)(result.attemptedMaximum,std::sqrt(Dot(delta[i],delta[i])));auto d=Scale(delta[i],amplitude);double length=std::sqrt(Dot(d,d));
   if(length>0){points[i]=Add(points[i],d);++result.changed;result.maximumDisplacement=(std::max)(result.maximumDisplacement,length);}}
  return result;
 }
};
}
