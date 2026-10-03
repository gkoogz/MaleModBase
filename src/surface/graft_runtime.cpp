#include <malemod/surface/graft_runtime.hpp>
#include <Eigen/SparseCholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace malemod::surface {
namespace {
using Vector=Eigen::Vector3d;
using Sparse=Eigen::SparseMatrix<double>;
using Entry=Eigen::Triplet<double>;
Vector V(PrecisePoint p){return Vector(p[0],p[1],p[2]);}
double Smoother(double x){x=std::clamp(x,0.,1.);return x*x*x*(x*(x*6-15)+10);}
bool Finite(PrecisePoint p){return V(p).allFinite();}
double Recruitment(Vector p,const GraftFrame& f){
 const Vector root=V(f.root)/f.sourceLengthScale,axis=V(f.axis),up=V(f.up),q=p-root;
 const double radius=f.radius/f.sourceLengthScale,length=f.length/f.sourceLengthScale;
 double s=q.dot(axis),y=q.y(),z=q.dot(up),rho=std::sqrt(y*y+z*z),upper=(z/std::max(rho,1e-8)+1)*.5;
 const double growth=Smoother((radius-2.9)/4.72),reach=5+growth*(2.5+2.5*upper);
 double w=Smoother((s+reach)/3)*(1-Smoother((s/length-.12)/.26));
 w*=1-Smoother((rho-(radius*1.55+2))/3);
 const double ventralReach=4+4*Smoother((radius-2.7)/1.1);
 w*=Smoother((p.x()-(2+ventralReach*(1-upper)))/3);
 w*=Smoother((p.z()-(root.z()-radius*2.1-2))/4);return w;
}
}
double GraftRecruitmentWeight(PrecisePoint point,const GraftFrame& frame){
 if(!Finite(point)||!Finite(frame.root)||!Finite(frame.axis)||!Finite(frame.up)||!std::isfinite(frame.sourceLengthScale)||frame.sourceLengthScale<=0||!std::isfinite(frame.radius)||frame.radius<=0||!std::isfinite(frame.length)||frame.length<=0)throw std::invalid_argument("Invalid graft recruitment input");
 const auto axis=V(frame.axis),up=V(frame.up);
 if(std::abs(axis.norm()-1)>1e-5||std::abs(up.norm()-1)>1e-5||std::abs(axis.dot(up))>1e-5)throw std::invalid_argument("Invalid graft recruitment basis");
 return Recruitment(V(point)/frame.sourceLengthScale,frame);
}
struct GraftPlan::Impl {
 double scale;
 std::size_t count;
 std::vector<std::uint32_t> masters,free,fixed,protectedVertices,prescribedVertices;
 std::vector<Vector> points;
 Eigen::VectorXd area;
 Sparse projection,boundary,attraction,baseMetric;
 std::vector<int> patternOuter,patternInner;
 Eigen::SimplicialLDLT<Sparse> factor;
 Impl(const GraftDomain& domain,const GraftFrame& frame):scale(frame.sourceLengthScale),count(domain.points.size()){
  if(!count||count>1000000||!Finite(frame.root)||!Finite(frame.axis)||!Finite(frame.up)||!std::isfinite(scale)||scale<=0||!std::isfinite(frame.radius)||frame.radius<=0||!std::isfinite(frame.length)||frame.length<=0)throw std::invalid_argument("Invalid graft frame/domain");
  const auto axis=V(frame.axis),up=V(frame.up);
  if(std::abs(axis.norm()-1)>1e-5||std::abs(up.norm()-1)>1e-5||std::abs(axis.dot(up))>1e-5)throw std::invalid_argument("Graft axis/up must be orthonormal");
  points.reserve(count);for(auto p:domain.points){if(!Finite(p))throw std::invalid_argument("Non-finite graft rest point");points.push_back(V(p)/scale);}
  std::vector<bool> slaves(count,false),locked(count,false);
  for(const auto& e:domain.seams){
   if(e.slave>=count||e.a>=count||e.b>=count||e.slave==e.a||e.slave==e.b||e.a==e.b||slaves[e.slave]||!std::isfinite(e.weight)||e.weight<0||e.weight>1)throw std::invalid_argument("Invalid original-edge seam");
   slaves[e.slave]=true;
  }
  for(auto e:domain.seams)if(slaves[e.a]||slaves[e.b])throw std::invalid_argument("Seam donors must be independent masters");
  for(auto id:domain.protectedVertices){if(id>=count||slaves[id])throw std::invalid_argument("Protected boundary must be an independent master");locked[id]=true;}
  protectedVertices=domain.protectedVertices;
  for(auto id:domain.prescribedVertices){if(id>=count||slaves[id]||locked[id])throw std::invalid_argument("Prescribed boundary must be an independent unprotected master");locked[id]=true;}
  prescribedVertices=domain.prescribedVertices;
  std::vector<unsigned> masterOf(count,0);
  for(unsigned i=0;i<count;i++)if(!slaves[i]){masterOf[i]=unsigned(masters.size());masters.push_back(i);}
  std::vector<Entry> entries;entries.reserve(masters.size()+domain.seams.size()*2);
  for(unsigned i=0;i<masters.size();i++)entries.emplace_back(masters[i],i,1);
  for(auto e:domain.seams){
   points[e.slave]=points[e.a]*(1-e.weight)+points[e.b]*e.weight;
   entries.emplace_back(e.slave,masterOf[e.a],1-e.weight);entries.emplace_back(e.slave,masterOf[e.b],e.weight);
  }
  projection.resize(int(count),int(masters.size()));projection.setFromTriplets(entries.begin(),entries.end());
  area=Eigen::VectorXd::Zero(count);
  entries.clear();
  if(domain.triangles.empty())throw std::invalid_argument("Empty graft triangles");
  for(auto face:domain.triangles){
   if(*std::max_element(face.begin(),face.end())>=count)throw std::invalid_argument("Graft triangle outside domain");
   Vector p[3]={points[face[0]],points[face[1]],points[face[2]]};
   const double twice=(p[1]-p[0]).cross(p[2]-p[0]).norm();if(twice<1e-14)continue;
   for(unsigned j=0;j<3;j++){
    auto a=face[(j+1)%3],b=face[(j+2)%3];const double w=std::clamp((p[(j+1)%3]-p[j]).dot(p[(j+2)%3]-p[j])/(2*twice),0.,20.);
    entries.emplace_back(a,a,w);entries.emplace_back(b,b,w);entries.emplace_back(a,b,-w);entries.emplace_back(b,a,-w);
    area[face[j]]+=twice/6;
   }
  }
  Sparse curvature(count,count);curvature.setFromTriplets(entries.begin(),entries.end());
  std::vector<Entry> inverseArea;
  for(unsigned i=0;i<count;i++){
   area[i]=std::max(area[i],.005);
   inverseArea.emplace_back(i,i,1/area[i]);
  }
  Sparse inverse(count,count);inverse.setFromTriplets(inverseArea.begin(),inverseArea.end());
  Sparse cp=curvature*projection;
  baseMetric=cp.transpose()*inverse*cp*8+projection.transpose()*curvature*projection*2;
  Update(frame);
 }
 void Update(const GraftFrame& frame){
  if(!Finite(frame.root)||!Finite(frame.axis)||!Finite(frame.up)||frame.sourceLengthScale!=scale||!std::isfinite(frame.radius)||frame.radius<=0||!std::isfinite(frame.length)||frame.length<=0)throw std::invalid_argument("Invalid updated graft frame/scale");
  const auto axis=V(frame.axis),up=V(frame.up);
  if(std::abs(axis.norm()-1)>1e-5||std::abs(up.norm()-1)>1e-5||std::abs(axis.dot(up))>1e-5)throw std::invalid_argument("Invalid updated graft basis");
  Eigen::VectorXd screen(count),mask(count);std::vector<Entry> screenEntries;
  screenEntries.reserve(count);
  for(unsigned i=0;i<count;i++){
   mask[i]=Recruitment(points[i],frame);screen[i]=area[i]*(2.5+2*std::pow(1-mask[i],4));
   screenEntries.emplace_back(i,i,screen[i]);
  }
  Sparse screenMatrix(count,count);screenMatrix.setFromTriplets(screenEntries.begin(),screenEntries.end());
  Sparse metric=baseMetric+projection.transpose()*screenMatrix*projection;
  std::vector<bool> locked(count,false);for(auto id:protectedVertices)locked[id]=true;
  for(auto id:prescribedVertices)locked[id]=true;
  free.clear();fixed.clear();
  std::vector<int> freeOf(masters.size(),-1),fixedOf(masters.size(),-1);
  for(unsigned i=0;i<masters.size();i++){
   if(mask[masters[i]]>1e-4&&!locked[masters[i]]){freeOf[i]=int(free.size());free.push_back(i);}
   else{fixedOf[i]=int(fixed.size());fixed.push_back(i);}
  }
  std::vector<Entry> freeEntries,boundaryEntries;
  for(int col=0;col<metric.outerSize();col++)for(Sparse::InnerIterator it(metric,col);it;++it){
   auto row=it.row();if(freeOf[row]<0)continue;
   if(freeOf[col]>=0)freeEntries.emplace_back(freeOf[row],freeOf[col],it.value());
   else boundaryEntries.emplace_back(freeOf[row],fixedOf[col],it.value());
  }
  Sparse freeMetric(free.size(),free.size());freeMetric.setFromTriplets(freeEntries.begin(),freeEntries.end());
  boundary.resize(free.size(),fixed.size());boundary.setFromTriplets(boundaryEntries.begin(),boundaryEntries.end());
  attraction=projection.transpose()*screenMatrix;
  if(!free.empty()){
   freeMetric.makeCompressed();
   std::vector<int> outer(freeMetric.outerIndexPtr(),freeMetric.outerIndexPtr()+freeMetric.outerSize()+1);
   std::vector<int> inner(freeMetric.innerIndexPtr(),freeMetric.innerIndexPtr()+freeMetric.nonZeros());
   if(outer!=patternOuter||inner!=patternInner){factor.analyzePattern(freeMetric);patternOuter=std::move(outer);patternInner=std::move(inner);}
   factor.factorize(freeMetric);if(factor.info()!=Eigen::Success)throw std::runtime_error("Graft factorization failed");
  }
 }
 std::vector<PrecisePoint> Solve(const std::vector<PrecisePoint>& input)const{
  if(input.size()!=count)throw std::invalid_argument("Graft displacement topology differs");
  for(auto id:protectedVertices)for(double value:input[id])if(!std::isfinite(value)||std::abs(value)>1e-10)throw std::invalid_argument("Graft displacement moves a protected part boundary");
  Eigen::MatrixXd target(count,3),solved(masters.size(),3),fixedValues(fixed.size(),3);
  for(unsigned i=0;i<count;i++){if(!Finite(input[i]))throw std::invalid_argument("Non-finite graft displacement");target.row(i)=(V(input[i])/scale).transpose();}
  for(unsigned i=0;i<masters.size();i++)solved.row(i)=target.row(masters[i]);
  if(!free.empty()){
   const Eigen::MatrixXd allRHS=attraction*target;Eigen::MatrixXd rhs(free.size(),3);
   for(unsigned i=0;i<free.size();i++)rhs.row(i)=allRHS.row(free[i]);
   for(unsigned i=0;i<fixed.size();i++)fixedValues.row(i)=solved.row(fixed[i]);
   rhs-=boundary*fixedValues;const Eigen::MatrixXd result=factor.solve(rhs);
   if(factor.info()!=Eigen::Success||!result.allFinite())throw std::runtime_error("Graft displacement solve failed");
   for(unsigned i=0;i<free.size();i++)solved.row(free[i])=result.row(i);
  }
  Eigen::MatrixXd expanded=projection*solved*scale;if(!expanded.allFinite())throw std::runtime_error("Non-finite graft result");
  std::vector<PrecisePoint> output(count);for(unsigned i=0;i<count;i++)output[i]={expanded(i,0),expanded(i,1),expanded(i,2)};return output;
 }
};
GraftPlan::GraftPlan(const GraftDomain& d,const GraftFrame& f):impl_(std::make_unique<Impl>(d,f)){}
GraftPlan::~GraftPlan()=default;
void GraftPlan::UpdateFrame(const GraftFrame& frame){impl_->Update(frame);}
std::vector<PrecisePoint> GraftPlan::SolveDisplacement(const std::vector<PrecisePoint>& displacement)const{return impl_->Solve(displacement);}
}
