#include <malemod/surface/root_articulation.hpp>
#include <Eigen/SparseCholesky>
#include <map>
#include <set>
namespace malemod::surface::root_articulation {
using V=Eigen::Vector3d;using Sparse=Eigen::SparseMatrix<double>;
static V Vec(Point p){return {p[0],p[1],p[2]};}
struct Junction::Impl {
 struct Row {std::vector<std::pair<unsigned,double>> entries;double weight;};
 std::vector<Point> reference;std::vector<bool> moving;std::vector<unsigned> free;std::vector<int> index;std::vector<Row> rows;
 std::vector<std::pair<unsigned,unsigned>> edges;
 bool analyzed=false;
 Eigen::SimplicialLDLT<Sparse> factor;
 Impl(const std::vector<Point>& p,const std::vector<Face>& faces,const std::vector<bool>& fixed,const std::vector<bool>& handles):reference(p),moving(handles),index(p.size(),-1){
  if(p.empty()||fixed.size()!=p.size()||handles.size()!=p.size())throw std::invalid_argument("Invalid articulation domain");
  for(unsigned i=0;i<p.size();i++){root_motion::Finite(p[i]);if(fixed[i]&&handles[i])throw std::invalid_argument("Conflicting articulation handles");if(!fixed[i]&&!handles[i]){index[i]=int(free.size());free.push_back(i);}}
  std::set<std::pair<unsigned,unsigned>> unique;for(auto f:faces){for(auto i:f)if(i>=p.size())throw std::invalid_argument("Invalid articulation face");for(unsigned j=0;j<3;j++)unique.insert(std::minmax(f[j],f[(j+1)%3]));}
  edges.assign(unique.begin(),unique.end());Update(p);
 }
 void Update(const std::vector<Point>& p){
  if(p.size()!=reference.size())throw std::invalid_argument("Articulation pose topology changed");
  for(auto v:p)root_motion::Finite(v);
  if(analyzed&&p==reference)return;
  reference=p;rows.clear();
  std::vector<std::vector<std::pair<unsigned,double>>> adjacent(p.size());
  for(auto [a,b]:edges){double w=1/std::max((Vec(p[a])-Vec(p[b])).norm(),1e-6);adjacent[a].emplace_back(b,w);adjacent[b].emplace_back(a,w);}
  std::vector<Eigen::Triplet<double>> entries;
  for(unsigned i=0;i<p.size();i++){
   bool needed=index[i]>=0;double diagonal=0;for(auto [j,w]:adjacent[i]){needed=needed||index[j]>=0;diagonal+=w;}if(!needed||diagonal==0)continue;
   Row row;row.entries.emplace_back(i,diagonal);for(auto [j,w]:adjacent[i])row.entries.emplace_back(j,-w);row.weight=1/diagonal;
   for(auto [a,wa]:row.entries)if(index[a]>=0)for(auto [b,wb]:row.entries)if(index[b]>=0)entries.emplace_back(index[a],index[b],wa*wb*row.weight);
   rows.push_back(std::move(row));
  }
  if(!free.empty()){Sparse matrix(free.size(),free.size());matrix.setFromTriplets(entries.begin(),entries.end());if(!analyzed)factor.analyzePattern(matrix);factor.factorize(matrix);if(factor.info()!=Eigen::Success)throw std::invalid_argument("Unanchored articulation component");}analyzed=true;
 }
 std::vector<Point> Apply(const std::vector<Point>& current,const Rotation& rotation)const{
  if(current.size()!=reference.size())throw std::invalid_argument("Articulation pose topology changed");
  for(auto p:current)root_motion::Finite(p);
  auto result=current;std::vector<V> delta(reference.size(),V::Zero());for(unsigned i=0;i<result.size();i++)if(moving[i]){result[i]=rotation.Apply(current[i]);delta[i]=Vec(result[i])-Vec(current[i]);}
  if(free.empty())return result;
  Eigen::MatrixXd rhs=Eigen::MatrixXd::Zero(free.size(),3);
  for(const auto& row:rows){V boundary=V::Zero();for(auto [i,w]:row.entries)if(index[i]<0)boundary+=w*delta[i];for(auto [i,w]:row.entries)if(index[i]>=0)rhs.row(index[i])-=row.weight*w*boundary.transpose();}
  Eigen::MatrixXd solved=factor.solve(rhs);if(factor.info()!=Eigen::Success||!solved.allFinite())throw std::runtime_error("Articulation solve failed");
  for(unsigned j=0;j<free.size();j++){auto i=free[j];for(unsigned k=0;k<3;k++)result[i][k]+=solved(j,k);}return result;
 }
};
Junction::Junction(const std::vector<Point>& p,const std::vector<Face>& f,const std::vector<bool>& fixed,const std::vector<bool>& moving):impl_(std::make_unique<Impl>(p,f,fixed,moving)){}
Junction::~Junction()=default;
void Junction::UpdateReference(const std::vector<Point>& current){impl_->Update(current);}
std::vector<Point> Junction::Apply(const Rotation& r)const{return impl_->Apply(impl_->reference,r);}
std::vector<Point> Junction::Apply(const std::vector<Point>& current,const Rotation& r)const{return impl_->Apply(current,r);}
}
