#include <malemod/surface/compound_contact_io.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
using namespace malemod::surface;
int main(int argc,char** argv){try{
 if(argc!=3)throw std::invalid_argument("Usage: evaluate_compound_contact reference.bin output.obj");
 auto r=ReadCompoundContact(argv[1]);auto start=std::chrono::steady_clock::now();
 CompoundContactPlan plan(r);auto cache=plan.Build(r.source,r.support,r.supportFaces,r.chart,r.eligible,r.protectedSupport);
 auto points=cache.Evaluate(r.source,r.support);double restError=0;
 for(unsigned i=0;i<points.size();i++){auto d=Sub(points[i],cache.rest[i]);restError=(std::max)(restError,std::sqrt(Dot(d,d)));}
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 for(auto f:cache.faces)for(unsigned k=0;k<3;k++){auto a=f[k].vertex,b=f[(k+1)%3].vertex;auto& e=edges[std::minmax(a,b)];e.first++;e.second+=a<b?1:-1;}
 unsigned boundary=0;for(auto e:edges){if(e.second.first==1)boundary++;else if(e.second.first!=2||e.second.second!=0)throw std::runtime_error("Compound visible assembly has a nonmanifold or reversed edge");}
 std::ofstream out(argv[2]);out<<std::setprecision(17);for(auto p:points)out<<"v "<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';for(auto f:cache.faces)out<<"f "<<f[0].vertex+1<<' '<<f[1].vertex+1<<' '<<f[2].vertex+1<<'\n';
 std::cout<<"vertices="<<points.size()<<" faces="<<cache.faces.size()<<" boundary="<<boundary<<" restError="<<restError<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
