#include <malemod/surface/compound_contact_io.hpp>
#include <sstream>
#include <iostream>
using namespace malemod::surface;
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
CompoundContactReference fixture(){
 CompoundContactReference r;
 for(unsigned y=0;y<=8;y++)for(unsigned x=0;x<=8;x++){
  r.support.push_back({double(x)-4,double(y)-4,0});r.chart.push_back({double(x)-4,double(y)-4});r.protectedSupport.push_back(true);
 }
 for(unsigned y=0;y<8;y++)for(unsigned x=0;x<8;x++){
  unsigned a=y*9+x,b=a+1,c=a+9,d=c+1;r.supportFaces.push_back({a,c,b});r.supportFaces.push_back({b,c,d});r.eligible.push_back(true);r.eligible.push_back(true);
 }
 r.source={{-1,-1,-2},{1,-1,-2},{1,1,-2},{-1,1,-2},{0,0,-3}};
 r.sourceLoop={0,3,2,1};r.footprint={{-1.4,-1.4},{-1.4,1.4},{1.4,1.4},{1.4,-1.4}};
 for(unsigned j=0;j<4;j++)r.sourceFaces.push_back({4,(j+1)%4,j});
 r.curve={0,1,2,3};r.parameters={0,.25,.5,.75,1};
 for(unsigned j=0;j<4;j++){
  auto p=r.source[r.sourceLoop[j]];PresentationPoint q{r.footprint[j][0],r.footprint[j][1],0};r.firstTangents.push_back(Sub(q,p));r.lastTangents.push_back(Sub(q,p));
 }
 for(double t:r.parameters)for(unsigned j=0;j<4;j++){
  auto p=r.source[r.sourceLoop[j]];PresentationPoint q{r.footprint[j][0],r.footprint[j][1],0};r.patch.push_back(Add(Scale(p,1-t),Scale(q,t)));
 }return r;
}
int main(){try{
 auto r=fixture();CompoundContactPlan plan(r);auto cache=plan.Build(r.source,r.support,r.supportFaces,r.chart,r.eligible,r.protectedSupport);
 auto p=cache.Evaluate(r.source,r.support);double restError=0;for(unsigned i=0;i<p.size();i++){auto d=Sub(p[i],cache.rest[i]);restError=(std::max)(restError,std::sqrt(Dot(d,d)));}require(restError<1e-10,"Rest reproduction differs");
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 for(auto f:cache.faces)for(unsigned k=0;k<3;k++){auto a=f[k].vertex,b=f[(k+1)%3].vertex;auto& e=edges[std::minmax(a,b)];e.first++;e.second+=a<b?1:-1;}
 unsigned boundary=0;for(auto item:edges){auto e=item.second;if(e.first==1)boundary++;else require(e.first==2&&e.second==0,"Nonmanifold assembly");}require(boundary==32,"Added an external boundary");
 for(double angle:{-.7,0.,.6})for(double scale:{.4,1.,2.}){
  auto transform=[&](PresentationPoint p){double c=std::cos(angle),s=std::sin(angle);return Add(Scale(PresentationPoint{c*p[0]-s*p[2],p[1],s*p[0]+c*p[2]},scale),{4,2,1});};
  auto source=r.source,support=r.support;for(auto& v:source)v=transform(v);for(auto& v:support)v=transform(v);auto moved=cache.Evaluate(source,support);
  for(unsigned i=0;i<moved.size();i++){auto d=Sub(moved[i],transform(p[i]));require(Dot(d,d)<1e-18,"Rigid/scale covariance differs");}
 }
 auto source=r.source;for(auto& v:source)v[2]-=.3;auto moved=cache.Evaluate(source,r.support);
 for(unsigned i=0;i<r.support.size();i++)require(moved[i]==r.support[i],"Independent suspension moved support");
 auto reject=[&](CompoundContactReference bad){bool failed=false;try{CompoundContactPlan other(std::move(bad));}catch(const std::invalid_argument&){failed=true;}require(failed,"Malformed reference accepted");};
 auto bad=r;bad.sourceLoop[0]=9999;reject(bad);bad=r;bad.curve[1]=bad.curve[0];reject(bad);bad=r;bad.parameters.back()=.9;reject(bad);bad=r;bad.support[0][0]=std::numeric_limits<double>::quiet_NaN();reject(bad);
 // A short forged header must be rejected before allocating its claimed mesh.
 std::string bytes="MMCCNT02";std::array<unsigned,7> counts{500000,1000000,500000,1000000,4096,50000,256};bytes.append(reinterpret_cast<const char*>(counts.data()),28);std::istringstream input(bytes);bool rejected=false;try{ReadCompoundContact(input);}catch(const std::runtime_error&){rejected=true;}require(rejected,"Oversize reference allocation accepted");
 std::cout<<"compound assembly, covariance, independent motion and input validation pass\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
