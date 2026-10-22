#include <malemod/surface/compound_graft.hpp>
#include <iostream>
using namespace malemod::surface;
using V=PresentationPoint;
void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool Near(V a,V b){auto d=Sub(a,b);return Dot(d,d)<1e-18;}
int main(){try{
 std::vector<V> source{{0,0,0},{1,0,0},{0,1,0}},support{{0,0,2},{1,0,2},{0,1,2}};
 CompoundSurfaceDonor donor{{0,1,2},{.2,.3,.5}};
 std::vector<V> patch{{.3,.5,0},{.4,.5,1},{.3,.5,2}};
 CompoundGraftPlan plan(source,support,{donor},{donor},{{0,0,2}},{{0,0,2}},{0,.5,1},patch);
 auto rest=plan.Evaluate(source,support);for(unsigned i=0;i<rest.size();i++)Check(Near(rest[i],patch[i]),"Rest patch changed");
 for(double angle:{-1.2,-.4,0.,.4,1.2})for(double scale:{.25,1.,3.}){
  auto transform=[&](V p){return Add(V{2,-1,3},Scale(V{std::cos(angle)*p[0]+std::sin(angle)*p[2],p[1],-std::sin(angle)*p[0]+std::cos(angle)*p[2]},scale));};
  auto a=source,b=support;for(auto& p:a)p=transform(p);for(auto& p:b)p=transform(p);
  auto result=plan.Evaluate(a,b);for(unsigned i=0;i<result.size();i++)Check(Near(result[i],transform(patch[i])),"Common pose/scale is not equivariant");
 }
 auto moved=support;for(auto& p:moved)p[0]+=.7;auto result=plan.Evaluate(source,moved);
 Check(Near(result.front(),patch.front()),"Support motion changed retained sac boundary");
 Check(Near(result.back(),Add(patch.back(),V{.7,0,0})),"Rim does not follow evaluated support");
 Check(!Near(result[1],patch[1]),"Connecting patch stayed rigid");
 auto swing=source;for(auto& p:swing)p[1]+=.3;result=plan.Evaluate(swing,support);
 Check(Near(result.back(),patch.back()),"Sac swing fed back into the shaft/root");
 Check(Near(result.front(),Add(patch.front(),V{0,.3,0})),"Sac boundary lost independent motion");
 auto collapsed=support;collapsed[2]=collapsed[1];bool rejected=false;
 try{plan.Evaluate(source,collapsed);}catch(const std::invalid_argument&){rejected=true;}
 Check(rejected,"Collapsed support silently accepted");
 Check(source[0]==V{0,0,0}&&support[0]==V{0,0,2},"Evaluation mutated owners");
 {
  // A narrow annular connector can fold through itself when its first
  // tangent turns inward, even though all end points remain distinct.
  constexpr unsigned count=32,rows=25;std::vector<V> a,b,liveA,skin,first,last;
  std::vector<CompoundSurfaceDonor> ends;std::vector<double> parameters;
  for(unsigned row=0;row<rows;row++)parameters.push_back(.5*(1-std::cos(3.141592653589793*row/(rows-1))));
  for(unsigned j=0;j<count;j++){
   double angle=6.283185307179586*j/count;V radial{std::cos(angle),std::sin(angle),0},side{-std::sin(angle),std::cos(angle),0},p=Scale(radial,.25);
   a.insert(a.end(),{p,Add(p,side),Add(p,V{0,0,1})});
   liveA.insert(liveA.end(),{p,Add(p,side),Sub(p,radial)});
   auto q=Add(p,V{0,0,2});b.insert(b.end(),{q,Add(q,side),Add(q,V{0,0,1})});
   ends.push_back({{3*j,3*j+1,3*j+2},{1,0,0}});first.push_back({0,0,2});last.push_back({0,0,2});
  }
  for(double t:parameters)for(unsigned j=0;j<count;j++)skin.push_back(Add(a[3*j],V{0,0,2*t}));
  CompoundGraftPlan narrow(a,b,ends,ends,first,last,parameters,skin);
  auto identity=narrow.Evaluate(a,b);for(unsigned i=0;i<skin.size();i++)Check(Near(identity[i],skin[i]),"Strip guard changed valid rest geometry");
  auto limited=narrow.Evaluate(liveA,b);
  auto rebased=narrow.Rebased(liveA,b);auto restAgain=rebased.Evaluate(liveA,b);
  for(unsigned i=0;i<limited.size();i++)Check(Near(limited[i],restAgain[i]),"Rebasing changed the validated material shape");
  for(unsigned j=0;j<count;j++){Check(Near(limited[j],a[3*j]),"Strip limiter moved source boundary");Check(Near(limited[(rows-1)*count+j],b[3*j]),"Strip limiter moved support boundary");}
  for(unsigned row=0;row+1<rows;row++)for(unsigned j=0;j<count;j++){
   unsigned k=(j+1)%count,x=row*count+j,y=row*count+k,z=(row+1)*count+j,w=(row+1)*count+k;
   for(auto f:{std::array<unsigned,3>{y,x,z},std::array<unsigned,3>{y,z,w}}){auto n=Cross(Sub(limited[f[1]],limited[f[0]]),Sub(limited[f[2]],limited[f[0]])),ref=Cross(Sub(skin[f[1]],skin[f[0]]),Sub(skin[f[2]],skin[f[0]]));Check(Dot(n,ref)>0,"Transported strip still reversed");}
  }
 }
 std::cout<<"PASS compound surface donors, independent endpoint motion, angles, scales, residual and collapse gate\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
