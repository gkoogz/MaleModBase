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
 std::cout<<"PASS compound surface donors, independent endpoint motion, angles, scales, residual and collapse gate\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
