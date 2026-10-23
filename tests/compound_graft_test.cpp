#include <malemod/surface/compound_graft.hpp>
#include <malemod/surface/strip_lighting.hpp>
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
 for(double radius:{.05,.25}){
  // A narrow annular connector can fold through itself when its first
  // tangent turns inward, even though all end points remain distinct.
  constexpr unsigned count=32,rows=25;std::vector<V> a,b,liveA,skin,first,last;
  std::vector<CompoundSurfaceDonor> ends;std::vector<double> parameters;
  for(unsigned row=0;row<rows;row++)parameters.push_back(.5*(1-std::cos(3.141592653589793*row/(rows-1))));
  for(unsigned j=0;j<count;j++){
   double angle=6.283185307179586*j/count;V radial{std::cos(angle),std::sin(angle),0},side{-std::sin(angle),std::cos(angle),0},p=Scale(radial,radius);
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
 {
  // A tilted generated meridian must leave a cylindrical retained surface
  // in its tangent plane, while the remote join and both rims stay exact.
  constexpr unsigned count=32;std::vector<double> parameters{0,.00001,.0001,.005,.025,.08,.15,.2,.5,.9,.99,1};
  std::vector<V> skin,normals;
  for(unsigned j=0;j<count;j++){double a=6.283185307179586*j/count;normals.push_back({std::cos(a),std::sin(a),0});}
  for(double t:parameters)for(auto n:normals)skin.push_back(Add(Scale(n,1+.3*t),V{0,0,t}));
  auto corrected=skin;CompoundGraftPlan::ConformSourceEntry(corrected,parameters,normals);
  for(unsigned j=0;j<count;j++){
   Check(Near(corrected[j],skin[j]),"Entry fillet moved retained boundary");
   auto entry=Sub(corrected[count+j],corrected[j]);
   Check(std::abs(Dot(entry,normals[j]))<1e-6,"Entry tangent still has a visible radial step");
  }
  for(unsigned j=0;j<count;j++)Check(Near(corrected[corrected.size()-count+j],skin[skin.size()-count+j]),"Entry fillet moved support boundary");
  for(unsigned j=0;j<count;j++){auto before=Sub(skin[skin.size()-count+j],skin[skin.size()-2*count+j]),after=Sub(corrected[corrected.size()-count+j],corrected[corrected.size()-2*count+j]);Check(std::sqrt(Dot(Sub(after,before),Sub(after,before)))/std::sqrt(Dot(before,before))<.0001,"Entry correction changed upper tangent");}
  auto moved=skin,rotated=normals;for(auto& p:moved)p=Add(Scale(V{p[2],p[1],-p[0]},3),{20,30,40});for(auto& n:rotated)n={n[2],n[1],-n[0]};
  CompoundGraftPlan::ConformSourceEntry(moved,parameters,rotated);
  for(unsigned i=0;i<skin.size();i++){auto p=corrected[i];Check(Near(moved[i],Add(Scale(V{p[2],p[1],-p[0]},3),{20,30,40})),"Entry fillet lost pose/scale covariance");}
 }
 {
  // The old tangent-plane projection was released over a fixed 20% band.
  // A constant initial slope therefore rebounded to nearly twice the
  // intended slope just above that band. Bound the derivative correction
  // instead, across short/long and strongly oblique nonplanar contours.
  constexpr unsigned count=24,rows=101;std::vector<double> t;
  for(unsigned row=0;row<rows;row++)t.push_back(double(row)/(rows-1));
  for(double height:{.08,1.,5.}){
   std::vector<V> skin,normals;
   for(unsigned j=0;j<count;j++){double a=6.283185307179586*j/count;normals.push_back({std::cos(a),std::sin(a),0});}
   for(double u:t)for(unsigned j=0;j<count;j++){auto n=normals[j];double a=6.283185307179586*j/count;skin.push_back(Add(Scale(n,1+.3*height*u),V{0,0,height*u+.35*std::cos(a)+.2*std::sin(2*a)}));}
   auto revised=skin;CompoundGraftPlan::ConformSourceEntry(revised,t,normals);
   double newPeak=0,oldPeak=0;
   auto old=skin;for(unsigned row=1;row+1<rows&&t[row]<.2;row++)for(unsigned j=0;j<count;j++){double u=t[row]/.2,w=1-u*u*u*(10+u*(-15+6*u));old[row*count+j]=Sub(old[row*count+j],Scale(normals[j],Dot(Sub(skin[row*count+j],skin[j]),normals[j])*w));}
   for(unsigned row=0;row+1<rows;row++)for(unsigned j=0;j<count;j++){
    auto derivative=[&](const auto& points){return Dot(Sub(points[(row+1)*count+j],points[row*count+j]),normals[j])/(height*(t[row+1]-t[row]));};
    newPeak=(std::max)(newPeak,derivative(revised));oldPeak=(std::max)(oldPeak,derivative(old));
   }
   Check(oldPeak>.5,"Synthetic regression does not reproduce old shoulder");
   Check(newPeak<.376&&newPeak<oldPeak*.75,"Derivative continuation retains projection-release shoulder");
   for(unsigned j=0;j<count;j++){Check(revised[j]==skin[j],"Shoulder repair moved retained lobe rim");Check(revised[(rows-1)*count+j]==skin[(rows-1)*count+j],"Shoulder repair moved support rim");}
   for(unsigned row=0;row+1<rows;row++)for(unsigned j=0;j<count;j++){
    unsigned k=(j+1)%count,a=row*count+j,b=row*count+k,c=(row+1)*count+j,d=(row+1)*count+k;
    for(auto f:{std::array<unsigned,3>{a,b,c},std::array<unsigned,3>{b,d,c}}){auto normal=[&](const auto& p){return Cross(Sub(p[f[1]],p[f[0]]),Sub(p[f[2]],p[f[0]]));};auto n=normal(skin),m=normal(revised);Check(Dot(n,m)>=.099999*Dot(n,n),"Derivative repair overturned nonplanar skin");}
   }
  }
 }
 {
  std::vector<V> field;std::vector<double> coordinate;
  for(unsigned i=0;i<128;i++){double a=6.283185307179586*i/128;coordinate.push_back(a);field.push_back(Unit(V{.2*std::sin(24*a),0,1}));}
  PeriodicNormalField normalField(field,coordinate,6.283185307179586);
  Check(normalField.At(0)==field,"Harmonic continuation changed the seam normal");
  auto near=normalField.At(.001),far=normalField.At(.04);double before=0,after=0;
  for(unsigned i=0;i<field.size();i++){before+=near[i][0]*near[i][0];after+=far[i][0]*far[i][0];}
  Check(after<before*.001,"Boundary ripple was extruded into the strip");
  for(auto& n:field)n={n[2],n[1],-n[0]};PeriodicNormalField rotated(field,coordinate,6.283185307179586);auto rotatedFar=rotated.At(.04);
  for(unsigned i=0;i<far.size();i++)Check(Near(rotatedFar[i],{far[i][2],far[i][1],-far[i][0]}),"Normal continuation depends on world orientation");
 }
 {
  std::vector<double> coordinates{0,.01,.3,1.,2.,3.,4.,5.,6.};std::vector<V> points;std::vector<std::vector<unsigned>> rings(3);
  for(unsigned row=0;row<3;row++)for(double t:coordinates){rings[row].push_back(unsigned(points.size()));points.push_back({std::cos(t),std::sin(t),double(row)});}
  auto before=StripDifferentials(points,rings,coordinates,6.283185307179586);
  for(auto& p:points)p={-p[1],p[0],p[2]};auto after=StripDifferentials(points,rings,coordinates,6.283185307179586);
  for(unsigned i=0;i<before.size();i++){
   auto t=before[i].tangent;Check(Near(after[i].tangent,{-t[1],t[0],t[2]}),"Strip tangent lost in-plane rotation");
   Check(Near(before[i].meridian,after[i].meridian),"Strip axial differential changed under axial rotation");
   Check(after[i].vertex==before[i].vertex,"Strip lighting changed vertex ownership");
  }
 }
 std::cout<<"PASS compound donors, motion, scales, strip orientation and live tangent directions\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
