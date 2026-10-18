#include <malemod/surface/skin_relief.hpp>
#include <iostream>
using namespace malemod::surface;
using V=PresentationPoint;
static void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 std::vector<V> p;std::vector<bool> active;std::vector<std::array<unsigned,3>> f;
 constexpr unsigned n=49;
 for(unsigned z=0;z<n;z++)for(unsigned y=0;y<n;y++){p.push_back({0.,-1.5+3.*y/(n-1),-1.+4.*z/(n-1)});active.push_back(z>0&&z<n-1&&y>0&&y<n-1);}
 for(unsigned z=0;z+1<n;z++)for(unsigned y=0;y+1<n;y++){unsigned a=z*n+y;f.push_back({a,a+1,a+n});f.push_back({a+1,a+n+1,a+n});}
 SkinRelief skin(p,f,active,{},PresentationBasis{{{1,0,0},{0,1,0},{0,0,1}}},1);
 auto a=p;auto result=skin.Apply(a);Check(result.changed>1500,"No skin relief");Check(result.amplitude==1,"Nominal relief needed backoff");Check(result.maximumDisplacement<.1,"Relief exceeds skin scale");
 auto same=p;skin.Apply(same);Check(same==a,"Relief is nondeterministic");
 for(unsigned i=0;i<p.size();i++)if(!active[i])Check(p[i]==a[i],"Protected attachment changed");
 auto scaled=p;for(auto& v:scaled)v=Add(Scale(v,3),V{100,-200,11});skin.Apply(scaled,3);
 for(unsigned i=0;i<p.size();i++)Check(Dot(Sub(scaled[i],Add(Scale(a[i],3),V{100,-200,11})),Sub(scaled[i],Add(Scale(a[i],3),V{100,-200,11})))<1e-20,"Relief is not scale/translation covariant");
 auto rotated=p;for(auto& v:rotated)v={v[2],v[0],v[1]};skin.Apply(rotated);
 for(unsigned i=0;i<p.size();i++)Check(Dot(Sub(rotated[i],V{a[i][2],a[i][0],a[i][1]}),Sub(rotated[i],V{a[i][2],a[i][0],a[i][1]}))<1e-20,"Features slip under rotation");
 auto compact=p,extended=p;for(auto& v:compact)v[2]*=.6;for(auto& v:extended)v[2]*=1.5;std::vector<V> compactNormals,extendedNormals;skin.Apply(compact,1,&compactNormals);skin.Apply(extended,1,&extendedNormals);
 double gathered=0,relaxed=0;for(unsigned i=0;i<p.size();i++)if(p[i][2]<0){gathered+=std::sqrt(Dot(compactNormals[i],compactNormals[i]));relaxed+=std::sqrt(Dot(extendedNormals[i],extendedNormals[i]));}Check(gathered>relaxed*1.2,"Folds do not gather under compression");
 for(auto f1:f){auto n0=Cross(Sub(p[f1[1]],p[f1[0]]),Sub(p[f1[2]],p[f1[0]]));auto n1=Cross(Sub(a[f1[1]],a[f1[0]]),Sub(a[f1[2]],a[f1[0]]));Check(Dot(n0,n1)>.2*Dot(n0,n0),"Relief reversed a triangle");}
 bool rejected=false;try{skin.Apply(a,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid scale accepted");
 // Retain near-coincident cut donors in a curved, nonuniformly stretched
 // strip. Their different triangle valence must not erase the whole field.
 auto needles=p;
 for(unsigned z=0;z<n;z++)for(unsigned y=0;y<n;y++){
  auto& v=needles[z*n+y];v[1]=-1.5+3.*(y/2)*2/(n-1)+(y%2?1e-5:0.);v[0]=.18*std::sin(1.7*v[2])*std::cos(.9*v[1]);
 }
 SkinRelief thin(needles,f,active,{},PresentationBasis{{{1,0,0},{0,1,0},{0,0,1}}},1);
 auto deformed=needles;for(auto& v:deformed){v[2]=v[2]*.8+.06*std::sin(v[1]);v[0]+=.1*v[2]*v[2];}
 auto relieved=deformed;auto thinResult=thin.Apply(relieved);Check(thinResult.amplitude>=.125,"Needle donors suppress all folds");
 for(unsigned z=2;z+2<n;z++)for(unsigned y=2;y+2<n;y+=2){auto i=z*n+y;auto d=Sub(Sub(relieved[i],deformed[i]),Sub(relieved[i+1],deformed[i+1]));Check(Dot(d,d)<1e-7,"Relief splits coincident material donors");}
 std::cout<<"Material fold anchoring, gathering, protected boundaries, scale, rotation and orientation pass\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
