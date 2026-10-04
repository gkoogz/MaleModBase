#pragma once
// A material sheet initialized by measured tissue walks. No radial primitive,
// anatomy replacement, game units, joints, or graphics API enters this module.
#include <functional>
#include <optional>
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
#include <cstdio>
#endif
#include "strap_route.hpp"
namespace malemod::garments::drape {
std::vector<Sample> PlaneRoute(const Input&,const Sample&,const Sample&,double,const std::function<void(Sample&)>&);
inline Point Barycentric(Point p,Point a,Point b,Point c){auto v=Sub(b,a),w=Sub(c,a),r=Sub(p,a);double vv=Dot(v,v),vw=Dot(v,w),ww=Dot(w,w),rv=Dot(r,v),rw=Dot(r,w),d=vv*ww-vw*vw;if(std::abs(d)<1e-24)return {1,0,0};double y=(ww*rv-vw*rw)/d,z=(vv*rw-vw*rv)/d;return {1-y-z,y,z};}
struct Sheet {
 std::vector<Sample> vertices;
 unsigned rows=0,columns=0;
 std::vector<std::vector<Sample>> walks;
};
inline Sample Mix(const Sample& a,const Sample& b,double t){return {Add(Mul(a.position,1-t),Mul(b.position,t)),Add(Mul(a.normal,1-t),Mul(b.normal,t)),detail::Blend(a.lineage,b.lineage,t)};}
inline void ValidateRegions(const Input& input){
 std::vector<unsigned char> ownership(input.anatomy.size());
 for(unsigned region=0;region<4;region++){
  const auto& indices=input.anatomyRegions[region];if(indices.empty())throw std::invalid_argument("Measured drape requires all four authored anatomy regions");
  for(auto index:indices){if(index>=input.anatomy.size()||ownership[index])throw std::invalid_argument("Drape semantic regions overlap or exceed the full anatomy surface");ownership[index]=1;}
 }
 if(input.anatomyTriangles.empty())throw std::invalid_argument("Measured drape requires the actual complete tissue topology");
}
// Select an actual exposed support in an authored region. The lateral target
// moves across that region's measured span; no fitted ellipsoid supplies it.
inline Sample Support(const Input& input,unsigned region,double lane,Point direction){
 const auto& indices=input.anatomyRegions[region];double low=1e100,high=-1e100;
 for(auto index:indices){double x=input.frame.Local(input.anatomy[index].position)[0];low=(std::min)(low,x);high=(std::max)(high,x);}
 double lateral=(low+high)*.5+lane*(high-low)*.5,best=-1e100;unsigned selected=indices.front();
 direction=Unit(direction);
 for(auto index:indices){const auto& sample=input.anatomy[index];auto local=input.frame.Local(sample.position);double facing=Dot(sample.normal,direction);double score=Dot(sample.position,direction)-4*std::abs(local[0]-lateral);if(facing<0)score-=high-low;
  if(score>best){best=score;selected=index;}}
 return input.anatomy[selected];
}
inline Sample Resample(const std::vector<Sample>& path,double fraction){
 std::vector<double> arc(path.size());for(unsigned i=1;i<path.size();i++)arc[i]=arc[i-1]+Length(Sub(path[i].position,path[i-1].position));
 if(arc.back()<1e-14)throw std::invalid_argument("Measured material walk is degenerate");double target=std::clamp(fraction,0.,1.)*arc.back();unsigned edge=0;while(edge+2<path.size()&&arc[edge+1]<target)edge++;
 double length=arc[edge+1]-arc[edge];return Mix(path[edge],path[edge+1],length>1e-20?(target-arc[edge])/length:0);
}
// The caller supplies exact physical segment contacts and measured-volume
// endpoint projection. Classification-only caps never become walk obstacles.
using Contact=std::function<std::optional<Sample>(Point,Point)>;
using Project=std::function<void(Sample&)>;
using Section=std::function<std::vector<Sample>(const Sample&,const Sample&)>;
inline std::vector<Sample> Tauten(const std::vector<Sample>& path,double preferredGap,Contact contact){
 if(path.size()<2||!std::isfinite(preferredGap)||preferredGap<=0)throw std::invalid_argument("Invalid material tension path");
  // String-pull only on the exterior side of every removed support. A
  // geometric shortcut through the back of a prominence is forbidden even
  // when its straight chord is collision free.
  std::vector<Sample> taut;taut.push_back(path.front());unsigned at=0;
  while(at+1<path.size()){
   unsigned next=at+1;
   for(unsigned candidate=unsigned(path.size()-1);candidate>at+1;candidate--){
    auto edge=Sub(path[candidate].position,path[at].position);double square=Dot(edge,edge);bool exterior=square>1e-20;
    for(unsigned knot=at+1;exterior&&knot<candidate;knot++){double t=std::clamp(Dot(Sub(path[knot].position,path[at].position),edge)/square,0.,1.);auto projected=Add(path[at].position,Mul(edge,t));if(Dot(Sub(projected,path[knot].position),Unit(path[knot].normal))<-preferredGap*1e-5)exterior=false;}
    if(exterior&&!contact(path[at].position,path[candidate].position)){next=candidate;break;}
   }
   taut.push_back(path[next]);at=next;
  }
  return taut;
}
inline Sheet Walk(const Input& input,const std::vector<Sample>& top,
                  const std::array<Sample,2>& bottom,unsigned rows,
                  double preferredGap,Contact contact,Project project,Section sectionRoute={}){
 ValidateRegions(input);if(top.size()<9||top.size()>129||rows<8||rows>64||!std::isfinite(preferredGap)||preferredGap<=0)throw std::invalid_argument("Measured drape grid outside bounded contract");
 Sheet result;result.rows=rows;result.columns=unsigned(top.size()-1);result.walks.resize(top.size());
 double leftCenter=0,rightCenter=0;for(auto id:input.anatomyRegions[2])leftCenter+=input.frame.Local(input.anatomy[id].position)[0];for(auto id:input.anatomyRegions[3])rightCenter+=input.frame.Local(input.anatomy[id].position)[0];leftCenter/=input.anatomyRegions[2].size();rightCenter/=input.anatomyRegions[3].size();unsigned negative=leftCenter<rightCenter?2:3,positive=5-negative;
 // The two sheet sides return to the measured attachment opening. This
 // places actual tissue inside the continuous cup instead of leaving open
 // triangular sides at the widest shaft/glans support.
 std::array<std::vector<Sample>,2> sidePaths;
 if(input.opening.size()>=3){
  double center=0;for(auto sample:input.opening)center+=input.frame.Local(sample.position)[0];center/=input.opening.size();
  for(unsigned side=0;side<2;side++){std::vector<Sample> arc;for(auto sample:input.opening)if((input.frame.Local(sample.position)[0]-center)*(side?1:-1)>=0)arc.push_back(sample);std::sort(arc.begin(),arc.end(),[&](const Sample&a,const Sample&b){return input.frame.Local(a.position)[2]>input.frame.Local(b.position)[2];});sidePaths[side].push_back(top[side?top.size()-1:0]);for(auto sample:arc){sample.position=Add(sample.position,Mul(Unit(sample.normal),preferredGap));project(sample);sidePaths[side].push_back(sample);}sidePaths[side].push_back(bottom[side]);}
 }
 for(unsigned col=0;col<top.size();col++){
  double t=double(col)/result.columns,u=2*t-1;auto lateral=Mul(input.frame.lateral,u*.7);
  auto measured=[&](unsigned region,Point direction){auto sample=Support(input,region,u,direction);sample.position=Add(sample.position,Mul(Unit(Length(sample.normal)>1e-14?sample.normal:direction),preferredGap));project(sample);return sample;};
  auto lower=[&](Point direction){auto a=measured(negative,Add(direction,lateral)),b=measured(positive,Add(direction,lateral));auto sample=Mix(a,b,t);project(sample);return sample;};
  std::vector<Sample> path{top[col],
   measured(0,Add(Add(input.frame.forward,Mul(input.frame.up,.45)),lateral)),
   measured(1,Add(input.frame.forward,lateral)),
   measured(1,Add(Add(Mul(input.frame.forward,.7),Mul(input.frame.up,-.7)),lateral)),
   lower(Add(Mul(input.frame.forward,.65),Mul(input.frame.up,-.35))),
   lower(Mul(input.frame.up,-1)),
   lower(Add(Mul(input.frame.forward,-.7),Mul(input.frame.up,-.3))),
   Mix(bottom[0],bottom[1],t)};
  auto section=sectionRoute?sectionRoute(top[col],Mix(bottom[0],bottom[1],t)):PlaneRoute(input,top[col],Mix(bottom[0],bottom[1],t),preferredGap,project);if(!section.empty())path=std::move(section);
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
  if(col==61){auto a=input.frame.Local(top[col].position),b=input.frame.Local(Mix(bottom[0],bottom[1],t).position);std::fprintf(stderr,"walk61 anchors %.9g %.9g %.9g / %.9g %.9g %.9g\n",a[0],a[1],a[2],b[0],b[1],b[2]);for(unsigned j=0;j<path.size();j++){auto p=input.frame.Local(path[j].position);std::fprintf(stderr,"walk61 hull %u %.9g %.9g %.9g\n",j,p[0],p[1],p[2]);}}
#endif
  if(!sectionRoute&&!sidePaths[0].empty()){unsigned side=u<0?0:1;double edgeWeight=std::pow(std::abs(u),4);for(unsigned knot=1;knot+1<path.size();knot++){auto edge=Resample(sidePaths[side],double(knot)/(path.size()-1));path[knot]=Mix(path[knot],edge,edgeWeight);project(path[knot]);}}
  project(path.back());
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
  if(col==61)for(unsigned j=0;j<path.size();j++){auto p=input.frame.Local(path[j].position);std::fprintf(stderr,"walk61 blended %u %.9g %.9g %.9g\n",j,p[0],p[1],p[2]);}
#endif
  // Static material fairing smooths each side path while retaining its
  // attachment endpoints. Only actual tissue can repel these positions;
  // this is not a permanent side seam or an attractive anatomy pin.
  if(col==0||col==result.columns)for(unsigned fair=0;fair<8;fair++){auto prior=path;for(unsigned k=1;k+1<path.size();k++){path[k].position=Add(Mul(prior[k].position,.5),Mul(Add(prior[k-1].position,prior[k+1].position),.25));project(path[k]);}}
  path=Tauten(path,preferredGap,contact);
  // A clear chord is the fabric bridge across a valley. Where a chord meets
  // tissue, advance via its actual contact point and face lineage instead.
  for(unsigned pass=0;pass<128;pass++){
   bool changed=false;std::vector<Sample> next;next.reserve(path.size()*2);next.push_back(path.front());
   for(unsigned i=1;i<path.size();i++){
    if(auto support=contact(path[i-1].position,path[i].position)){
     project(*support);double a=Length(Sub(support->position,path[i-1].position)),b=Length(Sub(support->position,path[i].position));
     if(a<preferredGap*1e-5||b<preferredGap*1e-5){support->position=Add(support->position,Mul(Unit(support->normal),preferredGap*.25));project(*support);a=Length(Sub(support->position,path[i-1].position));b=Length(Sub(support->position,path[i].position));if(a<preferredGap*1e-5||b<preferredGap*1e-5)throw std::invalid_argument("Measured material walk contact failed to advance");}next.push_back(*support);changed=true;
    }next.push_back(path[i]);
   }
   path.swap(next);path=Tauten(path,preferredGap,contact);if(path.size()>512)throw std::invalid_argument("Measured material walk exceeded contact budget");if(!changed)break;if(pass==127){
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
    std::fprintf(stderr,"walk failure col %u knots %zu\n",col,path.size());for(unsigned j=1;j<path.size();j++)if(auto h=contact(path[j-1].position,path[j].position)){auto a=input.frame.Local(path[j-1].position),b=input.frame.Local(path[j].position),p=input.frame.Local(h->position);auto escaped=*h;project(escaped);auto q=input.frame.Local(escaped.position);std::fprintf(stderr,"walk unresolved %u A %.9g %.9g %.9g B %.9g %.9g %.9g support %.9g %.9g %.9g projected %.9g %.9g %.9g normal %.9g %.9g %.9g\n",j,a[0],a[1],a[2],b[0],b[1],b[2],p[0],p[1],p[2],q[0],q[1],q[2],h->normal[0],h->normal[1],h->normal[2]);}
#endif
    throw std::invalid_argument("Measured material walk did not converge");}
  }
  path=Tauten(path,preferredGap,contact);
  result.walks[col]=std::move(path);
 }
 result.vertices.reserve((rows+1)*top.size());for(unsigned row=0;row<=rows;row++)for(unsigned col=0;col<top.size();col++)result.vertices.push_back(Resample(result.walks[col],double(row)/rows));
 return result;
}
}
