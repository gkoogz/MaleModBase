#pragma once
// Rest-only drape from measured section prominences. The convex route is an
// external material string, never a replacement collision/anatomy surface.
namespace malemod::garments::drape {
inline std::vector<Sample> PlaneRoute(const Input& input,const Sample& top,const Sample& bottom,double gap,const Project& project){
 auto down=Sub(bottom.position,top.position);if(Length(down)<gap)return {};down=Unit(down);auto forward=Sub(input.frame.forward,Mul(down,Dot(input.frame.forward,down)));if(Length(forward)<1e-6)forward=Sub(input.frame.up,Mul(down,Dot(input.frame.up,down)));forward=Unit(forward);auto normal=Unit(Cross(down,forward));
 struct Knot{Sample sample;double s,u;};std::vector<Knot> knots;auto add=[&](Sample sample){auto r=Sub(sample.position,top.position);knots.push_back({sample,Dot(r,down),Dot(r,forward)});};add(top);add(bottom);
 auto collect=[&](const std::vector<Sample>& surface,const std::vector<std::array<unsigned,3>>& faces,bool restrictToSpan){for(auto triangle:faces)for(unsigned edge=0;edge<3;edge++){auto a=surface[triangle[edge]],b=surface[triangle[(edge+1)%3]];double da=Dot(Sub(a.position,top.position),normal),db=Dot(Sub(b.position,top.position),normal);if((da>0)==(db>0)||std::abs(da-db)<1e-18)continue;auto sample=Mix(a,b,da/(da-db));double along=Dot(Sub(sample.position,top.position),down);if(restrictToSpan&&(along<0||along>Length(Sub(bottom.position,top.position))))continue;auto outside=Sub(sample.normal,Mul(normal,Dot(sample.normal,normal)));if(Length(outside)<1e-8)continue;sample.position=Add(sample.position,Mul(Unit(outside),gap));project(sample);add(sample);}};
 collect(input.anatomy,input.anatomyTriangles,false);
 // The string must also clear the measured hip/thigh sections in its sewn
 // longitudinal span. Excluding them from the initial support hull caused
 // repeated detours into the overlap between lobe and thigh surfaces.
 collect(input.bodySurface,input.bodyTriangles,true);
 if(knots.size()<5)return {};
 std::sort(knots.begin(),knots.end(),[](const Knot&a,const Knot&b){return a.s<b.s||a.s==b.s&&a.u<b.u;});
 auto turn=[](const Knot&a,const Knot&b,const Knot&c){return (b.s-a.s)*(c.u-a.u)-(b.u-a.u)*(c.s-a.s);};std::vector<Knot> hull;
 for(auto p:knots){while(hull.size()>1&&turn(hull[hull.size()-2],hull.back(),p)<=0)hull.pop_back();hull.push_back(p);}unsigned lower=unsigned(hull.size());for(unsigned i=unsigned(knots.size()-1);i-->0;){auto p=knots[i];while(hull.size()>lower&&turn(hull[hull.size()-2],hull.back(),p)<=0)hull.pop_back();hull.push_back(p);}if(hull.size()<4)return {};hull.pop_back();
 unsigned start=0,finish=0;double ds=1e100,df=1e100;for(unsigned i=0;i<hull.size();i++){double a=Length(Sub(hull[i].sample.position,top.position)),b=Length(Sub(hull[i].sample.position,bottom.position));if(a<ds){start=i;ds=a;}if(b<df){finish=i;df=b;}}
 if(start==finish)return {};std::array<std::vector<Sample>,2> routes;double score[2]{};for(unsigned direction=0;direction<2;direction++){auto& path=routes[direction];path.push_back(top);unsigned at=start;for(unsigned count=0;count<hull.size();count++){path.push_back(hull[at].sample);if(at==finish)break;at=direction?(at+hull.size()-1)%hull.size():(at+1)%hull.size();}path.push_back(bottom);for(unsigned k=1;k<path.size();k++){auto midpoint=Mul(Add(path[k-1].position,path[k].position),.5);score[direction]+=Dot(Sub(midpoint,top.position),forward)*Length(Sub(path[k].position,path[k-1].position));}}
 return routes[score[1]>score[0]?1:0];
}
}
