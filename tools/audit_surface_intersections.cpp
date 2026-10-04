// SDK-free offline triangle consistency audit. Zero-thickness triangle
// intersections are separate from external tissue clearance and cloth physics.
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>
using V=std::array<double,3>;
static V Sub(V a,V b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
static V Cross(V a,V b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
static double Dot(V a,V b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static double Length(V a){return std::sqrt(Dot(a,a));}
struct Face{unsigned id;std::array<unsigned,3> vertices;std::array<V,3> p;V low,high,normal;double area2;};
static bool Intersects(const Face& a,const Face& b,double tolerance){
 auto separated=[&](V axis){double n=Length(axis);if(n<1e-20)return false;for(auto& x:axis)x/=n;double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;for(auto p:a.p){auto d=Dot(p,axis);amin=(std::min)(amin,d);amax=(std::max)(amax,d);}for(auto p:b.p){auto d=Dot(p,axis);bmin=(std::min)(bmin,d);bmax=(std::max)(bmax,d);}return amax<bmin-tolerance||bmax<amin-tolerance;};
 if(separated(a.normal)||separated(b.normal))return false;
 std::array<V,3> ae,be;for(unsigned k=0;k<3;k++){ae[k]=Sub(a.p[(k+1)%3],a.p[k]);be[k]=Sub(b.p[(k+1)%3],b.p[k]);}
 for(auto x:ae)for(auto y:be)if(separated(Cross(x,y)))return false;
 // Coplanar triangles need in-plane edge normals, which are missing from
 // the ordinary edge-cross-edge axes when both triangles share a plane.
 for(auto edge:ae)if(separated(Cross(a.normal,edge)))return false;
 for(auto edge:be)if(separated(Cross(b.normal,edge)))return false;
 return true;
}
static Face At(std::array<V,3> p,unsigned id=0){Face f{};f.id=id;f.p=p;f.low=f.high=p[0];for(auto x:p)for(unsigned k=0;k<3;k++){f.low[k]=(std::min)(f.low[k],x[k]);f.high[k]=(std::max)(f.high[k],x[k]);}f.normal=Cross(Sub(p[1],p[0]),Sub(p[2],p[0]));f.area2=Length(f.normal);return f;}
static bool Coplanar(const Face& a,const Face& b,double tolerance){
 if(Length(Cross(a.normal,b.normal))>a.area2*b.area2*1e-9)return false;
 for(auto p:b.p)if(std::abs(Dot(Sub(p,a.p[0]),a.normal))>tolerance*a.area2)return false;
 return true;
}
enum class IntersectionKind {Touch,Crossing,CoplanarOverlap};
static IntersectionKind Kind(const Face& a,const Face& b,double tolerance,double areaThreshold){
 if(Coplanar(a,b,tolerance)){
  using P=std::array<double,2>;unsigned omit=0;for(unsigned k=1;k<3;k++)if(std::abs(a.normal[k])>std::abs(a.normal[omit]))omit=k;
  auto xy=[&](V p){p=Sub(p,a.p[0]);P q{};unsigned at=0;for(unsigned k=0;k<3;k++)if(k!=omit)q[at++]=p[k];return q;};
  auto side=[](P p,P x,P y){return (y[0]-x[0])*(p[1]-x[1])-(y[1]-x[1])*(p[0]-x[0]);};
  std::vector<P> polygon{xy(a.p[0]),xy(a.p[1]),xy(a.p[2])};std::array<P,3> clip{xy(b.p[0]),xy(b.p[1]),xy(b.p[2])};double orientation=side(clip[2],clip[0],clip[1])>=0?1:-1;
  for(unsigned edge=0;edge<3&&!polygon.empty();edge++){
   std::vector<P> output;auto x=clip[edge],y=clip[(edge+1)%3];
   for(unsigned k=0;k<polygon.size();k++){auto p=polygon[k],q=polygon[(k+1)%polygon.size()];double dp=side(p,x,y)*orientation,dq=side(q,x,y)*orientation;bool ip=dp>=0,iq=dq>=0;if(ip)output.push_back(p);if(ip!=iq){double t=dp/(dp-dq);output.push_back({p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])});}}
   polygon=std::move(output);
  }
  double area2=0;for(unsigned k=0;k<polygon.size();k++){auto p=polygon[k],q=polygon[(k+1)%polygon.size()];area2+=p[0]*q[1]-p[1]*q[0];}
  double area=std::abs(area2)*.5*a.area2/std::abs(a.normal[omit]);return area>areaThreshold?IntersectionKind::CoplanarOverlap:IntersectionKind::Touch;
 }
 V na=a.normal,nb=b.normal;for(auto& x:na)x/=a.area2;for(auto& x:nb)x/=b.area2;auto direction=Cross(na,nb);double n=Length(direction);if(n<1e-12)return IntersectionKind::Touch;for(auto& x:direction)x/=n;
 auto interval=[&](const Face& f,const Face& plane,V normal){
  std::array<double,3> distance;double minimum=1e100,maximum=-1e100;for(unsigned k=0;k<3;k++){distance[k]=Dot(Sub(f.p[k],plane.p[0]),normal);minimum=(std::min)(minimum,distance[k]);maximum=(std::max)(maximum,distance[k]);}
  bool straddles=minimum<-tolerance&&maximum>tolerance;double low=1e100,high=-1e100;
  auto put=[&](V p){double t=Dot(Sub(p,a.p[0]),direction);low=(std::min)(low,t);high=(std::max)(high,t);};
  for(unsigned k=0;k<3;k++){if(std::abs(distance[k])<=tolerance)put(f.p[k]);unsigned next=(k+1)%3;if((distance[k]<0)!=(distance[next]<0)){double t=distance[k]/(distance[k]-distance[next]);V p;for(unsigned axis=0;axis<3;axis++)p[axis]=f.p[k][axis]+t*(f.p[next][axis]-f.p[k][axis]);put(p);}}
  return std::array<double,3>{low,high,straddles?1.:0.};
 };
 auto ai=interval(a,b,nb),bi=interval(b,a,na);double overlap=(std::min)(ai[1],bi[1])-(std::max)(ai[0],bi[0]);return ai[2]&&bi[2]&&overlap>tolerance?IntersectionKind::Crossing:IntersectionKind::Touch;
}
int main(int argc,char** argv){try{
 if(argc==2&&std::string(argv[1])=="--self-test"){
  auto a=At({V{0,0,0},V{2,0,0},V{0,2,0}});
  if(!Intersects(a,At({V{.2,.2,-1},V{.2,.2,1},V{1,1,0}}),1e-9))throw std::runtime_error("Crossing triangles missed");
  if(Intersects(a,At({V{0,0,1},V{2,0,1},V{0,2,1}}),1e-9))throw std::runtime_error("Parallel separated triangles intersected");
  if(Intersects(a,At({V{3,3,0},V{4,3,0},V{3,4,0}}),1e-9))throw std::runtime_error("Coplanar separated triangles intersected");
  auto nested=At({V{.2,.2,0},V{1,.2,0},V{.2,1,0}});if(!Intersects(a,nested,1e-9)||Kind(a,nested,1e-9,1e-12)!=IntersectionKind::CoplanarOverlap)throw std::runtime_error("Coplanar positive-area overlap missed");
  auto touching=At({V{2,0,0},V{3,0,0},V{2,1,0}});if(Kind(a,touching,1e-9,1e-12)!=IntersectionKind::Touch)throw std::runtime_error("Coplanar edge/vertex touch mislabeled as overlap");
  auto proper=At({V{.5,-.5,-1},V{.5,1.5,1},V{.5,1.5,-1}});if(!Intersects(a,proper,1e-9)||Kind(a,proper,1e-9,1e-12)!=IntersectionKind::Crossing)throw std::runtime_error("Proper noncoplanar crossing missed");
  std::cout<<"PASS proper crossing, positive-area coplanar overlap, contact touch and separation\n";return 0;
 }
 if(argc!=5&&argc!=6)throw std::runtime_error("OBJ first-face face-count report-json [--all-pairs]");
 bool allPairs=argc==6;if(allPairs&&std::string(argv[5])!="--all-pairs")throw std::runtime_error("Unknown audit option");
 std::ifstream source(argv[1]);if(!source)throw std::runtime_error("Cannot read actual surface OBJ");std::vector<V> vertices;std::vector<std::array<unsigned,3>> triangles;std::string line;
 while(std::getline(source,line)){std::istringstream row(line);std::string tag;row>>tag;if(tag=="v"){V p;row>>p[0]>>p[1]>>p[2];if(!row||!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))throw std::runtime_error("Invalid OBJ vertex");vertices.push_back(p);}else if(tag=="f"){std::array<unsigned,3> face;for(auto& id:face){std::string index;row>>index;unsigned value=unsigned(std::stoul(index));if(!value)throw std::runtime_error("Zero OBJ index");id=value-1;}triangles.push_back(face);}}
 unsigned begin=unsigned(std::stoul(argv[2])),count=unsigned(std::stoul(argv[3]));if(!count||std::size_t(begin)+count>triangles.size())throw std::runtime_error("Actual surface face range invalid");
 std::vector<Face> faces;V low{},high{};bool first=true;for(unsigned id=begin;id<begin+count;id++){auto t=triangles[id];for(auto v:t)if(v>=vertices.size())throw std::runtime_error("Actual surface topology exceeds vertices");auto f=At({vertices[t[0]],vertices[t[1]],vertices[t[2]]},id);f.vertices=t;faces.push_back(f);if(first){low=f.low;high=f.high;first=false;}else for(unsigned k=0;k<3;k++){low[k]=(std::min)(low[k],f.low[k]);high[k]=(std::max)(high[k],f.high[k]);}}
 double diagonal=Length(Sub(high,low)),tolerance=diagonal*1e-9,areaThreshold=diagonal*diagonal*1e-12,minArea=1e100;unsigned degenerate=0;std::vector<unsigned> degenerateFaces;
 for(auto f:faces){minArea=(std::min)(minArea,f.area2*.5);if(f.area2<=areaThreshold){degenerate++;if(degenerateFaces.size()<100)degenerateFaces.push_back(f.id);}}
 std::sort(faces.begin(),faces.end(),[](const Face&a,const Face&b){return a.low[0]<b.low[0];});std::uint64_t candidates=0,intersections=0,coplanar=0,noncoplanar=0,touches=0,allowedAdjacentTouches=0;struct Pair{unsigned a,b;IntersectionKind kind;};std::vector<Pair> pairs,touchPairs;
 for(unsigned i=0;i<faces.size();i++){const auto& a=faces[i];if(a.area2<=areaThreshold)continue;for(unsigned j=i+1;j<faces.size()&&faces[j].low[0]<=a.high[0]+tolerance;j++){const auto& b=faces[j];if(b.area2<=areaThreshold||a.high[1]<b.low[1]-tolerance||b.high[1]<a.low[1]-tolerance||a.high[2]<b.low[2]-tolerance||b.high[2]<a.low[2]-tolerance)continue;bool adjacent=false;for(auto x:a.vertices)for(auto y:b.vertices)adjacent=adjacent||x==y;candidates++;if(Intersects(a,b,tolerance)){auto kind=Kind(a,b,tolerance,areaThreshold);if(adjacent&&kind==IntersectionKind::Touch){allowedAdjacentTouches++;continue;}intersections++;if(kind==IntersectionKind::CoplanarOverlap)coplanar++;else if(kind==IntersectionKind::Crossing)noncoplanar++;else touches++;auto& samples=kind==IntersectionKind::Touch?touchPairs:pairs;if(allPairs||samples.size()<100)samples.push_back({a.id,b.id,kind});}}}
 std::ofstream report(argv[4]);report<<std::setprecision(17)<<"{\"faceStart\":"<<begin<<",\"faceCount\":"<<count<<",\"geometryDiagonal\":"<<diagonal<<",\"intersectionTolerance\":"<<tolerance<<",\"coplanarOverlapAreaThreshold\":"<<areaThreshold<<",\"twiceAreaDegeneracyThreshold\":"<<areaThreshold<<",\"minimumTriangleArea\":"<<minArea<<",\"degenerateFaceCount\":"<<degenerate<<",\"degenerateFaces\":[";for(unsigned k=0;k<degenerateFaces.size();k++)report<<(k?",":"")<<degenerateFaces[k];report<<"],\"triangleAABBCandidates\":"<<candidates<<",\"unexpectedIntersectionCount\":"<<intersections<<",\"sheetCoplanarOverlaps\":"<<coplanar<<",\"sheetCrossings\":"<<noncoplanar<<",\"sheetTouchPairs\":"<<touches<<",\"firstIntersectionPairs\":[";for(unsigned k=0;k<pairs.size();k++)report<<(k?",":"")<<"{\"faces\":["<<pairs[k].a<<','<<pairs[k].b<<"],\"kind\":\""<<(pairs[k].kind==IntersectionKind::Crossing?"crossing":pairs[k].kind==IntersectionKind::CoplanarOverlap?"coplanar-overlap":"touch")<<"\"}";report<<"],\"firstContactTouchPairs\":[";for(unsigned k=0;k<touchPairs.size();k++)report<<(k?",[":"[")<<touchPairs[k].a<<','<<touchPairs[k].b<<']';report<<"],\"allIntersectionPairsReported\":"<<(allPairs?"true":"false")<<",\"allowedTopologicalTouches\":"<<allowedAdjacentTouches<<",\"excludesOnlySharedVertexTouches\":true,\"includesCoplanarOverlap\":true,\"meaning\":\"zero-thickness surface geometry diagnostic; external collider clearance and dynamic cloth are separate\"}\n";if(!report)throw std::runtime_error("Cannot save actual surface audit");std::cout<<"faces="<<count<<" degenerate="<<degenerate<<" unexpectedIntersections="<<intersections<<'\n';return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}


