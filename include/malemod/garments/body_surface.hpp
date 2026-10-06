#pragma once
// Closest-surface contacts over the caller's measured body triangles. The
// adapter owns skinning/SDK data. This BVH only stores caller-space geometry.
namespace malemod::garments::detail {
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
struct SurfacePerf {unsigned points=0,faces=0;double pointSeconds=0,faceSeconds=0;};
inline thread_local SurfacePerf surfacePerf;
struct SurfaceTimer{bool face;std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();~SurfaceTimer(){double dt=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();if(face){surfacePerf.faces++;surfacePerf.faceSeconds+=dt;}else{surfacePerf.points++;surfacePerf.pointSeconds+=dt;}}};
#define MALEMOD_SURFACE_TIME(type) SurfaceTimer queryTimer{type};
#else
#define MALEMOD_SURFACE_TIME(type)
#endif
inline double BoxDistance(Point p,Point lo,Point hi){double d=0;for(unsigned k=0;k<3;k++){double q=(std::max)({lo[k]-p[k],0.,p[k]-hi[k]});d+=q*q;}return d;}
inline bool FaceBoxesOverlap(const std::array<Point,3>& face,Point lo,Point hi,double margin){
 for(unsigned k=0;k<3;k++)if((std::max)({face[0][k],face[1][k],face[2][k]})<lo[k]-margin||(std::min)({face[0][k],face[1][k],face[2][k]})>hi[k]+margin)return false;
 // A long oblique triangle's coordinate box may cover a large empty volume.
 // Its plane is another conservative separating axis for the entire BVH box.
 // Use relative coordinates and round outward; never prune within the margin.
 const auto n=Cross(Sub(face[1],face[0]),Sub(face[2],face[0]));
 const double square=Dot(n,n);if(square<1e-28)return true;
 const auto extent=Mul(Sub(hi,lo),.5),relative=Add(Sub(lo,face[0]),extent);
 double projection=0,radius=0,magnitude=0;
 for(unsigned k=0;k<3;k++){projection+=n[k]*relative[k];radius+=std::abs(n[k])*extent[k];magnitude+=std::abs(n[k]*relative[k]);}
 const double padding=margin*std::sqrt(square);
 const double roundoff=32*std::numeric_limits<double>::epsilon()*(magnitude+radius+padding);
 return std::abs(projection)<=radius+padding+roundoff;
}
inline bool FaceBoxesOverlap(const SurfaceFaceQuery& query,Point lo,Point hi,double margin){
 for(unsigned k=0;k<3;k++)if(query.high[k]<lo[k]-margin||query.low[k]>hi[k]+margin)return false;
 const auto n=query.normal;const double square=query.square;if(square<1e-28)return true;
 const auto extent=Mul(Sub(hi,lo),.5),relative=Add(Sub(lo,query.face[0]),extent);
 double projection=0,radius=0,magnitude=0;for(unsigned k=0;k<3;k++){projection+=n[k]*relative[k];radius+=std::abs(n[k])*extent[k];magnitude+=std::abs(n[k]*relative[k]);}
 const double padding=margin*query.normalLength;const double roundoff=32*std::numeric_limits<double>::epsilon()*(magnitude+radius+padding);
 return std::abs(projection)<=radius+padding+roundoff;
}
inline bool SegmentFace(Point a,Point b,const std::array<Point,3>& face,Point& intersection){auto n=Cross(Sub(face[1],face[0]),Sub(face[2],face[0])),delta=Sub(b,a);double den=Dot(n,delta);if(std::abs(den)<1e-24)return false;double t=Dot(n,Sub(face[0],a))/den;if(t<0||t>1)return false;auto p=Add(a,Mul(delta,t)),q=ClosestTriangle(p,face[0],face[1],face[2]);if(Dot(Sub(p,q),Sub(p,q))>1e-24)return false;intersection=p;return true;}
inline bool SeparatedTriangles(const std::array<Point,3>& a,const std::array<Point,3>& b,double margin){
 auto separated=[&](Point axis){double square=Dot(axis,axis);if(square<1e-28)return false;double amin=Dot(a[0],axis),amax=amin,bmin=Dot(b[0],axis),bmax=bmin;for(unsigned k=1;k<3;k++){double p=Dot(a[k],axis),q=Dot(b[k],axis);amin=(std::min)(amin,p);amax=(std::max)(amax,p);bmin=(std::min)(bmin,q);bmax=(std::max)(bmax,q);}double gap=(std::max)({amin-bmax,bmin-amax,0.});return gap*gap>margin*margin*square*(1+1e-12);};
 std::array<Point,3> ea,eb;for(unsigned k=0;k<3;k++){ea[k]=Sub(a[(k+1)%3],a[k]);eb[k]=Sub(b[(k+1)%3],b[k]);}
 if(separated(Cross(ea[0],ea[1]))||separated(Cross(eb[0],eb[1])))return true;
 for(auto x:ea)for(auto y:eb)if(separated(Cross(x,y)))return true;return false;
}
inline bool SeparatedTriangles(const SurfaceFaceQuery& query,const std::array<Point,3>& b,double margin){
 const auto& a=query.face;auto separated=[&](Point axis){double square=Dot(axis,axis);if(square<1e-28)return false;double amin=Dot(a[0],axis),amax=amin,bmin=Dot(b[0],axis),bmax=bmin;for(unsigned k=1;k<3;k++){double p=Dot(a[k],axis),q=Dot(b[k],axis);amin=(std::min)(amin,p);amax=(std::max)(amax,p);bmin=(std::min)(bmin,q);bmax=(std::max)(bmax,q);}double gap=(std::max)({amin-bmax,bmin-amax,0.});return gap*gap>margin*margin*square*(1+1e-12);};
 const auto& ea=query.edges;std::array<Point,3> eb;for(unsigned k=0;k<3;k++)eb[k]=Sub(b[(k+1)%3],b[k]);
 if(separated(query.satNormal)||separated(Cross(eb[0],eb[1])))return true;
 for(auto x:ea)for(auto y:eb)if(separated(Cross(x,y)))return true;return false;
}
inline double SurfaceTriangleDistance(const std::array<Point,3>& a,const std::array<Point,3>& b,Point& pa,Point& pb){for(unsigned k=0;k<3;k++){Point p;if(SegmentFace(a[k],a[(k+1)%3],b,p)||SegmentFace(b[k],b[(k+1)%3],a,p)){pa=pb=p;return 0;}}return ClosestTriangles(a,b,pa,pb);}
inline void BodyCollider::Update(const std::vector<Sample>& vertices,const std::vector<std::array<std::uint32_t,3>>& faces,Point origin,double scale){
 if(vertices.empty()!=faces.empty()||vertices.size()>131072||faces.size()>262144||!std::isfinite(scale)||scale<=0)throw std::invalid_argument("Invalid measured body surface");
 if(vertices.empty()){Clear();return;}bool rebuild=triangles_!=faces||points_.size()!=vertices.size(),changed=rebuild;double travel=0;points_.resize(vertices.size());for(unsigned i=0;i<vertices.size();i++){if(!Finite(vertices[i].position))throw std::invalid_argument("Nonfinite measured body surface");auto next=Mul(Sub(vertices[i].position,origin),1/scale);if(!Finite(next))throw std::invalid_argument("Nonfinite normalized body surface");if(!rebuild){changed=changed||next!=points_[i];travel=(std::max)(travel,Length(Sub(next,points_[i])));}points_[i]=next;}for(auto f:faces)for(auto i:f)if(i>=points_.size())throw std::invalid_argument("Body triangle index outside surface");motion_.Advance(travel,rebuild);
 // Exact equality only: repeated paused/static substeps need no normal/bounds
 // recomputation or BVH refit. All current inputs were still validated above.
 if(!changed)return;
 faceNormals_.resize(faces.size());faceRawNormals_.resize(faces.size());faceLo_.resize(faces.size());faceHi_.resize(faces.size());
 for(unsigned i=0;i<faces.size();i++){auto f=faces[i];auto a=points_[f[0]],b=points_[f[1]],c=points_[f[2]],n=Cross(Sub(b,a),Sub(c,a));faceRawNormals_[i]=n;faceNormals_[i]=Dot(n,n)<1e-28?Point{}:Unit(n);for(unsigned k=0;k<3;k++){faceLo_[i][k]=(std::min)({a[k],b[k],c[k]});faceHi_[i][k]=(std::max)({a[k],b[k],c[k]});}}
 if(rebuild){triangles_=faces;order_.resize(faces.size());for(unsigned i=0;i<order_.size();i++)order_[i]=i;nodes_.clear();nodes_.reserve(faces.size()/2+1);Build(0,unsigned(order_.size()));}else Refit(0);
}
inline unsigned BodyCollider::Build(unsigned begin,unsigned end){unsigned at=unsigned(nodes_.size());nodes_.push_back({});nodes_[at].begin=begin;nodes_[at].end=end;Refit(at);if(end-begin>8){auto box=Sub(nodes_[at].hi,nodes_[at].lo);unsigned axis=box[1]>box[0]?1:0;if(box[2]>box[axis])axis=2;unsigned mid=(begin+end)/2;std::nth_element(order_.begin()+begin,order_.begin()+mid,order_.begin()+end,[&](unsigned a,unsigned b){double ca=0,cb=0;for(unsigned k=0;k<3;k++){ca+=points_[triangles_[a][k]][axis];cb+=points_[triangles_[b][k]][axis];}return ca<cb;});unsigned left=Build(begin,mid),right=Build(mid,end);nodes_[at].left=left;nodes_[at].right=right;}return at;}
inline void BodyCollider::Refit(unsigned at){auto& node=nodes_[at];node.lo={1e100,1e100,1e100};node.hi={-1e100,-1e100,-1e100};if(node.left){Refit(node.left);Refit(node.right);for(unsigned k=0;k<3;k++){node.lo[k]=(std::min)(nodes_[node.left].lo[k],nodes_[node.right].lo[k]);node.hi[k]=(std::max)(nodes_[node.left].hi[k],nodes_[node.right].hi[k]);}}else for(unsigned j=node.begin;j<node.end;j++)for(auto id:triangles_[order_[j]])for(unsigned k=0;k<3;k++){node.lo[k]=(std::min)(node.lo[k],points_[id][k]);node.hi[k]=(std::max)(node.hi[k],points_[id][k]);}}
inline BodyCollider::Hit BodyCollider::Closest(Point point,unsigned seedTriangle)const{MALEMOD_SURFACE_TIME(false) Hit hit;if(!Empty()){if(seedTriangle<triangles_.size())Consider(seedTriangle,point,hit);Search(0,point,hit);}return hit;}
inline BodyCollider::Hit BodyCollider::Near(Point point,double radius,unsigned seedTriangle)const{MALEMOD_SURFACE_TIME(false) if(!std::isfinite(radius)||radius<=0)throw std::invalid_argument("Invalid surface neighborhood");Hit hit;hit.distance=radius;if(!Empty()){if(seedTriangle<triangles_.size())Consider(seedTriangle,point,hit);Search(0,point,hit);}return hit;}
inline bool BodyCollider::CollectPoints(unsigned at,Point point,double radius,std::vector<unsigned>& candidates)const{
 const auto& node=nodes_[at];const double square=radius*radius;
 if(BoxDistance(point,node.lo,node.hi)>square)return true;
 if(node.left)return CollectPoints(node.left,point,radius,candidates)&&CollectPoints(node.right,point,radius,candidates);
 for(unsigned j=node.begin;j<node.end;j++){auto id=order_[j];if(BoxDistance(point,faceLo_[id],faceHi_[id])<=square){if(candidates.size()==256)return false;candidates.push_back(id);}}
 return true;
}
inline BodyCollider::Hit BodyCollider::NearCached(Point point,double radius,PointNeighborhood& neighborhood,unsigned seedTriangle)const{
 if(!std::isfinite(radius)||radius<=0||!Finite(point))throw std::invalid_argument("Invalid cached point neighborhood");
 Hit hit;hit.distance=hit.signedDistance=radius;hit.triangle=UINT32_MAX;if(Empty())return hit;
 const std::array<Point,3> repeated{point,point,point};const auto owner=reinterpret_cast<std::uintptr_t>(this);const auto stamp=MotionStamp();
 if(!neighborhood.bound_.Covers(repeated,radius,stamp,owner)){
  const double padded=radius*4;if(!std::isfinite(padded))throw std::invalid_argument("Point neighborhood radius overflow");
  neighborhood.candidates_.clear();
  if(!CollectPoints(0,point,padded,neighborhood.candidates_)){neighborhood.bound_={};neighborhood.candidates_.clear();return Near(point,radius,seedTriangle);}
  neighborhood.bound_.Remember(repeated,padded,stamp,owner);
 }
 if(seedTriangle<triangles_.size())Consider(seedTriangle,point,hit);
 for(auto id:neighborhood.candidates_)Consider(id,point,hit);
 return hit;
}
inline void BodyCollider::Consider(unsigned id,Point p,Hit& hit)const{
 auto n=faceNormals_[id];if(n==Point{}||BoxDistance(p,faceLo_[id],faceHi_[id])>hit.distance*hit.distance)return;
 auto f=triangles_[id];auto q=ClosestTriangle(p,points_[f[0]],points_[f[1]],points_[f[2]]),delta=Sub(p,q);double square=Dot(delta,delta);
 if(square<hit.distance*hit.distance){double d=std::sqrt(square),signedDistance=Dot(delta,n)>=0?d:-d;hit={q,signedDistance>=0&&d>1e-14?Mul(delta,1/d):n,d,signedDistance,id,p};}
}
inline void BodyCollider::Search(unsigned at,Point p,Hit& hit)const{const auto& node=nodes_[at];if(BoxDistance(p,node.lo,node.hi)>hit.distance*hit.distance)return;if(node.left){double a=BoxDistance(p,nodes_[node.left].lo,nodes_[node.left].hi),b=BoxDistance(p,nodes_[node.right].lo,nodes_[node.right].hi);Search(a<b?node.left:node.right,p,hit);Search(a<b?node.right:node.left,p,hit);return;}for(unsigned j=node.begin;j<node.end;j++)Consider(order_[j],p,hit);}
inline BodyCollider::Hit BodyCollider::ClosestFace(const std::array<Point,3>& face,double margin,unsigned seedTriangle)const{MALEMOD_SURFACE_TIME(true) if(!std::isfinite(margin)||margin<=0||!Finite(face[0])||!Finite(face[1])||!Finite(face[2]))throw std::invalid_argument("Invalid triangle contact neighborhood");SurfaceFaceQuery query(face);Hit hit;hit.distance=hit.signedDistance=margin;hit.triangle=unsigned(-1);if(!Empty()){if(seedTriangle<triangles_.size())ConsiderFace(seedTriangle,query,hit);SearchFace(0,query,margin,hit);}return hit;}
inline BodyCollider::Hit BodyCollider::Sweep(Point from,Point to)const{Hit hit;if(!Empty())SearchSweep(0,from,to,hit);return hit;}
inline bool RayBox(Point from,Point direction,Point lo,Point hi){
 double nearDistance=0,farDistance=1e100;
 for(unsigned k=0;k<3;k++){if(std::abs(direction[k])<1e-30){if(from[k]<lo[k]||from[k]>hi[k])return false;}else{double a=(lo[k]-from[k])/direction[k],b=(hi[k]-from[k])/direction[k];if(a>b)std::swap(a,b);nearDistance=(std::max)(nearDistance,a);farDistance=(std::min)(farDistance,b);if(nearDistance>farDistance)return false;}}
 return farDistance>=0;
}
inline void BodyCollider::SearchRay(unsigned at,Point from,Point direction,unsigned& intersections,bool& ambiguous,bool& boundary)const{
 const auto& node=nodes_[at];if(ambiguous||boundary||!RayBox(from,direction,node.lo,node.hi))return;
 if(node.left){SearchRay(node.left,from,direction,intersections,ambiguous,boundary);SearchRay(node.right,from,direction,intersections,ambiguous,boundary);return;}
 for(unsigned j=node.begin;j<node.end;j++){
  unsigned id=order_[j];if(faceNormals_[id]==Point{}||!RayBox(from,direction,faceLo_[id],faceHi_[id]))continue;
  auto f=triangles_[id];auto a=points_[f[0]],e1=Sub(points_[f[1]],a),e2=Sub(points_[f[2]],a),cross=Cross(direction,e2);double determinant=Dot(e1,cross);
  double magnitude=Length(e1)*Length(e2);if(std::abs(determinant)<=magnitude*1e-13){ambiguous=true;return;}
  auto offset=Sub(from,a);double u=Dot(offset,cross)/determinant;auto q=Cross(offset,e1);double v=Dot(direction,q)/determinant,t=Dot(e2,q)/determinant;
  constexpr double edgeTolerance=1e-11;double distanceTolerance=1e-11*(1+Length(Sub(nodes_[0].hi,nodes_[0].lo)));
  if(u<-edgeTolerance||v<-edgeTolerance||u+v>1+edgeTolerance||t<-distanceTolerance)continue;
  if(std::abs(t)<=distanceTolerance){boundary=true;return;}
  // Retry the whole ray at edges/vertices. Counting shared triangles twice
  // or merging nearly equal distant intersections can reverse parity.
  if(u<=edgeTolerance||v<=edgeTolerance||1-u-v<=edgeTolerance){ambiguous=true;return;}
  intersections++;
 }
}
inline BodyCollider::Side BodyCollider::Classify(Point point,bool verifiedClosed)const{
 if(!Finite(point))throw std::invalid_argument("Nonfinite surface membership point");
 if(!verifiedClosed||Empty())return Side::Indeterminate;
 const auto& box=nodes_[0];for(unsigned k=0;k<3;k++)if(point[k]<box.lo[k]||point[k]>box.hi[k])return Side::Outside;
 unsigned axis=0;for(unsigned k=1;k<3;k++)if(box.hi[k]-box.lo[k]>box.hi[axis]-box.lo[axis])axis=k;
 for(unsigned attempt=0;attempt<32;attempt++){
  Point direction{};if(!attempt)direction[axis]=1;
  else {double n=double(attempt);direction=Unit({std::sin(n*1.618033988749895+.317),std::cos(n*1.414213562373095+.719),std::sin(n*1.732050807568877+1.119)});}
  unsigned intersections=0;bool ambiguous=false,boundary=false;SearchRay(0,point,direction,intersections,ambiguous,boundary);
  if(boundary)return Side::Boundary;if(!ambiguous)return intersections%2?Side::Inside:Side::Outside;
 }
 return Side::Indeterminate;
}
inline void BodyCollider::SearchSweep(unsigned at,Point from,Point to,Hit& hit)const{
 const auto& node=nodes_[at];if(!FaceBoxesOverlap({from,to,to},node.lo,node.hi,0))return;
 if(node.left){SearchSweep(node.left,from,to,hit);SearchSweep(node.right,from,to,hit);return;}
 auto delta=Sub(to,from);for(unsigned j=node.begin;j<node.end;j++){unsigned id=order_[j];auto f=triangles_[id];std::array<Point,3> face{points_[f[0]],points_[f[1]],points_[f[2]]};auto n=faceRawNormals_[id];if(Dot(n,delta)>=-1e-24||faceNormals_[id]==Point{})continue;Point p;if(!SegmentFace(from,to,face,p))continue;double d=Length(Sub(p,from));if(d<hit.distance)hit={p,faceNormals_[id],d,0,id,p};}
}
inline void BodyCollider::ConsiderFace(unsigned id,const SurfaceFaceQuery& query,Hit& hit)const{auto n=faceNormals_[id];if(n==Point{}||!FaceBoxesOverlap(query,faceLo_[id],faceHi_[id],hit.distance))return;auto f=triangles_[id];std::array<Point,3> body{points_[f[0]],points_[f[1]],points_[f[2]]};if(SeparatedTriangles(query,body,hit.distance))return;Point p,q;double distance=SurfaceTriangleDistance(query.face,body,p,q);if(distance<hit.distance){auto delta=Sub(p,q);double signedDistance=Dot(delta,n)>=0?distance:-distance;hit={q,signedDistance>=0&&distance>1e-14?Mul(delta,1/distance):n,distance,signedDistance,id,p};}}
inline void BodyCollider::SearchFace(unsigned at,const SurfaceFaceQuery& query,double margin,Hit& hit)const{const auto& node=nodes_[at];if(!FaceBoxesOverlap(query,node.lo,node.hi,(std::min)(margin,hit.distance)))return;if(node.left){SearchFace(node.left,query,margin,hit);SearchFace(node.right,query,margin,hit);return;}for(unsigned j=node.begin;j<node.end;j++)ConsiderFace(order_[j],query,hit);}
inline bool BodyCollider::CollectFaces(unsigned at,const SurfaceFaceQuery& query,double radius,std::vector<unsigned>& candidates,std::vector<unsigned>* exclusions,bool* exclusionsComplete)const{
 auto exclude=[&](unsigned id){if(exclusions&&exclusionsComplete&&*exclusionsComplete){if(exclusions->size()==512){exclusions->clear();*exclusionsComplete=false;}else exclusions->push_back(id);}};
 const auto& node=nodes_[at];if(!FaceBoxesOverlap(query,node.lo,node.hi,radius)){exclude(at|0x80000000u);return true;}
 if(node.left)return CollectFaces(node.left,query,radius,candidates,exclusions,exclusionsComplete)&&CollectFaces(node.right,query,radius,candidates,exclusions,exclusionsComplete);
 for(unsigned j=node.begin;j<node.end;j++){unsigned id=order_[j];if(FaceBoxesOverlap(query,faceLo_[id],faceHi_[id],radius)){if(candidates.size()==512)return false;candidates.push_back(id);}else exclude(id);}
 return true;
}
inline BodyCollider::Hit BodyCollider::ClosestFaceCached(const std::array<Point,3>& face,double margin,FaceNeighborhood& neighborhood,unsigned seedTriangle)const{
 MALEMOD_SURFACE_TIME(true) if(!std::isfinite(margin)||margin<=0||!Finite(face[0])||!Finite(face[1])||!Finite(face[2]))throw std::invalid_argument("Invalid cached triangle contact neighborhood");
 SurfaceFaceQuery query(face);Hit hit;hit.distance=hit.signedDistance=margin;hit.triangle=unsigned(-1);if(Empty())return hit;
 const auto owner=reinterpret_cast<std::uintptr_t>(this);const auto stamp=MotionStamp();
 bool completeNeighborhood=neighborhood.bound_.Covers(face,margin,stamp,owner);
 if(!completeNeighborhood&&neighborhood.exclusionsComplete_&&neighborhood.owner_==owner&&neighborhood.topology_==stamp.topology){
  // Every source face is either retained or belongs to one of these excluded
  // bounds. Check their CURRENT boxes: a remote moving vertex need not force
  // reconstruction of unrelated neighborhoods. No omitted source face may
  // enter the current padded query, even across arbitrary surface motion.
  const double radius=margin*1.25;
  completeNeighborhood=std::isfinite(radius);
  for(unsigned id:neighborhood.exclusions_){
   if(!completeNeighborhood)break;
   if(id&0x80000000u){const auto& node=nodes_[id&0x7fffffffu];if(FaceBoxesOverlap(query,node.lo,node.hi,radius))completeNeighborhood=false;}
   else if(FaceBoxesOverlap(query,faceLo_[id],faceHi_[id],radius))completeNeighborhood=false;
  }
  if(completeNeighborhood)neighborhood.bound_.Remember(face,radius,stamp,owner);
 }
 if(!completeNeighborhood){
  double radius=margin*4;if(!std::isfinite(radius))throw std::invalid_argument("Triangle contact neighborhood radius overflow");
  neighborhood.candidates_.clear();neighborhood.rebuilt_++;
  // A wide cloth face may overlap the entire source mesh. Bound persistent
  // memory, but never treat a truncated neighborhood as complete contact data.
  bool complete=false;
  // Dense source regions need less travel padding, not truncated contacts.
  // Use the measured narrow padded set first. Larger sets add repeated exact
  // triangle work without improving coverage, and cannot rescue an overflowing
  // smaller set. Local exclusion witnesses recertify it after source motion.
  // Every retained set contains the entire requested physical contact margin.
  for(double factor:{1.25}){
   radius=margin*factor;neighborhood.candidates_.clear();neighborhood.exclusions_.clear();neighborhood.exclusionsComplete_=true;
   if(CollectFaces(0,query,radius,neighborhood.candidates_,&neighborhood.exclusions_,&neighborhood.exclusionsComplete_)){complete=true;break;}
  }
  if(!complete){
   neighborhood.candidates_.clear();neighborhood.exclusions_.clear();neighborhood.exclusionsComplete_=false;neighborhood.bound_={};neighborhood.fallback_++;
   if(seedTriangle<triangles_.size())ConsiderFace(seedTriangle,query,hit);
   SearchFace(0,query,margin,hit);return hit;
  }
  neighborhood.bound_.Remember(face,radius,stamp,owner);
  neighborhood.owner_=owner;neighborhood.topology_=stamp.topology;
 }else neighborhood.reused_++;
 if(seedTriangle<triangles_.size())ConsiderFace(seedTriangle,query,hit);
 for(auto id:neighborhood.candidates_)ConsiderFace(id,query,hit);return hit;
}
}

