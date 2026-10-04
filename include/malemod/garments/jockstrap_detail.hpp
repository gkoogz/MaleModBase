#pragma once
#include <limits>
namespace malemod::garments {
namespace detail {
constexpr double pi=3.14159265358979323846;
// Outward-only material tension envelope. Bounded angular slope prevents a
// tiny tangential displacement near the front pole from selecting a distant
// radial obstacle. Every authored anatomy support remains enclosed.
inline void TensionEnvelope(std::vector<double>& extent,unsigned rings,unsigned segments){
 double pole=0;for(unsigned x=0;x<segments;x++)pole=(std::max)(pole,extent[x]);for(unsigned x=0;x<segments;x++)extent[x]=pole;
 for(unsigned pass=0;pass<2*rings+segments;pass++){bool changed=false;pole=0;for(unsigned x=0;x<segments;x++)pole=(std::max)(pole,extent[x]);for(unsigned x=0;x<segments;x++)if(extent[x]<pole-1e-14){extent[x]=pole;changed=true;}for(unsigned at=0;at<extent.size();at++){unsigned y=at/segments,x=at%segments;double v=extent[at];v=(std::max)(v,extent[y*segments+(x+segments-1)%segments]-.7/segments);v=(std::max)(v,extent[y*segments+(x+1)%segments]-.7/segments);if(y)v=(std::max)(v,extent[at-segments]-.35/rings);if(y<rings)v=(std::max)(v,extent[at+segments]-.35/rings);if(v>extent[at]+1e-14){extent[at]=v;changed=true;}}if(!changed)break;}
 pole=0;for(unsigned x=0;x<segments;x++)pole=(std::max)(pole,extent[x]);for(unsigned x=0;x<segments;x++)extent[x]=pole;
}
inline double Smooth(double t){return t*t*(3-2*t);}
inline Lineage Blend(Lineage a,Lineage b,double t){
    if(!std::isfinite(t)||t<0||t>1)throw std::invalid_argument("Garment interpolation fraction outside [0,1]");std::vector<Donor> rows;
    for(unsigned family=0;family<2;family++)for(auto d:(family?b:a).donors){d.weight*=family?t:1-t;if(d.weight==0)continue;if(!std::isfinite(d.weight)||d.weight<0)throw std::invalid_argument("Invalid garment interpolation donor");auto it=std::find_if(rows.begin(),rows.end(),[&](auto r){return r.surface==d.surface&&r.vertex==d.vertex;});if(it==rows.end())rows.push_back(d);else it->weight+=d.weight;}
    Lineage out;if(rows.size()>out.donors.size())throw std::invalid_argument("Exact garment interpolation exceeds lineage capacity: "+std::to_string(rows.size()));double total=0;for(auto d:rows)total+=d.weight;if(std::abs(total-1)>1e-5)throw std::invalid_argument("Garment interpolation donors do not sum to one");for(unsigned i=0;i<rows.size();i++)out.donors[i]=rows[i];return out;
}
inline void ValidateSamples(const std::vector<Sample>& samples,std::size_t lo,std::size_t hi){if(samples.size()<lo||samples.size()>hi)throw std::invalid_argument("Garment sample count outside bounded contract");for(const auto& s:samples){if(!Finite(s.position)||!Finite(s.normal))throw std::invalid_argument("Nonfinite garment surface sample");double sum=0;for(const auto& d:s.lineage.donors){if(!std::isfinite(d.weight)||d.weight<0)throw std::invalid_argument("Invalid garment donor");sum+=d.weight;}if(std::abs(sum-1)>1e-5)throw std::invalid_argument("Garment donor weights must sum to one");}}
inline Sample RingSample(const std::vector<Sample>& ring,double t){double at=t*ring.size();auto i=std::size_t(std::floor(at))%ring.size();double u=at-std::floor(at);const auto& a=ring[i];const auto& b=ring[(i+1)%ring.size()];return {Add(Mul(a.position,1-u),Mul(b.position,u)),Add(Mul(a.normal,1-u),Mul(b.normal,u)),Blend(a.lineage,b.lineage,u)};}
inline std::uint32_t VertexAt(Mesh& m,Point p,double u,double v,Lineage l,Point contactNormal={}){m.vertices.push_back({p,contactNormal,{},{u,v},l});return std::uint32_t(m.vertices.size()-1);}
inline void Tri(Mesh& m,unsigned a,unsigned b,unsigned c,MaterialSlot mat,Point desired){if(Dot(Cross(Sub(m.vertices[b].position,m.vertices[a].position),Sub(m.vertices[c].position,m.vertices[a].position)),desired)<0)std::swap(b,c);m.triangles.push_back({{a,b,c},mat});}
inline void Quad(Mesh& m,unsigned a,unsigned b,unsigned c,unsigned d,MaterialSlot mat,Point desired){Tri(m,a,b,c,mat,desired);Tri(m,a,c,d,mat,desired);}
inline void Shading(Mesh& m){
    std::vector<Point> bitangent(m.vertices.size());for(auto& v:m.vertices){v.normal={};v.tangent={};}
    for(const auto& f:m.triangles){auto& a=m.vertices[f.vertices[0]];auto& b=m.vertices[f.vertices[1]];auto& c=m.vertices[f.vertices[2]];auto e=Sub(b.position,a.position),g=Sub(c.position,a.position),n=Cross(e,g);if(Length(n)<1e-16)continue;double du=b.uv[0]-a.uv[0],dv=b.uv[1]-a.uv[1],eu=c.uv[0]-a.uv[0],ev=c.uv[1]-a.uv[1],det=du*ev-dv*eu;Point tangent=std::abs(det)>1e-14?Mul(Sub(Mul(e,ev),Mul(g,dv)),1/det):e,bt=std::abs(det)>1e-14?Mul(Sub(Mul(g,du),Mul(e,eu)),1/det):g;
        for(auto id:f.vertices){m.vertices[id].normal=Add(m.vertices[id].normal,n);m.vertices[id].tangent=Add(m.vertices[id].tangent,tangent);bitangent[id]=Add(bitangent[id],bt);}}
    for(unsigned i=0;i<m.vertices.size();i++){auto& v=m.vertices[i];v.normal=Length(v.normal)>1e-14?Unit(v.normal):Point{0,0,1};auto t=Sub(v.tangent,Mul(v.normal,Dot(v.tangent,v.normal)));if(Length(t)<1e-12)t=Cross(v.normal,std::abs(v.normal[2])<.9?Point{0,0,1}:Point{0,1,0});v.tangent=Unit(t);v.tangentSign=Dot(Cross(v.normal,v.tangent),bitangent[i])<0?-1:1;}
}
inline Sample WaistAngle(const std::vector<Sample>& waist,Point center,const Frame& frame,double target){
    for(unsigned i=0;i<waist.size();i++){const auto& a=waist[i];const auto& b=waist[(i+1)%waist.size()];auto pa=Sub(frame.Local(a.position),center),pb=Sub(frame.Local(b.position),center);double aa=std::atan2(pa[0],pa[1]),ab=std::atan2(pb[0],pb[1]),edge=std::atan2(std::sin(ab-aa),std::cos(ab-aa)),delta=std::atan2(std::sin(target-aa),std::cos(target-aa));if(std::abs(edge)<1e-10)continue;double t=delta/edge;if(t>=0&&t<=1)return {Add(Mul(a.position,1-t),Mul(b.position,t)),Add(Mul(a.normal,1-t),Mul(b.normal,t)),Blend(a.lineage,b.lineage,t)};
    }throw std::invalid_argument("Measured waist does not enclose the garment frame");
}
inline double CapsuleDistance(Point p,const Capsule& c){auto a=Sub(c.b,c.a);double d=Dot(a,a),t=d>1e-20?std::clamp(Dot(Sub(p,c.a),a)/d,0.,1.):0;return Length(Sub(p,Add(c.a,Mul(a,t))))-c.radius;}
inline bool Project(Point& p,const Capsule& c,double gap,Point fallback){auto axis=Sub(c.b,c.a);double n=Dot(axis,axis),t=n>1e-20?std::clamp(Dot(Sub(p,c.a),axis)/n,0.,1.):0;auto center=Add(c.a,Mul(axis,t)),d=Sub(p,center);double distance=Length(d),r=c.radius+gap;if(distance>=r)return false;p=Add(center,Mul(distance>1e-14?Mul(d,1/distance):fallback,r));return true;}
inline Point ClosestTriangle(Point p,Point a,Point b,Point c){
 auto ab=Sub(b,a),ac=Sub(c,a),ap=Sub(p,a);double d1=Dot(ab,ap),d2=Dot(ac,ap);if(d1<=0&&d2<=0)return a;auto bp=Sub(p,b);double d3=Dot(ab,bp),d4=Dot(ac,bp);if(d3>=0&&d4<=d3)return b;double vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return Add(a,Mul(ab,d1/(d1-d3)));auto cp=Sub(p,c);double d5=Dot(ab,cp),d6=Dot(ac,cp);if(d6>=0&&d5<=d6)return c;double vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return Add(a,Mul(ac,d2/(d2-d6)));double va=d3*d6-d5*d4;if(va<=0&&d4-d3>=0&&d5-d6>=0)return Add(b,Mul(Sub(c,b),(d4-d3)/((d4-d3)+(d5-d6))));double total=va+vb+vc;if(std::abs(total)<1e-20)return a;return Add(a,Add(Mul(ab,vb/total),Mul(ac,vc/total)));
}
inline void ClosestSegments(Point a,Point b,Point c,Point d,Point& p,Point& q){auto u=Sub(b,a),v=Sub(d,c),r=Sub(a,c);double aa=Dot(u,u),ee=Dot(v,v),f=Dot(v,r),x=0,y=0;if(aa<=1e-20)y=ee>1e-20?std::clamp(f/ee,0.,1.):0;else{double cc=Dot(u,r);if(ee<=1e-20)x=std::clamp(-cc/aa,0.,1.);else{double bb=Dot(u,v),denom=aa*ee-bb*bb;if(denom>1e-20)x=std::clamp((bb*f-cc*ee)/denom,0.,1.);y=(bb*x+f)/ee;if(y<0){y=0;x=std::clamp(-cc/aa,0.,1.);}else if(y>1){y=1;x=std::clamp((bb-cc)/aa,0.,1.);}}}p=Add(a,Mul(u,x));q=Add(c,Mul(v,y));}
inline double ClosestTriangles(std::array<Point,3> a,std::array<Point,3> b,Point& pa,Point& pb){double best=1e100;auto candidate=[&](Point p,Point q){double d=Dot(Sub(p,q),Sub(p,q));if(d<best){best=d;pa=p;pb=q;}};for(auto p:a)candidate(p,ClosestTriangle(p,b[0],b[1],b[2]));for(auto q:b)candidate(ClosestTriangle(q,a[0],a[1],a[2]),q);for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++){Point p,q;ClosestSegments(a[i],a[(i+1)%3],b[j],b[(j+1)%3],p,q);candidate(p,q);}return std::sqrt(best);}
inline double TriangleCapsule(Point a,Point b,Point c,const Capsule& volume,Point& cloth,Point& axis,double margin=0){
 // Exact nearby tests are retained; disjoint expanded boxes only reject pairs.
 for(unsigned k=0;k<3;k++){double low=(std::min)({a[k],b[k],c[k]}),high=(std::max)({a[k],b[k],c[k]}),r=volume.radius+margin;if(high<(std::min)(volume.a[k],volume.b[k])-r||low>(std::max)(volume.a[k],volume.b[k])+r)return 1e100;}
 auto centroid=Mul(Add(Add(a,b),c),1./3);double bound=(std::max)({Length(Sub(a,centroid)),Length(Sub(b,centroid)),Length(Sub(c,centroid))});if(CapsuleDistance(centroid,volume)>bound+margin)return 1e100;
 double best=1e100;auto candidate=[&](Point p,Point q){double d=Dot(Sub(p,q),Sub(p,q));if(d<best){best=d;cloth=p;axis=q;}};candidate(ClosestTriangle(volume.a,a,b,c),volume.a);candidate(ClosestTriangle(volume.b,a,b,c),volume.b);for(auto edge:{std::array<Point,2>{a,b},std::array<Point,2>{b,c},std::array<Point,2>{c,a}}){Point p,q;ClosestSegments(edge[0],edge[1],volume.a,volume.b,p,q);candidate(p,q);}
 auto n=Cross(Sub(b,a),Sub(c,a)),direction=Sub(volume.b,volume.a);double denominator=Dot(n,direction);if(std::abs(denominator)>1e-20){double t=Dot(n,Sub(a,volume.a))/denominator;if(t>=0&&t<=1){auto p=Add(volume.a,Mul(direction,t)),q=ClosestTriangle(p,a,b,c);candidate(q,p);}}
 return std::sqrt(best)-volume.radius;
}
inline void Tube(Mesh& m,const std::vector<Sample>& path,double width,double thick,bool closed,const Frame& frame,MaterialSlot material){
    const unsigned count=unsigned(path.size()),base=unsigned(m.vertices.size());double length=0;Point previousTangent{},previousNormal{};
    for(unsigned i=0;i<count+(closed?1:0);i++){const unsigned k=i%count;if(i)length+=Length(Sub(path[k].position,path[(i-1)%count].position));auto tangent=Unit(Sub(path[(std::min)(count-1,k+1)].position,path[k?k-1:0].position));if(closed)tangent=Unit(Sub(path[(k+1)%count].position,path[(k+count-1)%count].position));auto normal=Sub(path[k].normal,Mul(tangent,Dot(path[k].normal,tangent)));if(Length(normal)<1e-8)normal=Cross(tangent,std::abs(Dot(tangent,frame.up))<.9?frame.up:frame.forward);normal=Unit(normal);
        if(i){auto axis=Cross(previousTangent,tangent);double cosine=std::clamp(Dot(previousTangent,tangent),-1.,1.);auto transported=previousNormal;if(cosine>-.999999)transported=Add(Add(transported,Cross(axis,transported)),Mul(Cross(axis,Cross(axis,transported)),1/(1+cosine)));transported=Sub(transported,Mul(tangent,Dot(transported,tangent)));if(Length(transported)>1e-10){transported=Unit(transported);if(Dot(normal,transported)<0)normal=Mul(normal,-1);normal=Unit(Add(Mul(transported,.85),Mul(normal,.15)));}}
        previousTangent=tangent;previousNormal=normal;auto side=Unit(Cross(tangent,normal));
        for(unsigned j=0;j<4;j++){double w=j==0||j==3?-width*.5:width*.5,h=j<2?thick*.5:-thick*.5;VertexAt(m,Add(path[k].position,Add(Mul(side,w),Mul(normal,h))),length/width,double(j),path[k].lineage,normal);}
    }
    unsigned segments=closed?count:count-1;
    for(unsigned i=0;i<segments;i++)for(unsigned j=0;j<4;j++){unsigned a=base+i*4+j,b=base+i*4+(j+1)%4,c=b+4,d=a+4;Point center=Mul(Add(path[i%count].position,path[(i+1)%count].position),.5);Quad(m,a,b,c,d,material,Sub(m.vertices[a].position,center));}
    if(!closed){Quad(m,base,base+1,base+2,base+3,material,Sub(path[0].position,path[1].position));auto b=base+(count-1)*4;Quad(m,b,b+1,b+2,b+3,material,Sub(path.back().position,path[count-2].position));}
}
}
#include "band_rest.hpp"
inline Session::Session(Parameters p):parameters_(p){if(!std::isfinite(p.sideCoverageExtra)||p.sideCoverageExtra<0||p.sideCoverageExtra>1.5)throw std::invalid_argument("Measured side coverage extra outside finite bounded contract");for(double v:{p.bandWidth,p.bandThickness,p.strapWidth,p.hemWidth,p.hemThickness,p.clearance})if(!std::isfinite(v)||v<=0||v>.2)throw std::invalid_argument("Garment dimensions must be finite measured fractions");if(!std::isfinite(p.panelHalfAngle)||p.panelHalfAngle<.3||p.panelHalfAngle>1.4||!std::isfinite(p.supportFraction)||p.supportFraction<0||p.supportFraction>.15||p.pouchRings<8||p.pouchRings>64||p.pouchSegments<16||p.pouchSegments>128||(p.simulate&&(p.pouchRings%2||p.pouchSegments%2)))throw std::invalid_argument("Garment strength/tessellation outside budget");for(double v:{p.mechanics.massFraction,p.mechanics.stretchCompliance,p.mechanics.bendCompliance,p.mechanics.dampingRate,p.mechanics.hemComplianceMultiplier})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid cloth mechanics");if(!std::isfinite(p.mechanics.extensionLimit)||p.mechanics.extensionLimit<1||p.mechanics.extensionLimit>1.15||!std::isfinite(p.mechanics.friction)||p.mechanics.friction<0||p.mechanics.friction>1)throw std::invalid_argument("Cloth stretch/friction outside material budget");output_.mesh.vertices.reserve(14000);output_.mesh.triangles.reserve(28000);localAnatomy_.reserve(32768);}
inline void Session::Reset(){output_=Output{};localAnatomy_.clear();bodyCollider_.Clear();anatomyCollider_.Clear();closedBody_.Clear();closedAnatomy_.Clear();rootCap_.clear();rootVertices_.clear();closedBodyFaces_.clear();closedAnatomyFaces_.clear();for(auto& memos:pointMemos_)memos.clear();for(auto& memos:faceMemos_)memos.clear();restMesh_={};restInput_={};vertexNodes_.clear();renderBindings_.clear();solverTriangles_.clear();inverseMass_.clear();previousBodies_.clear();previousBodySurface_.clear();previousAnatomySurface_.clear();restCircumference_=0;positions_.clear();velocities_.clear();previousTargets_.clear();anchors_.clear();edges_.clear();sewing_.clear();pinned_.clear();clock_=accumulator_=0;resetCount_++;}
inline const Output& Session::Fit(Style style,const Input& input){
    using namespace detail;
    if(!input.anatomyRegions[0].empty())return FitSheet(style,input);
    if(style!=Style::Naked&&style!=Style::WhiteJockstrap)throw std::invalid_argument("Unknown garment style");
    output_.style=style;output_.characterEpoch=input.characterEpoch;output_.topologyRevision=input.topologyRevision;output_.mesh.vertices.clear();output_.mesh.triangles.clear();output_.support.clear();output_.projectedContacts=0;output_.contactBudgetSatisfied=true;output_.coverageMargin=0;output_.band={};
    if(style==Style::Naked)return output_;
    const std::vector<Capsule> noVolumes;const auto& bodyVolumes=input.bodySurface.empty()?input.bodyContacts:noVolumes;
    input.frame.Validate();ValidateSamples(input.waist,8,128);ValidateSamples(input.opening,8,128);ValidateSamples(input.anatomy,4,131072);for(const auto& path:input.rearStraps)ValidateSamples(path,3,128);
    if(bodyVolumes.size()>64||!Finite(input.gravity)||!std::isfinite(input.deltaTime)||input.deltaTime<0||input.deltaTime>1)throw std::invalid_argument("Invalid garment contact/timing input");
    for(const auto& c:bodyVolumes)if(!Finite(c.a)||!Finite(c.b)||!std::isfinite(c.radius)||c.radius<0)throw std::invalid_argument("Invalid body volume");
    for(const auto& f:input.anatomyTriangles)for(auto i:f)if(i>=input.anatomy.size())throw std::invalid_argument("Anatomy triangle index outside surface");
    double circumference=0;for(std::size_t i=0;i<input.waist.size();i++)circumference+=Length(Sub(input.waist[i].position,input.waist[(i+1)%input.waist.size()].position));if(!std::isfinite(circumference)||circumference<1e-12)throw std::invalid_argument("Degenerate measured waist");output_.measuredCircumference=circumference;
    const double width=parameters_.bandWidth*circumference,thick=parameters_.bandThickness*circumference,gap=parameters_.clearance*circumference,hem=parameters_.hemWidth*circumference;
    auto& m=output_.mesh;const auto& frame=input.frame;
    auto bandRest=detail::BuildBand(input,parameters_,output_);auto& measuredBand=bandRest.measured;const unsigned waist=measuredBand.empty()?unsigned(input.waist.size()):unsigned(measuredBand.front().size());double waistHeight=bandRest.height,bandGap=bandRest.gap;
    auto bandAnchor=[&](Point p){auto local=frame.Local(p);local[2]=waistHeight;return frame.World(local);};
    const unsigned back=7*(waist+1);
    localAnatomy_.resize(input.anatomy.size());Point lo{1e100,1e100,1e100},hi{-1e100,-1e100,-1e100};for(std::size_t i=0;i<input.anatomy.size();i++){auto p=frame.Local(input.anatomy[i].position);localAnatomy_[i]=p;for(unsigned k=0;k<3;k++){lo[k]=(std::min)(lo[k],p[k]);hi[k]=(std::max)(hi[k],p[k]);}}
    double baseY=0;for(const auto& s:input.opening)baseY+=frame.Local(s.position)[1];baseY/=input.opening.size();baseY=(std::min)(baseY,lo[1]-gap);
    Point center{(lo[0]+hi[0])*.5,baseY,(lo[2]+hi[2])*.5};Point radius{(std::max)((hi[0]-lo[0])*.5+gap,gap*2),(std::max)(hi[1]-baseY+gap,gap*2),(std::max)((hi[2]-lo[2])*.5+gap,gap*2)};
    // Radial stretch cloth is fitted to the actual live surface, not its AABB
    // ellipsoid. Each measured vertex supports all corners of its angular cell;
    // this retains shaft, glans and individual lobe bulges with fixed topology.
    const unsigned rings=parameters_.pouchRings,segments=parameters_.pouchSegments;
    const unsigned gridCount=(rings+1)*segments;
    std::vector<double> extent(gridCount,0),next(gridCount,0);
    std::vector<int> donor(gridCount,-1),nextDonor(gridCount,-1);
    auto coordinates=[&](Point p){p=Sub(p,center);Point q{p[0]/radius[0],p[1]/radius[1],p[2]/radius[2]};double d=Length(q),angle=std::atan2(q[2],q[0]);if(angle<0)angle+=2*pi;return Point{std::acos(std::clamp(q[1]/(std::max)(d,1e-20),0.,1.))/(pi*.5)*rings,angle/(2*pi)*segments,d};};
    for(unsigned i=0;i<localAnatomy_.size();i++){
        auto c=coordinates(localAnatomy_[i]);unsigned row=(std::min)(rings-1,unsigned(c[0])),col=unsigned(c[1])%segments;
        for(unsigned y=row;y<=row+1;y++)for(unsigned x=0;x<2;x++){unsigned at=y*segments+(col+x)%segments;if(c[2]>extent[at]){extent[at]=c[2];donor[at]=int(i);}}
    }
    // Empty directional cells are suspended fabric between measured supports.
    // Bounded diffusion fills gaps only; occupied cells are never averaged away.
    for(unsigned pass=0;pass<rings+segments/2;pass++){
        next=extent;nextDonor=donor;bool changed=false;
        for(unsigned y=0;y<=rings;y++)for(unsigned x=0;x<segments;x++){unsigned at=y*segments+x;if(extent[at]>0)continue;double sum=0,best=0;unsigned count=0;int source=-1;
            for(auto n:{y*segments+(x+segments-1)%segments,y*segments+(x+1)%segments,(y?y-1:y)*segments+x,(std::min)(rings,y+1)*segments+x})if(extent[n]>0){sum+=extent[n];count++;if(extent[n]>best){best=extent[n];source=donor[n];}}
            if(count){next[at]=sum/count;nextDonor[at]=source;changed=true;}
        }extent.swap(next);donor.swap(nextDonor);if(!changed)break;
    }
    // Smooth only outwards: cloth bridges sharp valleys without penetrating any
    // measured support or erasing the exterior shape of the live anatomy.
    TensionEnvelope(extent,rings,segments);
    unsigned poleDonor=0;double poleExtent=0;for(unsigned x=0;x<segments;x++)if(extent[x]>poleExtent){poleExtent=extent[x];poleDonor=unsigned((std::max)(0,donor[x]));}for(unsigned x=0;x<segments;x++){extent[x]=poleExtent;donor[x]=int(poleDonor);}
    const double tessellation=1/std::cos(std::sqrt(std::pow(pi/(2*rings),2)+std::pow(2*pi/segments,2)));
    Point openingCenter{};for(auto sample:input.opening)openingCenter=Add(openingCenter,frame.Local(sample.position));openingCenter=Mul(openingCenter,1./input.opening.size());
    auto openingAt=[&](double theta){
      struct Knot{double angle;unsigned id;};std::vector<Knot> knots;for(unsigned k=0;k<input.opening.size();k++){auto p=Sub(frame.Local(input.opening[k].position),openingCenter);double a=std::atan2(p[2],p[0]);if(a<0)a+=2*pi;knots.push_back({a,k});}std::sort(knots.begin(),knots.end(),[](auto a,auto b){return a.angle<b.angle;});theta=std::fmod(theta+2*pi,2*pi);unsigned high=0;while(high<knots.size()&&knots[high].angle<theta)high++;unsigned lo=(high+unsigned(knots.size())-1)%unsigned(knots.size()),hi=high%unsigned(knots.size());double a=knots[lo].angle,b=knots[hi].angle;if(b<=a)b+=2*pi;if(theta<a)theta+=2*pi;double t=(theta-a)/(std::max)(b-a,1e-12);auto left=input.opening[knots[lo].id],right=input.opening[knots[hi].id];return Sample{Add(Mul(left.position,1-t),Mul(right.position,t)),Add(Mul(left.normal,1-t),Mul(right.normal,t)),Blend(left.lineage,right.lineage,t)};
    };
    auto radial=[&](unsigned y,unsigned x){double phi=pi*.5*y/rings,theta=2*pi*(x%segments)/segments;Point direction{radius[0]*std::sin(phi)*std::cos(theta),radius[1]*std::cos(phi),radius[2]*std::sin(phi)*std::sin(theta)};double e=extent[y*segments+x%segments];return Add(center,Mul(direction,e*tessellation+gap/(std::max)(Length(direction),gap)));};
    auto fitted=[&](Point p){auto c=coordinates(p);unsigned y=(std::min)(rings-1,unsigned(c[0])),x=unsigned(c[1])%segments;double fy=c[0]-y,fx=c[1]-std::floor(c[1]);return (1-fy)*((1-fx)*extent[y*segments+x]+fx*extent[y*segments+(x+1)%segments])+fy*((1-fx)*extent[(y+1)*segments+x]+fx*extent[(y+1)*segments+(x+1)%segments]);};
    output_.coverageMargin=1e100;for(auto p:localAnatomy_){auto c=coordinates(p);output_.coverageMargin=(std::min)(output_.coverageMargin,(fitted(p)*tessellation-c[2])*(std::min)({radius[0],radius[1],radius[2]})+gap);}
    const unsigned pouchBegin=unsigned(m.vertices.size());VertexAt(m,frame.World(radial(0,0)),.5,0,input.anatomy[poleDonor].lineage);
    for(unsigned j=1;j<=rings;j++)for(unsigned i=0;i<=segments;i++){auto d=donor[j*segments+i%segments];auto point=frame.World(radial(j,i));Lineage lineage=input.anatomy[unsigned((std::max)(0,d))].lineage;
      if(!input.anatomyTriangles.empty()){auto attachment=openingAt(2*pi*(i%segments)/segments);auto normal=attachment.normal;normal=Length(normal)>1e-12?Unit(normal):frame.forward;auto rim=Add(attachment.position,Mul(normal,gap+hem*.5+thick));double blend=std::clamp((double(j)/rings-.75)/.25,0.,1.);blend=blend*blend*(3-2*blend);point=Add(Mul(point,1-blend),Mul(rim,blend));if(j==rings)lineage=attachment.lineage;}
      VertexAt(m,point,double(i)/segments,double(j)/rings,lineage);}

    for(unsigned i=0;i<segments;i++)Tri(m,pouchBegin,pouchBegin+1+i,pouchBegin+2+i,MaterialSlot::WhiteRibbed,frame.forward);
    for(unsigned j=0;j+1<rings;j++)for(unsigned i=0;i<segments;i++){unsigned a=pouchBegin+1+j*(segments+1)+i;Quad(m,a,a+1,a+segments+2,a+segments+1,MaterialSlot::WhiteRibbed,Sub(m.vertices[a].position,frame.World(center)));}
    const unsigned pouchEnd=unsigned(m.vertices.size());
    std::vector<Sample> rim;rim.reserve(segments);for(unsigned i=0;i<segments;i++){auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+i];rim.push_back({v.position,frame.forward,v.lineage});}
    Tube(m,rim,hem,thick*.45,true,frame,MaterialSlot::WhiteElastic);
    // Join the measured body opening to the pouch. The rear remains open; this
    // seam and the two under-glute straps are deliberate jockstrap structure.
    const unsigned join=unsigned(m.vertices.size());for(unsigned row=0;row<2;row++)for(unsigned i=0;i<=segments;i++){auto s=openingAt(2*pi*(i%segments)/segments);const auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+i];VertexAt(m,row?v.position:s.position,double(i)/segments,double(row),row?v.lineage:s.lineage);}
    for(unsigned i=0;i<segments;i++)Quad(m,join+i,join+i+1,join+segments+2+i,join+segments+1+i,MaterialSlot::WhiteRibbed,frame.forward);
    // Front suspension panel sews the upper pouch arc into the measured front
    // waistband. Its shape has no invented bone names or fixed world height.
    Point waistCenter{};for(const auto& v:input.waist)waistCenter=Add(waistCenter,frame.Local(v.position));waistCenter=Mul(waistCenter,1./input.waist.size());
    const unsigned panelBase=unsigned(m.vertices.size()),panelColumns=24,panelRows=8;
    for(unsigned row=0;row<=panelRows;row++)for(unsigned col=0;col<=panelColumns;col++){
        double u=double(col)/panelColumns,t=double(row)/panelRows,theta=pi/3+u*pi/3;
        auto top=WaistAngle(input.waist,waistCenter,frame,(1-2*u)*parameters_.panelHalfAngle);
        auto bottom=RingSample(rim,theta/(2*pi));Point normal=Unit(Length(top.normal)>1e-12?top.normal:frame.forward);
        auto start=Add(bandAnchor(top.position),Add(Mul(normal,gap+thick),Mul(frame.up,-width*.42)));
        auto point=Add(Mul(start,1-t),Mul(bottom.position,t));VertexAt(m,point,u,t,Blend(top.lineage,bottom.lineage,t));
    }
    for(unsigned row=0;row<panelRows;row++)for(unsigned col=0;col<panelColumns;col++){auto a=panelBase+row*(panelColumns+1)+col;Quad(m,a,a+1,a+panelColumns+2,a+panelColumns+1,MaterialSlot::WhiteRibbed,frame.forward);}
    const unsigned strapBegin=unsigned(m.vertices.size());
    std::vector<std::array<unsigned,4>> strapSections;
    std::vector<unsigned> strapRouteStarts;
    for(const auto& measuredPath:input.rearStraps){auto path=measuredPath;const double side=frame.Local(path.back().position)[0];auto sewn=RingSample(rim,(side<0?4*pi/3:5*pi/3)/(2*pi));path.push_back(sewn);
      // A long sewn connector cannot follow a curved body volume as one flat
      // ribbon quad. Refine actual measured edges and retain both donor fields.
      std::vector<double> arc(path.size(),0);for(unsigned k=1;k<path.size();k++)arc[k]=arc[k-1]+Length(Sub(path[k].position,path[k-1].position));if(arc.back()<circumference*1e-10)throw std::invalid_argument("Degenerate measured strap route");
      // Uniform arc spacing prevents near-coincident measured donor samples
      // from becoming arbitrarily short material constraints after fitting.
      const unsigned count=(std::max)(33u,unsigned(path.size()-1)*4+1);std::vector<Sample> refined;refined.reserve(count);unsigned edge=0;
      for(unsigned k=0;k<count;k++){double at=arc.back()*k/(count-1);while(edge+2<path.size()&&arc[edge+1]<=at)++edge;double length=arc[edge+1]-arc[edge],t=length>1e-20?std::clamp((at-arc[edge])/length,0.,1.):0;const auto& a=path[edge];const auto& b=path[edge+1];refined.push_back({Add(Mul(a.position,1-t),Mul(b.position,t)),Add(Mul(a.normal,1-t),Mul(b.normal,t)),Blend(a.lineage,b.lineage,t)});}
      strapRouteStarts.push_back(unsigned(strapSections.size()));unsigned first=unsigned(m.vertices.size());Tube(m,refined,parameters_.strapWidth*circumference,thick*.5,false,frame,MaterialSlot::WhiteElastic);for(unsigned i=first;i<m.vertices.size();i+=4)strapSections.push_back({i,i+1,i+2,i+3});}
    // Body volume contacts are bounded and resolved without deleting triangles.
    // Reproject pouch rows onto their coverage envelope so a thigh projection
    // cannot push cloth through the anatomical volume.
    auto reconcileContactSeams=[&](){
      auto weld=[&](unsigned a,unsigned b){auto p=Mul(Add(m.vertices[a].position,m.vertices[b].position),.5);m.vertices[a].position=m.vertices[b].position=p;};
      for(unsigned layer=0;layer<2;layer++)for(unsigned row=0;row<7;row++){unsigned a=layer*7*(waist+1)+row*(waist+1);weld(a,a+waist);}
      for(unsigned row=0;row<rings;row++){unsigned a=pouchBegin+1+row*(segments+1);weld(a,a+segments);}
      for(unsigned row=0;row<2;row++){unsigned a=join+row*(segments+1);weld(a,a+segments);}
      // These are physical stitches, not merely coincident initial authoring
      // points. Contact fitting must publish one actual material boundary.
      for(unsigned k=0;k<=segments;k++){Point center{};for(unsigned j=0;j<4;j++)center=Add(center,m.vertices[pouchEnd+4*k+j].position);center=Mul(center,.25);unsigned rim=pouchBegin+1+(rings-1)*(segments+1)+k;m.vertices[rim].position=center;m.vertices[join+segments+1+k].position=center;}
      std::vector<Sample> stitchedRim;for(unsigned k=0;k<segments;k++){const auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+k];stitchedRim.push_back({v.position,frame.forward,v.lineage});}
      for(unsigned col=0;col<=panelColumns;col++)m.vertices[panelBase+panelRows*(panelColumns+1)+col].position=RingSample(stitchedRim,(1./6+double(col)/panelColumns/6)).position;
    };
    // Contact capsules describe one union. Projecting against them separately
    // can oscillate between overlapping thigh/pelvis volumes. Move along the
    // measured garment/body outward direction to the union's outer boundary.
    auto nearestAxis=[&](Point p,const Capsule& c){auto axis=Sub(c.b,c.a);double square=Dot(axis,axis),t=square>1e-20?std::clamp(Dot(Sub(p,c.a),axis)/square,0.,1.):0;return Add(c.a,Mul(axis,t));};
    auto directions=[&](Point point){std::vector<Point> candidates;for(auto c:bodyVolumes){auto radial=Sub(point,nearestAxis(point,c));if(Length(radial)>1e-12)candidates.push_back(Unit(radial));}for(auto a:{frame.lateral,frame.forward,frame.up}){candidates.push_back(a);candidates.push_back(Mul(a,-1));}return candidates;};
    auto escapePoint=[&](Point& point,bool front){bool touching=false;for(auto c:bodyVolumes)touching|=CapsuleDistance(point,c)<gap*.3;if(!touching)return;Point best{};double minimum=1e100;
      for(auto preferred:front?std::vector<Point>{frame.forward}:directions(point)){double high=gap;auto clear=[&](double distance){auto p=Add(point,Mul(preferred,distance));for(auto c:bodyVolumes)if(CapsuleDistance(p,c)<gap*.5)return false;return true;};
        for(unsigned k=0;k<16&&!clear(high)&&high<minimum;k++)high=(std::min)(high*2,minimum);if(!clear(high))continue;double low=0;for(unsigned k=0;k<20;k++){double mid=(low+high)*.5;if(clear(mid))high=mid;else low=mid;}if(high<minimum){minimum=high;best=preferred;}}
      if(minimum<1e100){point=Add(point,Mul(best,minimum));output_.projectedContacts++;}
    };
    std::vector<Point> rest;rest.reserve(m.vertices.size());for(auto v:m.vertices)rest.push_back(v.position);
    std::vector<unsigned> sectionOf(m.vertices.size(),unsigned(-1));for(unsigned i=0;i<strapSections.size();i++)for(auto id:strapSections[i])sectionOf[id]=i;
    std::vector<std::vector<unsigned>> bandSections;for(unsigned col=0;col<=waist;col++){std::vector<unsigned> ids;for(unsigned layer=0;layer<2;layer++)for(unsigned row=0;row<7;row++)ids.push_back(layer*7*(waist+1)+row*(waist+1)+col);bandSections.push_back(ids);for(auto id:ids)sectionOf[id]=unsigned(strapSections.size())+col;}
    std::vector<double> strapSides(strapSections.size()),strapConfidence(strapSections.size());for(unsigned route=0;route<strapRouteStarts.size();route++){unsigned end=route+1<strapRouteStarts.size()?strapRouteStarts[route+1]:unsigned(strapSections.size());double side=frame.Local(input.rearStraps[route].front().position)[0]<waistCenter[0]?-1:1;double confidence=std::abs(frame.Local(input.rearStraps[route].front().position)[0]-waistCenter[0])/circumference;for(unsigned k=strapRouteStarts[route];k<end;k++){strapSides[k]=side;strapConfidence[k]=confidence;}}
    auto strapDirections=[&](Point p,double side,double confidence){if(confidence<parameters_.strapWidth)return std::vector<Point>{Mul(frame.lateral,side)};std::vector<Point> choices;for(auto volume:bodyVolumes){auto v=Sub(p,nearestAxis(p,volume));v=Sub(v,Mul(frame.up,Dot(v,frame.up)));if(Length(v)>1e-12){v=Unit(v);if(Dot(v,frame.lateral)*side>=-1e-8)choices.push_back(v);}}choices.push_back(Mul(frame.lateral,side));choices.push_back(frame.forward);choices.push_back(Mul(frame.forward,-1));return choices;};
    std::vector<std::array<unsigned,4>> hemSections;for(unsigned k=pouchEnd;k<join;k+=4){hemSections.push_back({k,k+1,k+2,k+3});for(unsigned j=0;j<4;j++)sectionOf[k+j]=unsigned(strapSections.size()+bandSections.size()+hemSections.size()-1);}
    auto rigidSections=[&](){for(auto ids:strapSections){Point delta{};for(auto id:ids)delta=Add(delta,Sub(m.vertices[id].position,rest[id]));delta=Mul(delta,.25);for(auto id:ids)m.vertices[id].position=Add(rest[id],delta);}for(auto ids:hemSections){Point delta{};for(auto id:ids)delta=Add(delta,Sub(m.vertices[id].position,rest[id]));delta=Mul(delta,.25);for(auto id:ids)m.vertices[id].position=Add(rest[id],delta);}};
    auto escapeSection=[&](const auto& ids,const std::vector<Point>& preferredDirections){Point center{};bool touching=false;for(auto id:ids){center=Add(center,m.vertices[id].position);for(auto volume:bodyVolumes)touching|=CapsuleDistance(m.vertices[id].position,volume)<gap*.3;}if(!touching)return;center=Mul(center,1./ids.size());Point best{};double minimum=1e100;for(auto preferred:preferredDirections){auto clear=[&](double distance){for(auto id:ids)for(auto volume:bodyVolumes)if(CapsuleDistance(Add(m.vertices[id].position,Mul(preferred,distance)),volume)<gap*.5)return false;return true;};double high=gap;for(unsigned k=0;k<16&&!clear(high)&&high<minimum;k++)high=(std::min)(high*2,minimum);if(!clear(high))continue;double low=0;for(unsigned k=0;k<20;k++){double middle=(low+high)*.5;if(clear(middle))high=middle;else low=middle;}if(high<minimum){minimum=high;best=preferred;}}if(minimum<1e100){for(auto id:ids)m.vertices[id].position=Add(m.vertices[id].position,Mul(best,minimum));output_.projectedContacts++;}};
    auto escapeTriangle=[&](const Triangle& triangle){
      std::array<Point,3> points;Point center{};bool front=true;
      for(unsigned k=0;k<3;k++){unsigned id=triangle.vertices[k];points[k]=m.vertices[id].position;center=Add(center,points[k]);front&=id>=pouchBegin&&id<strapBegin;}center=Mul(center,1./3);
      auto clear=[&](Point direction,double distance){auto d=Mul(direction,distance);auto moved=[&](unsigned k){auto id=triangle.vertices[k];return id>=panelBase&&id<=panelBase+panelColumns?points[k]:Add(points[k],d);};for(auto volume:bodyVolumes){Point p,q;if(TriangleCapsule(moved(0),moved(1),moved(2),volume,p,q,gap*.5)<gap*.5)return false;}return true;};
      if(clear(frame.forward,0))return;Point best{};double minimum=1e100;
      bool band=triangle.vertices[0]<pouchBegin;auto radial=Sub(center,frame.World(waistCenter));radial=Sub(radial,Mul(frame.up,Dot(radial,frame.up)));double side=0,confidence=0;bool sewn=false;for(auto id:triangle.vertices)if(id>=strapBegin){auto section=sectionOf[id];side=strapSides[section];confidence=strapConfidence[section];sewn|=section+1==strapSections.size()||std::find(strapRouteStarts.begin(),strapRouteStarts.end(),section+1)!=strapRouteStarts.end();}auto choices=front?std::vector<Point>{frame.forward}:band?std::vector<Point>{Unit(radial)}:sewn?std::vector<Point>{frame.forward}:strapDirections(center,side,confidence);for(auto preferred:choices){double high=gap;for(unsigned k=0;k<16&&!clear(preferred,high)&&high<minimum;k++)high=(std::min)(high*2,minimum);if(!clear(preferred,high))continue;double low=0;for(unsigned k=0;k<20;k++){double mid=(low+high)*.5;if(clear(preferred,mid))high=mid;else low=mid;}if(high<minimum){minimum=high;best=preferred;}}
      if(minimum>=1e100)return;auto delta=Mul(best,minimum);std::array<unsigned,3> groups{};unsigned count=0;
      for(auto id:triangle.vertices){if(id>=panelBase&&id<=panelBase+panelColumns)continue;unsigned group=sectionOf[id]==unsigned(-1)?id:unsigned(m.vertices.size())+sectionOf[id];bool found=false;for(unsigned k=0;k<count;k++)found|=groups[k]==group;if(!found)groups[count++]=group;}
      for(unsigned k=0;k<count;k++){if(groups[k]<m.vertices.size())m.vertices[groups[k]].position=Add(m.vertices[groups[k]].position,delta);else {unsigned group=groups[k]-unsigned(m.vertices.size());if(group<strapSections.size()){for(auto id:strapSections[group])m.vertices[id].position=Add(m.vertices[id].position,delta);}else if(group<strapSections.size()+bandSections.size()){for(auto id:bandSections[group-unsigned(strapSections.size())])m.vertices[id].position=Add(m.vertices[id].position,delta);}else for(auto id:hemSections[group-unsigned(strapSections.size()+bandSections.size())])m.vertices[id].position=Add(m.vertices[id].position,delta);}}output_.projectedContacts++;
    };
    auto relaxStraps=[&](){for(unsigned i=1;i<strapSections.size();i++){auto previous=strapSections[i-1],next=strapSections[i];if(std::find(strapRouteStarts.begin(),strapRouteStarts.end(),i)!=strapRouteStarts.end())continue; // Separate measured routes have distinct end caps.
      Point a{},b{},ar{},br{};for(unsigned k=0;k<4;k++){a=Add(a,m.vertices[previous[k]].position);b=Add(b,m.vertices[next[k]].position);ar=Add(ar,rest[previous[k]]);br=Add(br,rest[next[k]]);}a=Mul(a,.25);b=Mul(b,.25);ar=Mul(ar,.25);br=Mul(br,.25);auto d=Sub(b,a);double length=Length(d),original=Length(Sub(br,ar));if(length<=original*1.4||length<1e-12)continue;auto correction=Mul(d,(length-original*1.4)/length*.45);for(auto id:previous)m.vertices[id].position=Add(m.vertices[id].position,correction);for(auto id:next)m.vertices[id].position=Sub(m.vertices[id].position,correction);
    }};
    auto sewHem=[&](){std::vector<Sample> contour;for(unsigned k=0;k<segments;k++){const auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+k];contour.push_back({v.position,frame.forward,v.lineage});}Mesh trim;Tube(trim,contour,hem,thick*.45,true,frame,MaterialSlot::WhiteElastic);for(unsigned k=0;k<trim.vertices.size();k++){m.vertices[pouchEnd+k]=trim.vertices[k];rest[pouchEnd+k]=trim.vertices[k].position;}unsigned first=28*waist+segments+(rings-1)*segments*2;for(unsigned k=0;k<trim.triangles.size();k++){auto triangle=trim.triangles[k];for(auto& id:triangle.vertices)id+=pouchEnd;m.triangles[first+k]=triangle;}};
    auto sewPanel=[&](){std::vector<Sample> contour;for(unsigned k=0;k<waist;k++){double t=(.5-.42)/(.5-.10);auto point=Add(Mul(m.vertices[k].position,1-t),Mul(m.vertices[waist+1+k].position,t));contour.push_back({point,measuredBand.empty()?input.waist[k].normal:measuredBand[0][k].normal,Blend(m.vertices[k].lineage,m.vertices[waist+1+k].lineage,t)});}for(unsigned col=0;col<=panelColumns;col++){double u=double(col)/panelColumns;auto sample=WaistAngle(contour,waistCenter,frame,(1-2*u)*parameters_.panelHalfAngle);Point sewn=sample.position;double distance=1e100;for(unsigned id=0;id<28*waist;id++){auto face=m.triangles[id];auto point=ClosestTriangle(sample.position,m.vertices[face.vertices[0]].position,m.vertices[face.vertices[1]].position,m.vertices[face.vertices[2]].position);double d=Dot(Sub(point,sample.position),Sub(point,sample.position));if(d<distance){distance=d;sewn=point;}}m.vertices[panelBase+col].position=sewn;m.vertices[panelBase+col].lineage=sample.lineage;}};
    auto solveContacts=[&](unsigned budget,bool relax){for(unsigned iteration=0;!bodyVolumes.empty()&&iteration<budget;iteration++){auto before=output_.projectedContacts;
      if(relax)relaxStraps();rigidSections();for(unsigned k=0;k<strapSections.size();k++){auto section=strapSections[k];Point anchor{};for(auto id:section)anchor=Add(anchor,rest[id]);bool sewn=k+1==strapSections.size()||std::find(strapRouteStarts.begin(),strapRouteStarts.end(),k+1)!=strapRouteStarts.end();escapeSection(section,sewn?std::vector<Point>{frame.forward}:strapDirections(Mul(anchor,.25),strapSides[k],strapConfidence[k]));}for(unsigned col=0;col<bandSections.size();col++){auto radial=input.waist[col%waist].normal;radial=Unit(Sub(radial,Mul(frame.up,Dot(radial,frame.up))));escapeSection(bandSections[col],std::vector<Point>{radial});}
      for(auto section:hemSections)escapeSection(section,std::vector<Point>{frame.forward});
      for(unsigned i=0;i<m.vertices.size();i++){if(i>=strapBegin||i<pouchBegin||(i>=pouchEnd&&i<join))continue;auto& v=m.vertices[i];escapePoint(v.position,i>=pouchBegin&&i<strapBegin);if(i>=pouchBegin&&i<pouchEnd){auto p=Sub(frame.Local(v.position),center);auto c=coordinates(Add(p,center));double r=Length(p),minimum=fitted(Add(p,center))*tessellation+gap/(std::max)(r,gap);if(c[2]<minimum&&c[2]>1e-12)v.position=frame.World(Add(center,Mul(p,minimum/c[2])));}}
      for(const auto& triangle:m.triangles)escapeTriangle(triangle);
      reconcileContactSeams();if(output_.projectedContacts==before)break;
    }
    };
    bool separated=true;for(auto confidence:strapConfidence)separated&=confidence>=parameters_.strapWidth;solveContacts(separated?8:4,separated);
    // Route fitting precedes material tessellation. A capsule union may require
    // a longer path around a thigh; distributing that path by arclength avoids
    // concentrating the whole recruitment into one distorted ribbon quad.
    if(!bodyVolumes.empty()){
      for(unsigned materialPass=0;materialPass<(separated?2u:3u);materialPass++){sewHem();for(unsigned route=0;route<strapRouteStarts.size();route++){
        unsigned first=strapRouteStarts[route],end=route+1<strapRouteStarts.size()?strapRouteStarts[route+1]:unsigned(strapSections.size());
        std::vector<Sample> path;std::vector<double> arc;for(unsigned k=first;k<end;k++){Point point{};for(auto id:strapSections[k])point=Add(point,m.vertices[id].position);auto id=strapSections[k][0];path.push_back({Mul(point,.25),m.vertices[id].normal,m.vertices[id].lineage});arc.push_back(path.size()==1?0:arc.back()+Length(Sub(path.back().position,path[path.size()-2].position)));}
        // Sew the rebuilt route to the already contact-fitted band and pouch,
        // rather than preserving an obsolete pre-projection attachment point.
        if(materialPass==0){unsigned nearest=0;double minimum=1e100;for(unsigned col=0;col<waist;col++){double d=Length(Sub(m.vertices[col].position,input.rearStraps[route].front().position));if(d<minimum){minimum=d;nearest=col;}}path.front().position=m.vertices[nearest].position;path.front().lineage=m.vertices[nearest].lineage;
        std::vector<Sample> liveRim;for(unsigned col=0;col<segments;col++){const auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+col];liveRim.push_back({v.position,frame.forward,v.lineage});}auto attachment=RingSample(liveRim,(strapSides[first]<0?4*pi/3:5*pi/3)/(2*pi));path.back()=attachment;}
        arc[0]=0;for(unsigned k=1;k<path.size();k++)arc[k]=arc[k-1]+Length(Sub(path[k].position,path[k-1].position));
        std::vector<Sample> materialPath;unsigned edge=0;for(unsigned k=0;k<path.size();k++){double at=arc.back()*k/(path.size()-1);while(edge+2<path.size()&&arc[edge+1]<=at)++edge;double length=arc[edge+1]-arc[edge],t=length>1e-20?std::clamp((at-arc[edge])/length,0.,1.):0;const auto& a=path[edge];const auto& b=path[edge+1];materialPath.push_back({Add(Mul(a.position,1-t),Mul(b.position,t)),Add(Mul(a.normal,1-t),Mul(b.normal,t)),Blend(a.lineage,b.lineage,t)});}
        Mesh strip;Tube(strip,materialPath,parameters_.strapWidth*circumference,thick*.5,false,frame,MaterialSlot::WhiteElastic);unsigned base=strapSections[first][0];for(unsigned k=0;k<strip.vertices.size();k++){m.vertices[base+k]=strip.vertices[k];rest[base+k]=strip.vertices[k].position;}
      }
      sewPanel();solveContacts(4,false);}
      sewPanel();reconcileContactSeams();
    }
    // Sew actual material surfaces, not nominal centerline points. Translate
    // one intact end section by the shortest feasible cloth-to-cloth stitch;
    // reject candidates that would penetrate any calibrated body volume.
    for(unsigned route=0;route<strapRouteStarts.size();route++)for(unsigned endIndex=0;endIndex<2;endIndex++){
      unsigned first=strapRouteStarts[route],end=route+1<strapRouteStarts.size()?strapRouteStarts[route+1]:unsigned(strapSections.size());auto ids=strapSections[endIndex?end-1:first];
      std::vector<unsigned> adjacent;for(unsigned k=0;k<m.triangles.size();k++){bool touches=false;for(auto id:m.triangles[k].vertices)for(auto endpoint:ids)touches|=id==endpoint;if(touches)adjacent.push_back(k);}
      auto proposed=[&](unsigned id,Point delta){for(auto endpoint:ids)if(id==endpoint)return Add(m.vertices[id].position,delta);return m.vertices[id].position;};
      auto feasible=[&](Point delta){for(auto k:adjacent){const auto& face=m.triangles[k];for(auto volume:bodyVolumes){Point p,q;if(TriangleCapsule(proposed(face.vertices[0],delta),proposed(face.vertices[1],delta),proposed(face.vertices[2],delta),volume,p,q,gap*.3)<gap*.3-1e-9*circumference)return false;}}return true;};
      unsigned targetFirst=endIndex?28*waist+segments+(rings-1)*segments*2:0,targetEnd=endIndex?targetFirst+segments*8:28*waist;Point best{};double distance=circumference*.04;
      for(unsigned k=targetFirst;k<targetEnd;k++){const auto& face=m.triangles[k];std::array<Point,3> target{m.vertices[face.vertices[0]].position,m.vertices[face.vertices[1]].position,m.vertices[face.vertices[2]].position};for(auto corners:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{0,2,3}}){std::array<Point,3> section{m.vertices[ids[corners[0]]].position,m.vertices[ids[corners[1]]].position,m.vertices[ids[corners[2]]].position};Point a,b;double gapToCloth=ClosestTriangles(section,target,a,b);if(gapToCloth>=distance-circumference*1e-10)continue;auto delta=Sub(b,a);if(feasible(delta)){distance=gapToCloth;best=delta;}}}
      // A moving hem can require the sewn ribbon end to turn. Translation of
      // a fixed material frame may place its far corner inside the body even
      // though the nearest stitch is only millimetres away. Fit one intact end
      // cross section to the actual hem tangent and body radial frame, retaining
      // axis signs and choosing the least-displacing feasible configuration.
      // This is conditional material fitting, never a relaxed collision margin.
      if(distance>=circumference*.04&&endIndex){
        std::array<Point,4> original,chosen{};Point endpointCenter{};
        for(unsigned j=0;j<4;j++){original[j]=m.vertices[ids[j]].position;endpointCenter=Add(endpointCenter,original[j]);}
        endpointCenter=Mul(endpointCenter,.25);
        double materialWidth=parameters_.strapWidth*circumference,cost=4*materialWidth*materialWidth;
        bool found=false;
        auto hemCenter=[&](unsigned section){Point p{};for(unsigned j=0;j<4;j++)p=Add(p,m.vertices[pouchEnd+(section%segments)*4+j].position);return Mul(p,.25);};
        for(unsigned k=targetFirst;k<targetEnd;k++){
          const auto& face=m.triangles[k];Point targetCenter{};
          for(auto id:face.vertices)targetCenter=Add(targetCenter,m.vertices[id].position);
          targetCenter=Mul(targetCenter,1./3);
          if(Length(Sub(targetCenter,endpointCenter))>2*materialWidth)continue;
          unsigned section=(face.vertices[0]-pouchEnd)/4;
          auto widthDirection=Sub(hemCenter(section+1),hemCenter(section+segments-1));
          if(Length(widthDirection)<circumference*1e-10)continue;
          widthDirection=Unit(widthDirection);
          if(Dot(widthDirection,Sub(original[1],original[0]))<0)widthDirection=Mul(widthDirection,-1);
          std::vector<Point> normals;
          for(auto volume:bodyVolumes){auto n=Sub(targetCenter,nearestAxis(targetCenter,volume));if(Length(n)>1e-12)normals.push_back(Unit(n));}
          normals.push_back(frame.forward);
          for(auto normal:normals){
            normal=Sub(normal,Mul(widthDirection,Dot(normal,widthDirection)));
            if(Length(normal)<1e-12)continue;
            normal=Unit(normal);
            if(Dot(normal,Sub(original[0],original[3]))<0)normal=Mul(normal,-1);
            for(unsigned j=0;j<4;j++){double w=(j==0||j==3?-1:1)*materialWidth*.5,h=(j<2?1:-1)*thick*.25;m.vertices[ids[j]].position=Add(endpointCenter,Add(Mul(widthDirection,w),Mul(normal,h)));}
            std::array<Point,3> target{m.vertices[face.vertices[0]].position,m.vertices[face.vertices[1]].position,m.vertices[face.vertices[2]].position};
            for(auto corners:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{0,2,3}}){
              std::array<Point,3> ribbon{m.vertices[ids[corners[0]]].position,m.vertices[ids[corners[1]]].position,m.vertices[ids[corners[2]]].position};Point a,b;
              double d=ClosestTriangles(ribbon,target,a,b);auto delta=Sub(b,a);
              if(d>=circumference*.04||!feasible(delta))continue;
              double score=0;for(unsigned j=0;j<4;j++){auto change=Sub(Add(m.vertices[ids[j]].position,delta),original[j]);score+=Dot(change,change);}
              if(score>=cost-circumference*circumference*1e-12)continue;
              found=true;cost=score;for(unsigned j=0;j<4;j++)chosen[j]=Add(m.vertices[ids[j]].position,delta);
            }
          }
        }
        for(unsigned j=0;j<4;j++)m.vertices[ids[j]].position=found?chosen[j]:original[j];
        if(found){distance=0;best={};}
      }
      if(distance<circumference*.04){for(auto id:ids)m.vertices[id].position=Add(m.vertices[id].position,best);}else output_.contactBudgetSatisfied=false;
    }
    // Keep knit texel density tied to actual published material length.
    for(unsigned route=0;route<strapRouteStarts.size();route++){unsigned first=strapRouteStarts[route],end=route+1<strapRouteStarts.size()?strapRouteStarts[route+1]:unsigned(strapSections.size());double length=0;Point previous{};for(unsigned k=first;k<end;k++){Point point{};for(auto id:strapSections[k])point=Add(point,m.vertices[id].position);point=Mul(point,.25);if(k>first)length+=Length(Sub(point,previous));previous=point;for(auto id:strapSections[k])m.vertices[id].uv[0]=length/(parameters_.strapWidth*circumference);}}
    sewPanel();
    // Measured skin replaces anatomy-solver capsules for garment contact.
    // Move intact material sections, preserving band/ribbon/hem cross sections.
    bodyCollider_.Update(input.bodySurface,input.bodyTriangles,frame.origin,circumference);
    if(!input.anatomyTriangles.empty())anatomyCollider_.Update(input.anatomy,input.anatomyTriangles,frame.origin,circumference);else anatomyCollider_.Clear();
    ClassifySurfaces(input,frame.origin,circumference,true);
    if(!bodyCollider_.Empty()||!anatomyCollider_.Empty()){
      auto normalized=[&](Point point){return Mul(Sub(point,frame.origin),1/circumference);};
      auto moveSection=[&](unsigned group,Point delta){if(group<strapSections.size())for(auto v:strapSections[group])m.vertices[v].position=Add(m.vertices[v].position,delta);else if(group<strapSections.size()+bandSections.size())for(auto v:bandSections[group-strapSections.size()])m.vertices[v].position=Add(m.vertices[v].position,delta);else for(auto v:hemSections[group-strapSections.size()-bandSections.size()])m.vertices[v].position=Add(m.vertices[v].position,delta);};
      auto moveGroup=[&](unsigned id,Point delta){
        unsigned rimFirst=pouchBegin+1+(rings-1)*(segments+1);
        if(id>=join+segments+1&&id<join+2*(segments+1))id=rimFirst+id-(join+segments+1);
        if(id>=panelBase+panelRows*(panelColumns+1)&&id<panelBase+(panelRows+1)*(panelColumns+1)){double at=(1./6+double(id-panelBase-panelRows*(panelColumns+1))/panelColumns/6)*segments;unsigned col=unsigned(at)%segments;moveSection(unsigned(strapSections.size()+bandSections.size())+col,delta);moveSection(unsigned(strapSections.size()+bandSections.size())+(col+1)%segments,delta);return;}
        if(id>=rimFirst&&id<=rimFirst+segments){moveSection(unsigned(strapSections.size()+bandSections.size())+(id-rimFirst)%segments,delta);return;}
        unsigned group=sectionOf[id];if(group==unsigned(-1)){m.vertices[id].position=Add(m.vertices[id].position,delta);return;}if(group<strapSections.size())for(auto v:strapSections[group])m.vertices[v].position=Add(m.vertices[v].position,delta);else if(group<strapSections.size()+bandSections.size())for(auto v:bandSections[group-strapSections.size()])m.vertices[v].position=Add(m.vertices[v].position,delta);else for(auto v:hemSections[group-strapSections.size()-bandSections.size()])m.vertices[v].position=Add(m.vertices[v].position,delta);};
      for(unsigned pass=0;pass<32;pass++){
        unsigned before=output_.projectedContacts;
        for(auto* collider:{&bodyCollider_,&anatomyCollider_})if(!collider->Empty()){bool anatomical=collider==&anatomyCollider_;
        for(unsigned id=0;id<m.vertices.size();id++){auto hit=anatomical?collider->Near(normalized(m.vertices[id].position),parameters_.clearance*.3):collider->Closest(normalized(m.vertices[id].position));hit.signedDistance=ClassifiedDistance(hit,normalized(m.vertices[id].position),anatomical);if((!anatomical||hit.distance<parameters_.clearance*.3)&&hit.signedDistance<parameters_.clearance*.3){moveGroup(id,Mul(hit.normal,(parameters_.clearance*.3-hit.signedDistance+1e-5)*circumference));output_.projectedContacts++;}}
        for(const auto& face:m.triangles){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalized(m.vertices[face.vertices[k]].position);auto hit=collider->ClosestFace(points,parameters_.clearance*.3);if(hit.signedDistance>=parameters_.clearance*.3)continue;auto delta=Mul(hit.normal,(parameters_.clearance*.3-hit.signedDistance+1e-5)*circumference);std::set<unsigned> groups;for(auto id:face.vertices){unsigned group=sectionOf[id];if(group==unsigned(-1)||groups.insert(group).second)moveGroup(id,delta);}output_.projectedContacts++;}
        }sewPanel();reconcileContactSeams();if(output_.projectedContacts==before)break;
      }
      for(const auto& vertex:m.vertices)if(!bodyCollider_.Empty()&&ClassifiedDistance(bodyCollider_.Closest(normalized(vertex.position)),normalized(vertex.position),false)<parameters_.clearance*.3-1e-8)output_.contactBudgetSatisfied=false;
      for(const auto& face:m.triangles){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalized(m.vertices[face.vertices[k]].position);for(auto* collider:{&bodyCollider_,&anatomyCollider_})if(!collider->Empty()&&collider->ClosestFace(points,parameters_.clearance*.3).distance<parameters_.clearance*.3-1e-8)output_.contactBudgetSatisfied=false;}
      if(!anatomyCollider_.Empty()){output_.coverageMargin=1e100;for(const auto& face:m.triangles){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalized(m.vertices[face.vertices[k]].position);auto hit=anatomyCollider_.ClosestFace(points,parameters_.clearance*.3);if(hit.distance<parameters_.clearance*.3)output_.coverageMargin=(std::min)(output_.coverageMargin,(hit.distance-parameters_.clearance*.3)*circumference);}if(output_.coverageMargin==1e100)output_.coverageMargin=parameters_.clearance*.3*circumference;}

    }
    // Explicit residual check exposes an unsatisfied contact budget rather than
    // silently hiding clipping. Adapters must surface this status in validation.
    for(const auto& v:m.vertices)for(const auto& c:bodyVolumes)if(CapsuleDistance(v.position,c)<gap*.3-1e-9*circumference)output_.contactBudgetSatisfied=false;
    for(const auto& triangle:m.triangles)for(const auto& contact:bodyVolumes){Point cloth,axis;if(TriangleCapsule(m.vertices[triangle.vertices[0]].position,m.vertices[triangle.vertices[1]].position,m.vertices[triangle.vertices[2]].position,contact,cloth,axis,gap*.3)<gap*.3-1e-9*circumference)output_.contactBudgetSatisfied=false;}
    if(!bodyCollider_.Empty()){output_.band.minimumInnerClearance=1e100;output_.band.maximumInnerClearance=-1e100;for(unsigned i=back;i<pouchBegin;i++){double d=bodyCollider_.Closest(Mul(Sub(m.vertices[i].position,frame.origin),1/circumference)).signedDistance*circumference;output_.band.minimumInnerClearance=(std::min)(output_.band.minimumInnerClearance,d);output_.band.maximumInnerClearance=(std::max)(output_.band.maximumInnerClearance,d);}}
    Shading(m);
    auto alias=[&](unsigned a,unsigned b){auto n=Add(m.vertices[a].normal,m.vertices[b].normal);n=Length(n)>1e-12?Unit(n):m.vertices[a].normal;auto t=Add(m.vertices[a].tangent,m.vertices[b].tangent);t=Sub(t,Mul(n,Dot(t,n)));t=Length(t)>1e-12?Unit(t):m.vertices[a].tangent;m.vertices[a].normal=m.vertices[b].normal=n;m.vertices[a].tangent=m.vertices[b].tangent=t;m.vertices[b].tangentSign=m.vertices[a].tangentSign;};
    for(unsigned layer=0;layer<2;layer++)for(unsigned row=0;row<7;row++){unsigned a=layer*7*(waist+1)+row*(waist+1);alias(a,a+waist);}
    for(unsigned row=0;row<rings;row++){unsigned a=pouchBegin+1+row*(segments+1);alias(a,a+segments);}
    for(unsigned row=0;row<2;row++){unsigned a=join+row*(segments+1);alias(a,a+segments);}

    // Small support acceleration is an optional solver coupling, not permission
    // to remove/hide the internal anatomical mesh. Bounded 64 source contacts.
    const double upward=(std::max)(0.,-Dot(input.gravity,frame.up));if(upward>0&&output_.contactBudgetSatisfied){
        struct ContactCandidate{unsigned index;double weight,separation;};std::vector<ContactCandidate> candidates;candidates.reserve(localAnatomy_.size()/2);
        for(unsigned i=0;i<localAnatomy_.size();i++){auto p=Sub(localAnatomy_[i],center);double length=Length(p);if(p[2]>=0||length<1e-12)continue;auto c=coordinates(localAnatomy_[i]);double separation=(fitted(localAnatomy_[i])*tessellation-c[2])*length/(std::max)(c[2],1e-12)+gap;
            double proximity=std::clamp(1-(separation-gap)/(circumference*.05),0.,1.),weight=proximity*(-p[2]/length);if(weight>1e-6)candidates.push_back({i,weight,separation});}
        // Even sampling retains both sides and source identity. Influence follows
        // actual nearby lower cloth support, never raw anatomical vertex count.
        unsigned stride=(std::max)(1u,unsigned(candidates.size()/64));double sum=0;for(unsigned k=0;k<candidates.size()&&output_.support.size()<64;k+=stride){auto c=candidates[k];output_.support.push_back({input.anatomy[c.index].lineage,Mul(frame.up,upward*parameters_.supportFraction),c.weight,c.separation});sum+=c.weight;}
        if(sum>0)for(auto& contact:output_.support)contact.influence/=sum;
    }
    return output_;
}
}
