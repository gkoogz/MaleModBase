#pragma once
#include <limits>
namespace malemod::garments {
namespace detail {
constexpr double pi=3.14159265358979323846;
inline double Smooth(double t){return t*t*(3-2*t);}
inline Lineage Blend(Lineage a,Lineage b,double t){
    std::vector<Donor> rows;
    for(unsigned k=0;k<8;k++){auto d=k<4?a.donors[k]:b.donors[k-4];d.weight*=k<4?1-t:t;if(d.weight<=1e-14)continue;auto i=std::find_if(rows.begin(),rows.end(),[&](const Donor& r){return r.surface==d.surface&&r.vertex==d.vertex;});if(i==rows.end())rows.push_back(d);else i->weight+=d.weight;}
    std::stable_sort(rows.begin(),rows.end(),[](const Donor& a,const Donor& b){return a.weight>b.weight;});Lineage out;double total=0;for(unsigned k=0;k<std::min<std::size_t>(4,rows.size());k++)total+=rows[k].weight;if(total<=0)throw std::invalid_argument("Garment sample has no source lineage");for(unsigned k=0;k<std::min<std::size_t>(4,rows.size());k++){out.donors[k]=rows[k];out.donors[k].weight/=total;}return out;
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
inline double TriangleCapsule(Point a,Point b,Point c,const Capsule& volume,Point& cloth,Point& axis,double margin=0){
 auto centroid=Mul(Add(Add(a,b),c),1./3);double bound=(std::max)({Length(Sub(a,centroid)),Length(Sub(b,centroid)),Length(Sub(c,centroid))});if(CapsuleDistance(centroid,volume)>bound+margin)return 1e100;
 double best=1e100;auto candidate=[&](Point p,Point q){double d=Dot(Sub(p,q),Sub(p,q));if(d<best){best=d;cloth=p;axis=q;}};candidate(ClosestTriangle(volume.a,a,b,c),volume.a);candidate(ClosestTriangle(volume.b,a,b,c),volume.b);for(auto edge:{std::array<Point,2>{a,b},std::array<Point,2>{b,c},std::array<Point,2>{c,a}}){Point p,q;ClosestSegments(edge[0],edge[1],volume.a,volume.b,p,q);candidate(p,q);}
 auto n=Cross(Sub(b,a),Sub(c,a)),direction=Sub(volume.b,volume.a);double denominator=Dot(n,direction);if(std::abs(denominator)>1e-20){double t=Dot(n,Sub(a,volume.a))/denominator;if(t>=0&&t<=1){auto p=Add(volume.a,Mul(direction,t)),q=ClosestTriangle(p,a,b,c);candidate(q,p);}}
 return std::sqrt(best)-volume.radius;
}
inline void Tube(Mesh& m,const std::vector<Sample>& path,double width,double thick,bool closed,const Frame& frame,MaterialSlot material){
    const unsigned count=unsigned(path.size()),base=unsigned(m.vertices.size());double length=0;
    for(unsigned i=0;i<count+(closed?1:0);i++){const unsigned k=i%count;if(i)length+=Length(Sub(path[k].position,path[(i-1)%count].position));auto tangent=Unit(Sub(path[(std::min)(count-1,k+1)].position,path[k?k-1:0].position));if(closed)tangent=Unit(Sub(path[(k+1)%count].position,path[(k+count-1)%count].position));auto normal=Sub(path[k].normal,Mul(tangent,Dot(path[k].normal,tangent)));if(Length(normal)<1e-8)normal=Cross(tangent,std::abs(Dot(tangent,frame.up))<.9?frame.up:frame.forward);normal=Unit(normal);auto side=Unit(Cross(tangent,normal));
        for(unsigned j=0;j<4;j++){double w=j==0||j==3?-width*.5:width*.5,h=j<2?thick*.5:-thick*.5;VertexAt(m,Add(path[k].position,Add(Mul(side,w),Mul(normal,h))),length/width,double(j),path[k].lineage,normal);}
    }
    unsigned segments=closed?count:count-1;
    for(unsigned i=0;i<segments;i++)for(unsigned j=0;j<4;j++){unsigned a=base+i*4+j,b=base+i*4+(j+1)%4,c=b+4,d=a+4;Point center=Mul(Add(path[i%count].position,path[(i+1)%count].position),.5);Quad(m,a,b,c,d,material,Sub(m.vertices[a].position,center));}
    if(!closed){Quad(m,base,base+1,base+2,base+3,material,Sub(path[0].position,path[1].position));auto b=base+(count-1)*4;Quad(m,b,b+1,b+2,b+3,material,Sub(path.back().position,path[count-2].position));}
}
}
inline Session::Session(Parameters p):parameters_(p){for(double v:{p.bandWidth,p.bandThickness,p.strapWidth,p.hemWidth,p.clearance})if(!std::isfinite(v)||v<=0||v>.2)throw std::invalid_argument("Garment dimensions must be finite measured fractions");if(!std::isfinite(p.supportFraction)||p.supportFraction<0||p.supportFraction>.15||p.pouchRings<8||p.pouchRings>64||p.pouchSegments<16||p.pouchSegments>128)throw std::invalid_argument("Garment strength/tessellation outside budget");output_.mesh.vertices.reserve(14000);output_.mesh.triangles.reserve(28000);localAnatomy_.reserve(32768);}
inline void Session::Reset(){output_=Output{};localAnatomy_.clear();}
inline const Output& Session::Update(Style style,const Input& input){
    using namespace detail;
    if(style!=Style::Naked&&style!=Style::WhiteJockstrap)throw std::invalid_argument("Unknown garment style");
    output_.style=style;output_.characterEpoch=input.characterEpoch;output_.topologyRevision=input.topologyRevision;output_.mesh.vertices.clear();output_.mesh.triangles.clear();output_.support.clear();output_.projectedContacts=0;output_.contactBudgetSatisfied=true;output_.coverageMargin=0;
    if(style==Style::Naked)return output_;
    input.frame.Validate();ValidateSamples(input.waist,8,128);ValidateSamples(input.opening,8,128);ValidateSamples(input.anatomy,4,131072);for(const auto& path:input.rearStraps)ValidateSamples(path,3,128);
    if(input.bodyContacts.size()>64||!Finite(input.gravity)||!std::isfinite(input.deltaTime)||input.deltaTime<0||input.deltaTime>1)throw std::invalid_argument("Invalid garment contact/timing input");
    for(const auto& c:input.bodyContacts)if(!Finite(c.a)||!Finite(c.b)||!std::isfinite(c.radius)||c.radius<0)throw std::invalid_argument("Invalid body volume");
    for(const auto& f:input.anatomyTriangles)for(auto i:f)if(i>=input.anatomy.size())throw std::invalid_argument("Anatomy triangle index outside surface");
    double circumference=0;for(std::size_t i=0;i<input.waist.size();i++)circumference+=Length(Sub(input.waist[i].position,input.waist[(i+1)%input.waist.size()].position));if(!std::isfinite(circumference)||circumference<1e-12)throw std::invalid_argument("Degenerate measured waist");output_.measuredCircumference=circumference;
    const double width=parameters_.bandWidth*circumference,thick=parameters_.bandThickness*circumference,gap=parameters_.clearance*circumference,hem=parameters_.hemWidth*circumference;
    auto& m=output_.mesh;const auto& frame=input.frame;const unsigned waist=unsigned(input.waist.size());
    // A closed thick waistband with separate thin stripe bands, not a flat
    // image decal. Duplicate seams retain equal positions and original donors.
    const std::array<double,7> rows{{-.5,-.39,-.345,-.28,-.235,.39,.5}};
    for(unsigned layer=0;layer<2;layer++){
        const unsigned base=unsigned(m.vertices.size());
        for(unsigned j=0;j<rows.size();j++)for(unsigned i=0;i<=waist;i++){const auto& s=input.waist[i%waist];auto radial=Sub(s.normal,Mul(frame.up,Dot(s.normal,frame.up)));if(Length(radial)<1e-10){auto p=frame.Local(s.position);radial=Add(Mul(frame.lateral,p[0]),Mul(frame.forward,p[1]));}radial=Unit(radial);VertexAt(m,Add(s.position,Add(Mul(frame.up,rows[j]*width),Mul(radial,gap+(layer?0:thick)))),double(i)/waist,double(j)/6,s.lineage);}
        for(unsigned j=0;j+1<rows.size();j++)for(unsigned i=0;i<waist;i++){unsigned a=base+j*(waist+1)+i;auto normal=input.waist[i].normal;if(layer)normal=Mul(normal,-1);MaterialSlot material=MaterialSlot::WhiteElastic;if(!layer&&j==1)material=MaterialSlot::RedStripe;if(!layer&&j==3)material=MaterialSlot::BlueStripe;Quad(m,a,a+1,a+waist+2,a+waist+1,material,normal);}
    }
    // Close upper/lower physical band edges. Separate normals preserve the
    // narrow thickness instead of a zero-volume strip.
    const unsigned back=7*(waist+1);for(unsigned row:{0u,6u})for(unsigned i=0;i<waist;i++){auto a=row*(waist+1)+i,b=a+1;Quad(m,a,b,b+back,a+back,MaterialSlot::WhiteElastic,Mul(frame.up,row?1:-1));}
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
    for(unsigned pass=0;pass<2;pass++){next=extent;for(unsigned y=1;y<rings;y++)for(unsigned x=0;x<segments;x++){unsigned at=y*segments+x;double average=(extent[at- segments]+extent[at+segments]+extent[y*segments+(x+1)%segments]+extent[y*segments+(x+segments-1)%segments])*.25;next[at]=(std::max)(extent[at],extent[at]*.75+average*.25);}extent.swap(next);}
    unsigned poleDonor=0;double poleExtent=0;for(unsigned x=0;x<segments;x++)if(extent[x]>poleExtent){poleExtent=extent[x];poleDonor=unsigned((std::max)(0,donor[x]));}for(unsigned x=0;x<segments;x++){extent[x]=poleExtent;donor[x]=int(poleDonor);}
    const double tessellation=1/std::cos(std::sqrt(std::pow(pi/(2*rings),2)+std::pow(2*pi/segments,2)));
    auto radial=[&](unsigned y,unsigned x){double phi=pi*.5*y/rings,theta=2*pi*(x%segments)/segments;Point direction{radius[0]*std::sin(phi)*std::cos(theta),radius[1]*std::cos(phi),radius[2]*std::sin(phi)*std::sin(theta)};double e=extent[y*segments+x%segments];return Add(center,Mul(direction,e*tessellation+gap/(std::max)(Length(direction),gap)));};
    auto fitted=[&](Point p){auto c=coordinates(p);unsigned y=(std::min)(rings-1,unsigned(c[0])),x=unsigned(c[1])%segments;double fy=c[0]-y,fx=c[1]-std::floor(c[1]);return (1-fy)*((1-fx)*extent[y*segments+x]+fx*extent[y*segments+(x+1)%segments])+fy*((1-fx)*extent[(y+1)*segments+x]+fx*extent[(y+1)*segments+(x+1)%segments]);};
    output_.coverageMargin=1e100;for(auto p:localAnatomy_){auto c=coordinates(p);output_.coverageMargin=(std::min)(output_.coverageMargin,(fitted(p)*tessellation-c[2])*(std::min)({radius[0],radius[1],radius[2]})+gap);}
    const unsigned pouchBegin=unsigned(m.vertices.size());VertexAt(m,frame.World(radial(0,0)),.5,0,input.anatomy[poleDonor].lineage);
    for(unsigned j=1;j<=rings;j++)for(unsigned i=0;i<=segments;i++){auto d=donor[j*segments+i%segments];VertexAt(m,frame.World(radial(j,i)),double(i)/segments,double(j)/rings,input.anatomy[unsigned((std::max)(0,d))].lineage);}
    for(unsigned i=0;i<segments;i++)Tri(m,pouchBegin,pouchBegin+1+i,pouchBegin+2+i,MaterialSlot::WhiteRibbed,frame.forward);
    for(unsigned j=0;j+1<rings;j++)for(unsigned i=0;i<segments;i++){unsigned a=pouchBegin+1+j*(segments+1)+i;Quad(m,a,a+1,a+segments+2,a+segments+1,MaterialSlot::WhiteRibbed,Sub(m.vertices[a].position,frame.World(center)));}
    const unsigned pouchEnd=unsigned(m.vertices.size());
    std::vector<Sample> rim;rim.reserve(segments);for(unsigned i=0;i<segments;i++){auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+i];rim.push_back({v.position,frame.forward,v.lineage});}
    Tube(m,rim,hem,thick*.45,true,frame,MaterialSlot::WhiteElastic);
    // Join the measured body opening to the pouch. The rear remains open; this
    // seam and the two under-glute straps are deliberate jockstrap structure.
    const unsigned join=unsigned(m.vertices.size());for(unsigned row=0;row<2;row++)for(unsigned i=0;i<=segments;i++){auto s=RingSample(input.opening,double(i)/segments);const auto& v=m.vertices[pouchBegin+1+(rings-1)*(segments+1)+i];VertexAt(m,row?v.position:s.position,double(i)/segments,double(row),row?v.lineage:s.lineage);}
    for(unsigned i=0;i<segments;i++)Quad(m,join+i,join+i+1,join+segments+2+i,join+segments+1+i,MaterialSlot::WhiteRibbed,frame.forward);
    // Front suspension panel sews the upper pouch arc into the measured front
    // waistband. Its shape has no invented bone names or fixed world height.
    Point waistCenter{};for(const auto& v:input.waist)waistCenter=Add(waistCenter,frame.Local(v.position));waistCenter=Mul(waistCenter,1./input.waist.size());
    const unsigned panelBase=unsigned(m.vertices.size()),panelColumns=24,panelRows=8;
    for(unsigned row=0;row<=panelRows;row++)for(unsigned col=0;col<=panelColumns;col++){
        double u=double(col)/panelColumns,t=double(row)/panelRows,theta=pi/3+u*pi/3;
        auto top=WaistAngle(input.waist,waistCenter,frame,(.5-u)*1.2);
        auto bottom=RingSample(rim,theta/(2*pi));Point normal=Unit(Length(top.normal)>1e-12?top.normal:frame.forward);
        auto start=Add(top.position,Add(Mul(normal,gap+thick),Mul(frame.up,-width*.42)));
        auto point=Add(Mul(start,1-t),Mul(bottom.position,t));VertexAt(m,point,u,t,Blend(top.lineage,bottom.lineage,t));
    }
    for(unsigned row=0;row<panelRows;row++)for(unsigned col=0;col<panelColumns;col++){auto a=panelBase+row*(panelColumns+1)+col;Quad(m,a,a+1,a+panelColumns+2,a+panelColumns+1,MaterialSlot::WhiteRibbed,frame.forward);}
    for(const auto& measuredPath:input.rearStraps){auto path=measuredPath;const double side=frame.Local(path.back().position)[0];auto sewn=RingSample(rim,(side<0?4*pi/3:5*pi/3)/(2*pi));path.push_back(sewn);Tube(m,path,parameters_.strapWidth*circumference,thick*.5,false,frame,MaterialSlot::WhiteElastic);}
    // Body volume contacts are bounded and resolved without deleting triangles.
    // Reproject pouch rows onto their coverage envelope so a thigh projection
    // cannot push cloth through the anatomical volume.
    auto reconcileContactSeams=[&](){
      auto weld=[&](unsigned a,unsigned b){auto p=Mul(Add(m.vertices[a].position,m.vertices[b].position),.5);m.vertices[a].position=m.vertices[b].position=p;};
      for(unsigned layer=0;layer<2;layer++)for(unsigned row=0;row<7;row++){unsigned a=layer*7*(waist+1)+row*(waist+1);weld(a,a+waist);}
      for(unsigned row=0;row<rings;row++){unsigned a=pouchBegin+1+row*(segments+1);weld(a,a+segments);}
      for(unsigned row=0;row<2;row++){unsigned a=join+row*(segments+1);weld(a,a+segments);}
    };
    // Contact capsules describe one union. Projecting against them separately
    // can oscillate between overlapping thigh/pelvis volumes. Move along the
    // measured garment/body outward direction to the union's outer boundary.
    auto escapePoint=[&](Point& point,Point preferred){bool touching=false;for(auto c:input.bodyContacts)touching|=CapsuleDistance(point,c)<gap*.3;if(!touching)return;preferred=Length(preferred)>1e-12?Unit(preferred):frame.forward;double high=gap;
      auto clear=[&](double distance){auto p=Add(point,Mul(preferred,distance));for(auto c:input.bodyContacts)if(CapsuleDistance(p,c)<gap*.31)return false;return true;};
      for(unsigned k=0;k<24&&!clear(high);k++)high*=2;if(!clear(high))return;double low=0;for(unsigned k=0;k<24;k++){double mid=(low+high)*.5;if(clear(mid))high=mid;else low=mid;}point=Add(point,Mul(preferred,high));output_.projectedContacts++;
    };
    auto escapeTriangle=[&](const Triangle& triangle){auto& a=m.vertices[triangle.vertices[0]].position;auto& b=m.vertices[triangle.vertices[1]].position;auto& c=m.vertices[triangle.vertices[2]].position;Point preferred{};for(auto i:triangle.vertices)preferred=Add(preferred,m.vertices[i].normal);preferred=Length(preferred)>1e-12?Unit(preferred):frame.forward;
      auto clear=[&](double distance){auto delta=Mul(preferred,distance);for(auto contact:input.bodyContacts){Point cloth,axis;if(TriangleCapsule(Add(a,delta),Add(b,delta),Add(c,delta),contact,cloth,axis,gap*.31)<gap*.31)return false;}return true;};if(clear(0))return;double high=gap;for(unsigned k=0;k<24&&!clear(high);k++)high*=2;if(!clear(high))return;double low=0;for(unsigned k=0;k<24;k++){double mid=(low+high)*.5;if(clear(mid))high=mid;else low=mid;}auto delta=Mul(preferred,high);a=Add(a,delta);b=Add(b,delta);c=Add(c,delta);output_.projectedContacts++;
    };
    for(unsigned iteration=0;!input.bodyContacts.empty()&&iteration<16;iteration++){auto before=output_.projectedContacts;
      for(unsigned i=0;i<m.vertices.size();i++){auto& v=m.vertices[i];escapePoint(v.position,v.normal);if(i>=pouchBegin&&i<pouchEnd){auto p=Sub(frame.Local(v.position),center);auto c=coordinates(Add(p,center));double r=Length(p),minimum=fitted(Add(p,center))*tessellation+gap/(std::max)(r,gap);if(c[2]<minimum&&c[2]>1e-12)v.position=frame.World(Add(center,Mul(p,minimum/c[2])));}}
      for(const auto& triangle:m.triangles)escapeTriangle(triangle);
      reconcileContactSeams();if(output_.projectedContacts==before)break;
    }
    if(!input.bodyContacts.empty())reconcileContactSeams();
    // Explicit residual check exposes an unsatisfied contact budget rather than
    // silently hiding clipping. Adapters must surface this status in validation.
    for(const auto& v:m.vertices)for(const auto& c:input.bodyContacts)if(CapsuleDistance(v.position,c)<gap*.3-1e-9*circumference)output_.contactBudgetSatisfied=false;
    for(const auto& triangle:m.triangles)for(const auto& contact:input.bodyContacts){Point cloth,axis;if(TriangleCapsule(m.vertices[triangle.vertices[0]].position,m.vertices[triangle.vertices[1]].position,m.vertices[triangle.vertices[2]].position,contact,cloth,axis,gap*.3)<gap*.3-1e-9*circumference)output_.contactBudgetSatisfied=false;}
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
