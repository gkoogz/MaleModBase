#include <malemod/surface/compound_contact_io.hpp>
#include <malemod/surface/compound_exterior.hpp>
#include <sstream>
#include <iostream>
using namespace malemod::surface;
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
CompoundContactReference fixture(){
 CompoundContactReference r;
 for(unsigned y=0;y<=8;y++)for(unsigned x=0;x<=8;x++){
  r.support.push_back({double(x)-4,double(y)-4,0});r.chart.push_back({double(x)-4,double(y)-4});r.protectedSupport.push_back(true);
 }
 for(unsigned y=0;y<8;y++)for(unsigned x=0;x<8;x++){
  unsigned a=y*9+x,b=a+1,c=a+9,d=c+1;r.supportFaces.push_back({a,c,b});r.supportFaces.push_back({b,c,d});r.eligible.push_back(true);r.eligible.push_back(true);
 }
 r.source={{-1,-1,-2},{1,-1,-2},{1,1,-2},{-1,1,-2},{0,0,-3}};
 r.sourceLoop={0,3,2,1};r.footprint={{-1.4,-1.4},{-1.4,1.4},{1.4,1.4},{1.4,-1.4}};
 for(unsigned j=0;j<4;j++)r.sourceFaces.push_back({4,(j+1)%4,j});
 r.curve={0,1,2,3};r.parameters={0,.25,.5,.75,1};
 for(unsigned j=0;j<4;j++){
  auto p=r.source[r.sourceLoop[j]];PresentationPoint q{r.footprint[j][0],r.footprint[j][1],0};r.firstTangents.push_back(Sub(q,p));r.lastTangents.push_back(Sub(q,p));
 }
 for(double t:r.parameters)for(unsigned j=0;j<4;j++){
  auto p=r.source[r.sourceLoop[j]];PresentationPoint q{r.footprint[j][0],r.footprint[j][1],0};r.patch.push_back(Add(Scale(p,1-t),Scale(q,t)));
 }return r;
}
// Two meshes describe the same planar boundary. The second adds a tiny
// cluster of collinear samples; this must not change the frame's density.
struct SourceGaugeFixture {std::vector<PresentationPoint> source;CompoundContactCache cache;};
SourceGaugeFixture sourceGaugeFixture(bool clustered){
 SourceGaugeFixture f;auto& c=f.cache;f.source={{-2,0,0},{-1,0,0}};
 if(clustered)for(double x:{-.0001,0.,.0001})f.source.push_back({x,0,0});else f.source.push_back({0,0,0});
 for(auto p:std::vector<PresentationPoint>{{1,0,0},{2,0,0},{2,3,0},{-2,3,0}})f.source.push_back(p);
 const unsigned count=unsigned(f.source.size());for(unsigned i=0;i<count;i++)c.sourceLoop.push_back(i);
 f.source.push_back({0,1.5,0});std::vector<double> arc{0};
 for(unsigned i=0;i<count;i++){
  c.sourceFaces.push_back({count,i,(i+1)%count});auto edge=Sub(f.source[(i+1)%count],f.source[i]);arc.push_back(arc.back()+std::sqrt(Dot(edge,edge)));
  auto side=Scale(Sub(f.source[(i+1)%count],f.source[(i+count-1)%count]),.5);c.sourceReferenceSideLength.push_back(std::sqrt(Dot(side,side)));
 }
 c.framePeriod=arc.back();
 for(unsigned k=0;k<256;k++){
  const double u=arc.back()*k/256;auto next=std::upper_bound(arc.begin(),arc.end(),u);const unsigned j=unsigned(next-arc.begin())-1;
  const double t=(u-arc[j])/(arc[j+1]-arc[j]);c.curveKnots.push_back(j+t);c.frameCoordinates.push_back(u);c.sourceDifferentials.push_back({j,j,{0,1-t,t},t});
 }return f;
}
void sourceGaugeRegression(){
 auto regular=sourceGaugeFixture(false),clustered=sourceGaugeFixture(true);
 const auto a=regular.cache.SourceDifferentialPoints(regular.source),b=clustered.cache.SourceDifferentialPoints(clustered.source);
 unsigned samples=0;
 for(unsigned i=0;i<256;i++)if(a[3*i][0]>-.5&&a[3*i][0]<.5&&std::abs(a[3*i][1])<1e-10){
  samples++;for(unsigned k=1;k<3;k++){auto d=Sub(Sub(a[3*i+k],a[3*i]),Sub(b[3*i+k],b[3*i]));require(Dot(d,d)<1e-20,"Source frame depends on collinear tessellation density");}
 }
 require(samples>8,"Source gauge regression lacks shared samples");
 // Fixed reference divisors retain actual live strain. Normalizing each live
 // vector instead would discard uniform scaling and fail this check.
 for(double scale:{.25,1.,3.})for(double angle:{-.7,.2,.9}){
  auto transform=[&](PresentationPoint p){double c=std::cos(angle),s=std::sin(angle);return Add(Scale(PresentationPoint{c*p[0]-s*p[2],p[1],s*p[0]+c*p[2]},scale),{4,2,-3});};
  auto source=clustered.source;for(auto& p:source)p=transform(p);auto moved=clustered.cache.SourceDifferentialPoints(source);
  for(unsigned i=0;i<moved.size();i++){auto d=Sub(moved[i],transform(b[i]));require(Dot(d,d)<1e-20,"Source gauge discarded live scale or rotation");}
 }
}

int main(){try{
 {
  // A chord toward the side of a cylindrical support lies inside the solid.
  // The connector must use the original outer surface while keeping both rims.
  std::vector<PresentationPoint> support;std::vector<std::array<unsigned,3>> faces;
  constexpr unsigned n=64;
  for(double x:{-3.,3.})for(unsigned j=0;j<n;j++){double a=2*3.141592653589793*j/n;support.push_back({x,std::cos(a),std::sin(a)});}
  for(unsigned j=0;j<n;j++){unsigned k=(j+1)%n;faces.push_back({j,k,n+j});faces.push_back({k,n+k,n+j});}
  std::vector<PresentationPoint> points{{0,0,-2},{0,.3,-.4},{0,1,0}};auto original=points;
  auto count=ConformCompoundExterior(points,{{0},{1},{2}},support,faces,std::vector<bool>(faces.size(),true),{{-2,0,0},{2,0,0}});
  require(count==1&&std::hypot(points[1][1],points[1][2])>.998,"Connector still cuts through shaft volume");
  require(points[0]==original[0]&&points[2]==original[2],"Exterior contact moved a shared rim");
  require(ConformCompoundExterior(points,{{0},{1},{2}},support,faces,std::vector<bool>(faces.size(),true),{{-2,0,0},{2,0,0}})==0,"Exterior contact accumulates");
 }
 sourceGaugeRegression();
 auto r=fixture();CompoundContactPlan plan(r);auto cache=plan.Build(r.source,r.support,r.supportFaces,r.chart,r.eligible,r.protectedSupport);
 auto p=cache.Evaluate(r.source,r.support);double restError=0;for(unsigned i=0;i<p.size();i++){auto d=Sub(p[i],cache.rest[i]);restError=(std::max)(restError,std::sqrt(Dot(d,d)));}require(restError<1e-10,"Rest reproduction differs");
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 for(auto f:cache.faces)for(unsigned k=0;k<3;k++){auto a=f[k].vertex,b=f[(k+1)%3].vertex;auto& e=edges[std::minmax(a,b)];e.first++;e.second+=a<b?1:-1;}
 unsigned boundary=0;for(auto item:edges){auto e=item.second;if(e.first==1)boundary++;else require(e.first==2&&e.second==0,"Nonmanifold assembly");}require(boundary==32,"Added an external boundary");
 for(double angle:{-.7,0.,.6})for(double scale:{.4,1.,2.}){
  auto transform=[&](PresentationPoint p){double c=std::cos(angle),s=std::sin(angle);return Add(Scale(PresentationPoint{c*p[0]-s*p[2],p[1],s*p[0]+c*p[2]},scale),{4,2,1});};
  auto source=r.source,support=r.support;for(auto& v:source)v=transform(v);for(auto& v:support)v=transform(v);auto moved=cache.Evaluate(source,support);
  for(unsigned i=0;i<moved.size();i++){auto d=Sub(moved[i],transform(p[i]));require(Dot(d,d)<1e-18,"Rigid/scale covariance differs");}
 }
 auto source=r.source;for(auto& v:source)v[2]-=.3;auto moved=cache.Evaluate(source,r.support);
 for(unsigned i=0;i<r.support.size();i++)require(moved[i]==r.support[i],"Independent suspension moved support");
 {
  auto free=cache.SuspensionContactTranslation(r.source,r.support,1.);
  require(Dot(free,free)<1e-20,"Contact changed the authored reference");
  auto compressed=r.source;for(auto& v:compressed)v[2]+=1.9;
  auto shift=cache.SuspensionContactTranslation(compressed,r.support,1.);
  require(std::abs(shift[2]+.3)<1e-10&&shift[0]==0&&shift[1]==0,"Suspended rim did not preserve its contact gap");
  for(auto& v:compressed)v=Add(v,shift);
  auto again=cache.SuspensionContactTranslation(compressed,r.support,1.);
  require(Dot(again,again)<1e-20,"Suspension contact accumulates displacement");
  auto contact=cache.Evaluate(compressed,r.support,nullptr,1.);
  for(unsigned i=0;i<r.support.size();i++)require(contact[i]==r.support[i],"Suspension contact moved retained support");
 }
 auto reject=[&](CompoundContactReference bad){bool failed=false;try{CompoundContactPlan other(std::move(bad));}catch(const std::invalid_argument&){failed=true;}require(failed,"Malformed reference accepted");};
 auto bad=r;bad.sourceLoop[0]=9999;reject(bad);bad=r;bad.curve[1]=bad.curve[0];reject(bad);bad=r;bad.parameters.back()=.9;reject(bad);bad=r;bad.support[0][0]=std::numeric_limits<double>::quiet_NaN();reject(bad);
 {
  // A close pair of immutable rim samples needs a coarser interior meridian,
  // not a displaced rim or a hole. This also exercises generated-strip flips.
  CompoundContactCache narrow;const std::vector<double> angles{0,1e-5,.8,1.6,2.4,3.2,4,5};
  constexpr unsigned rows=11;const unsigned columns=unsigned(angles.size());
  for(unsigned row=0;row<rows;row++)for(auto a:angles)narrow.rest.push_back({std::cos(a),std::sin(a),double(row)/(rows-1)});
  for(unsigned row=0;row+1<rows;row++)for(unsigned j=0;j<columns;j++){
   auto corner=[&](unsigned r,unsigned c){return CompoundContactCorner{r*columns+c,2,0,{},double(c),r};};unsigned k=(j+1)%columns;
   narrow.faces.push_back({corner(row,k),corner(row,j),corner(row+1,j)});narrow.faces.push_back({corner(row,k),corner(row+1,j),corner(row+1,k)});
  }
  narrow.rest.push_back({1000,0,0}); // Scene extent sets the area tolerance.
  std::vector<bool> protect(narrow.rest.size());for(unsigned j=0;j<columns;j++)protect[j]=protect[(rows-1)*columns+j]=true;
  auto original=narrow.rest;narrow.Clean(protect);require(narrow.rest==original,"Cleanup moved retained geometry");
  require(narrow.collapsedEdges>0,"Skinny meridian was not conditioned");
  std::map<std::pair<unsigned,unsigned>,unsigned> usage;for(auto face:narrow.faces)for(unsigned k=0;k<3;k++)usage[std::minmax(face[k].vertex,face[(k+1)%3].vertex)]++;
  unsigned open=0;for(auto item:usage){require(item.second<=2,"Cleanup made a branched strip");open+=item.second==1;}
  require(open==2*columns,"Cleanup changed the immutable rim");
 }
 {
  // A narrow fan on gently curved generated skin. Its center is farther
  // along the skin than the point budget, but the retriangulation's normal
  // error is below one tenth of that budget. The boundary is immutable.
  CompoundContactCache ear;
  ear.rest={{0,0,0},{.004,0,0},{.014,.000005,.000002},{.009,.02,.0001},{-.011,.02,0},{-.005,-.02,0},{.005,-.015,.00005},{100,0,0}};
  for(unsigned j=1;j<=6;j++)ear.faces.push_back({CompoundContactCorner{0,2,0,{},0,1},CompoundContactCorner{j,2,0,{},double(j),0},CompoundContactCorner{j==6?1:j+1,2,0,{},double(j==6?1:j+1),0}});
  std::vector<bool> protect(ear.rest.size(),true);protect[0]=false;
  CompoundContactCache retained;retained.rest=ear.rest;retained.faces=ear.faces;for(auto& face:retained.faces)for(auto& c:face)c.domain=1;
  bool guarded=false;try{retained.Clean(protect);}catch(const std::runtime_error&){guarded=true;}
  require(guarded||retained.collapsedEdges==0,"Generated skin error budget leaked onto retained source");
  const auto original=ear.rest;ear.Clean(protect);
  require(ear.collapsedEdges==1,"Curved generated ear could not collapse within positional budget");
  require(ear.rest==original,"Curved ear cleanup changed vertex positions");
  std::map<std::pair<unsigned,unsigned>,unsigned> usage;
  for(auto face:ear.faces){for(auto c:face)require(c.domain==2&&c.parent==0&&c.vertex!=0,"Cleanup crossed material parent or retained removed center");for(unsigned k=0;k<3;k++)usage[std::minmax(face[k].vertex,face[(k+1)%3].vertex)]++;}
  unsigned open=0;for(auto item:usage){require(item.second<=2,"Curved ear cleanup became nonmanifold");open+=item.second==1;}
  require(open==6,"Curved ear cleanup changed protected boundary");
 }
 // A short forged header must be rejected before allocating its claimed mesh.
 std::string bytes="MMCCNT02";std::array<unsigned,7> counts{500000,1000000,500000,1000000,4096,50000,256};bytes.append(reinterpret_cast<const char*>(counts.data()),28);std::istringstream input(bytes);bool rejected=false;try{ReadCompoundContact(input);}catch(const std::runtime_error&){rejected=true;}require(rejected,"Oversize reference allocation accepted");
 std::cout<<"compound assembly, covariance, independent motion and input validation pass\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
