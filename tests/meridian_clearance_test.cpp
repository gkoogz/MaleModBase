#include <malemod/garments/meridian_clearance.hpp>
#include <malemod/garments/taut_contact.hpp>
#include <cstdio>
#include <malemod/garments/meridian_rig.hpp>
using namespace malemod::garments::meridian;
static void Require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 // A triangle can clear a convex primitive along its own face normal while
 // spanning all preselected plane directions. Do not force it to a distant
 // axis plane. The exact support test still rejects an intersecting chord.
 {Hull sphere={{{1,0,0},1},{{-1,0,0},1},{{0,1,0},1},{{0,-1,0},1},{{0,0,1},1},{{0,0,-1},1}};
  sphere.support=[](Vec n){return std::sqrt(Dot(n,n));};
  Vec a={1.9f,-.4f,0},b={-.4f,1.9f,0},c={.75f,.75f,1};
  Require(ExactTriangleSeparated(sphere,a,b,c),"Exact separating direction missed");
  Require(!ExactTriangleSeparated(sphere,{-2,0,0},{2,0,0},{0,2,0}),"Exact support accepted chord penetration");
  Hull shifted=sphere;shifted.support=[](Vec n){return 2*n[0]+std::sqrt(Dot(n,n));};
  auto cover=ConvexCover({sphere,shifted},{0,0,1});
  for(auto plane:cover){Require(plane.offset>=sphere.support(plane.normal),"Common envelope cuts first support");Require(plane.offset>=shifted.support(plane.normal),"Common envelope cuts second support");}
 }
 // A tip folded back through the boundary must not reverse or collapse the
 // display chart. Its boundary and collision primitives remain unchanged.
 {std::vector<Vec> p={{0,-2,-2},{0,2,-2},{0,2,2},{0,-2,2},{-5,0,0},{2,0,0},{3,1,1},{3,-1,-1}};auto before=p;
  auto axis=PrepareEnvelopePole(p,4,4,5,8);
  Require(axis[0]>.999f&&p[4][0]>3,"Folded tip collapsed the sewn-boundary chart");
 for(unsigned i:{0u,1u,2u,3u,5u,6u,7u})Require(p[i]==before[i],"Envelope pole moved boundary or primitive");
 }
 {std::vector<Vec> p={{0,-2,-2},{0,2,-2},{0,2,2},{0,-2,2},{-5,0,-8},{2,0,0},{3,1,1},{3,-1,-1}};auto before=p;
  auto chart=PrepareAnchoredEnvelope(p,4,4,5,8,{-5,0,-8});Require(std::abs(p[4][2]+8)<1e-6f,"Chart lifted the render tip away from the anatomy");
  for(Vec q:before)Require(Dot(Sub(chart.Inverse(chart.Forward(q)),q),Sub(chart.Inverse(chart.Forward(q)),q))<1e-8f,"Envelope shear failed round trip");
  Hull sphere; sphere.support=[](Vec n){return std::sqrt(Dot(n,n));};auto transformed=chart.Transform(sphere);
  for(unsigned k=0;k<100;k++){float a=k*2.39996323f,z=1-2*(k+.5f)/100,r=std::sqrt(1-z*z);Vec n={r*std::cos(a),r*std::sin(a),z};
   for(Vec q:std::vector<Vec>{{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}})Require(Dot(n,chart.Forward(q))<=transformed.support(n)+1e-5f,"Sheared support cuts the current collider");
  }
  for(unsigned i:{0u,1u,2u,3u,5u,6u,7u})Require(p[i]==before[i],"Anchored chart moved a sewn point or collider");
 }
 auto ring=FitCircularSection({2,0,0},{0,2,0},{-2,0,0},{0,-2,0});Vec dome[8*32+2];AlignDomeRim(ring,{0,0,3});WriteDome(dome,ring,{0,0,3},8,32);
 for(unsigned i=0;i<32;i++)Require(std::abs(Dot(dome[i],dome[i])-4)<1e-5f,"Dome rim lost its exact circle");Require(dome[256]==Vec{0,0,3},"Dome apex moved");
 Vec lobe[9*32+2];WriteOvoid(lobe,{0,0,3},{0,0,-3},{2,0,0},{-2,0,0},{0,1,0},{0,-1,0},8,32);
 Require(lobe[288]==Vec{0,0,3}&&lobe[289]==Vec{0,0,-3},"Ovoid poles drifted");Require(std::abs(lobe[128][0]-2)<1e-5f,"Ovoid equator changed");
 const Vec controls[]={{0,0,3},{0,0,-3},{2,0,0},{-2,0,0},{0,1,0},{0,-1,0}};
 for(unsigned k=0;k<100;k++){float a=k*2.39996323f,z=1-2*(k+.5f)/100,r=std::sqrt(1-z*z);Vec n={r*std::cos(a),r*std::sin(a),z};float ds=DomeSupport(ring,{0,0,3},n),ls=OvoidSupport(controls,n);for(auto p:dome)Require(Dot(n,p)<=ds+1e-5f,"Analytic dome support excludes its surface");for(auto p:lobe)Require(Dot(n,p)<=ls+1e-5f,"Analytic ovoid support excludes its surface");}
 constexpr unsigned cols=16,rows=8;std::vector<Vec> reference;std::vector<Face> faces;
 for(unsigned row=0;row<rows;row++)for(unsigned col=0;col<cols;col++){
  float a=6.28318531f*col/cols,r=row? .2f:3.f;
  reference.push_back({r*std::cos(a),r*std::sin(a),-3+7.f*row/rows});
 }
 reference.push_back({0,0,4});
 Require(WithinMeridianSampling(reference,reference,cols,rows),"Unchanged envelope failed physical budget");
 {auto spike=reference;spike[cols*4][0]+=100;Require(!WithinMeridianSampling(spike,reference,cols,rows),"Physical budget accepted a transformed spike");}
 for(unsigned row=0;row+1<rows;row++)for(unsigned col=0;col<cols;col++){
  unsigned x=row*cols+col,y=row*cols+(col+1)%cols;
  faces.push_back({std::uint16_t(x),std::uint16_t(y),std::uint16_t(y+cols)});faces.push_back({std::uint16_t(x),std::uint16_t(y+cols),std::uint16_t(x+cols)});
 }
 for(unsigned col=0;col<cols;col++)faces.push_back({std::uint16_t((rows-1)*cols+col),std::uint16_t((rows-1)*cols+(col+1)%cols),std::uint16_t(cols*rows)});
 const Vec normals[]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
 // A previously walked surface must keep its stations when its triangles
 // already clear. The old second walk redistributed them and could fail the
 // caller's physical sampling budget despite needing no contact correction.
 {auto seed=reference;for(unsigned row=1;row<rows;row++)for(unsigned col=0;col<cols;col++)
   seed[row*cols+col][2]+= .1f*std::sin(float(col));
  auto solved=seed;
  ClearMeridians(solved,cols,rows,faces.data(),unsigned(faces.size()),{},{0,0,1},.04f,12,nullptr,0,true);
  for(unsigned i=0;i<seed.size();i++)Require(Dot(Sub(solved[i],seed[i]),Sub(solved[i],seed[i]))<1e-10f,"Clear seed was redistributed");
  Require(WithinMeridianSampling(solved,seed,cols,rows),"Unneeded second walk spent the physical budget");
 }
 // A hem close to a collider cap should escape locally, rather than being
 // dragged all the way around the side by a purely radial fitting rule.
 {std::vector<Vec> seam;for(unsigned col=0;col<cols;col++){float a=6.28318531f*col/cols;seam.push_back({.8f*std::cos(a),.8f*std::sin(a),-.98f});}
  const Vec box[]={{-1,-1,-1},{1,1,1}};auto hull=SupportHull(box,2,normals,6,0);
  auto delta=FitSeam(seam,cols,{0,0,4},{0,0,1},{hull},.12f,.5f);
  for(unsigned col=0;col<cols;col++){Require(Dot(delta[col],delta[col])<.25f,"Cap fitting stretched the fixed trim excessively");bool clear=false;
   for(auto plane:hull)if(Signed(plane,seam[col])>=.059f&&Signed(plane,seam[(col+1)%cols])>=.059f)clear=true;
   Require(clear,"Fitted sewn edge still crosses the cap");}
 }
 std::vector<unsigned> cache;
 // Triangle correction used to leave a local return along one ray. Fairing
 // must remove it without moving the outline/pole or cutting through a solid.
 {auto points=reference;unsigned id=4*cols;points[id]={8,0,points[id][2]};const auto seam=points.front(),pole=points.back();
  auto before=Dot(Sub(points[id],points[id-cols]),Sub(points[id],points[id+cols]));
  Require(before>0,"Spike fixture did not reproduce a path reversal");
  Require(FairMeridianReversals(points,cols,rows,faces.data(),unsigned(faces.size()),{})>0,"Spike was not faired");
  Require(Dot(Sub(points[id],points[id-cols]),Sub(points[id],points[id+cols]))<=0,"Isolated spike survived");
  Require(points.front()==seam&&points.back()==pole,"Fairing moved fixed attachments");
 }
 for(float size:{.6f,1.f,1.3f,.8f}){
  const Vec corners[]={{-size,-size,-1},{size,size,1}};
  std::vector<Hull> hulls={SupportHull(corners,2,normals,6,0)};auto points=reference;
  auto receipt=ClearMeridians(points,cols,rows,faces.data(),unsigned(faces.size()),hulls,{0,0,1},.04f,12,&cache);
  Require(receipt.minimumSeparation>=-1e-5f,"Reported penetration");
  for(unsigned i=0;i<cols;i++)Require(points[i]==reference[i],"Sewn boundary moved");Require(points.back()==reference.back(),"Pole moved");
  // Independently test full affine triangles against all six cube supports.
  for(Face face:faces){bool clear=false;for(const Plane& plane:hulls[0])if(Signed(plane,points[face[0]])>=-1e-5f&&Signed(plane,points[face[1]])>=-1e-5f&&Signed(plane,points[face[2]])>=-1e-5f)clear=true;Require(clear,"Edge or face crosses the solid");}
  hulls[0].support=[size](Vec n){return size*(std::abs(n[0])+std::abs(n[1]))+std::abs(n[2]);};
  std::vector<float> heights(cols*rows);for(unsigned i=0;i<heights.size();i++)heights[i]=1.f-float(i/cols)/rows;
  points=reference;std::vector<Vec> seed;float padding=0;
  WalkCertifiedTautEnvelope(points,cols,rows,heights.data(),faces.data(),unsigned(faces.size()),hulls,{0,0,1},.04f,&padding,&seed);
  Require(TautTriangleClearance(points,faces.data(),unsigned(faces.size()),hulls)>=-1e-5f,"Certified cover left a triangle penetration");
  Require(WithinMeridianSampling(points,seed,cols,rows),"Normal refinement exceeded seed sampling");
  for(unsigned i=0;i<cols;i++)Require(points[i]==reference[i],"Certified cover moved sewn anchors");Require(points.back()==reference.back(),"Certified cover moved terminal anchor");
  auto unchanged=points;RefineTautContacts(points,cols,rows,faces.data(),unsigned(faces.size()),hulls,{0,0,1});Require(points==unchanged,"Clear surface received unnecessary correction");
 }
 // A distant exact support must skip contact work, but moving that same
 // primitive over the garment on the next pose must be checked again.
 {const Vec farCorners[]={{99,-1,-1},{101,1,1}},nearCorners[]={{-10,-10,-10},{10,10,10}};
  Hull distant=SupportHull(farCorners,2,normals,6,0);
  distant.support=[](Vec n){return 100*n[0]+std::abs(n[0])+std::abs(n[1])+std::abs(n[2]);};
  auto points=reference;RefineTautContacts(points,cols,rows,faces.data(),unsigned(faces.size()),{distant},{0,0,1});
  Require(points==reference&&TautTriangleClearance(points,faces.data(),unsigned(faces.size()),{distant})>=0,"Distant contact changed the surface");
  distant=SupportHull(nearCorners,2,normals,6,0);
  distant.support=[](Vec n){return 10*(std::abs(n[0])+std::abs(n[1])+std::abs(n[2]));};
  bool rejected=false;try{RefineTautContacts(points,cols,rows,faces.data(),unsigned(faces.size()),{distant},{0,0,1});}catch(const std::exception&){rejected=true;}
  Require(rejected&&points==reference,"Moving contact reused stale distant bounds");}
 // An invalid anchored pose must leave the caller's last geometry untouched.
 const Vec enclosing[]={{-10,-10,-10},{10,10,10}};auto bad=reference;bool rejected=false;
 try{ClearMeridians(bad,cols,rows,faces.data(),unsigned(faces.size()),{SupportHull(enclosing,2,normals,6,0)},{0,0,1});}catch(const std::exception&){rejected=true;}
 Require(rejected&&bad==reference,"Failed wrapping was published");
 {auto enclosed=SupportHull(enclosing,2,normals,6,0);enclosed.support=[](Vec n){return 10*(std::abs(n[0])+std::abs(n[1])+std::abs(n[2]));};bad=reference;rejected=false;
  try{RefineTautContacts(bad,cols,rows,faces.data(),unsigned(faces.size()),{enclosed},{0,0,1});}catch(const std::exception&){rejected=true;}
  Require(rejected&&bad==reference,"Rejected normal contact was published");
 }
 // A narrow oblate support between the render rows used to be missed by the
 // coarse walk. Its contact bulge must survive the virtual walk and adaptive
 // resampling, without adding rows or moving either sewn anchor.
 {constexpr unsigned c=32,r=24;std::vector<Vec> points;
  for(unsigned row=0;row<r;row++)for(unsigned col=0;col<c;col++){
   float a=6.28318531f*col/c,t=float(row)/r;points.push_back({3*(1-t)*std::cos(a),3*(1-t)*std::sin(a),-4+8*t});
  }points.push_back({0,0,4});const auto original=points;Hull oblate;
  for(unsigned k=0;k<512;k++){float a=k*2.39996323f,z=1-2*(k+.5f)/512,q=std::sqrt(1-z*z);Vec n={q*std::cos(a),q*std::sin(a),z};
   oblate.push_back({n,.26f*n[2]+std::sqrt(4*(n[0]*n[0]+n[1]*n[1])+.0064f*n[2]*n[2])});
  }
  ClearMeridians(points,c,r,nullptr,0,{oblate},{0,0,1});
  Require(points.size()==original.size(),"Detailed walk increased the render mesh");
  for(unsigned col=0;col<c;col++){
   Require(points[col]==original[col],"Detailed walk moved the sewn outline");
   float atPeak=0;
   for(unsigned row=1;row<=r;row++){
    Vec a=points[(row-1)*c+col],b=row==r?points.back():points[row*c+col];
    if(a[2]<=.26f&&b[2]>=.26f){float t=(.26f-a[2])/(b[2]-a[2]);Vec p=Add(a,Mul(Sub(b,a),t));atPeak=std::sqrt(p[0]*p[0]+p[1]*p[1]);}
    Vec radial={b[0],b[1],0},longitude={std::cos(6.28318531f*col/c),std::sin(6.28318531f*col/c),0};
    Require(std::abs(Cross(radial,longitude)[2])<1e-4f,"Detailed walk left its longitude plane");
   }
   Require(atPeak>1.85f,"Sub-row curved support vanished from the taut path");
  }Require(points.back()==original.back(),"Detailed walk moved the pole");
 }
 std::puts("PASS: changing obstacles, whole-face separation, fixed anchors, verified cache and transactional rejection");return 0;
 }catch(const std::exception& e){std::printf("FAIL: %s\n",e.what());return 1;}}
