#include <malemod/garments/jockstrap.hpp>
#include <iostream>
using namespace malemod::garments;
static void Check(bool c,const char* text){if(!c)throw std::runtime_error(text);}
static Input Fixture(){Input in;const unsigned count=128;const double pi=3.14159265358979323846;
 for(unsigned row=0;row<33;row++){double z=.3+.05*row,r=1+.14*(z-1);
  for(unsigned i=0;i<count;i++){double theta=2*pi*i/count;Sample s;s.position={r*std::sin(theta),.72*r*std::cos(theta),z};s.normal=Unit({std::sin(theta),std::cos(theta)/.72,-.14});s.lineage.donors[0]={Surface::Body,unsigned(in.bodySurface.size()),1};in.bodySurface.push_back(s);if(row==14)in.waist.push_back(s);}
 }
 for(unsigned row=0;row<32;row++)for(unsigned i=0;i<count;i++){unsigned j=(i+1)%count,a=row*count+i,b=row*count+j,c=b+count,d=a+count;in.bodyTriangles.push_back({a,b,c});in.bodyTriangles.push_back({a,c,d});}
 return in;}
static double Circumference(const Input& in){double c=0;for(unsigned i=0;i<in.waist.size();i++)c+=Length(Sub(in.waist[i].position,in.waist[(i+1)%in.waist.size()].position));return c;}
static detail::BandRest Build(const Input& in,Output& out){out.measuredCircumference=Circumference(in);return detail::BuildBand(in,Parameters{},out);}
static void Audit(const Input& in,const Output& out,const detail::BandRest& band){
 const double width=Parameters{}.bandWidth*out.measuredCircumference;
 const auto guide=band_seating::Guide(in,width);
 Check(band.measured.size()==7,"Seated band lost material stripe rows");
 for(const auto& row:band.measured){double lo=1e100,hi=-1e100;
  for(auto sample:row){auto local=in.frame.Local(sample.position);lo=(std::min)(lo,local[2]);hi=(std::max)(hi,local[2]);Point reconstructed{};double sum=0;
   for(auto d:sample.lineage.donors)if(d.weight){Check(d.surface==Surface::Body&&d.vertex<in.bodySurface.size()&&d.weight>=0,"Seated band invented or discarded body donors");sum+=d.weight;reconstructed=Add(reconstructed,Mul(in.bodySurface[d.vertex].position,d.weight));}
   Check(std::abs(sum-1)<1e-10&&Length(Sub(reconstructed,sample.position))<width*1e-9,"Seated cut does not reconstruct exact measured skin");
  }
  Check(hi-lo>width*.35,"Seated band remains a level horizontal cut");
 }
 // Rows can have quite different perimeters when a large native pelvic graft
 // reaches the lower edge. They must nevertheless share one material column
 // correspondence; independently restarting at each forward-most vertex folds
 // the finite-thickness strip even while its individual skin cuts are exact.
 for(unsigned col=0;col<band.measured[0].size();col++){
  auto first=Sub(in.frame.Local(band.measured[0][col].position),guide.center);
  for(const auto& row:band.measured){auto p=Sub(in.frame.Local(row[col].position),guide.center);
   Check(std::abs(first[0]*p[1]-first[1]*p[0])<out.measuredCircumference*out.measuredCircumference*1e-10&&first[0]*p[0]+first[1]*p[1]>0,"Measured stripe rows twist their material columns");
  }
 }
 // Finite cloth follows the complete measured skin normal. Its upward/downward
 // component must survive a sloped recruitment ramp, rather than flattening the
 // layer separation into the horizontal pelvis plane.
 const unsigned count=unsigned(band.measured.front().size()),columns=count+1,layer=7*columns;
 Check(count>=(std::max)(96u,unsigned(in.waist.size())),"Native route count limited the band silhouette resolution");
 for(unsigned row=0;row<7;row++)for(unsigned col=0;col<count;col++){
  auto separation=Sub(out.mesh.vertices[row*columns+col].position,out.mesh.vertices[layer+row*columns+col].position);
  auto expected=Mul(Unit(band.measured[row][col].normal),Parameters{}.bandThickness*out.measuredCircumference);
  Check(Length(Sub(separation,expected))<out.measuredCircumference*1e-10,"Band thickness flattened the measured pelvic slope");
  Check(Length(Sub(band.measured[row][col].normal,band.measured[0][col].normal))<1e-12,"Stripe rows independently twist their thickness director");
  auto local=Sub(in.frame.Local(band.measured[row][col].position),guide.center),normal=in.frame.Local(Add(in.frame.origin,band.measured[row][col].normal));
  Check(std::abs(local[0]*normal[1]-local[1]*normal[0])<out.measuredCircumference*1e-10,"Finite layer shifts its material angular correspondence");
 }
 const auto& row=band.measured[3];unsigned front=0,hip=0;
 for(unsigned i=0;i<row.size();i++){if(in.frame.Local(row[i].position)[1]>in.frame.Local(row[front].position)[1])front=i;if(in.frame.Local(row[i].position)[0]>in.frame.Local(row[hip].position)[0])hip=i;}
 Check(in.frame.Local(row[hip].position)[2]-in.frame.Local(row[front].position)[2]>width*.4,"Band does not dip at front and rise smoothly at hip");
 // Compare corresponding angular locations, independent of each edge's
 // different circumference and arc-length sampling phase.
 for(unsigned i=0;i<row.size();i++){auto p=in.frame.Local(row[i].position);double angle=std::atan2(p[0],p[1]);std::array<Point,2> ends;
  for(unsigned edge=0;edge<2;edge++){const auto& samples=band.measured[edge?6:0];double best=1e100;for(auto sample:samples){auto q=in.frame.Local(sample.position);double delta=std::abs(std::atan2(std::sin(std::atan2(q[0],q[1])-angle),std::cos(std::atan2(q[0],q[1])-angle)));if(delta<best){best=delta;ends[edge]=q;}}}
  double span=Length(Sub(ends[1],ends[0]));Check(span>width*.9&&span<width*1.1,"Curved band narrows or flares instead of preserving fabric width");
 }
}
int main(){try{auto in=Fixture();auto fullWaist=in.waist;in.waist.clear();for(unsigned i=0;i<fullWaist.size();i+=4)in.waist.push_back(fullWaist[i]);Output out;auto band=Build(in,out);Audit(in,out,band);
 auto moved=in;moved.frame.origin={870,-932,512};moved.frame.lateral={0,-1,0};moved.frame.forward={0,0,-1};moved.frame.up={1,0,0};
 for(auto* samples:{&moved.waist,&moved.bodySurface})for(auto& s:*samples){s.position=moved.frame.World(Mul(s.position,100));s.normal=Add(Add(Mul(moved.frame.lateral,s.normal[0]),Mul(moved.frame.forward,s.normal[1])),Mul(moved.frame.up,s.normal[2]));}
 Output transformed;auto second=Build(moved,transformed);Audit(moved,transformed,second);
 for(unsigned row=0;row<7;row++)for(unsigned i=0;i<band.measured[row].size();i++)Check(Length(Sub(second.measured[row][i].position,moved.frame.World(Mul(band.measured[row][i].position,100))))<1e-7,"Seated skin attachments depend on source units or world origin");
 bool rejected=false;try{band_seating::Guide(in,0);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Invalid seating width not rejected");
 std::cout<<"PASS nonlevel front/hip waistband seating, exact skin donors, seven stripe rows, consistent surface width and rigid100x unit covariance\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
