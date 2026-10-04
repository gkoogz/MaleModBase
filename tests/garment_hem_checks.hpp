#pragma once
#include <malemod/garments/jockstrap.hpp>
#include <string>
namespace malemod::garments::hem_checks {
struct Measurements {double maximumCenterOffset=0,maximumTurnRadians=0,maximumSectionError=0;};
inline Measurements Check(const Output& out,const Parameters& parameters={},bool smoothRest=false){
 auto require=[](bool value,const char* message){if(!value)throw std::runtime_error(message);};
 const auto& layout=out.layout;const auto& mesh=out.mesh;double C=out.measuredCircumference;
 require(layout.revision==2&&layout.sheet.rows>=2&&layout.sheet.columns>=2&&std::isfinite(C)&&C>0,"Missing actual walking-sheet hem layout");
 require(layout.topSeam.size()==layout.sheet.columns+1&&layout.bottomSeams[0].size()>=2&&layout.bottomSeams[1].size()>=2,"Hem attachment seams are incomplete");
 Measurements result;const double width=parameters.hemWidth*C,thickness=parameters.hemThickness*C,tolerance=C*1e-7;
 for(unsigned side=0;side<2;side++){
  const auto& boundary=layout.sideBoundary[side];const auto& hem=layout.sideHems[side];
  require(boundary.size()==layout.sheet.rows+1&&hem.sections==boundary.size()&&hem.corners==4,"Side hem omits a walked boundary section");
  require(boundary.front()==(side?layout.topSeam.back():layout.topSeam.front()),"Side hem does not begin at an end of the front waistband arc");
  require(boundary.back()==(side?layout.bottomSeams[side].back():layout.bottomSeams[side].front()),"Side hem does not meet its underside strap seam");
  require(hem.start>=layout.sheet.start+(layout.sheet.rows+1)*(layout.sheet.columns+1)&&hem.start+std::size_t(hem.sections)*4<=mesh.vertices.size(),"Hem range overlaps sheet or exceeds actual mesh");
  std::vector<Point> centers;
  for(unsigned section=0;section<hem.sections;section++){
   unsigned id=layout.sheet.start+section*(layout.sheet.columns+1)+(side?layout.sheet.columns:0);
   require(boundary[section]==id,"Side hem jumps across the material sheet");auto edge=mesh.vertices.at(id).position;Point center{};
   for(unsigned corner=0;corner<4;corner++){const auto& vertex=mesh.vertices[hem.start+section*4+corner];require(Finite(vertex.position)&&Finite(vertex.normal)&&Finite(vertex.tangent),"Nonfinite hem geometry or shading");center=Add(center,Mul(vertex.position,.25));}
   double offset=Length(Sub(center,edge));result.maximumCenterOffset=(std::max)(result.maximumCenterOffset,offset);
   require(offset<=parameters.bandThickness*C*.35+tolerance,"Side hem separated from its material boundary");
   for(unsigned corner=0;corner<4;corner++){double distance=Length(Sub(mesh.vertices[hem.start+section*4+corner].position,mesh.vertices[hem.start+section*4+(corner+1)%4].position));double expected=corner%2?thickness:width;result.maximumSectionError=(std::max)(result.maximumSectionError,std::abs(distance-expected));require(std::abs(distance-expected)<tolerance,"Hem cross section collapsed or stretched independently of fabric");}
   centers.push_back(center);
  }
  for(unsigned section=1;section<hem.sections;section++){
   require(Length(Sub(centers[section],centers[section-1]))>tolerance,"Hem material progression contains a collapsed segment");
   std::array<bool,8> connected{};
   for(const auto& triangle:mesh.triangles){if(triangle.material!=MaterialSlot::WhiteElastic)continue;bool previous=false,next=false,inRange=true;for(auto id:triangle.vertices){inRange=inRange&&id>=hem.start+(section-1)*4&&id<hem.start+(section+1)*4;previous=previous||(id>=hem.start+(section-1)*4&&id<hem.start+section*4);next=next||(id>=hem.start+section*4&&id<hem.start+(section+1)*4);}if(inRange&&previous&&next)for(auto id:triangle.vertices)connected[id-(hem.start+(section-1)*4)]=true;}
   for(bool value:connected)require(value,"Continuous side hem has an unsewn gap between sections");
  }
  for(unsigned section=1;section+1<centers.size();section++){auto a=Unit(Sub(centers[section],centers[section-1])),b=Unit(Sub(centers[section+1],centers[section]));double angle=std::acos(std::clamp(Dot(a,b),-1.,1.));result.maximumTurnRadians=(std::max)(result.maximumTurnRadians,angle);}
 }
 if(smoothRest)require(result.maximumTurnRadians<=3.14159265358979323846/3,"Rest side hem has a sharp kink rather than a smooth arc");
 return result;
}
}
