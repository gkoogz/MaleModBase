#pragma once
#include <malemod/garments/jockstrap.hpp>
#include <string>
namespace garment_shape {
using namespace malemod::garments;
struct Metrics {double bandStretch=0,sectionError=0,spacingRatio=0,uvStrain=0,strapEdge=0,sewnGap=0,bandSewnGap=0,bandSurfaceGap=0,hemSurfaceGap=0,minSignedDistance=1e100;};
inline double TriangleGap(std::array<Point,3> a,std::array<Point,3> b){double gap=1e100;for(auto p:a)gap=(std::min)(gap,Length(Sub(p,detail::ClosestTriangle(p,b[0],b[1],b[2]))));for(auto p:b)gap=(std::min)(gap,Length(Sub(p,detail::ClosestTriangle(p,a[0],a[1],a[2]))));for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++){Point p,q;detail::ClosestSegments(a[i],a[(i+1)%3],b[j],b[(j+1)%3],p,q);gap=(std::min)(gap,Length(Sub(p,q)));}return gap;}
inline Metrics Measure(const Input& in,const Output& out,Parameters p={}){
 Metrics x;double c=out.measuredCircumference;unsigned waist=unsigned(in.waist.size()),band=14*(waist+1),pouch=band,segments=p.pouchSegments,rings=p.pouchRings;
 const bool sheet=out.layout.revision==2;unsigned strap=sheet?out.layout.straps[0].start:band+1+rings*(segments+1)+(segments+1)*4+2*(segments+1)+9*25;
 auto need=[](bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);};need(out.mesh.vertices.size()>=strap,"Garment shape layout incomplete");
 // A stripe is a constant-height ring in the measured pelvic frame. Body seam
 // zigzags remain donor lineage rather than zigzags in the elastic material.
 for(unsigned layer=0;layer<2;layer++)for(unsigned row=0;row<7;row++){double z=in.frame.Local(out.mesh.vertices[layer*7*(waist+1)+row*(waist+1)].position)[2];for(unsigned k=1;k<=waist;k++)need(std::abs(in.frame.Local(out.mesh.vertices[layer*7*(waist+1)+row*(waist+1)+k].position)[2]-z)<c*1e-7,"Waistband stripe height ripple");}
 auto restingInput=in;restingInput.bodyContacts.clear();p.simulate=false;Session resting(p);const auto& templateCloth=resting.Update(Style::WhiteJockstrap,restingInput);
 detail::BodyCollider actualBody;actualBody.Update(in.bodySurface,in.bodyTriangles,in.frame.origin,c);
 for(unsigned k=0;k<waist;k++)for(unsigned row=0;row<7;row++){unsigned a=row*(waist+1)+k,b=a+1;double rest=Length(Sub(templateCloth.mesh.vertices[a].position,templateCloth.mesh.vertices[b].position));if(rest>c*1e-9)x.bandStretch=(std::max)(x.bandStretch,Length(Sub(out.mesh.vertices[a].position,out.mesh.vertices[b].position))/rest);}
 std::vector<Sample> rim;if(!sheet)for(unsigned k=0;k<segments;k++){const auto& v=out.mesh.vertices[pouch+1+(rings-1)*(segments+1)+k];rim.push_back({v.position,{0,0,1},v.lineage});}
 unsigned offset=strap,route=0;for(auto path:in.rearStraps){unsigned count=sheet?out.layout.straps[route].sections:(std::max)(33u,unsigned(path.size())*4+1);std::vector<Point> centers;double total=0,maximum=0;
  for(unsigned k=0;k<count;k++){Point center{};for(unsigned j=0;j<4;j++)center=Add(center,out.mesh.vertices.at(offset+4*k+j).position);centers.push_back(Mul(center,.25));
   for(auto pair:{std::array<unsigned,2>{0,1},std::array<unsigned,2>{2,3},std::array<unsigned,2>{0,3},std::array<unsigned,2>{1,2}}){double expected=pair[0]==0&&pair[1]==1||pair[0]==2&&pair[1]==3?p.strapWidth*c:p.bandThickness*c*.5;double measured=Length(Sub(out.mesh.vertices[offset+4*k+pair[0]].position,out.mesh.vertices[offset+4*k+pair[1]].position));x.sectionError=(std::max)(x.sectionError,std::abs(measured-expected));}
   if(k){double length=Length(Sub(centers[k],centers[k-1]));total+=length;maximum=(std::max)(maximum,length);double uv=out.mesh.vertices[offset+4*k].uv[0]-out.mesh.vertices[offset+4*(k-1)].uv[0];if(uv>1e-12)x.uvStrain=(std::max)(x.uvStrain,length/(uv*p.strapWidth*c));}
  }
  unsigned anchor=0;double closest=1e100;for(unsigned k=0;k<waist;k++){double distance=Length(Sub(out.mesh.vertices[k].position,path.front().position));if(distance<closest){closest=distance;anchor=k;}}x.bandSewnGap=(std::max)(x.bandSewnGap,Length(Sub(centers.front(),out.mesh.vertices[anchor].position)));
  double gap=1e100;unsigned hemFirst=sheet?out.layout.sheetFaces.start+(out.layout.sheet.rows-1)*out.layout.sheet.columns*2:28*waist+segments+(rings-1)*segments*2,hemEnd=sheet?out.layout.sheetFaces.start+out.layout.sheetFaces.count:hemFirst+segments*8;for(unsigned k=hemFirst;k<hemEnd;k++){auto f=out.mesh.triangles[k];std::array<Point,3> target{out.mesh.vertices[f.vertices[0]].position,out.mesh.vertices[f.vertices[1]].position,out.mesh.vertices[f.vertices[2]].position};for(auto ids:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{0,2,3}}){std::array<Point,3> section{};for(unsigned j=0;j<3;j++)section[j]=out.mesh.vertices[offset+(count-1)*4+ids[j]].position;gap=(std::min)(gap,TriangleGap(section,target));}}x.hemSurfaceGap=(std::max)(x.hemSurfaceGap,gap);
  double bandGap=1e100;for(unsigned k=0;k<28*waist;k++){auto f=out.mesh.triangles[k];std::array<Point,3> target{out.mesh.vertices[f.vertices[0]].position,out.mesh.vertices[f.vertices[1]].position,out.mesh.vertices[f.vertices[2]].position};for(auto ids:{std::array<unsigned,3>{0,1,2},std::array<unsigned,3>{0,2,3}}){std::array<Point,3> section{};for(unsigned j=0;j<3;j++)section[j]=out.mesh.vertices[offset+ids[j]].position;bandGap=(std::min)(bandGap,TriangleGap(section,target));}}x.bandSurfaceGap=(std::max)(x.bandSurfaceGap,bandGap);
  x.spacingRatio=(std::max)(x.spacingRatio,maximum/(total/(count-1)));
  if(sheet){double nearest=1e100;for(const auto& seam:out.layout.bottomSeams)for(unsigned id:seam)nearest=(std::min)(nearest,Length(Sub(centers.back(),out.mesh.vertices[id].position)));x.sewnGap=(std::max)(x.sewnGap,nearest);}else{double side=in.frame.Local(path.back().position)[0];auto sewn=detail::RingSample(rim,(side<0?4*detail::pi/3:5*detail::pi/3)/(2*detail::pi));x.sewnGap=(std::max)(x.sewnGap,Length(Sub(centers.back(),sewn.position)));}offset+=count*4;route++;
 }
 need(offset==out.mesh.vertices.size(),"Unexpected ribbon topology");
 for(auto triangle:out.mesh.triangles){for(unsigned k=0;k<3;k++){unsigned a=triangle.vertices[k],b=triangle.vertices[(k+1)%3];if(a>=strap&&b>=strap)x.strapEdge=(std::max)(x.strapEdge,Length(Sub(out.mesh.vertices[a].position,out.mesh.vertices[b].position)));}if(!actualBody.Empty()){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=Mul(Sub(out.mesh.vertices[triangle.vertices[k]].position,in.frame.origin),1/c);x.minSignedDistance=(std::min)(x.minSignedDistance,actualBody.ClosestFace(points,p.clearance*.3).signedDistance*c);}else for(auto volume:in.bodyContacts){Point a,b;double d=detail::TriangleCapsule(out.mesh.vertices[triangle.vertices[0]].position,out.mesh.vertices[triangle.vertices[1]].position,out.mesh.vertices[triangle.vertices[2]].position,volume,a,b,c*2);x.minSignedDistance=(std::min)(x.minSignedDistance,d);}}
 need(x.sewnGap<c*.04,"Strap endpoint detached from fitted pouch "+std::to_string(x.sewnGap/c));
 need(x.hemSurfaceGap<c*1e-6,"Pouch material sewing surface gap");need(x.bandSurfaceGap<c*1e-6,"Band material sewing surface gap");
 need(x.bandSewnGap<c*.04,"Strap endpoint detached from fitted waistband "+std::to_string(x.bandSewnGap/c));
 need(x.bandStretch<1.8,"Waist material longitudinal strain "+std::to_string(x.bandStretch));need(x.sectionError<c*1e-7,"Strap width/thickness distortion");need(x.spacingRatio<3,"Localized strap material recruitment "+std::to_string(x.spacingRatio));need(x.uvStrain<3,"Strap material UV strain "+std::to_string(x.uvStrain));return x;
}
}
