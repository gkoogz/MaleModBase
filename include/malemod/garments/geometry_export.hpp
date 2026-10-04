#pragma once
// Shared diagnostic/source export. No game buffer layout or graphics SDK.
#include "jockstrap.hpp"
#include <ostream>
#include <iomanip>
namespace malemod::garments::geometry_export {
inline void WriteOBJ(std::ostream& stream,const Mesh& mesh){
 stream<<std::setprecision(17);
 for(const auto& v:mesh.vertices){if(!Finite(v.position)||!Finite(v.normal)||!std::isfinite(v.uv[0])||!std::isfinite(v.uv[1]))throw std::invalid_argument("Nonfinite garment export vertex");stream<<"v "<<v.position[0]<<' '<<v.position[1]<<' '<<v.position[2]<<'\n';}
 for(const auto& v:mesh.vertices)stream<<"vt "<<v.uv[0]<<' '<<v.uv[1]<<'\n';
 for(const auto& v:mesh.vertices)stream<<"vn "<<v.normal[0]<<' '<<v.normal[1]<<' '<<v.normal[2]<<'\n';
 for(auto t:mesh.triangles){for(auto id:t.vertices)if(id>=mesh.vertices.size())throw std::invalid_argument("Garment export face exceeds actual vertices");stream<<"g material"<<unsigned(t.material)<<"\nf";for(auto id:t.vertices){auto index=std::uint64_t(id)+1;stream<<' '<<index<<'/'<<index<<'/'<<index;}stream<<'\n';}
 if(!stream)throw std::runtime_error("Cannot export complete garment OBJ");
}
inline void WriteLayout(std::ostream& stream,const MaterialLayout& layout){
 if(layout.revision<2||layout.jointRevision!=1||!std::isfinite(layout.upperArcRadians)||!std::isfinite(layout.measuredCircumference)||layout.measuredCircumference<=0||!std::isfinite(layout.bandThicknessNormalized)||layout.bandThicknessNormalized<=0)throw std::invalid_argument("Garment layout lacks authoritative material/joint contract");
 if(!std::isfinite(layout.sideCoverageExtra)||layout.sideCoverageExtra<0||layout.sideCoverageExtra>1.5)throw std::invalid_argument("Invalid authored side coverage refinement");
 if(!std::isfinite(layout.sideCoverageAspect)||layout.sideCoverageAspect<0||!std::isfinite(layout.sideCoverageEffective)||layout.sideCoverageEffective<0||layout.sideCoverageEffective>layout.sideCoverageExtra)throw std::invalid_argument("Invalid measured side coverage activation");
 stream<<std::setprecision(17);
 auto indices=[&](const auto& values){stream<<'[';for(unsigned k=0;k<values.size();k++)stream<<(k?",":"")<<values[k];stream<<']';};
 auto ribbon=[&](const RibbonLayout& r){stream<<"{\"start\":"<<r.start<<",\"sections\":"<<r.sections<<",\"corners\":"<<r.corners<<'}';};
 auto text=[&](const std::string& value){stream<<'"';for(unsigned char c:value){if(c=='"'||c=='\\')stream<<'\\'<<c;else if(c<32)throw std::invalid_argument("Invalid control character in authored joint name");else stream<<c;}stream<<'"';};
 auto point=[&](const MaterialPoint& p){if(p.vertices.empty()||p.vertices.size()>4||p.vertices.size()!=p.weights.size())throw std::invalid_argument("Invalid exported material joint binding");double sum=0;for(auto w:p.weights){if(!std::isfinite(w)||w<0)throw std::invalid_argument("Invalid exported joint weight");sum+=w;}if(std::abs(sum-1)>1e-9)throw std::invalid_argument("Exported joint weights do not sum to one");stream<<"{\"vertices\":";indices(p.vertices);stream<<",\"weights\":";indices(p.weights);stream<<'}';};
 stream<<"{\"revision\":"<<layout.revision<<",\"jointRevision\":"<<layout.jointRevision<<",\"upperArcRadians\":"<<layout.upperArcRadians<<",\"measuredCircumference\":"<<layout.measuredCircumference<<",\"bandThicknessNormalized\":"<<layout.bandThicknessNormalized;
 stream<<",\"sideCoverageExtra\":"<<layout.sideCoverageExtra<<",\"sideCoverageAspect\":"<<layout.sideCoverageAspect<<",\"sideCoverageEffective\":"<<layout.sideCoverageEffective;
 stream<<",\"band\":{\"start\":"<<layout.band.start<<",\"rows\":"<<layout.band.rows<<",\"columns\":"<<layout.band.columns<<",\"layers\":"<<layout.bandLayers<<"},\"sheet\":{\"start\":"<<layout.sheet.start<<",\"rows\":"<<layout.sheet.rows<<",\"columns\":"<<layout.sheet.columns<<",\"faceStart\":"<<layout.sheetFaces.start<<",\"faceCount\":"<<layout.sheetFaces.count<<"},\"topSeam\":";indices(layout.topSeam);
 stream<<",\"bottomSeams\":[";for(unsigned side=0;side<2;side++){if(side)stream<<',';indices(layout.bottomSeams[side]);}stream<<"],\"straps\":[";for(unsigned side=0;side<2;side++){if(side)stream<<',';ribbon(layout.straps[side]);}stream<<"],\"sideHems\":[";for(unsigned side=0;side<2;side++){if(side)stream<<',';ribbon(layout.sideHems[side]);}stream<<"],\"sideBoundary\":[";for(unsigned side=0;side<2;side++){if(side)stream<<',';indices(layout.sideBoundary[side]);}stream<<"],\"authoredJoints\":[";
 for(unsigned k=0;k<layout.authoredJoints.size();k++){const auto& joint=layout.authoredJoints[k];if(!Finite(joint.restOffset))throw std::invalid_argument("Nonfinite authored seam export offset");if(k)stream<<',';stream<<"{\"name\":";text(joint.name);stream<<",\"a\":";point(joint.a);stream<<",\"b\":";point(joint.b);stream<<",\"restOffset\":["<<joint.restOffset[0]<<','<<joint.restOffset[1]<<','<<joint.restOffset[2]<<"],\"offsetProvenance\":\"authored-thickness-contract\"}";}
 stream<<"]}\n";if(!stream)throw std::runtime_error("Cannot export complete garment material layout");
}
}
