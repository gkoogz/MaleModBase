#pragma once
namespace malemod::garments::trim_support {
// Reserve the last eighth for the mobile underside connection. Everything
// before it follows the measured body, including the complete glute crease.
inline unsigned LastSupportedSection(unsigned sections){
 if(sections<3)throw std::invalid_argument("Incomplete supported strap");
 return (sections-1)*7/8;
}
inline bool SupportedVertex(const MaterialLayout& layout,unsigned vertex){
 if(vertex<layout.bandLayers*(layout.band.rows+1)*(layout.band.columns+1))return true;
 for(const auto& strap:layout.straps)
  if(vertex>=strap.start&&vertex<strap.start+strap.sections*strap.corners)
   return vertex>=strap.start+strap.corners&&(vertex-strap.start)/strap.corners<=LastSupportedSection(strap.sections);
 return false;
}
}
