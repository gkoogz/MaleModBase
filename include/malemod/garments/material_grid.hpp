#pragma once
namespace malemod::garments::cloth_detail {
inline bool SheetParticle(unsigned row,unsigned col,unsigned rows){return true;}
inline std::vector<std::array<unsigned,3>> SheetTriangles(unsigned start,unsigned rows,unsigned cols){
 std::vector<std::array<unsigned,3>> faces;auto id=[&](unsigned r,unsigned c){return start+r*(cols+1)+c;};
 for(unsigned r=0;r<rows;r++)for(unsigned c=0;c<cols;c++){unsigned a=id(r,c),b=id(r,c+1),d=id(r+1,c+1),e=id(r+1,c);
  // Every corner triangle includes an interior material node. A diagonal
  // joining two boundary nodes across a reflex sewn corner creates a fixed
  // inverted triangle that no numerical interpolation can unfold.
  if((r==0&&c+1==cols)||(r+1==rows&&c==0)){faces.push_back({a,b,e});faces.push_back({b,d,e});}
  else{faces.push_back({a,b,d});faces.push_back({a,d,e});}
 }return faces;
}
}
