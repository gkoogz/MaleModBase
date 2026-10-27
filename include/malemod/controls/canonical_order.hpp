#pragma once
#include <array>
#include <cmath>
namespace malemod::controls {
// Public menu: state, overall, length, width, glans, scrotum, hang, angle,
// forward, vertical, eight physics controls. Canonical Wolverine keeps the
// seven original sliders first and the later independent glans/hang last.
inline std::array<float,18> CanonicalPacketOrder(const std::array<float,18>& menu){
 return {menu[0],menu[1],menu[2],menu[3],menu[5],menu[7],menu[8],menu[9],menu[4],menu[6],
         menu[10],menu[11],menu[12],menu[13],menu[14],menu[15],menu[16],menu[17]};
}
inline bool ValidMenuValue(unsigned index,float value){
 if(index>=18||!std::isfinite(value))return false;
 if(!index)return value>=0&&value<=2&&value==int(value);
 return value>=(index==2||index==4?0.f:1.f)&&value<=100;
}
}
