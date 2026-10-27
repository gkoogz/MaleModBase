#pragma once
#include "canonical_order.hpp"
#include <istream>
#include <ostream>
#include <iomanip>
namespace malemod::controls {
struct MenuPreferences {std::array<float,18> controls{2,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};unsigned garment=0;unsigned throbMode=0;bool idleChatter=false;};
inline bool ReadPreferences(std::istream& input,MenuPreferences& output){
 unsigned version=0;MenuPreferences parsed;
 if(!(input>>version)||(version!=2&&version!=3))return false;
 for(unsigned i=0;i<18;i++)if(!(input>>parsed.controls[i])||!ValidMenuValue(i,parsed.controls[i]))return false;
 if(!(input>>parsed.garment)||parsed.garment>1)return false;
 if(version==3){unsigned idle=0;if(!(input>>parsed.throbMode>>idle)||parsed.throbMode>3||idle>1)return false;parsed.idleChatter=idle!=0;}
 input>>std::ws;if(!input.eof())return false;output=parsed;return true;
}
inline void WritePreferences(std::ostream& output,const MenuPreferences& value){
 output<<3<<'\n'<<std::setprecision(9);
 for(float v:value.controls)output<<v<<' ';output<<'\n'<<value.garment<<'\n'<<value.throbMode<<' '<<unsigned(value.idleChatter)<<'\n';
}
}
