#pragma once
#include "canonical_order.hpp"
#include <algorithm>
#include <cmath>
namespace malemod::controls::menu {
// Wolverine gameplay menu, with Top and the unrequested bottoms omitted.
inline constexpr unsigned RowCount=21;
inline constexpr int Width=400,Height=700,CollapsedWidth=156,CollapsedHeight=34;
inline constexpr int TrackLeft=174,TrackRight=334,StatusY=622;
enum class Action { SetThrob,SetIdleChatter,SetGarment,ToggleTeaching,CancelTeaching,ResetDefaults,ToggleLighting };
struct State {
 unsigned throbMode=0;bool idleChatter=false;unsigned garment=0;
 bool teachingActive=false;float teachingSeconds=0;
 bool anatomyVisible=true;bool transformAvailable=true;
 std::array<float,18> displayControls{};bool displayAnimated=false;
 bool operator==(const State& b)const{return throbMode==b.throbMode&&idleChatter==b.idleChatter&&garment==b.garment&&teachingActive==b.teachingActive&&teachingSeconds==b.teachingSeconds&&anatomyVisible==b.anatomyVisible&&transformAvailable==b.transformAvailable&&displayAnimated==b.displayAnimated&&(!displayAnimated||displayControls==b.displayControls);}
 bool operator!=(const State& b)const{return !(*this==b);}
};
constexpr int CoreIndex(unsigned row){return row==0?0:row>=3&&row<20?int(row)-2:-1;}
constexpr unsigned MenuRow(unsigned core){return core?core+2:0;}
constexpr int RowY(unsigned row){return 60+24*int(row)+(row>=3?18:0)+(row>=12?18:0)+(row>=20?18:0);}
inline float AdjustCore(const std::array<float,18>& values,unsigned core,int direction,bool shift){
 if(core==0)return float((int(values[0])+direction+3)%3);
 const float low=core==2||core==4?0.f:1.f;
 return (std::max)(low,(std::min)(100.f,values[core]+direction*(shift?5.f:1.f)));
}
inline float SliderValue(unsigned core,int x){const float low=core==2||core==4?0.f:1.f;return std::round(low+(100-low)*(std::max)(0.f,(std::min)(1.f,float(x-TrackLeft)/(TrackRight-TrackLeft))));}
}
