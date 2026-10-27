#include <malemod/controls/canonical_menu.hpp>
#include <malemod/controls/menu_preferences.hpp>
#include <cassert>
#include <sstream>
int main(){
 namespace menu=malemod::controls::menu;
 using namespace malemod::controls;
 static_assert(menu::RowCount==21&&menu::Width==400&&menu::Height==700);
 static_assert(menu::RowY(0)==60&&menu::RowY(3)==150&&menu::RowY(12)==384&&menu::RowY(20)==594);
 for(unsigned core=0;core<18;core++)assert(menu::CoreIndex(menu::MenuRow(core))==int(core));
 assert(menu::CoreIndex(1)==-1&&menu::CoreIndex(2)==-1&&menu::CoreIndex(20)==-1);
 menu::State a,b;b.displayControls[1]=88;assert(a==b);b.displayAnimated=true;assert(a!=b);a=b;assert(a==b);a.displayControls[1]=87;assert(a!=b);
 MenuPreferences preferences;auto values=preferences.controls;
 assert(menu::AdjustCore(values,0,1,false)==0&&menu::AdjustCore(values,0,-1,true)==1);
 values[2]=0;values[4]=100;values[1]=1;values[10]=98;
 assert(menu::AdjustCore(values,2,-1,true)==0&&menu::AdjustCore(values,1,-1,true)==1);
 assert(menu::AdjustCore(values,4,1,true)==100&&menu::AdjustCore(values,10,1,true)==100);
 assert(menu::SliderValue(2,-100)==0&&menu::SliderValue(1,-100)==1&&menu::SliderValue(1,900)==100);
 preferences.throbMode=3;preferences.idleChatter=true;preferences.garment=1;
 std::stringstream saved;WritePreferences(saved,preferences);MenuPreferences restored;
 assert(ReadPreferences(saved,restored)&&restored.throbMode==3&&restored.idleChatter&&restored.garment==1);
 std::stringstream legacy;legacy<<"2\n";for(float v:preferences.controls)legacy<<v<<' ';legacy<<"\n1\n";
 assert(ReadPreferences(legacy,restored)&&restored.throbMode==0&&!restored.idleChatter&&restored.garment==1);
 for(const char* suffix:{"4 0","0 2","-1 0","3 1 extra"}){
  std::stringstream invalid;invalid<<"3\n";for(float v:preferences.controls)invalid<<v<<' ';invalid<<"\n0\n"<<suffix;
  MenuPreferences untouched=preferences;assert(!ReadPreferences(invalid,untouched));assert(untouched.throbMode==3&&untouched.idleChatter&&untouched.garment==1);
 }
}
