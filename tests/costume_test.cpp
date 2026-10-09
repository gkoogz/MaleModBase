#include <malemod/controls/costume.hpp>
#include <cassert>
using namespace malemod::controls;
int main(){
 for(int top=0;top<2;top++)for(int bottom=0;bottom<4;bottom++) {
  auto c=Costume::Load(top,bottom,1-bottom);
  assert(unsigned(c.top)==unsigned(top)&&unsigned(c.bottom)==unsigned(bottom));
 }
 auto legacy=Costume::Load(-1,-1,1);assert(legacy.top==Top::Naked&&legacy.bottom==Bottom::Jockstrap);
 auto invalid=Costume::Load(55,9,1);assert(invalid.top==Top::Naked&&invalid.bottom==Bottom::Naked);
}
