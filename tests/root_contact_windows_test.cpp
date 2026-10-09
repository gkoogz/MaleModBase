// Match adapters that include the real Win32 headers before the shared helper.
#include <windows.h>
#include <malemod/surface/root_contact.hpp>
#include <malemod/surface/tangent_root.hpp>
#ifndef max
#error The Win32 macro collision regression requires the actual max macro.
#endif
int main(){
 using namespace malemod::surface::root_contact;
 auto basis=malemod::surface::tangent_root::Jacobian({},2);
 auto mobility=malemod::surface::tangent_root::Mobility(basis,2);
 if(mobility[0][0]!=0)return 2;
 auto driven=malemod::surface::tangent_root::DrivenVelocity(basis,0,0,{1,0,0},1);if(driven!=Point{1,0,0})return 3;
 auto state=FromJoint({9,0,84.3},{10,0,84.3},{0,0,-.5},0);
 return std::abs(state.pitchVelocity-.5)>1e-12;
}
