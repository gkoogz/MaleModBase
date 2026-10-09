// Match adapters that include the real Win32 headers before the shared helper.
#include <windows.h>
#include <malemod/surface/root_contact.hpp>
#ifndef max
#error The Win32 macro collision regression requires the actual max macro.
#endif
int main(){
 using namespace malemod::surface::root_contact;
 auto state=FromJoint({9,0,84.3},{10,0,84.3},{0,0,-.5},0);
 return std::abs(state.pitchVelocity-.5)>1e-12;
}
