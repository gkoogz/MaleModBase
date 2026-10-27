#include <malemod/surface/normal_frame.hpp>
#include <cassert>
#include <limits>
using namespace malemod::surface::normal_frame;
static bool Near(Vector a,Vector b){float error=0;for(unsigned i=0;i<3;i++)error+=(a[i]-b[i])*(a[i]-b[i]);return error<1e-10f;}
static Vector Apply(Matrix m,Vector v){Vector q{};for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)q[i]+=m[i*3+j]*v[j];return q;}
int main(){
 const Vector normal{0,0,1},tangent{1,0,0};
 assert(Transfer(normal,normal,tangent,tangent)==tangent);
 assert(Near(Transfer(normal,{0,1,0},normal,tangent),{0,1,0}));
 assert(Near(Transfer(normal,{0,0,-1},normal,tangent),{0,0,-1}));
 assert(Near(Transfer(normal,{0,0,-1},tangent,tangent),tangent));
 for(unsigned i=0;i<100;i++){
  auto destination=Unit({std::sin(i*.131f),std::cos(i*.237f),.37f});
  auto movedNormal=Transfer(normal,destination,normal,tangent),movedTangent=Transfer(normal,destination,tangent,tangent);
  const auto matrix=Rotation(normal,destination);Vector rotated{};for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)rotated[a]+=matrix[a*3+b]*normal[b];assert(Near(rotated,destination));
  assert(Near(movedNormal,destination));assert(std::abs(Dot(movedNormal,movedTangent))<1e-5f);assert(std::abs(Dot(movedTangent,movedTangent)-1)<1e-5f);
 }
 bool rejected=false;try{Transfer({0,0,0},normal,tangent,tangent);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 const Matrix identity{1,0,0,0,1,0,0,0,1};
 assert(FrameRotation(normal,tangent,normal,tangent)==identity);
 const auto twist=FrameRotation(normal,tangent,normal,{0,1,0});
 assert(Near(Apply(twist,normal),normal));assert(Near(Apply(twist,tangent),{0,1,0}));assert(Near(Apply(twist,{0,1,0}),{-1,0,0}));
 // A rigid three-axis permutation must preserve arbitrary material directions.
 const auto rigid=FrameRotation(normal,tangent,{1,0,0},{0,1,0});
 assert(Near(Apply(rigid,normal),{1,0,0}));assert(Near(Apply(rigid,tangent),{0,1,0}));assert(Near(Apply(rigid,{0,1,0}),{0,0,1}));
 assert(Near(Apply(rigid,{.2f,.3f,.4f}),{.4f,.2f,.3f}));
 // Source packet quantization may add normal contamination and scale.
 assert(FrameRotation({0,0,2},{3,0,4},{0,0,7},{0,8,-3})==twist);
 assert(FrameRotation(normal,{0,0,0},{0,1,0},tangent)==Rotation(normal,{0,1,0}));
 assert(FrameRotation(normal,tangent,{0,1,0},{0,2,0})==Rotation(normal,{0,1,0}));
 const float nan=std::numeric_limits<float>::quiet_NaN();
 assert(FrameRotation(normal,{nan,0,0},{0,1,0},tangent)==Rotation(normal,{0,1,0}));
 assert(FrameRotation({.3f,.4f,.5f},{0,0,0},{.7f,.2f,.9f},tangent)==Rotation({.3f,.4f,.5f},{.7f,.2f,.9f}));
 rejected=false;try{FrameRotation({0,0,0},tangent,normal,tangent);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
}
