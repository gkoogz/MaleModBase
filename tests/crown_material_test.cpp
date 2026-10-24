#include <malemod/surface/crown_material.hpp>
#include <iostream>
#include <array>
struct V {float x=0,y=0,z=0;V operator+(V b)const{return {x+b.x,y+b.y,z+b.z};}V operator-(V b)const{return {x-b.x,y-b.y,z-b.z};}V operator*(float s)const{return {x*s,y*s,z*s};}V operator/(float s)const{return *this*(1/s);}};
float norm(V p){return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);}
int main(){try{
 constexpr unsigned rows=9,cols=24,first=3,count=first+rows*cols+1;
 std::array<V,count> reference{},flat{};
 for(unsigned r=0;r<rows;r++)for(unsigned c=0;c<cols;c++){
  float t=float(r)/(rows-1),a=6.283185307f*c/cols,radius=1.f+.5f*std::sin(3.141592654f*t);
  V p{t*3,radius*std::cos(a),radius*std::sin(a)};reference[first+r*cols+c]=p;
  // Flatten axial depth without changing the requested radius; rotate and
  // translate the whole fixture to make world-axis shortcuts fail.
  flat[first+r*cols+c]=V{8+2*p.z,3+2*p.y,5-.1f*p.x};
 }
 reference.back()={3.1f,0,0};flat.back()={8,3,4.69f};
 const auto original=flat;auto identity=flat;
 malemod::surface::RestoreCrownMaterial(identity.data(),reference.data(),count,first,rows,cols,3,V{0,0,-1},0);
 for(unsigned i=0;i<count;i++)if(norm(identity[i]-original[i])!=0)throw std::runtime_error("Zero blend changes original shape");
 malemod::surface::RestoreCrownMaterial(flat.data(),reference.data(),count,first,rows,cols,3,V{0,0,-1},1);
 for(unsigned i=0;i<first+cols;i++)if(norm(flat[i]-original[i])!=0)throw std::runtime_error("Root seam or external geometry changed");
 for(unsigned i=first+3*cols;i<count;i++){
  auto p=reference[i];V expected{8+2*p.z,3+2*p.y,5-2*p.x};
  if(norm(flat[i]-expected)>2e-5f)throw std::runtime_error("Free crown loses isotropic volume or control shape");
 }
 // A small blend must remain continuous and retain the exact seam.
 auto near=original;malemod::surface::RestoreCrownMaterial(near.data(),reference.data(),count,first,rows,cols,3,V{0,0,-1},1e-5f);
 for(unsigned i=0;i<count;i++)if(norm(near[i]-original[i])>.001f)throw std::runtime_error("Control transition jumps");
 bool rejected=false;try{malemod::surface::RestoreCrownMaterial(flat.data(),reference.data(),count,first,rows,cols,rows,V{1,0,0},1);}catch(const std::invalid_argument&){rejected=true;}
 if(!rejected)throw std::runtime_error("Invalid crown topology accepted");
 std::cout<<"PASS exact seam; restored isotropic crown under rotated frame; continuous control blend\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
