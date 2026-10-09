#pragma once
// SDK-free deformation for authored meridian meshes. No path search or cloth solver.
#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace malemod::garments::meridian {
using Vec=std::array<float,3>;
using Face=std::array<std::uint16_t,3>;
inline Vec Add(Vec a,Vec b){for(int k=0;k<3;k++)a[k]+=b[k];return a;}
inline Vec Sub(Vec a,Vec b){for(int k=0;k<3;k++)a[k]-=b[k];return a;}
inline Vec Mul(Vec a,float b){for(float& v:a)v*=b;return a;}
inline float Dot(Vec a,Vec b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Vec Cross(Vec a,Vec b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Vec Unit(Vec a){float l=std::sqrt(Dot(a,a));if(!std::isfinite(l)||l<1e-8f)throw std::runtime_error("Degenerate meridian donor frame");return Mul(a,1/l);}
struct Binding {std::uint32_t source[3];float weights[3];Vec offset;std::uint8_t surface;};
struct Vertex {Vec position,normal;std::array<float,2> uv;};
template<class Fetch> inline Vec Transport(const Binding& b,Fetch fetch){
 Vec p[3];for(unsigned j=0;j<3;j++)p[j]=fetch(b.surface,b.source[j]);
 Vec e=Unit(Sub(p[1],p[0])),n=Unit(Cross(Sub(p[1],p[0]),Sub(p[2],p[0]))),v=Cross(n,e),q{};
 for(unsigned j=0;j<3;j++)q=Add(q,Mul(p[j],b.weights[j]));
 q=Add(q,Add(Mul(e,b.offset[0]),Add(Mul(v,b.offset[1]),Mul(n,b.offset[2]))));
 for(float value:q)if(!std::isfinite(value))throw std::runtime_error("Nonfinite meridian position");
 return q;
}
class Mesh {
 std::vector<Vertex> vertices_;
 public:
 const std::vector<Vertex>& Vertices()const{return vertices_;}
 template<class Fetch> void Update(const Binding* bindings,unsigned count,const Face* faces,unsigned faceCount,Fetch fetch){
  vertices_.resize(count); // Capacity is reused after initialization.
  for(unsigned i=0;i<count;i++){
   const auto& b=bindings[i];Vec p[3];for(unsigned j=0;j<3;j++)p[j]=fetch(b.surface,b.source[j]);
   Vec e=Unit(Sub(p[1],p[0])),n=Unit(Cross(Sub(p[1],p[0]),Sub(p[2],p[0]))),v=Cross(n,e),q{};
   for(unsigned j=0;j<3;j++)q=Add(q,Mul(p[j],b.weights[j]));
   auto& out=vertices_[i];out.position=Add(q,Add(Mul(e,b.offset[0]),Add(Mul(v,b.offset[1]),Mul(n,b.offset[2]))));out.normal={};
   for(float value:out.position)if(!std::isfinite(value))throw std::runtime_error("Nonfinite meridian position");
  }
  for(unsigned i=0;i<faceCount;i++){
   auto f=faces[i];for(auto id:f)if(id>=count)throw std::runtime_error("Meridian face outside mesh");
   Vec n=Cross(Sub(vertices_[f[1]].position,vertices_[f[0]].position),Sub(vertices_[f[2]].position,vertices_[f[0]].position));
   for(auto id:f)vertices_[id].normal=Add(vertices_[id].normal,n);
  }
  for(auto& v:vertices_)v.normal=Unit(v.normal);
 }
};
}
