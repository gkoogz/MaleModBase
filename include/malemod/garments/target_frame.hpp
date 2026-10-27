#pragma once
#include <malemod/garments/meridian_runtime.hpp>
namespace malemod::garments::meridian {
// Character retargeting only: transport a shell offset using the target's
// current smooth skin frame. Preserve the actual triangle/barycentric anchor.
// The authored source's flat-facet Transport remains unchanged.
template<class Position,class Normal> inline Vec TransportSmooth(const Binding& b,Position position,Normal normal){
 Vec p[3],q{},n{};
 for(unsigned k=0;k<3;k++){p[k]=position(b.surface,b.source[k]);q=Add(q,Mul(p[k],b.weights[k]));n=Add(n,Mul(normal(b.source[k]),b.weights[k]));}
 n=Unit(n);auto edge=Sub(p[1],p[0]);auto tangent=Unit(Sub(edge,Mul(n,Dot(edge,n))));auto bitangent=Cross(n,tangent);
 q=Add(q,Add(Mul(tangent,b.offset[0]),Add(Mul(bitangent,b.offset[1]),Mul(n,b.offset[2]))));
 for(float value:q)if(!std::isfinite(value))throw std::runtime_error("Nonfinite target garment frame");return q;
}
}
