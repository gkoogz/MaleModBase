#pragma once
#include <cmath>

namespace malemod {
struct V3 { float x, y, z; };
inline V3 operator+(V3 a, V3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x*s,a.y*s,a.z*s}; }
inline V3 operator/(V3 a, float s) { return {a.x/s,a.y/s,a.z/s}; }
inline float Dot(V3 a, V3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline V3 Cross(V3 a, V3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline float Length(V3 a) { return std::sqrt(Dot(a,a)); }
inline V3 Unit(V3 a) { float n=Length(a); return n>1e-6f ? a/n : V3{1,0,0}; }
}
