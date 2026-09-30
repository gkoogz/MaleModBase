#pragma once
#include "math.hpp"
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace malemod {
struct Transform {
 V3 x{1,0,0},y{0,1,0},z{0,0,1},translation{};
 V3 Vector(V3 p)const{return x*p.x+y*p.y+z*p.z;}
 V3 Point(V3 p)const{return Vector(p)+translation;}
};
struct SkinBinding {std::array<std::uint16_t,4> joints{};std::array<float,4> weights{};};
// Matrices are current character-local joint transforms times inverse bind.
// This normalized-position path is shared; engine-packed buffers are adapters.
inline void SkinPositions(const std::vector<V3>& rest,const std::vector<SkinBinding>& bindings,const std::vector<Transform>& palette,std::vector<V3>& output){
 if(rest.size()!=bindings.size())throw std::invalid_argument("Skin binding count differs");
 for(const auto& b:bindings){float sum=0;for(unsigned k=0;k<4;k++){float w=b.weights[k];if(!std::isfinite(w)||w<0||(w>0&&b.joints[k]>=palette.size()))throw std::invalid_argument("Invalid skin binding");sum+=w;}if(std::abs(sum-1)>1e-4f)throw std::invalid_argument("Skin weights must sum to one");}
 output.resize(rest.size());for(size_t i=0;i<rest.size();i++){V3 p{};for(unsigned k=0;k<4;k++)if(bindings[i].weights[k]>0)p=p+palette[bindings[i].joints[k]].Point(rest[i])*bindings[i].weights[k];output[i]=p;}
}
struct AttachmentSocket {std::uint16_t joint=0;Transform local;};
inline Transform Compose(const Transform& a,const Transform& b){return {a.Vector(b.x),a.Vector(b.y),a.Vector(b.z),a.Point(b.translation)};}
inline Transform ResolveSocket(const AttachmentSocket& socket,const std::vector<Transform>& pose){if(socket.joint>=pose.size())throw std::invalid_argument("Socket joint not observed");return Compose(pose[socket.joint],socket.local);}
}
