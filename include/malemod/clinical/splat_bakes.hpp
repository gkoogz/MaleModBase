#pragma once
#include "common.hpp"
#include <fstream>
#include <iterator>

namespace malemod::clinical::volumeFluid {
// Immutable baked data may be shared; each SplatModel owns its deposition state.
struct SplatBakes {
 std::shared_ptr<const std::vector<std::uint16_t>> pixels;
 int width=0,frames[2]{};
 bool Open() const { return bool(pixels); }
 bool Load(const std::vector<std::uint8_t>& bytes) {
  auto word=[&](size_t i){return std::uint32_t(bytes[i])|(std::uint32_t(bytes[i+1])<<8)|(std::uint32_t(bytes[i+2])<<16)|(std::uint32_t(bytes[i+3])<<24);};
  // Validate the complete original SPK1 format before replacing existing data.
  if(bytes.size()!=16+192u*192u*90u*2u || word(0)!=0x314B5053 || word(4)!=192 || word(8)!=26 || word(12)!=64) return false;
  auto decoded=std::make_shared<std::vector<std::uint16_t>>();decoded->reserve((bytes.size()-16)/2);
  for(size_t i=16;i<bytes.size();i+=2) decoded->push_back(std::uint16_t(bytes[i])|(std::uint16_t(bytes[i+1])<<8));
  pixels=decoded;width=192;frames[0]=26;frames[1]=64;return true;
 }
 bool LoadFile(const std::string& path) {
  std::ifstream input(path,std::ios::binary);if(!input)return false;
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),{});
  return Load(bytes);
 }
 float Sample(int kind,int frame,float u,float v) const {
  if(!pixels||kind<0||kind>1||frame<0||frame>=frames[kind]||!std::isfinite(u)||!std::isfinite(v)||u<0||v<0||u>width-1||v>width-1)return 0;
  int x=int(u),y=int(v),xx=min(width-1,x+1),yy=min(width-1,y+1);float fx=u-x,fy=v-y;
  const auto p=pixels->data()+((kind?frames[0]:0)+frame)*width*width;
  return ((p[y*width+x]*(1-fx)+p[y*width+xx]*fx)*(1-fy)+(p[yy*width+x]*(1-fx)+p[yy*width+xx]*fx)*fy)/65535.f;
 }
};
}
