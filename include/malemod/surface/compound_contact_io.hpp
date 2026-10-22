#pragma once
#include "compound_contact.hpp"
#include <fstream>
#include <filesystem>
namespace malemod::surface {
inline CompoundContactReference ReadCompoundContact(std::istream& in){
 auto read=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::runtime_error("Truncated compound reference");};
 char magic[8];read(magic,8);if(std::string(magic,8)!="MMCCNT02")throw std::runtime_error("Compound reference version differs");
 std::array<unsigned,7> n;read(n.data(),28);
 if(!n[0]||n[0]>500000||!n[1]||n[1]>1000000||!n[2]||n[2]>500000||!n[3]||n[3]>1000000||n[4]<3||n[4]>4096||n[5]<3||n[5]>50000||n[6]<3||n[6]>256)throw std::runtime_error("Invalid compound reference dimensions");
 const std::uint64_t required=std::uint64_t(n[0])*41+std::uint64_t(n[1])*13+std::uint64_t(n[2])*24+std::uint64_t(n[3])*12+std::uint64_t(n[4])*20+std::uint64_t(n[5])*56+std::uint64_t(n[6])*8+std::uint64_t(n[5])*n[6]*24;
 const auto begin=in.tellg();in.seekg(0,std::ios::end);const auto end=in.tellg();in.seekg(begin);
 if(begin<0||end<begin||required>64*1024*1024||std::uint64_t(end-begin)<required)throw std::runtime_error("Compound reference byte budget differs");
 CompoundContactReference r;r.support.resize(n[0]);r.supportFaces.resize(n[1]);r.chart.resize(n[0]);std::vector<unsigned char> eligible(n[1]);r.source.resize(n[2]);r.sourceFaces.resize(n[3]);r.sourceLoop.resize(n[4]);r.footprint.resize(n[4]);r.curve.resize(n[5]);r.parameters.resize(n[6]);r.firstTangents.resize(n[5]);r.lastTangents.resize(n[5]);r.patch.resize(std::size_t(n[5])*n[6]);
 read(r.support.data(),n[0]*24);read(r.supportFaces.data(),n[1]*12);read(r.chart.data(),n[0]*16);read(eligible.data(),n[1]);r.eligible.assign(eligible.begin(),eligible.end());
 std::vector<unsigned char> protectedSupport(n[0]);read(protectedSupport.data(),n[0]);r.protectedSupport.assign(protectedSupport.begin(),protectedSupport.end());
 read(r.source.data(),n[2]*24);read(r.sourceFaces.data(),n[3]*12);read(r.sourceLoop.data(),n[4]*4);read(r.footprint.data(),n[4]*16);read(r.curve.data(),n[5]*8);read(r.parameters.data(),n[6]*8);read(r.firstTangents.data(),n[5]*24);read(r.lastTangents.data(),n[5]*24);read(r.patch.data(),r.patch.size()*24);
 return r;
}
inline CompoundContactReference ReadCompoundContact(const std::filesystem::path& path){
 if(std::filesystem::file_size(path)>64*1024*1024)throw std::invalid_argument("Oversize compound reference");
 std::ifstream in(path,std::ios::binary);auto r=ReadCompoundContact(in);
 if(in.peek()!=EOF)throw std::runtime_error("Trailing compound reference data");return r;
}
}
