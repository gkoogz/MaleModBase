#pragma once
#include "jockstrap.hpp"
#include <istream>
#include <ostream>
// Exact SDK-free development input recording. Never distribute captured data.
// Scalar encoding is little-endian IEEE754; game adapters own file locations.
namespace malemod::garments::input_recording {
template<class IO> void Fields(IO& io,Input& v){
 io(v.frame.origin);io(v.frame.lateral);io(v.frame.forward);io(v.frame.up);
 io(v.characterEpoch);io(v.topologyRevision);io(v.restRevision);io(v.gravity);io(v.deltaTime);io(v.anatomyMass);
 auto samples=[&](std::vector<Sample>& rows){io.List(rows,[](auto& a,auto& s){a(s.position);a(s.normal);for(auto& d:s.lineage.donors){a(d.surface);a(d.vertex);a(d.weight);}});};
 samples(v.waist);samples(v.opening);samples(v.anatomy);samples(v.bodySurface);for(auto& route:v.rearStraps)samples(route);
 auto faces=[&](auto& rows){io.List(rows,[](auto& a,auto& f){a(f);});};faces(v.bodyTriangles);faces(v.anatomyTriangles);
 io.List(v.rootSubdivisions,[](auto& a,auto& r){a(r.vertex);a(r.a);a(r.b);a(r.t);});
 for(auto& region:v.anatomyRegions)io.List(region,[](auto& a,auto& id){a(id);});
 io.List(v.bodyContacts,[](auto& a,auto& c){a(c.a);a(c.b);a(c.radius);});
}
struct Writer {std::ostream& out;template<class T> void operator()(const T& v){out.write(reinterpret_cast<const char*>(&v),sizeof(v));if(!out)throw std::runtime_error("Garment recording write failed");}template<class T,class F> void List(std::vector<T>& rows,F f){std::uint32_t n=unsigned(rows.size());(*this)(n);for(auto& r:rows)f(*this,r);}};
struct Reader {std::istream& in;template<class T> void operator()(T& v){in.read(reinterpret_cast<char*>(&v),sizeof(v));if(!in)throw std::runtime_error("Incomplete garment recording");}template<class T,class F> void List(std::vector<T>& rows,F f){std::uint32_t n;(*this)(n);if(n>524288)throw std::runtime_error("Garment recording count limit");rows.resize(n);for(auto& r:rows)f(*this,r);}};
inline void Write(std::ostream& out,const Input& input){out.write("MMINPUT1",8);Writer writer{out};Fields(writer,const_cast<Input&>(input));}
inline Input Read(std::istream& in){char magic[8];in.read(magic,8);if(!in||std::string(magic,8)!="MMINPUT1")throw std::runtime_error("Garment recording version");Reader reader{in};Input result;Fields(reader,result);return result;}
}
