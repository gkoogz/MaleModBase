#pragma once
#include "runtime.hpp"
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>

// Versioned numerical messages. Transport, process handles, engine resources
// and graphics buffers belong to the adapter. Never transmit C++ object layouts.
namespace malemod::surface::wire {
constexpr std::uint32_t version=4;
constexpr std::size_t maximumBytes=16*1024*1024;
constexpr std::uint32_t maximumVertices=60000,maximumIndices=360000;
using Bytes=std::vector<std::uint8_t>;
static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Wire requires IEEE binary32");
inline bool LittleEndian(){const std::uint32_t x=1;return *reinterpret_cast<const std::uint8_t*>(&x)==1;}
struct Request {Controls controls;Frame frame;bool reset=false;};
struct Writer {
 Bytes bytes;
 void U32(std::uint32_t x){for(int i=0;i<4;i++)bytes.push_back(std::uint8_t(x>>(i*8)));}
 void Float(float x){if(!std::isfinite(x))throw std::invalid_argument("Non-finite wire value");std::uint32_t b;std::memcpy(&b,&x,4);U32(b);}
 void Double(double x){static_assert(sizeof(double)==8&&std::numeric_limits<double>::is_iec559);if(!std::isfinite(x))throw std::invalid_argument("Non-finite wire clock");std::uint64_t b;std::memcpy(&b,&x,8);U32(std::uint32_t(b));U32(std::uint32_t(b>>32));}
 void Point3(Point p){Float(p.x);Float(p.y);Float(p.z);}
 void UVs(const std::vector<std::array<float,2>>& uv){
  if(LittleEndian()&&sizeof(std::array<float,2>)==8){
   for(auto p:uv)if(!std::isfinite(p[0])||!std::isfinite(p[1]))throw std::invalid_argument("Non-finite wire value");
   const auto n=uv.size()*8;if(n>maximumBytes||bytes.size()>maximumBytes-n)throw std::invalid_argument("Oversize wire packet");
   const auto offset=bytes.size();bytes.resize(offset+n);if(n)std::memcpy(bytes.data()+offset,uv.data(),n);
  }else for(auto p:uv){Float(p[0]);Float(p[1]);}
 }
 void IDs(const std::vector<std::uint32_t>& ids){
  if(LittleEndian()){
   const auto n=ids.size()*4;if(n>maximumBytes||bytes.size()>maximumBytes-n)throw std::invalid_argument("Oversize wire packet");
   const auto offset=bytes.size();bytes.resize(offset+n);if(n)std::memcpy(bytes.data()+offset,ids.data(),n);
  }else for(auto id:ids)U32(id);
 }
 template<class T>void Points(const T& points){
  if(LittleEndian()&&sizeof(Point)==12){
   for(auto p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::invalid_argument("Non-finite wire value");
   const auto n=points.size()*12;if(n>maximumBytes||bytes.size()>maximumBytes-n)throw std::invalid_argument("Oversize wire packet");
   const auto offset=bytes.size();bytes.resize(offset+n);if(n)std::memcpy(bytes.data()+offset,points.data(),n);
  }else for(auto p:points)Point3(p);
 }
};
struct Reader {
 const Bytes& bytes;std::size_t offset=0;
 explicit Reader(const Bytes& b):bytes(b){if(b.size()>maximumBytes)throw std::invalid_argument("Oversize wire packet");}
 std::uint32_t U32(){if(bytes.size()-offset<4)throw std::invalid_argument("Truncated wire packet");std::uint32_t x=0;for(int i=0;i<4;i++)x|=std::uint32_t(bytes[offset++])<<(8*i);return x;}
 float Float(){auto b=U32();float x;std::memcpy(&x,&b,4);if(!std::isfinite(x))throw std::invalid_argument("Non-finite wire value");return x;}
 double Double(){std::uint64_t b=U32();b|=std::uint64_t(U32())<<32;double x;std::memcpy(&x,&b,8);if(!std::isfinite(x))throw std::invalid_argument("Non-finite wire clock");return x;}
 Point Point3(){Point p;p.x=Float();p.y=Float();p.z=Float();return p;}
 void UVs(std::vector<std::array<float,2>>& uv){
  if(LittleEndian()&&sizeof(std::array<float,2>)==8){
   const auto n=uv.size()*8;if(n>bytes.size()-offset)throw std::invalid_argument("Truncated wire packet");
   if(n)std::memcpy(uv.data(),bytes.data()+offset,n);offset+=n;
   for(auto p:uv)if(!std::isfinite(p[0])||!std::isfinite(p[1]))throw std::invalid_argument("Non-finite wire value");
  }else for(auto& p:uv){p[0]=Float();p[1]=Float();}
 }
 void IDs(std::vector<std::uint32_t>& ids){
  if(LittleEndian()){
   const auto n=ids.size()*4;if(n>bytes.size()-offset)throw std::invalid_argument("Truncated wire packet");
   if(n)std::memcpy(ids.data(),bytes.data()+offset,n);offset+=n;
  }else for(auto& id:ids)id=U32();
 }
 template<class T>void Points(T& points){
  if(LittleEndian()&&sizeof(Point)==12){
   const auto n=points.size()*12;if(n>bytes.size()-offset)throw std::invalid_argument("Truncated wire packet");
   if(n)std::memcpy(points.data(),bytes.data()+offset,n);offset+=n;
   for(auto p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))throw std::invalid_argument("Non-finite wire value");
  }else for(auto& p:points)p=Point3();
 }
 void End(){if(offset!=bytes.size())throw std::invalid_argument("Trailing wire data");}
};
inline void Validate(const Controls& c){
 for(unsigned i=0;i<c.values.size();i++){
  float x=c.values[i],lo=i==0||i==2||i==4?0.f:1.f,hi=i==0?2.f:100.f;
  if(!std::isfinite(x)||x<lo||x>hi||(i==0&&x!=std::floor(x)))throw std::invalid_argument("Invalid wire control");
 }
}
inline void Validate(const Frame& f){
 if(!std::isfinite(f.seconds)||f.seconds<0||f.seconds>.15f||!std::isfinite(f.pitchForce)||!std::isfinite(f.yawForce))throw std::invalid_argument("Invalid wire frame");
 auto finite=[](Point p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
 if(f.thighEndpoints)for(auto p:*f.thighEndpoints)if(!finite(p))throw std::invalid_argument("Invalid wire thigh endpoint");
 if(f.collarQueries.size()>20000)throw std::invalid_argument("Too many collar queries");
 for(auto p:f.collarQueries)if(!finite(p))throw std::invalid_argument("Invalid collar query");
 const auto& c=f.clinical;
 if(c.throbMode>3||!std::isfinite(c.time)||c.time<0||c.time>60||!std::isfinite(c.sizeTime)||c.sizeTime<0||c.sizeTime>=3||!std::isfinite(c.twitchTime)||c.twitchTime<0||c.twitchTime>=5.75f||!std::isfinite(c.lateralWobbleDegrees)||std::abs(c.lateralWobbleDegrees)>90)throw std::invalid_argument("Invalid clinical projection");
 for(float gain:c.lateralGain)if(!std::isfinite(gain)||std::abs(gain)>2)throw std::invalid_argument("Invalid clinical lateral variation");
 for(float gain:c.angleGain)if(!std::isfinite(gain)||gain<.05f||gain>2)throw std::invalid_argument("Invalid clinical pulse variation");
 if(f.collision){
  if(!f.thighEndpoints)throw std::invalid_argument("Collision calibration requires measured thigh endpoints");
  for(float r:f.collision->thighRadii)if(!std::isfinite(r)||r<=0)throw std::invalid_argument("Invalid wire thigh radius");
  for(auto p:f.collision->pelvisEndpoints)if(!finite(p))throw std::invalid_argument("Invalid wire pelvis endpoint");
  if(!std::isfinite(f.collision->pelvisRadius)||f.collision->pelvisRadius<=0)throw std::invalid_argument("Invalid wire pelvis radius");
 }
}
inline Bytes Encode(const Request& q){
 Validate(q.controls);Validate(q.frame);
 Writer w;w.U32(version);w.U32(q.reset?1:0);
 for(float x:q.controls.values)w.Float(x);
 w.Float(q.frame.seconds);w.Float(q.frame.pitchForce);w.Float(q.frame.yawForce);
 w.U32(q.frame.thighEndpoints?1:0);if(q.frame.thighEndpoints)w.Points(*q.frame.thighEndpoints);
 w.U32(q.frame.collision?1:0);if(q.frame.collision){for(float r:q.frame.collision->thighRadii)w.Float(r);w.Points(q.frame.collision->pelvisEndpoints);w.Float(q.frame.collision->pelvisRadius);}
 w.U32(std::uint32_t(q.frame.collarQueries.size()));w.Points(q.frame.collarQueries);
 const auto& c=q.frame.clinical;w.U32(c.active?1:0);w.Double(c.time);w.U32(c.throbMode);w.Float(c.sizeTime);w.Float(c.twitchTime);w.Float(c.lateralWobbleDegrees);for(float x:c.angleGain)w.Float(x);for(float x:c.lateralGain)w.Float(x);
 return w.bytes;
}
inline Request DecodeRequest(const Bytes& bytes){
 Reader r(bytes);if(r.U32()!=version)throw std::invalid_argument("Wire version mismatch");
 Request q;auto reset=r.U32();if(reset>1)throw std::invalid_argument("Invalid reset flag");q.reset=reset!=0;
 for(float& x:q.controls.values)x=r.Float();Validate(q.controls);
 q.frame.seconds=r.Float();q.frame.pitchForce=r.Float();q.frame.yawForce=r.Float();
 if(q.frame.seconds<0||q.frame.seconds>.15f)throw std::invalid_argument("Invalid wire timestep");
 auto thigh=r.U32();if(thigh>1)throw std::invalid_argument("Invalid thigh flag");
 if(thigh){q.frame.thighEndpoints=std::array<Point,4>{};r.Points(*q.frame.thighEndpoints);}
 auto calibrated=r.U32();if(calibrated>1)throw std::invalid_argument("Invalid collision flag");
 if(calibrated){q.frame.collision=CollisionCalibration{};for(float& radius:q.frame.collision->thighRadii)radius=r.Float();r.Points(q.frame.collision->pelvisEndpoints);q.frame.collision->pelvisRadius=r.Float();}
 auto queries=r.U32();if(queries>20000)throw std::invalid_argument("Too many collar queries");q.frame.collarQueries.resize(queries);r.Points(q.frame.collarQueries);
 auto active=r.U32();if(active>1)throw std::invalid_argument("Invalid clinical active flag");auto& c=q.frame.clinical;c.active=active;c.time=r.Double();c.throbMode=r.U32();c.sizeTime=r.Float();c.twitchTime=r.Float();c.lateralWobbleDegrees=r.Float();for(float& x:c.angleGain)x=r.Float();for(float& x:c.lateralGain)x=r.Float();
 Validate(q.frame);
 r.End();return q;
}
inline void WriteSurface(Writer& w,const Surface& s){
 auto n=s.positions.size();
 if(n>maximumVertices||s.normals.size()!=n||s.tangents.size()!=n||s.uv.size()!=n||s.sourceVertexIDs.size()!=n)throw std::invalid_argument("Invalid wire surface layout");
 w.U32(std::uint32_t(n));w.Points(s.positions);w.Points(s.normals);w.Points(s.tangents);
 w.UVs(s.uv);w.IDs(s.sourceVertexIDs);
}
inline Surface ReadSurface(Reader& r){
 auto n=r.U32();if(n>maximumVertices||std::size_t(n)*48>r.bytes.size()-r.offset)throw std::invalid_argument("Invalid wire vertex count");
 Surface s;s.positions.resize(n);s.normals.resize(n);s.tangents.resize(n);s.uv.resize(n);s.sourceVertexIDs.resize(n);
 r.Points(s.positions);r.Points(s.normals);r.Points(s.tangents);
 r.UVs(s.uv);r.IDs(s.sourceVertexIDs);return s;
}
inline Bytes Encode(const Output& o){
 auto capacity=512+(o.anatomy.positions.size()+o.body[0].positions.size()+o.body[1].positions.size())*48+o.anatomyIndices.size()*4;
 if(capacity>maximumBytes)throw std::invalid_argument("Oversize wire output");
 Writer w;w.bytes.reserve(capacity);w.U32(version);WriteSurface(w,o.anatomy);for(const auto& b:o.body)WriteSurface(w,b);
 if(o.anatomyIndices.size()>maximumIndices||o.anatomyIndices.size()%3)throw std::invalid_argument("Invalid wire topology");
 w.U32(std::uint32_t(o.anatomyIndices.size()));
 std::vector<std::uint32_t> indices;indices.reserve(o.anatomyIndices.size());
 for(auto i:o.anatomyIndices){if(i>=o.anatomy.positions.size())throw std::invalid_argument("Invalid wire triangle");indices.push_back(i);}w.IDs(indices);
 w.Float(o.proximalRadius);w.Float(o.restLength);w.Points(o.shaftGuide);w.Points(o.restGuide);w.Points(o.lobeCenters);w.Points(o.lobeAnchors);w.Points(o.lobeRadii);
 for(const auto& axes:o.lobeAxes)w.Points(axes);w.Point3(o.rootDirection);for(float x:o.bendMultipliers)w.Float(x);
 w.Point3(o.collarMetric.root);w.Point3(o.collarMetric.axis);w.Point3(o.collarMetric.up);
 w.Float(o.collarMetric.radius);w.Float(o.collarMetric.length);w.U32(o.collarMetric.generation);
 if(o.collarDisplacements.size()>20000)throw std::invalid_argument("Too many collar targets");w.U32(std::uint32_t(o.collarDisplacements.size()));w.Points(o.collarDisplacements);
 w.Point3(o.nozzlePosition);w.Point3(o.nozzleDirection);
 if(w.bytes.size()>maximumBytes)throw std::invalid_argument("Oversize wire output");return w.bytes;
}
inline Output DecodeOutput(const Bytes& bytes){
 Reader r(bytes);if(r.U32()!=version)throw std::invalid_argument("Wire version mismatch");
 Output o;o.anatomy=ReadSurface(r);for(auto& b:o.body)b=ReadSurface(r);
 auto n=r.U32();if(n>maximumIndices||n%3||std::size_t(n)*4>r.bytes.size()-r.offset)throw std::invalid_argument("Invalid wire index count");
 o.anatomyIndices.resize(n);for(auto& i:o.anatomyIndices){auto id=r.U32();if(id>=o.anatomy.positions.size()||id>65535)throw std::invalid_argument("Invalid wire triangle");i=std::uint16_t(id);}
 o.proximalRadius=r.Float();o.restLength=r.Float();r.Points(o.shaftGuide);r.Points(o.restGuide);r.Points(o.lobeCenters);r.Points(o.lobeAnchors);r.Points(o.lobeRadii);
 for(auto& axes:o.lobeAxes)r.Points(axes);o.rootDirection=r.Point3();for(float& x:o.bendMultipliers)x=r.Float();
 o.collarMetric.root=r.Point3();o.collarMetric.axis=r.Point3();o.collarMetric.up=r.Point3();
 o.collarMetric.radius=r.Float();o.collarMetric.length=r.Float();o.collarMetric.generation=r.U32();
 auto targets=r.U32();if(targets>20000)throw std::invalid_argument("Too many collar targets");o.collarDisplacements.resize(targets);r.Points(o.collarDisplacements);o.nozzlePosition=r.Point3();o.nozzleDirection=r.Point3();r.End();return o;
}
}
