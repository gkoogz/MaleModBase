#include <malemod/surface/wire.hpp>
#include <cstdio>
#include <limits>
using namespace malemod::surface;
int main(){
 wire::Request q;q.reset=true;q.frame.seconds=.01f;q.frame.pitchForce=.2f;q.frame.yawForce=-.3f;
 q.frame.thighEndpoints=std::array<Point,4>{{{1,2,3},{4,5,6},{7,8,9},{10,11,12}}};
 q.frame.collision=CollisionCalibration{{4,5},{{{1,2,3},{1,2,9}}},6};
 for(unsigned i=1;i<18;i++)q.controls.values[i]=float(i+1);
 auto bytes=wire::Encode(q);auto decoded=wire::DecodeRequest(bytes);
 if(wire::Encode(decoded)!=bytes)return 1;
 Output o{};Surface s;s.positions={{1,2,3},{2,3,4},{3,4,5}};s.normals={{0,0,1},{0,0,1},{0,0,1}};s.tangents={{1,0,0},{1,0,0},{1,0,0}};s.uv={{{0,0}},{{.5f,1}},{{1,0}}};s.sourceVertexIDs={10,11,12};
 o.anatomy=s;o.body={s,s};o.anatomyIndices={0,1,2};o.restLength=24;o.proximalRadius=3;o.rootDirection={1,0,0};
 o.collarMetric={{10,0,83},{1,0,0},{0,0,1},3,24,27};
 auto encoded=wire::Encode(o);if(wire::Encode(wire::DecodeOutput(encoded))!=encoded)return 2;
 auto expectReject=[](auto fn){try{fn();return false;}catch(const std::invalid_argument&){return true;}};
 auto truncated=encoded;truncated.pop_back();if(!expectReject([&]{wire::DecodeOutput(truncated);}))return 3;
 auto trailing=bytes;trailing.push_back(0);if(!expectReject([&]{wire::DecodeRequest(trailing);}))return 4;
 auto bad=bytes;bad[0]=wire::version+1;if(!expectReject([&]{wire::DecodeRequest(bad);}))return 5;
 q.controls.values[0]=.5f;if(!expectReject([&]{wire::Encode(q);}))return 6;
 q.controls.values[0]=2;q.controls.values[5]=std::numeric_limits<float>::quiet_NaN();if(!expectReject([&]{wire::Encode(q);}))return 7;
 // Reject a malicious count before allocating the advertised array.
 bad=encoded;bad[4]=bad[5]=bad[6]=bad[7]=255;if(!expectReject([&]{wire::DecodeOutput(bad);}))return 8;
 q.controls.values[5]=50;q.frame.collision->pelvisRadius=-1;if(!expectReject([&]{wire::Encode(q);}))return 9;
 q.frame.collision->pelvisRadius=6;q.frame.thighEndpoints.reset();if(!expectReject([&]{wire::Encode(q);}))return 10;
 s.positions[0].z=std::numeric_limits<float>::infinity();o.anatomy=s;if(!expectReject([&]{wire::Encode(o);}))return 11;
 std::puts("PASS lossless controls/frames/surfaces and malformed packet rejection");
}
