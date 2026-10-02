#include <malemod/surface/runtime.hpp>
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace malemod::surface;
float Difference(const Output& a,const Output& b){
 float error=0;
 for(unsigned i=0;i<a.anatomy.positions.size();i++){
  auto p=a.anatomy.positions[i],q=b.anatomy.positions[i];
  error=std::max(error,std::sqrt((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z)));
 }
 return error;
}
int main(){
 Session reference,calibrated,larger;
 Frame frame;frame.thighEndpoints=std::array<Point,4>{{{2,-7.8f,79},{1,-8.2f,43},{2,7.8f,79},{1,8.2f,43}}};
 frame.collision=CollisionCalibration{{7.2f,7.2f},{{{3,0,70},{5.4f,0,86}}},6.4f};
 Frame expanded=frame;expanded.collision->thighRadii={9,9};
 for(unsigned i=0;i<120;i++){reference.Step();calibrated.Step(frame);larger.Step(expanded);}
 auto baseline=reference.Read(),equivalent=calibrated.Read();
 const auto identity=Difference(baseline,equivalent),changed=Difference(baseline,larger.Read());
 if(identity>1e-4f||changed<.01f)return 1;
 Frame invalid=frame;invalid.thighEndpoints.reset();
 try{calibrated.Step(invalid);return 2;}catch(const std::invalid_argument&){}
 if(Difference(equivalent,calibrated.Read())!=0)return 3;
 // Exercise endpoint history across simulation substeps, including zero elapsed
 // time and a long frame. All returned geometry must remain finite.
 for(unsigned i=0;i<40;i++){
  frame.seconds=i==0?0:i==1?.1f:1.f/60.f;
  frame.collision->pelvisEndpoints[0].y=.1f*std::sin(float(i)*.2f);
  frame.collision->pelvisEndpoints[1].y=frame.collision->pelvisEndpoints[0].y;
  calibrated.Step(frame);calibrated.Read();
 }
 std::printf("PASS calibrated reference error %.9g; changed measured radius response %.9g; rejected input recovery and moving contacts\n",identity,changed);
}
