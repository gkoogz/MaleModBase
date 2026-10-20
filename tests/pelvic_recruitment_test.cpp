#include <malemod/surface/pelvic_recruitment.hpp>
#include <iostream>
using namespace malemod::surface;
using namespace malemod::surface::root_motion;
void Check(bool x,const char* why){if(!x)throw std::runtime_error(why);}
double Norm(PrecisePoint p){return std::sqrt(Dot(p,p));}
int main(){try{
 std::vector<PrecisePoint> opening={{0,0,2},{0,2,0},{0,0,-2},{0,-2,0}};
 std::vector<PrecisePoint> rest=opening;rest.insert(rest.end(),{{0,0,10},{0,0,100},{-100,0,3},{10,0,3}});
 PelvicRecruitment field(rest,opening);GraftFrame f{{0,0,0},{1,0,0},{0,0,1},8,30,1};
 auto neutral=f;neutral.radius=2;for(unsigned i=0;i<rest.size();i++)Check(field.BodyDisplacement(i,neutral,2)==PrecisePoint{},"Neutral pelvic shape changed");
 Check(field.BodyDisplacement(5,f,2)==PrecisePoint{}&&field.BodyDisplacement(6,f,2)==PrecisePoint{},"Exterior/posterior changed");
 auto larger=field.BodyDisplacement(0,f,2);Check(std::abs(larger[2]+2-8)<1e-12,"Collar does not reach actual shaft radius");
 Check(-field.BodyDisplacement(2,f,2)[2]+2>8,"Underside lacks extra section space");
 auto half=f;half.radius=4;Check(Norm(field.BodyDisplacement(4,f,2))>Norm(field.BodyDisplacement(4,half,2)),"Pelvic support fails to grow progressively");
 auto tilted=f;tilted.axis={.6,0,.8};tilted.up={-.8,0,.6};
 auto pose=Add(rest[0],field.BodyDisplacement(0,tilted,2));const double depth=.95*8,compensation=std::max(0.,8*.8-depth*.6);const auto anchor=Add(Scale(tilted.axis,depth),{compensation,0,0});const auto radial=Add(pose,Scale(anchor,-1));Check(std::abs(Dot(radial,tilted.axis))<1e-10&&std::abs(Norm(radial)-8)<1e-10,"Tilt pinches the expanded opening");
 std::vector<PrecisePoint> moving(rest.size(),{1,2,3});std::vector<double> weights(rest.size(),1);weights[7]=0;auto p=field.Evaluate(moving,tilted,2,weights);
 Check(p[7]==moving[7]&&p[5]==moving[5],"Excluded material or distal anatomy changed");
 for(unsigned i=0;i<4;i++)Check(Norm(Add(p[i],Scale(field.BodyDisplacement(i,tilted,2),-1)))<1e-10,"Opening sides disagree");
 std::vector<PrecisePoint> rings;for(unsigned i=0;i<1001;i++)rings.push_back({0,0,2+i*.025});PelvicRecruitment annulus(rings,opening);double last=-INFINITY;
 for(unsigned i=0;i<rings.size();i++){const auto q=Add(rings[i],annulus.BodyDisplacement(i,f,2));Check(q[2]>last,"Radial ramp reverses rings");last=q[2];}
 std::cout<<"PASS neutral/exterior, progressive size, underside, rigid section, exclusions and monotone radial ramp\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
