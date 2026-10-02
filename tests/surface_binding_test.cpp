#include <malemod/surface/binding.hpp>
#include <cstdio>
#include <limits>
using namespace malemod::surface;
int main(){
 CoordinateCalibration c{{{{0,-1,0},{1,0,0},{0,0,1}}},{10.368809700012207,.12673234939575195,83.42259979248047},{0,.11,.975},.010255218584141075};
 c.Validate();
 auto p=c.PointToTarget(c.sourceRoot);if(p!=c.targetRoot)return 1;
 for(auto point:std::array<PrecisePoint,3>{{{1,2,3},{80,-20,100},{0,0,0}}}){
  auto q=c.PointToSource(c.PointToTarget(point));for(unsigned i=0;i<3;i++)if(std::abs(point[i]-q[i])>1e-11)return 2;
 }
 if(c.VectorToTarget({1,0,0})!=PrecisePoint{0,1,0}||c.VectorToSource({0,1,0})!=PrecisePoint{1,0,0})return 3;
 if(std::abs(c.LengthToSource(c.LengthToTarget(7.2))-7.2)>1e-12)return 4;
 DeltaBinding d{3,{0,2,3},{0,2,1},{.25,.75,1}};
 std::vector<PrecisePoint> neutral{{1,2,3},{4,5,6},{7,8,9}},now{{2,4,6},{5,7,9},{8,10,12}};
 auto delta=d.Apply(now,neutral);if(delta!=std::vector<PrecisePoint>{{1,2,3},{1,2,3}})return 5;
 auto reject=[](auto fn){try{fn();return false;}catch(const std::invalid_argument&){return true;}};
 c.basis[0][1]=1;if(!reject([&]{c.Validate();}))return 6;
 d.weights[0]=.5;if(!reject([&]{d.Apply(now,neutral);}))return 7;
 d.weights[0]=.25;d.donors[0]=3;if(!reject([&]{d.Apply(now,neutral);}))return 8;
 d.donors[0]=0;now[0][2]=std::numeric_limits<double>::quiet_NaN();if(!reject([&]{d.Apply(now,neutral);}))return 9;
 std::puts("PASS calibrated position/vector/length conversions, exact donor deltas and malformed binding rejection");
}
