#pragma once
#include <functional>

namespace malemod::garments::drape {
// Rest-only fairing of an actual measured ribbon centerline. Apply this to
// the equally spaced dense material route, as well as authored route knots.
// End attachments and each sample's exact ancestry remain unchanged; the
// caller's physical union projection rejects inward movement on every sweep.
inline std::vector<Sample> SmoothRoute(std::vector<Sample> path,unsigned sweeps,
                                     const std::function<void(Sample&)>& project){
 if(path.size()<3)throw std::invalid_argument("Measured material route is incomplete");
 for(auto& sample:path)if(!Finite(sample.position))throw std::invalid_argument("Measured material route is nonfinite");
 for(unsigned pass=0;pass<sweeps;pass++){
  const auto old=path;
  for(unsigned k=1;k+1<path.size();k++){
   double left=Length(Sub(old[k].position,old[k-1].position));
   double right=Length(Sub(old[k+1].position,old[k].position));
   double total=left+right;
   if(total<=1e-14)throw std::invalid_argument("Measured material route is degenerate");
   auto chord=Mul(Add(old[k-1].position,old[k+1].position),.5);
   path[k].position=Mul(Add(old[k].position,chord),.5);
   project(path[k]);
   if(!Finite(path[k].position))throw std::invalid_argument("Measured material projection is nonfinite");
  }
 }
 return path;
}
// One rest bending-energy descent step on complete ribbon sections. The
// move callback translates all four corners together, retaining width,
// thickness, frame and exact donor ancestry. The owner MUST run its physical
// full-vertex/full-face projection after each step, then verify final clearance.
// This is not a centerline collision approximation or a moving-cloth refit.
inline bool BendRibbonGroups(const Mesh& mesh,const RibbonLayout& route,
                             const std::function<void(unsigned,Point)>& moveGroup,
                             double strength=.05){
 if(route.corners!=4||route.sections<5||std::size_t(route.start)+std::size_t(route.sections)*4>mesh.vertices.size()||!std::isfinite(strength)||strength<=0||strength>1./16)
  throw std::invalid_argument("Invalid measured ribbon bending contract");
 std::vector<Point> center(route.sections);
 for(unsigned k=0;k<route.sections;k++){
  for(unsigned j=0;j<4;j++){auto p=mesh.vertices[route.start+4*k+j].position;if(!Finite(p))throw std::invalid_argument("Nonfinite measured ribbon section");center[k]=Add(center[k],p);}
  center[k]=Mul(center[k],.25);
 }
 // Differentiate the complete sum of squared interior second differences.
 // The one-sided natural boundary rows are necessary: excluding the first
 // and penultimate free sections leaves the approach to a sewn cap unfaired.
 std::vector<Point> second(route.sections);
 for(unsigned k=1;k+1<route.sections;k++)second[k]=Add(Sub(center[k-1],Mul(center[k],2)),center[k+1]);
 bool moved=false;
 for(unsigned k=1;k+1<route.sections;k++){
  auto fourth=Mul(second[k],-2);
  if(k>1)fourth=Add(fourth,second[k-1]);
  if(k+2<route.sections)fourth=Add(fourth,second[k+1]);
  auto delta=Mul(fourth,-strength);
  moveGroup(route.start+k*4,delta);moved=moved||Length(delta)>0;
 }
 return moved;
}
}
