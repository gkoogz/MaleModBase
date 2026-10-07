#include <malemod/garments/input_recording.hpp>
#include <malemod/garments/geometry_export.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#ifdef MALEMOD_REPLAY_CPU_POUCH
#include <malemod/garments/pouch_session.hpp>
#endif
using namespace malemod::garments;

// Replays private recordings without an engine or graphics SDK. Captures stay
// outside Git; failed contact/material budgets must fail the process as well.
int main(int argc,char** argv) {
 try {
  if(argc<2)throw std::invalid_argument("Usage: replay_supported_garment DIRECTORY [interpolation-hz]");
  const std::filesystem::path directory(argv[1]);
  const double hz=argc>2?std::stod(argv[2]):0;
  if(!std::isfinite(hz)||hz<0||hz>120)throw std::invalid_argument("Interpolation rate outside 0..120 Hz");
  auto read=[&](const std::string& name){std::ifstream file(directory/(name+".input"),std::ios::binary);return input_recording::Read(file);};
  auto reference=read("reference"),previous=read("prepared");
#ifdef MALEMOD_REPLAY_CPU_POUCH
  PouchSession session(SupportedPouchParameters());
#else
  Session session(SupportedPouchParameters());
#endif
  auto output=session.InitializeDraped(reference,previous);
  bool passed=output.contactBudgetSatisfied&&output.physics.materialBudgetSatisfied;
  std::cout<<"request,substep,seconds,solver_ms,contact,material,stretch,render_stretch,seam,clock,worst_a,worst_b,rest_length,current_length,coverage,body_intersections,pouch_penetrations\n"<<std::setprecision(10);
  unsigned requests=0,failures=0;double simulated=0,wallMs=0;
  for(unsigned request=1;std::filesystem::exists(directory/("request-"+std::to_string(request)+".input"));request++) {
   auto current=read("request-"+std::to_string(request));requests++;
   if(request==1){session.PlacePrepared(previous,current);current.deltaTime=0;previous=current;}
   const unsigned count=hz>0?std::max(1u,unsigned(std::ceil(current.deltaTime*hz))):1;
   for(unsigned sub=1;sub<=count;sub++) {
    auto pose=current;const double fraction=double(sub)/count;
    auto mix=[&](auto& destination,const auto& source){
     if(destination.size()!=source.size())throw std::invalid_argument("Replay topology changed");
     for(unsigned i=0;i<destination.size();i++){
      destination[i].position=Add(Mul(source[i].position,1-fraction),Mul(destination[i].position,fraction));
      destination[i].normal=Unit(Add(Mul(source[i].normal,1-fraction),Mul(destination[i].normal,fraction)));
     }
    };
    mix(pose.waist,previous.waist);mix(pose.opening,previous.opening);
    mix(pose.bodySurface,previous.bodySurface);mix(pose.anatomy,previous.anatomy);
    for(unsigned side=0;side<2;side++)mix(pose.rearStraps[side],previous.rearStraps[side]);
    pose.frame.origin=Add(Mul(previous.frame.origin,1-fraction),Mul(current.frame.origin,fraction));
    pose.frame.lateral=Unit(Add(Mul(previous.frame.lateral,1-fraction),Mul(current.frame.lateral,fraction)));
    pose.frame.up=Unit(Add(Mul(previous.frame.up,1-fraction),Mul(current.frame.up,fraction)));
    pose.frame.forward=Unit(Cross(pose.frame.up,pose.frame.lateral));
    pose.frame.up=Unit(Cross(pose.frame.lateral,pose.frame.forward));
    pose.deltaTime=current.deltaTime/count;
    output=session.Update(Style::WhiteJockstrap,pose,TimeContinuity::Continuous);
    const auto& p=output.physics;simulated+=pose.deltaTime;wallMs+=p.solverMilliseconds;
    const bool accepted=output.contactBudgetSatisfied&&p.materialBudgetSatisfied;
    passed=passed&&accepted;if(!accepted)failures++;
    std::cout<<request<<','<<sub<<','<<pose.deltaTime<<','<<p.solverMilliseconds<<','<<output.contactBudgetSatisfied<<','<<p.materialBudgetSatisfied<<','<<p.maxStretchRatio<<','<<p.maxRenderStretchRatio<<','<<p.maxSeamGap<<','<<p.accumulatedSeconds<<','<<p.worstStretchA<<','<<p.worstStretchB<<','<<p.worstStretchRestLength<<','<<p.worstStretchCurrentLength<<','<<output.coverageMargin<<','<<output.bodyIntersectionCount<<','<<output.pouchPenetrationCount<<'\n'<<std::flush;
   }
   previous=current;
   std::ofstream mesh(directory/("checked-replay-"+std::to_string(request)+".obj"));
   geometry_export::WriteOBJ(mesh,output.mesh);
   std::ofstream layout(directory/("checked-replay-"+std::to_string(request)+".json"));geometry_export::WriteLayout(layout,output.layout);
   std::cerr<<"request="<<request<<" body_crossings band="<<output.bodyIntersectionsByPart[0]<<" pouch="<<output.bodyIntersectionsByPart[1]<<" straps="<<output.bodyIntersectionsByPart[2]<<" hems="<<output.bodyIntersectionsByPart[3]<<'\n';
  }
  if(!requests)throw std::invalid_argument("Replay contains no request-1.input");
  std::cerr<<"requests="<<requests<<" failed_updates="<<failures<<" simulated_seconds="<<simulated<<" solver_seconds="<<wallMs/1000<<"\n";
  return passed?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
