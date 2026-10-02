#include <malemod/motion_filter.hpp>
#include <iostream>
#include <limits>
#include "original-reference.inc"
int main(){
 try{
  using Tracker=malemod::motion::Tracker;Tracker tracker;std::uint32_t tick=1000;
  for(unsigned frame=0;frame<2000;frame++){
   const float angle=.25f*std::sin(frame*.07f);const float c=std::cos(angle),s=std::sin(angle);
   Tracker::Matrix matrix={c,-s,0,2.f*std::sin(frame*.12f),s,c,0,3.f*std::cos(frame*.11f),0,0,1,1.5f*std::sin(frame*.03f)};
   if(frame%47==0)matrix[3]+=400.f;
   tick+=frame%97==0?1000:frame%13==0?3:16;
   if(frame==1000){tick=0xfffffff0u;tracker.Reset();original_motion::Reset();}
   tracker.Track(tick,matrix,matrix,matrix);original_motion::oracleNow=tick;
   original_motion::TrackCharacterMotionMatrices(matrix.data(),matrix.data(),matrix.data(),"offline original");
   if(!original_motion::Equal(tracker.Read(),original_motion::Read()))throw std::runtime_error("Source filter state differs");
  }
  // Missing contact poses must preserve the force law without advertising
  // zero matrices as calibrated collision bones.
  Tracker poseOnly,full;tick=4000;
  for(unsigned frame=0;frame<400;frame++){
   Tracker::Matrix matrix={1,0,0,std::sin(frame*.13f),0,1,0,std::cos(frame*.17f),0,0,1,std::sin(frame*.07f)};
   tick+=frame%17==0?3:16;poseOnly.TrackPoseOnly(tick,matrix);full.Track(tick,matrix,matrix,matrix);
   auto actual=poseOnly.Read(),expected=full.Read();
   if(actual.motionCollisionBonesReady)throw std::runtime_error("Unavailable contact bones advertised");
   expected.motionCollisionBonesReady=false;
   std::memset(expected.motionLeftThighMatrix,0,sizeof(expected.motionLeftThighMatrix));
   std::memset(expected.motionRightThighMatrix,0,sizeof(expected.motionRightThighMatrix));
   if(!original_motion::Equal(actual,expected))throw std::runtime_error("Force-only path changes source motion law");
  }
  auto saved=tracker.Read();Tracker::Matrix invalid{};invalid[0]=std::numeric_limits<float>::quiet_NaN();
  try{tracker.Track(tick+16,invalid,invalid,invalid);throw std::runtime_error("Nonfinite matrix accepted");}catch(const std::invalid_argument&){}
  if(!original_motion::Equal(saved,tracker.Read()))throw std::runtime_error("Rejected input changed state");
  std::cout<<"PASS: 2000 exact source motion states, 400 force-only states, short intervals, long gaps, teleports, timer wrap, reset and invalid-input preservation.\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
