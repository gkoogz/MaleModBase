#include <malemod/surface/suspension_skin_frame.hpp>
#include <iostream>
#include <limits>
using namespace malemod::surface;
void Check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
void Near(PresentationPoint a,PresentationPoint b,double e=1e-11){Check(Dot(Sub(a,b),Sub(a,b))<e*e,"Frame differs");}
int main(){try{
 PresentationFrame rest{{2,-3,5},Basis({.2,-.3,.1,std::sqrt(.86)})};
 auto cord=Unit(PresentationPoint{.2,.3,1});auto anchor=Add(rest.origin,Scale(cord,4));
 auto same=SuspensionSkinFrame(rest,anchor,rest.origin,anchor);Check(same.basis==rest.basis&&same.origin==rest.origin,"Reference changed");
 {
  auto pair=std::array<PresentationFrame,2>{rest,rest};pair[0].origin[1]-=2;pair[1].origin[1]+=2;
  std::array<PresentationPoint,2> anchors{Add(pair[0].origin,{0,0,5}),Add(pair[1].origin,{0,0,5})};
  auto live=std::array<PresentationPoint,2>{Add(pair[0].origin,{4,-3,1}),Add(pair[1].origin,{-2,3,-1})};
  auto envelope=SuspensionEnvelopeFrames(pair,anchors,live,anchors);
  auto a=Sub(pair[0].origin,pair[1].origin),b=Sub(envelope[0].origin,envelope[1].origin);
  Check(std::abs(Dot(a,a)-Dot(b,b))<1e-10,"Independent contents shear the common skin rim");
  Near(Scale(Add(envelope[0].origin,envelope[1].origin),.5),Scale(Add(live[0],live[1]),.5));
  for(auto f:envelope)Quaternion(f.basis);
 }
 auto target=Unit(PresentationPoint{.7,-.2,1});PresentationPoint center{4,-6,8};auto endpoint=Add(center,Scale(target,2));auto frame=SuspensionSkinFrame(rest,anchor,center,endpoint);
 Near(Rotate(frame.basis,Unrotate(rest.basis,cord)),target);Check(frame.origin==center,"Skin body center changed");Quaternion(frame.basis);
 // Full and compact callers have no internal body-spin input. Identical
 // measured centers and attachments must therefore produce identical frames.
 Check(SuspensionSkinFrame(rest,anchor,center,endpoint).basis==frame.basis,"Publication route differs");
 auto q=Basis({-.1,.4,.2,std::sqrt(.79)});PresentationPoint translation{80,-7,22};double scale=3.7;
 auto point=[&](PresentationPoint p){return Add(Scale(Rotate(q,p),scale),translation);};
 PresentationFrame transformed{point(rest.origin),{}};for(unsigned j=0;j<3;j++)transformed.basis[j]=Rotate(q,rest.basis[j]);
 auto covariant=SuspensionSkinFrame(transformed,point(anchor),point(center),point(endpoint));Near(covariant.origin,point(frame.origin));for(unsigned j=0;j<3;j++)Near(covariant.basis[j],Rotate(q,frame.basis[j]));
 // Very short low-Hang cords cannot amplify body spin; direction controls
 // frame and length does not. Distances remain rigid, preserving lobe volume.
 auto shortFrame=SuspensionSkinFrame(rest,anchor,center,Add(center,Scale(target,1e-5)));for(unsigned j=0;j<3;j++)Near(shortFrame.basis[j],frame.basis[j],1e-9);
 for(auto p:{PresentationPoint{1,2,3},PresentationPoint{-4,1,.2}})Check(std::abs(Dot(Rotate(frame.basis,p),Rotate(frame.basis,p))-Dot(p,p))<1e-11,"Envelope volume/length changed");
 // Smooth oscillation across the vertical cord has no Euler wrap, no sign
 // switch and no accumulated drift after many evaluations.
 auto previous=rest.basis;
 for(unsigned i=0;i<=2000;i++){double t=6.283185307179586*i/2000;auto live=Unit(Add(cord,Scale(rest.basis[0],.2*std::sin(t))));auto current=SuspensionSkinFrame(rest,anchor,rest.origin,Add(rest.origin,Scale(live,4)));
  for(unsigned j=0;j<3;j++)Check(Dot(Sub(current.basis[j],previous[j]),Sub(current.basis[j],previous[j]))<1e-5,"Skin orientation jumped");previous=current.basis;}
 for(unsigned j=0;j<3;j++)Near(previous[j],rest.basis[j]);
 auto opposite=SuspensionSkinFrame(rest,anchor,center,Sub(center,Scale(cord,2)));Near(Rotate(opposite.basis,Unrotate(rest.basis,cord)),Scale(cord,-1));Quaternion(opposite.basis);
 // Use a non-tied authored frame when testing antipodal covariance.
 auto oppositeRotated=SuspensionSkinFrame(transformed,point(anchor),point(center),point(Sub(center,Scale(cord,2))));for(unsigned j=0;j<3;j++)Near(oppositeRotated.basis[j],Rotate(q,opposite.basis[j]));
 bool bad=false;try{SuspensionSkinFrame(rest,anchor,center,center);}catch(const std::invalid_argument&){bad=true;}Check(bad,"Collapsed live cord accepted");
 bad=false;try{SuspensionSkinFrame(rest,anchor,{std::numeric_limits<double>::infinity(),0,0},endpoint);}catch(const std::invalid_argument&){bad=true;}Check(bad,"Nonfinite center accepted");
 std::cout<<"PASS exact reference, cord alignment, covariance, rigid volume, short-span and temporal continuity\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
