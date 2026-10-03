#pragma once
#include "teaching_sequence.h"
#include <cmath>
#include <stdexcept>
namespace malemod::clinical {
// Original independent three-second size and 4.6-second return clocks.
struct AmbientClock {
 unsigned mode=0;float sizeTime=0,twitchTime=0;
 void SetMode(unsigned next){if(next>3)throw std::invalid_argument("Throb mode must be 0..3");if(next!=mode){mode=next;sizeTime=twitchTime=0;}}
 void Advance(float seconds){if(!std::isfinite(seconds)||seconds<0)throw std::invalid_argument("Invalid throb interval");if(!mode)return;const float dt=teaching::BoundedFrameAdvance(seconds);sizeTime=fmodf(sizeTime+dt,3.f);twitchTime+=dt;if(twitchTime>=5.75f)twitchTime-=4.6f;}
 static float Envelope(float cycle,float delay,float duration,bool twitch){float x=(cycle-delay)/duration;if(x<=0||x>=1)return 0;if(twitch){float rise=(duration>4?.20f:.14f)/duration;return x<rise?teaching::Ease(x/rise):1-teaching::Ease((x-rise)/(1-rise));}return x<.5f?teaching::Ease(x*2):1-teaching::Ease((x-.5f)*2);}
 float Pulse()const{return Envelope(sizeTime,.20f,1.05f,false)+Envelope(twitchTime,1.15f,1.05f,true);}
};
}
