// Continuous ventral envelope from the shaft into the existing lower pouch.
// Landmark positions come from the final animated skin, so the blend follows
// both ends without modifying the cage, solver, body weld or distal shaft.
#include "underside_blend_data.h"
#include "underside_legacy.h"
static float completeRapheActivation=0.f;
static float undersideBlendMoved[r14Count];
static void BlendContinuousUnderside(){
  static V3 original[r14Count],delta[r14Count];
  memcpy(original,r14Positions,sizeof(original));
  memset(delta,0,sizeof(delta));memset(undersideBlendMoved,0,sizeof(undersideBlendMoved));
  V3 start=original[36],end=original[318];
  V3 chord=end-start;chord.y=0.f;
  float span=Length(chord);if(span<1e-4f)return;
  V3 axis=chord/span,outward=Unit(Cross(axis,V3{0,1,0}));
  V3 startTangent=original[37]-start,endTangent=original[317]-end;
  startTangent.y=endTangent.y=0.f;
  startTangent=Unit(startTangent)*span;endTangent=Unit(endTangent)*span;
  // Keep the shaft tangent straight to the neck; round only the junction.
  // A single cubic started bending immediately and recreated the visible notch.
  V3 incoming=Unit(startTangent),outgoing=Unit(endTangent);
  float determinant=incoming.x*outgoing.z-incoming.z*outgoing.x;
  float firstLength=0.f,lastLength=0.f;
  bool cornerValid=fabsf(determinant)>.08f;
  if(cornerValid){
    firstLength=(chord.x*outgoing.z-chord.z*outgoing.x)/determinant;
    lastLength=(incoming.x*chord.z-incoming.z*chord.x)/determinant;
    cornerValid=firstLength>span*.08f&&lastLength>span*.08f&&firstLength+lastLength<span*3.f;
  }
  // Do not bridge folded-over surfaces when the shaft hangs into the pouch.
  // The existing local deformation handles that contact configuration.
  completeRapheActivation=Smoother01((sliderValues[0]-10.f)/15.f)*(1.f-Smoother01((incoming.z-.70f)/.20f));
  if((firstLength<0.f&&incoming.z>0.f)||completeRapheActivation<=0.f){
    completeRapheActivation=0.f;LegacyUnderside::BlendContinuousUnderside();
    memcpy(undersideBlendMoved,LegacyUnderside::undersideBlendMoved,sizeof(undersideBlendMoved));return;
  }
  V3 corner=start+incoming*firstLength;
  float rounding=min(firstLength,lastLength)*.20f;
  auto envelope=[&](float u)->V3{
    if(!cornerValid){float u2=u*u,u3=u2*u;return start*(2*u3-3*u2+1)+startTangent*(u3-2*u2+u)+end*(-2*u3+3*u2)+endTangent*(u3-u2);}
    float straightA=firstLength-rounding,curveLength=rounding*1.7f,straightB=lastLength-rounding;
    float travel=u*(straightA+curveLength+straightB);
    V3 a=corner-incoming*rounding,b=corner+outgoing*rounding;
    if(travel<=straightA)return start+incoming*travel;
    if(travel>=straightA+curveLength)return b+outgoing*(travel-straightA-curveLength);
    float t=(travel-straightA)/curveLength;return a*((1-t)*(1-t))+corner*(2*t*(1-t))+b*(t*t);
  };
  static float parameter[r14Count],lateral[r14Count],falloff[r14Count];
  static float nextParameter[r14NewStart],nextLateral[r14NewStart];
  for(UINT i=0;i<r14Count;i++){
    parameter[i]=undersideMaterial[i];lateral[i]=original[i].y;
    falloff[i]=undersideBlendMask[i]*Smoother01(parameter[i]/.065f)*Smoother01((1.f-parameter[i])/.12f);
  }
  float largestGap=0.f;
  for(UINT i=0;i<r14NewStart;i++){
    float u=max(0.f,min(1.f,parameter[i])),u2=u*u,u3=u2*u;
    V3 curve=envelope(u);
    largestGap=max(largestGap,max(0.f,Dot(curve-original[i],outward))*falloff[i]);
  }
  float activation=completeRapheActivation*Smoother01((largestGap/span-.005f)/.025f);
  if(activation<=0.f)return;
  for(UINT i=0;i<r14Count;i++)falloff[i]*=activation;
  // The old notch doubles back locally. Fair the two coordinates along the
  // new surface before filling it, so those folded rows cannot overlap.
  for(int pass=0;pass<40;pass++){
    for(UINT i=0;i<r14NewStart;i++){
      float u=0.f,y=0.f;UINT first=undersideRows[i],last=undersideRows[i+1];
      for(UINT k=first;k<last;k++){UINT j=undersideNeighbors[k];u+=parameter[j];y+=lateral[j];}
      float weight=.45f*falloff[i];
      nextParameter[i]=parameter[i]+weight*(u/float(last-first)-parameter[i]);
      nextLateral[i]=lateral[i]+weight*(y/float(last-first)-lateral[i]);
    }
    memcpy(parameter,nextParameter,sizeof(nextParameter));memcpy(lateral,nextLateral,sizeof(nextLateral));
  }
  for(UINT i=0;i<r14NewStart;i++){
    if(undersideBlendMask[i]<=0.f)continue;
    float raw=Dot(original[i]-start,axis)/span;
    float u=max(0.f,min(1.f,parameter[i]));
    float u2=u*u,u3=u2*u;
    V3 curve=envelope(u);
    float amount=max(0.f,Dot(curve-original[i],outward));
    amount=min(amount,span*.30f)*falloff[i];
    delta[i]=outward*amount+axis*((u-raw)*span*falloff[i]);
    delta[i].y+=(lateral[i]-original[i].y)*falloff[i];
  }
  // Advance in small increments: curved skin can legitimately turn more
  // than 90 degrees overall, but each increment must retain triangle area.
  static V3 step[r14Count];
  for(int increment=0;increment<24;increment++){
    for(UINT i=0;i<r14Count;i++)step[i]=(original[i]+delta[i]-r14Positions[i])/float(24-increment);
    float fraction=1.f;
    for(UINT k=0;k<r14IndexCount;k+=3){
      UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
      if(undersideBlendMask[a]+undersideBlendMask[b]+undersideBlendMask[c]<=0.f)continue;
      fraction=min(fraction,SurfaceCorrectionLimit(r14Positions[a],r14Positions[b],r14Positions[c],step[a],step[b],step[c],.35f));
    }
    for(UINT i=0;i<r14NewStart;i++)r14Positions[i]=r14Positions[i]+step[i]*fraction;
  }
  static V3 fairNext[r14NewStart];
  for(int pass=0;pass<32;pass++){
    for(UINT i=0;i<r14NewStart;i++){
      V3 mean{};UINT first=undersideRows[i],last=undersideRows[i+1];
      for(UINT k=first;k<last;k++)mean=mean+r14Positions[undersideNeighbors[k]];
      fairNext[i]=r14Positions[i]+(mean/float(last-first)-r14Positions[i])*(.4f*falloff[i]);
    }
    memcpy(r14Positions,fairNext,sizeof(fairNext));
  }
  // Fairing must not hollow the straight underside back out. Reassert the
  // live shaft tangent over the shaft-side strip, fading before the neck turn.
  if(cornerValid){
    V3 shaftOutward=Unit(Cross(incoming,V3{0,1,0}));
    for(int pass=0;pass<12;pass++){
      memset(step,0,sizeof(step));
      for(UINT i=0;i<r14NewStart;i++){
        float t=Dot(r14Positions[i]-start,incoming);
        float weight=undersideBlendMask[i]*Smoother01(t/max(1e-4f,span*.06f))
          *(1.f-Smoother01((t-(firstLength-rounding))/max(1e-4f,rounding)));
        float recess=max(0.f,Dot(start-r14Positions[i],shaftOutward));
        step[i]=shaftOutward*(recess*weight*.5f);
      }
      float safe=1.f;
      for(UINT k=0;k<r14IndexCount;k+=3){
        UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];
        if(undersideBlendMask[a]+undersideBlendMask[b]+undersideBlendMask[c]<=0.f)continue;
        safe=min(safe,SurfaceCorrectionLimit(r14Positions[a],r14Positions[b],r14Positions[c],step[a],step[b],step[c],.35f));
      }
      for(UINT i=0;i<r14NewStart;i++)r14Positions[i]=r14Positions[i]+step[i]*safe;
    }
  }
  // Relax the lateral shoulder of the constraint to avoid a faceted lip.
  for(int pass=0;pass<28;pass++){
    for(UINT i=0;i<r14NewStart;i++){
      V3 mean{};UINT first=undersideRows[i],last=undersideRows[i+1];
      for(UINT k=first;k<last;k++)mean=mean+r14Positions[undersideNeighbors[k]];
      fairNext[i]=r14Positions[i]+(mean/float(last-first)-r14Positions[i])*(.28f*falloff[i]);
    }
    memcpy(r14Positions,fairNext,sizeof(fairNext));
  }
  for(UINT i=0;i<r14NewStart;i++)undersideBlendMoved[i]=Length(r14Positions[i]-original[i]);
}
