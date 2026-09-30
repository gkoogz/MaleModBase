#pragma once
static float ventralTubeApplied[r14Count];
static void CompleteVentralTube(){
  memset(ventralTubeApplied,0,sizeof(ventralTubeApplied));
  if(!constraintSolverReady)return;
  V3 axis=Unit(r14Positions[36]-r14Positions[37]);
  V3 incoming=axis*-1.f,outgoing=Unit(r14Positions[317]-r14Positions[318]),chord=r14Positions[318]-r14Positions[36];
  float determinant=incoming.x*outgoing.z-incoming.z*outgoing.x;
  float intersection=fabsf(determinant)>.001f?(chord.x*outgoing.z-chord.z*outgoing.x)/determinant:0.f;
  float visibility=Smoother01((sliderUI[0]-8.f)/15.f);
  if(incoming.z>0.f)visibility*=Smoother01(intersection/max(.1f,logicalShaftBodyRadius));
  if(visibility<=0.f)return;
  static V3 origin[r14Count],step[r14Count],next[r14NewStart];
  memcpy(origin,r14Positions,sizeof(origin));memset(step,0,sizeof(step));
  V3 centers[41],tangents[41];
  for(int k=0;k<=40;k++)SampleShaftChain(float(k)*.78f/40.f,centers[k],tangents[k]);
  float radius=logicalShaftBodyRadius;
  V3 ridgeAnchor=origin[355],ridgeAxis=Unit(origin[355]-origin[354]);
  V3 ridgeOut=Unit(Cross(V3{0,1,0},ridgeAxis));
  for(UINT i=0;i<r14NewStart;i++){
    V3 p=origin[i];float best=1e30f,t=0.f;V3 center{},tangent{};
    for(int k=0;k<40;k++){
      V3 edge=centers[k+1]-centers[k];float u=max(0.f,min(1.f,Dot(p-centers[k],edge)/max(1e-8f,Dot(edge,edge))));
      V3 c=centers[k]+edge*u;float dist=Dot(p-c,p-c);
      if(dist<best){best=dist;t=(float(k)+u)*.78f/40.f;center=c;tangent=Unit(tangents[k]*(1.f-u)+tangents[k+1]*u);}
    }
    V3 lateral=Unit(V3{0,1,0}-tangent*tangent.y),ventral=Unit(Cross(lateral,tangent));
    float side=Dot(p-center,lateral)/radius,depth=Dot(p-center,ventral)/radius;
    // An embedded tube on the SHAFT axis. Deep pouch vertices are outside
    // its envelope and are never selected: there is no path around the sac.
    float halfWidth=.64f;
    if(fabsf(side)>=halfWidth||depth<.45f||depth>1.7f)continue;
    // Continue the existing distal crest instead of creating a new bump.
    // The round cross section sits below this shared crest line.
    float halfTube=.62f*radius;
    float crossY=Dot(p-ridgeAnchor,lateral);
    float roundDrop=halfTube-sqrtf(max(0.f,halfTube*halfTube-crossY*crossY));
    float amount=max(-.12f*radius,Dot(ridgeAnchor-p,ridgeOut)-roundDrop);
    ventral=ridgeOut;
    float contact=1.f;
    for(int sideIndex=0;sideIndex<2;sideIndex++){
      float support=max(eggRadii[sideIndex].x,max(eggRadii[sideIndex].y,eggRadii[sideIndex].z));
      contact=min(contact,Smoother01((Length(p-ballNodes[sideIndex])-support*.9f)/max(.001f,support*.45f)));
    }
    float shoulder=1.f-Smoother01((fabsf(side)-.42f)/.22f);
    float root=Smoother01(t/.10f),distal=1.f-Smoother01((t-.69f)/.08f);
    float membership=0.f;for(UINT k=r14Offsets[i];k<r14Offsets[i+1];k++)membership+=r14Weight[k]*phys_scrotum_weight[r14Sources[k]];
    float exposed=1.f-Smoother01((membership-.40f)/.35f);
    step[i]=ventral*(amount*shoulder*root*distal*exposed*contact*visibility);
    ventralTubeApplied[i]=Length(step[i]);
  }
  for(int pass=0;pass<4;pass++){
    for(UINT i=0;i<r14NewStart;i++){
      V3 average{};UINT first=undersideRows[i],last=undersideRows[i+1];
      for(UINT k=first;k<last;k++)average=average+step[undersideNeighbors[k]];
      next[i]=step[i]+(average/float(last-first)-step[i])*(.25f*(ventralTubeApplied[i]>0.f?1.f:0.f)*Smoother01(fabsf(origin[i].y)/max(.001f,radius*.25f)));
    }
    memcpy(step,next,sizeof(next));
  }
  float fraction=1.f;
  for(UINT k=0;k<r14IndexCount;k+=3){UINT a=r14Indices[k],b=r14Indices[k+1],c=r14Indices[k+2];if(ventralTubeApplied[a]+ventralTubeApplied[b]+ventralTubeApplied[c]<=0.f)continue;fraction=min(fraction,SurfaceCorrectionLimit(origin[a],origin[b],origin[c],step[a],step[b],step[c],.35f));}
  for(UINT i=0;i<r14NewStart;i++){r14Positions[i]=origin[i]+step[i]*fraction;ventralTubeApplied[i]=Length(step[i])*fraction;undersideBlendMoved[i]+=ventralTubeApplied[i];}
}
