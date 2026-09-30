#pragma once
// CPU-only closed annular tube: it has an outer wall, lumen, and sealed wall
// ends. No vertex buffer, material or draw path can expose this internal mesh.
static constexpr unsigned rapheTubeRings=65,rapheTubeSides=24;
static V3 rapheTubeWall[rapheTubeRings*rapheTubeSides*2];
static unsigned short rapheTubeFaces[rapheTubeRings*rapheTubeSides*12];
static V3 rapheTubeCenters[rapheTubeRings];
static float rapheTubeOuter[rapheTubeRings],rapheTubeParameters[rapheTubeRings];
static bool rapheTubeTopologyReady=false,rapheNormalNeeded[rsCount]{};
static float rapheTubeEnd=.88f,rapheTubeEndDepth=0.f,rapheSkinFraction=1.f;
static unsigned rapheSkinChanged=0;
static V3 rapheSkinDelta[rsCount];
static float rapheSkinBall[rsCount]{};
#ifdef RAPHE_PROFILE
static double rapheProfile[4]{};
#endif
struct RapheFrame {V3 p,t,side,down;};
static RapheFrame rapheChain[33];
struct RapheSegment {V3 edge;float inverseLength2;};
static RapheSegment rapheSegments[32];
static V3 rapheBoundsMin[8],rapheBoundsMax[8];
struct RapheSegments4 {__m128 x,y,z,dx,dy,dz,inverse,minimum;};
static RapheSegments4 rapheSegments4[8];
static V3 rapheSearchAxis;
static V3 rapheCollarAxis,rapheCollarDown,rapheCollarRoot;
static RapheFrame RapheChainFrame(float t){
 if(t<0){auto f=rapheChain[0];f.p=f.p+f.t*(t*constraintRestLength);return f;}
 float u=max(0.f,min(1.f,t))*32;unsigned a=min(31u,(unsigned)u);float blend=u-a;
 RapheFrame f;SampleShaftChain(t,f.p,f.t);f.side=Unit(rapheChain[a].side*(1-blend)+rapheChain[a+1].side*blend);
 f.side=Unit(f.side-f.t*Dot(f.side,f.t));f.down=Unit(Cross(f.side,f.t));return f;
}
static float RapheClosest(V3 p){
 float best=1e30f,t=0;
 // Conservative block bounds retain an exact closest-segment search while
 // avoiding projection against distant portions of the centerline.
 unsigned hint=(unsigned)max(0.f,min(7.f,Dot(p-rapheChain[0].p,rapheSearchAxis)*8));
 for(unsigned visit=0;visit<9;visit++){
  unsigned block=visit?visit-1:hint;if(visit&&block==hint)continue;
  V3 lo=rapheBoundsMin[block],hi=rapheBoundsMax[block];
  float bx=max(0.f,max(lo.x-p.x,p.x-hi.x)),by=max(0.f,max(lo.y-p.y,p.y-hi.y)),bz=max(0.f,max(lo.z-p.z,p.z-hi.z));
  if(bx*bx+by*by+bz*bz>best+1e-4f)continue;
  const auto& v=rapheSegments4[block];
  __m128 px=_mm_set1_ps(p.x),py=_mm_set1_ps(p.y),pz=_mm_set1_ps(p.z);
  __m128 dot=_mm_add_ps(_mm_add_ps(_mm_mul_ps(_mm_sub_ps(px,v.x),v.dx),_mm_mul_ps(_mm_sub_ps(py,v.y),v.dy)),_mm_mul_ps(_mm_sub_ps(pz,v.z),v.dz));
  __m128 u=_mm_max_ps(v.minimum,_mm_min_ps(_mm_set1_ps(1.f),_mm_mul_ps(dot,v.inverse)));
  __m128 x=_mm_sub_ps(px,_mm_add_ps(v.x,_mm_mul_ps(v.dx,u))),y=_mm_sub_ps(py,_mm_add_ps(v.y,_mm_mul_ps(v.dy,u))),z=_mm_sub_ps(pz,_mm_add_ps(v.z,_mm_mul_ps(v.dz,u)));
  __m128 distance=_mm_add_ps(_mm_add_ps(_mm_mul_ps(x,x),_mm_mul_ps(y,y)),_mm_mul_ps(z,z));
  float ds[4],us[4];_mm_storeu_ps(ds,distance);_mm_storeu_ps(us,u);
  // Preserve visit/tie order and exact arithmetic, including the root extension.
  for(unsigned lane=0;lane<4;lane++)if(ds[lane]<best){best=ds[lane];t=(block*4+lane+us[lane])/32.f;}
 }
 return t;
}
static float RapheRadius(float t){return logicalShaftBodyRadius*RapheTubeRadiusRatio(t/rapheTubeEnd);}
static float RapheCover(){return max(.035f,logicalShaftBodyRadius*.065f);}
static float RapheDepth(float t){
 float s=t/rapheTubeEnd,taper=Smoother01((s-.50f)/.50f);
 float rootDepth=logicalShaftBodyRadius*.985f;
 float endDepth=rapheTubeEndDepth-RapheRadius(rapheTubeEnd)-RapheCover();
 float depth=rootDepth*(1-taper)+endDepth*taper;
 if(t>rapheTubeEnd)depth-=logicalShaftBodyRadius*.18f*Smoother01((t-rapheTubeEnd)/.045f);
 return depth;
}
static void BuildRapheTube(){
 for(unsigned k=0;k<33;k++){
  auto& f=rapheChain[k];SampleShaftChain(k/32.f,f.p,f.t);
  V3 side=k?RotateFromTo(rapheChain[k-1].side,rapheChain[k-1].t,f.t):V3{0,1,0};
  f.side=Unit(side-f.t*Dot(side,f.t));if(Length(f.side)<.5f)f.side=Unit(Cross(f.t,V3{0,0,1}));f.down=Unit(Cross(f.side,f.t));
 }
 for(unsigned k=0;k<32;k++){V3 e=rapheChain[k+1].p-rapheChain[k].p;rapheSegments[k]={e,1.f/max(1e-8f,Dot(e,e))};}
 for(unsigned block=0;block<8;block++){
  unsigned k=block*4;auto& v=rapheSegments4[block];
  auto starts=[&](int axis){return _mm_set_ps((&rapheChain[k+3].p.x)[axis],(&rapheChain[k+2].p.x)[axis],(&rapheChain[k+1].p.x)[axis],(&rapheChain[k].p.x)[axis]);};
  auto edges=[&](int axis){return _mm_set_ps((&rapheSegments[k+3].edge.x)[axis],(&rapheSegments[k+2].edge.x)[axis],(&rapheSegments[k+1].edge.x)[axis],(&rapheSegments[k].edge.x)[axis]);};
  v.x=starts(0);v.y=starts(1);v.z=starts(2);v.dx=edges(0);v.dy=edges(1);v.dz=edges(2);
  v.inverse=_mm_set_ps(rapheSegments[k+3].inverseLength2,rapheSegments[k+2].inverseLength2,rapheSegments[k+1].inverseLength2,rapheSegments[k].inverseLength2);
  v.minimum=_mm_set_ps(0,0,0,block==0?-.30f*32:0.f);
 }
 V3 chord=rapheChain[32].p-rapheChain[0].p;rapheSearchAxis=chord/max(1e-8f,Dot(chord,chord));
 for(unsigned block=0;block<8;block++){
  V3 lo=rapheChain[block*4].p,hi=lo;
  for(unsigned j=0;j<=4;j++){V3 p=rapheChain[block*4+j].p;lo={min(lo.x,p.x),min(lo.y,p.y),min(lo.z,p.z)};hi={max(hi.x,p.x),max(hi.y,p.y),max(hi.z,p.z)};}
  if(block==0){V3 p=rapheChain[0].p-rapheSegments[0].edge*(.30f*32);lo={min(lo.x,p.x),min(lo.y,p.y),min(lo.z,p.z)};hi={max(hi.x,p.x),max(hi.y,p.y),max(hi.z,p.z)};}
  rapheBoundsMin[block]=lo;rapheBoundsMax[block]=hi;
 }
 rapheCollarAxis=RestShaftDirection();rapheCollarDown=Unit(Cross(V3{0,1,0},rapheCollarAxis));rapheCollarRoot=ShaftRoot();
 // Meet the existing ventral fold, including independent glans scaling.
 auto distal=RapheChainFrame(.85f);V3 fold{};float best=-1e30f;unsigned first=r14NewStart+24*r14SegmentCount;
 for(unsigned j=0;j<r14SegmentCount;j++){
  V3 a=rsPositions[first+j],b=rsPositions[first+(j+1)%r14SegmentCount];float x=Dot(a-distal.p,distal.side),y=Dot(b-distal.p,distal.side);
  if(x*y>0||fabsf(x-y)<1e-7f)continue;V3 p=a+(b-a)*(x/(x-y));float depth=Dot(p-distal.p,distal.down);if(depth>best){best=depth;fold=p;}
 }
 rapheTubeEnd=max(.65f,min(.95f,RapheClosest(fold)));auto f=RapheChainFrame(rapheTubeEnd);
 rapheTubeEndDepth=max(logicalShaftBodyRadius*.65f,Dot(fold-f.p,f.down));
 for(unsigned k=0;k<rapheTubeRings;k++){
  float t=-.20f+(rapheTubeEnd+.245f)*k/(rapheTubeRings-1);auto frame=RapheChainFrame(t);float r=RapheRadius(t);
  if(t>rapheTubeEnd)r*=1.f-.2f*Smoother01((t-rapheTubeEnd)/.045f);
  V3 center=frame.p+frame.down*RapheDepth(t);rapheTubeCenters[k]=center;rapheTubeOuter[k]=r;rapheTubeParameters[k]=t;
  for(unsigned side=0;side<rapheTubeSides;side++){
   float angle=6.28318530718f*side/rapheTubeSides;V3 radial=frame.side*cosf(angle)+frame.down*sinf(angle);
   rapheTubeWall[(k*2)*rapheTubeSides+side]=center+radial*r;
   rapheTubeWall[(k*2+1)*rapheTubeSides+side]=center+radial*(r*rapheTubeLumenRatio);
  }
 }
 if(!rapheTubeTopologyReady){
  unsigned at=0;auto quad=[&](unsigned a,unsigned b,unsigned c,unsigned d){for(unsigned i:{a,b,c,a,c,d})rapheTubeFaces[at++]=(unsigned short)i;};
  for(unsigned k=0;k+1<rapheTubeRings;k++)for(unsigned j=0;j<rapheTubeSides;j++){
   unsigned a=k*2*rapheTubeSides+j,b=k*2*rapheTubeSides+(j+1)%rapheTubeSides,c=a+2*rapheTubeSides,d=b+2*rapheTubeSides;
   quad(a,b,d,c);quad(a+rapheTubeSides,c+rapheTubeSides,d+rapheTubeSides,b+rapheTubeSides);
  }
  for(unsigned j=0;j<rapheTubeSides;j++){
   unsigned n=(j+1)%rapheTubeSides;quad(j,j+rapheTubeSides,n+rapheTubeSides,n);
   unsigned base=(rapheTubeRings-1)*2*rapheTubeSides;quad(base+j,base+n,base+n+rapheTubeSides,base+j+rapheTubeSides);
  }
  for(unsigned k=0;k<sizeof(rapheTubeFaces)/sizeof(rapheTubeFaces[0]);k+=3)std::swap(rapheTubeFaces[k+1],rapheTubeFaces[k+2]);
  rapheTubeTopologyReady=true;
 }
}
static V3 RapheCollarShift(V3 p){
 V3 q=p-rapheCollarRoot;float radius=logicalShaftBodyRadius;
 float t=Dot(q,rapheCollarAxis)/max(8.f,constraintRestLength),y=q.y,depth=Dot(q,rapheCollarDown);
 if(fabsf(t+.020f)>=.115f||depth<=radius*.85f)return {};
 float width=(radius*.58f+RapheCover())*1.65f,u=fabsf(y)/max(.01f,width);
 float cross=sqrtf(max(0.f,1-u*u))*(1.f-Smoother01((u-.75f)/.25f));
 float root=1.f-Smoother01(fabsf(t+.020f)/.115f);
 float lower=Smoother01((depth/radius-.85f)/.70f);
 return rapheCollarDown*(radius*.26f*cross*root*lower*ModeValue(1.f,.9f,.6f));
}
static void ApplyRapheTubeSkin(unsigned char* body){PerfScope perf(12);
#ifdef RAPHE_PROFILE
 double mark=PerfClock();
#endif
 if(!constraintSolverReady||!shaftRestFrameReady)return;BuildRapheTube();
#ifdef RAPHE_PROFILE
 rapheProfile[0]+=PerfClock()-mark;mark=PerfClock();
#endif
 static V3 original[rsCount],step[rsCount],bodyStep[paBodyNormalCount];
 static float membership[rsCount];static bool membershipReady=false;
 if(!membershipReady){
  for(unsigned i=0;i<r14Count;i++){float ball=0,shaft=0;for(unsigned k=r14Offsets[i];k<r14Offsets[i+1];k++){unsigned j=r14Sources[k];ball+=r14Weight[k]*phys_scrotum_weight[j];shaft+=r14Weight[k]*max(phys_shaft_weight[j],phys_attachment_weight[j]);}rapheSkinBall[i]=ball;membership[i]=Smoother01((shaft-.05f)/.5f)*(1.f-Smoother01((ball-.03f)/.12f));}
  for(unsigned k=0;k<rsFineCount;k++)for(unsigned j=0;j<3;j++){membership[r14Count+k]+=membership[rsFineSource[k*3+j]]*rsFineBary[k*3+j];rapheSkinBall[r14Count+k]+=rapheSkinBall[rsFineSource[k*3+j]]*rsFineBary[k*3+j];}membershipReady=true;
 }
 memcpy(original,rsPositions,sizeof(original));memset(rapheNormalNeeded,0,sizeof(rapheNormalNeeded));float radius=logicalShaftBodyRadius;
 GeometryFor(rsCount,[&](unsigned i){
  V3 p=original[i];float ball=rapheSkinBall[i];step[i]=ball>.15f?V3{}:RapheCollarShift(p)*(1.f-Smoother01((ball-.03f)/.12f));
  if(ball>.15f){
   // The tube passes through the neck. Preserve its silhouette and permit
   // only a small, local surface easing where the wall actually intersects it.
   if(ball<.65f){float t=RapheClosest(p);if(t>.02f&&t<rapheTubeEnd*.35f){auto f=RapheChainFrame(t);V3 radial=p-f.p;radial=radial-f.t*Dot(radial,f.t)-f.down*RapheDepth(t);float distance=Length(radial),penetration=RapheRadius(t)+RapheCover()-distance;if(penetration>0&&distance>1e-5f)step[i]=Unit(radial)*(radius*.035f*Smoother01(penetration/max(.01f,radius*.15f)));}}
   return;
  }
  if(membership[i]<.001f)return;
  float t=RapheClosest(p),s=t/rapheTubeEnd;if(s<.02f||s>1.025f)return;
  auto f=RapheChainFrame(t);V3 q=p-f.p;float y=Dot(q,f.side),depth=Dot(q,f.down);
  if(depth<radius*.35f||depth>radius*1.95f||fabsf(y)>radius*.78f)return;
  float skinRadius=RapheRadius(t)+RapheCover(),ay=fabsf(y),cap=min(ay,skinRadius*.90f);
  // Continue the circular cap along its tangent before joining the shaft.
  // Cutting a circle off at its lateral extent leaves a visible shoulder step.
  float rise=sqrtf(max(1e-8f,skinRadius*skinRadius-cap*cap));
  float tube=RapheDepth(t)+rise-(ay-cap)*cap/rise;
  float base=.99f*radius*sqrtf(max(0.f,1.f-y*y/(radius*radius*1.02f*1.02f)));
  float smooth=radius*.03f,h=max(0.f,min(1.f,.5f+.5f*(tube-base)/smooth));float target=base*(1-h)+tube*h+smooth*h*(1-h);
  float shoulder=1.f-Smoother01((fabsf(y)/radius-.56f)/.22f);
  float root=Smoother01((s-.02f)/.14f),end=1.f-Smoother01((s-.93f)/.095f);
  float shift=max(-radius*.48f,min(radius*.42f,target-depth))*shoulder*root*end*membership[i];
  step[i]=step[i]+f.down*shift;
 });
 for(unsigned k=0;k<paBodyNormalCount;k++)bodyStep[k]=RapheCollarShift(paBodyBefore[k]);
 for(unsigned k=0;k<paSeamCount;k++)step[paSeamR14[k]]=bodyStep[paSeamBody[k]];
 // A correction can only shrink during backoff. Build its face set once,
 // then reuse the unchanged triangle geometry through all quality passes.
 static unsigned activeFaces[rsIndexCount/3];static PreparedSurfaceLimit limits[rsIndexCount/3];unsigned activeCount=0;
 for(unsigned k=0;k<rsIndexCount;k+=3){unsigned a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];
  if(Dot(step[a],step[a])+Dot(step[b],step[b])+Dot(step[c],step[c])==0.f)continue;
  activeFaces[activeCount]=k;limits[activeCount++].Prepare(original[a],original[b],original[c]);}
 auto safeFraction=[&](unsigned face,V3 da,V3 db,V3 dc,float floor){
  const auto& limit=limits[face];V3 after=Cross(limit.e1+db-da,limit.e2+dc-da);
  if(Dot(limit.n,after)>=floor*Dot(limit.n,limit.n)&&Dot(after,after)>1e-12f)return 1.f;
  return limit.Evaluate(da,db,dc,floor);
 };
#ifdef RAPHE_PROFILE
 rapheProfile[1]+=PerfClock()-mark;mark=PerfClock();
#endif
 // Local backoff retains triangle orientation while allowing the smooth
 // profile to advance independently of unrelated small crease triangles.
 for(int pass=0;pass<6;pass++){
  bool changed=false;
  for(unsigned face=0;face<activeCount;face++){unsigned k=activeFaces[face],a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];
   float safe=safeFraction(face,step[a],step[b],step[c],.12f);if(safe<.9999f){step[a]=step[a]*safe;step[b]=step[b]*safe;step[c]=step[c]*safe;changed=true;}}
  for(unsigned k=0;k<paSeamCount;k++)bodyStep[paSeamBody[k]]=step[paSeamR14[k]];
  for(unsigned k=0;k<paBodyFaceCount;k++){unsigned a=paBodyNormalFaces[k*3],b=paBodyNormalFaces[k*3+1],c=paBodyNormalFaces[k*3+2];float safe=RSSafeFraction(paBodyBefore[a],paBodyBefore[b],paBodyBefore[c],bodyStep[a],bodyStep[b],bodyStep[c],.12f);if(safe<.9999f){bodyStep[a]=bodyStep[a]*safe;bodyStep[b]=bodyStep[b]*safe;bodyStep[c]=bodyStep[c]*safe;changed=true;}}
  // UV-split body vertices share one positional weld. A local safety limit
  // must apply to every member, or a collar correction can open those seams.
  V3 groupStep[paBodyGroupCount]{};float groupLength[paBodyGroupCount];for(auto& v:groupLength)v=1e30f;
  for(unsigned k=0;k<paBodyNormalCount;k++){unsigned g=paBodyNormalGroup[k];float length=Length(bodyStep[k]);if(length<groupLength[g]){groupLength[g]=length;groupStep[g]=bodyStep[k];}}
  for(unsigned k=0;k<paBodyNormalCount;k++)bodyStep[k]=groupStep[paBodyNormalGroup[k]];
  for(unsigned k=0;k<paSeamCount;k++)step[paSeamR14[k]]=bodyStep[paSeamBody[k]];if(!changed)break;
 }
 float fraction=1.f;
 for(unsigned face=0;face<activeCount;face++){unsigned k=activeFaces[face],a=rsIndices[k],b=rsIndices[k+1],c=rsIndices[k+2];fraction=min(fraction,safeFraction(face,step[a],step[b],step[c],.05f));rapheNormalNeeded[a]=rapheNormalNeeded[b]=rapheNormalNeeded[c]=true;}
 for(unsigned k=0;k<paBodyFaceCount;k++){unsigned a=paBodyNormalFaces[k*3],b=paBodyNormalFaces[k*3+1],c=paBodyNormalFaces[k*3+2];fraction=min(fraction,RSSafeFraction(paBodyBefore[a],paBodyBefore[b],paBodyBefore[c],bodyStep[a],bodyStep[b],bodyStep[c],.05f));}
 rapheSkinFraction=fraction;rapheSkinChanged=0;
#ifdef RAPHE_PROFILE
 rapheProfile[2]+=PerfClock()-mark;mark=PerfClock();
#endif
 for(unsigned i=0;i<rsCount;i++){rapheSkinDelta[i]=step[i]*fraction;rsPositions[i]=original[i]+rapheSkinDelta[i];rapheSkinChanged+=Length(rapheSkinDelta[i])>1e-5f;}
 for(unsigned k=0;k<paBodyNormalCount;k++){paBodyBefore[k]=paBodyBefore[k]+bodyStep[k]*fraction;memcpy(body+paBodyNormalIDs[k]*32,&paBodyBefore[k],12);}
#ifdef RAPHE_PROFILE
 rapheProfile[3]+=PerfClock()-mark;
#endif
}
