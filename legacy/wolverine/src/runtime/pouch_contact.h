#pragma once
// Geometry shared by the compliant dynamics solver and enclosing skin.
static bool cpReady=false;
static V3 cpBasis[2][3],cpOmega[2],cpNormal{0,1,0};
static float cpCompression[2]{},cpMinimumGap=1e9f,cpLastGap=0.f;
static V3 CPMul(V3 a,V3 b){return {a.x*b.x,a.y*b.y,a.z*b.z};}
static V3 CPDiv(V3 a,V3 b){return {a.x/b.x,a.y/b.y,a.z/b.z};}
static V3 CPRadii(int s){return CPMul(eggRadii[s],s?V3{5.724f/5.6240826f,4.86f/3.8670442f,7.81f/6.129288f}:V3{5.724f/5.6093187f,4.86f/3.871375f,7.93f/6.1588774f});}
// The fitted skin centres predate the larger contents/collision envelopes.
// Give both suspension targets room for their complete envelopes and skin;
// use the same target for shear, tether length, and initialization.
static V3 CPRestSlack(int s){
 float width=CPRadii(0).y+CPRadii(1).y;
 float desired=width*1.10f;
 float half=max(0.f,(desired-fabsf(constraintBallRest[1].y-constraintBallRest[0].y))*.5f);
 float forward=max(0.f,9.7f+1.35f*CPRadii(s).x-constraintBallRest[s].x);
 return {forward,(s?1.f:-1.f)*half,0};
}
static V3 CPRotate(V3 v,V3 axis,float angle){float c=cosf(angle),s=sinf(angle);return v*c+Cross(axis,v)*s+axis*(Dot(axis,v)*(1.f-c));}
// Positive rotation about lateral Y brings the upper poles forward (+X).
static V3 CPDesiredUp(int s){
 V3 suspension=BallAnchor(s)-ballNodes[s];
 // The cord can angle inward without forcing the entire ovoid to fan outward.
 suspension.y*=.10f;
 return CPRotate(Unit(suspension),{0,1,0},.436332313f);
}
static void CPEnsure(){if(cpReady)return;for(int s=0;s<2;s++){cpBasis[s][2]=CPDesiredUp(s);cpBasis[s][0]=Unit(Cross({0,1,0},cpBasis[s][2]));cpBasis[s][1]=Unit(Cross(cpBasis[s][2],cpBasis[s][0]));cpOmega[s]={};cpCompression[s]=0;}cpNormal={0,1,0};cpReady=true;}
static V3 CPOrient(V3 v,int s){return cpBasis[s][0]*v.x+cpBasis[s][1]*v.y+cpBasis[s][2]*v.z;}
static V3 CPUnorient(V3 v,int s){return {Dot(v,cpBasis[s][0]),Dot(v,cpBasis[s][1]),Dot(v,cpBasis[s][2])};}
static V3 CPPairPressure(V3 v,int s,bool inverse=false){float a=1.f-cpCompression[s],t=1.f/sqrtf(a);if(inverse){a=1.f/a;t=1.f/t;}return v*t+cpNormal*(Dot(v,cpNormal)*(a-t));}
static V3 CPTransform(V3 v,int s){return LobePressureOffset(CPPairPressure(CPOrient(v,s),s),s);}
static V3 CPInverse(V3 v,int s){return CPUnorient(CPPairPressure(LobePressureOffset(v,s,true),s,true),s);}
static V3 CPTranspose(V3 v,int s){return CPUnorient(CPPairPressure(LobePressureOffset(v,s),s),s);}
static V3 CPCenter(int s){return ballNodes[s]-cpBasis[s][2]*(CPRadii(s).z*.23f);}
static V3 CPSupportLocal(V3 n,V3 r){
 float h=sqrtf(n.x*n.x*r.x*r.x+n.y*n.y*r.y*r.y),v=n.z*r.z;
 if(h<1e-8f)return {0,0,copysignf(r.z,v)};
 // The tapered ellipse has strictly negative support-objective curvature.
 // Newton from the untapered ellipse converges without the quantization of
 // a 22-step float bisection, and is cheaper during normal refinement.
 double z=double(v)/sqrt(double(h)*h+double(v)*v),ratio=double(v)/h;
 z=max(-.999999999999,min(.999999999999,z));
 for(int i=0;i<6;i++){
  double radial=sqrt(max(1e-24,1.-z*z));
  double derivative=-.13*radial-(1.-.13*z)*z/radial+ratio;
  double curvature=(-1.+.39*z-.26*z*z*z)/(radial*radial*radial);
  z=max(-.999999999999,min(.999999999999,z-derivative/curvature));
 }
 float section=float((1.-.13*z)*sqrt(max(0.,1.-z*z)));
 return {r.x*r.x*n.x/h*section,r.y*r.y*n.y/h*section,float(r.z*z)};
}
static V3 CPSupport(int s,V3 n){return CPTransform(CPSupportLocal(CPTranspose(n,s),CPRadii(s)),s);}
static float CPLevel(V3 p,int s,V3 radius){V3 a=CPDiv(CPInverse(p-CPCenter(s),s),radius);float t=1.f-.13f*max(-1.f,min(1.f,a.z));a.x/=t;a.y/=t;return Length(a)-1.f;}
static void CPShift(int s,V3 shift){ballNodes[s]=ballNodes[s]+shift;ballPrevious[s]=ballPrevious[s]+shift;nutNodes[s]=nutNodes[s]+shift;nutPrevious[s]=nutPrevious[s]+shift;neckNodes[s]=neckNodes[s]+shift*.5f;neckPrevious[s]=neckPrevious[s]+shift*.5f;}
static float CPTetherLimit(int s){
 // Derive reach from the authored suspension, including size and Hang.
 // Contact is allowed modest slack but cannot indefinitely lengthen it.
 return max(2.45f,Length(constraintBallRest[s]+CPRestSlack(s)-RestBallAnchor(s)))*1.12f+CPRadii(s).y*.15f;
}
