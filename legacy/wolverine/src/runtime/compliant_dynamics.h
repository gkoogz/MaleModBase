#pragma once
// One XPBD solve for rod strain, suspension and all contacts.
// Positions are integrated once, constraints share generalized masses, and
// velocities are reconstructed once from the accepted configuration.
static const int pdBody0=shaftNodeCount,pdCount=shaftNodeCount+2;
struct PDConstraint {
 float normal=0.f; V3 tangent{};
 int a=-1,b=-1,rod=-1; float t=0.f,normalImpulse=0.f;
 V3 ra{},rb{},n{},fixedVelocity{},tangentImpulse{};
 bool anchorReady=false;V3 localA{},localB{},oldA{},oldB{},fixed{};float anchorT=0.f;V3 geometryNormal{};
};
static PDConstraint* pdContacts[128];
static int pdContactCount=0;
static bool pdVelocityPass=false;

static V3 pdPosition[pdCount],pdOldPosition[pdCount],pdVelocity[pdCount];
static V3 pdOldBasis[2][3],pdThigh[4],pdOldThigh[4];
static float pdInvMass[pdCount],pdInvInertia[2],pdMaxTetherRatio=0.f,pdMinimumGap=1e9f;
static bool pdReady=false;
static float pdPairForce=0.f,pdShaftForce[2]{};
static V3 pdPressureDirection[2]={{0,-1,0},{0,1,0}};
static void ResetCompliantDynamics(){pdReady=false;pdMaxTetherRatio=0.f;pdMinimumGap=1e9f;pdPairForce=0.f;pdShaftForce[0]=pdShaftForce[1]=0.f;}
static void PDRotateBody(int s,V3 rotation){
 float angle=Length(rotation);if(angle<1e-9f)return;V3 axis=rotation/angle;
 const float cosine=cosf(angle),sine=sinf(angle);
 for(int j=0;j<3;j++){
  const V3 v=cpBasis[s][j];
  cpBasis[s][j]=v*cosine+Cross(axis,v)*sine+axis*(Dot(axis,v)*(1.f-cosine));
 }
 cpBasis[s][2]=Unit(cpBasis[s][2]);cpBasis[s][0]=Unit(cpBasis[s][0]-cpBasis[s][2]*Dot(cpBasis[s][0],cpBasis[s][2]));cpBasis[s][1]=Cross(cpBasis[s][2],cpBasis[s][0]);
}
static void PDSync(){for(int s=0;s<2;s++)ballNodes[s]=pdPosition[pdBody0+s]+cpBasis[s][2]*(CPRadii(s).z*.23f);}
static float PDEffectiveMass(int id,V3 arm,V3 direction){
 if(id<0)return 0.f;float w=pdInvMass[id];if(id>=pdBody0){V3 cross=Cross(arm,direction);w+=pdInvInertia[id-pdBody0]*Dot(cross,cross);}return w;
}
static void PDApply(int id,V3 arm,V3 impulse){
 if(id<0)return;pdPosition[id]=pdPosition[id]+impulse*pdInvMass[id];
 if(id>=pdBody0)PDRotateBody(id-pdBody0,Cross(arm,impulse)*pdInvInertia[id-pdBody0]);
}
static V3 PDPreviousPoint(int id,V3 arm){
 if(id<pdBody0)return pdOldPosition[id]+arm;
 int s=id-pdBody0;V3 local=CPUnorient(arm,s);return pdOldPosition[id]+pdOldBasis[s][0]*local.x+pdOldBasis[s][1]*local.y+pdOldBasis[s][2]*local.z;
}
// Sequential impulses remove the artificial rebound of penetration recovery.
// The positional impulse is the initial impulse, so releasing it is bounded;
// both friction and normal response act on relative material-point velocity.
static V3 PDPointVelocity(int id,V3 arm){
 if(id<0)return {};V3 v=pdVelocity[id];
 if(id>=pdBody0)v=v+Cross(cpOmega[id-pdBody0],arm);return v;
}
static void PDVelocityImpulse(int id,V3 arm,V3 impulse){
 if(id<0)return;pdVelocity[id]=pdVelocity[id]+impulse*pdInvMass[id];
 if(id>=pdBody0)cpOmega[id-pdBody0]=cpOmega[id-pdBody0]+Cross(arm,impulse)*pdInvInertia[id-pdBody0];
}
static void PDRecord(PDConstraint& c,int a,V3 ra,int b,V3 rb,V3 n,V3 fixedVelocity,int rod=-1,float t=0.f){
 if(!pdVelocityPass)return;
 c.a=a;c.ra=ra;c.b=b;c.rb=rb;c.n=n;c.fixedVelocity=fixedVelocity;c.rod=rod;c.t=t;
 pdContacts[pdContactCount++]=&c;
}
static void PDSolveContactVelocities(float dt){
 for(int k=0;k<pdContactCount;k++){
  PDConstraint& c=*pdContacts[k];c.normalImpulse=c.normal/dt;c.tangentImpulse=c.tangent/dt;
 }
 for(int it=0;it<10;it++)for(int k=0;k<pdContactCount;k++){
  PDConstraint& c=*pdContacts[k];if(c.normal<=0.f)continue;
  auto relative=[&](){V3 v=PDPointVelocity(c.a,c.ra);
   return c.rod>=0?v-pdVelocity[c.rod]*(1-c.t)-pdVelocity[c.rod+1]*c.t:v-(c.b>=0?PDPointVelocity(c.b,c.rb):c.fixedVelocity);};
  auto mass=[&](V3 n){return PDEffectiveMass(c.a,c.ra,n)+(c.rod>=0?
   pdInvMass[c.rod]*(1-c.t)*(1-c.t)+pdInvMass[c.rod+1]*c.t*c.t:PDEffectiveMass(c.b,c.rb,n));};
  auto apply=[&](V3 impulse){PDVelocityImpulse(c.a,c.ra,impulse);
   if(c.rod>=0){PDVelocityImpulse(c.rod,{},impulse*(-(1-c.t)));PDVelocityImpulse(c.rod+1,{},impulse*(-c.t));}
   else PDVelocityImpulse(c.b,c.rb,impulse*-1.f);};
  float w=mass(c.n);if(w<1e-8f)continue;
  float next=max(0.f,c.normalImpulse-Dot(relative(),c.n)/w);
  apply(c.n*(next-c.normalImpulse));c.normalImpulse=next;
  V3 v=relative();v=v-c.n*Dot(v,c.n);float speed=Length(v);if(speed<1e-8f)continue;
  V3 tangent=v/speed;float wt=mass(tangent);if(wt<1e-8f)continue;
  V3 trial=c.tangentImpulse-v/wt;trial=trial-c.n*Dot(trial,c.n);
  float length=Length(trial),limit=.48f*c.normalImpulse;
  if(length>limit)trial=trial*(.32f*c.normalImpulse/max(length,1e-8f));
  apply(trial-c.tangentImpulse);c.tangentImpulse=trial;
 }
}
static void PDAnchorContact(PDConstraint& c,int a,V3 ra,int b,V3 rb,V3 fixed,V3 oldFixed,int rod=-1,float t=0.f){
 if(c.normal<=0.f)return;
 if(!c.anchorReady){
  c.localA=a>=pdBody0?CPUnorient(ra,a-pdBody0):ra;c.oldA=PDPreviousPoint(a,ra);
  c.localB=b>=pdBody0?CPUnorient(rb,b-pdBody0):rb;c.oldB=rod>=0?pdOldPosition[rod]*(1-t)+pdOldPosition[rod+1]*t:(b>=0?PDPreviousPoint(b,rb):oldFixed);
  c.fixed=fixed;c.anchorT=t;c.anchorReady=true;
 }
}
static void PDStaticFriction(PDConstraint& c,int a,V3 ra,int b,V3 rb,V3 fixed,V3 oldFixed,V3 normal,int rod=-1,float t=0.f){
 if(!c.anchorReady)return;
 ra=a>=pdBody0?CPOrient(c.localA,a-pdBody0):c.localA;
 rb=b>=pdBody0?CPOrient(c.localB,b-pdBody0):c.localB;t=c.anchorT;
 if(c.normal<=0.f){
  // A released normal constraint cannot retain a tangential attraction.
  V3 impulse=c.tangent*-1.f;c.tangent={};PDApply(a,ra,impulse);
  if(rod>=0){pdPosition[rod]=pdPosition[rod]-impulse*(pdInvMass[rod]*(1-t));pdPosition[rod+1]=pdPosition[rod+1]-impulse*(pdInvMass[rod+1]*t);}
  else PDApply(b,rb,impulse*-1.f);return;
 }
 V3 pointB=rod>=0?pdPosition[rod]*(1-t)+pdPosition[rod+1]*t:(b>=0?pdPosition[b]+rb:c.fixed);
 V3 slip=pdPosition[a]+ra-c.oldA-(pointB-c.oldB);slip=slip-normal*Dot(slip,normal);
 float length=Length(slip);if(length<1e-8f)return;V3 tangent=slip/length;
 float w=PDEffectiveMass(a,ra,tangent)+(rod>=0?pdInvMass[rod]*(1-t)*(1-t)+pdInvMass[rod+1]*t*t:PDEffectiveMass(b,rb,tangent));
 if(w<1e-8f)return;V3 trial=c.tangent-slip/w;trial=trial-normal*Dot(trial,normal);
 // Continuous projection onto the static cone. Kinetic friction is applied
 // in the velocity solve, never as a discontinuous position-level switch.
 float magnitude=Length(trial),limit=.48f*c.normal;if(magnitude>limit)trial=trial*(limit/max(magnitude,1e-8f));
 V3 impulse=trial-c.tangent;c.tangent=trial;PDApply(a,ra,impulse);
 if(rod>=0){pdPosition[rod]=pdPosition[rod]-impulse*(pdInvMass[rod]*(1-t));pdPosition[rod+1]=pdPosition[rod+1]-impulse*(pdInvMass[rod+1]*t);}
 else PDApply(b,rb,impulse*-1.f);
}
static void PDContact(PDConstraint& c,int a,V3 ra,int b,V3 rb,V3 fixed,V3 oldFixed,V3 normal,float separation,float compliance,float dt){
 PDRecord(c,a,ra,b,rb,normal,(fixed-oldFixed)/dt);
 float w=PDEffectiveMass(a,ra,normal)+PDEffectiveMass(b,rb,normal),alpha=compliance/(dt*dt);
 if(w<1e-9f)return;float next=max(0.f,c.normal-(separation+alpha*c.normal)/(w+alpha)),change=next-c.normal;c.normal=next;
 PDAnchorContact(c,a,ra,b,rb,fixed,oldFixed);
 PDApply(a,ra,normal*change);PDApply(b,rb,normal*(-change));
 PDStaticFriction(c,a,ra,b,rb,fixed,oldFixed,normal);
}
static void PDDistance(int a,int b,float rest,float compliance,float& lambda,float dt){
 V3 delta=pdPosition[b]-pdPosition[a];float distance=Length(delta);if(distance<1e-7f)return;
 float alpha=compliance/(dt*dt),dl=(-(distance-rest)-alpha*lambda)/(pdInvMass[a]+pdInvMass[b]+alpha);lambda+=dl;
 V3 impulse=delta*(dl/distance);pdPosition[a]=pdPosition[a]-impulse*pdInvMass[a];pdPosition[b]=pdPosition[b]+impulse*pdInvMass[b];
}
struct PDBendData {float alpha,gamma,factor;V3 oldValue;};
static PDBendData PDPrepareBend(int i,float compliance,float dt){
 float alpha=compliance/(dt*dt),w=pdInvMass[i-1]+4.f*pdInvMass[i]+pdInvMass[i+1];
 // Kelvin-Voigt bending: dissipate changes in curvature, leaving rigid
 // translation and a common swing untouched. The ratio follows BOUNCE.
 float ratio=.10f+.40f*(1.f-max(0.f,min(1.f,physUI[2]/100.f)));
 float gamma=2.f*ratio*sqrtf(compliance/max(w,1e-8f))/dt;
 V3 oldValue=pdOldPosition[i-1]-pdOldPosition[i]*2.f+pdOldPosition[i+1];
 return {alpha,gamma,-1.f/((1.f+gamma)*w+alpha),oldValue};
}
static void PDBendPrepared(int i,const PDBendData& data,V3& lambda){
 V3 value=pdPosition[i-1]-pdPosition[i]*2.f+pdPosition[i+1];
 V3 dl=(value+lambda*data.alpha+(value-data.oldValue)*data.gamma)*data.factor;lambda=lambda+dl;
 pdPosition[i-1]=pdPosition[i-1]+dl*pdInvMass[i-1];pdPosition[i]=pdPosition[i]-dl*(2.f*pdInvMass[i]);pdPosition[i+1]=pdPosition[i+1]+dl*pdInvMass[i+1];
}
static void PDBend(int i,float compliance,V3& lambda,float dt){PDBendPrepared(i,PDPrepareBend(i,compliance,dt),lambda);}
// The attachment is transported by the proximal Hermite span. Its Jacobian
// transmits the equal reaction to that span; treating it as a moving wall
// silently supplied energy whenever a large support pulled on the shaft.
static V3 pdPreviousAnchor[2],pdPreviousMaterial[2];
static V3 PDLateralSlack(int s){return CPRestSlack(s);}
static V3 PDMaterialTarget(int s){
 V3 rc{},rt{},lc{},lt{};SampleRestShaftFrame(.12f,rc,rt);SampleShaftChain(.12f,lc,lt);
 return lc+RotateFromTo(constraintBallRest[s]+PDLateralSlack(s)-rc,rt,lt);
}
static float PDSuspensionMassFiniteDifference(int s,V3 arm,V3 n,V3 gradient[shaftNodeCount],bool material=false){
 for(int i=0;i<shaftNodeCount;i++){shaftNodes[i]=pdPosition[i];gradient[i]={};}
 float w=PDEffectiveMass(pdBody0+s,arm,n);
 // Only the four nodes of the attachment's Hermite span contribute.
 // Nodes 0 and 1 are the prescribed pelvic boundary.
 const float epsilon=.005f;
 int span=int(.12f*(shaftNodeCount-1));
 for(int i=max(2,span-1);i<=min(shaftNodeCount-1,span+2);i++)for(int axis=0;axis<3;axis++){
  V3 direction=axis==0?V3{1,0,0}:axis==1?V3{0,1,0}:V3{0,0,1};
  shaftNodes[i]=pdPosition[i]+direction*epsilon;V3 plus=material?PDMaterialTarget(s):BallAnchor(s);
  shaftNodes[i]=pdPosition[i]-direction*epsilon;V3 minus=material?PDMaterialTarget(s):BallAnchor(s);
  shaftNodes[i]=pdPosition[i];gradient[i]=gradient[i]-direction*Dot((plus-minus)/(2*epsilon),n);
 }
 for(int i=2;i<shaftNodeCount;i++)w+=pdInvMass[i]*Dot(gradient[i],gradient[i]);return w;
}
// Exact derivative of the Hermite attachment and its minimal-rotation frame.
// Besides avoiding differencing noise, this removes repeated curve samples
// from the inner solve. The antiparallel singularity uses the finite fallback.
static float PDSuspensionMass(int s,V3 arm,V3 n,V3 gradient[shaftNodeCount],bool material=false){
 for(int i=0;i<shaftNodeCount;i++){shaftNodes[i]=pdPosition[i];gradient[i]={};}
 float u=.12f*(shaftNodeCount-1);int span=int(u);float q=u-span,q2=q*q,q3=q2*q;
 if(span<1||span+2>=shaftNodeCount)return PDSuspensionMassFiniteDifference(s,arm,n,gradient,material);
 float h00=2*q3-3*q2+1,h10=q3-2*q2+q,h01=-2*q3+3*q2,h11=q3-q2;
 float d00=6*q2-6*q,d10=3*q2-4*q+1,d01=-d00,d11=3*q2-2*q;
 float weights[4]={-.5f*h10,h00-.5f*h11,h01+.5f*h10,.5f*h11};
 float derivatives[4]={-.5f*d10,d00-.5f*d11,d01+.5f*d10,.5f*d11};
 V3 raw{};for(int i=0;i<4;i++)raw=raw+pdPosition[span-1+i]*derivatives[i];
 float length=Length(raw);V3 tangent=Unit(raw),rc{},rt{};SampleRestShaftFrame(.12f,rc,rt);rt=Unit(rt);
 V3 offset=(material?constraintBallRest[s]+PDLateralSlack(s):RestBallAnchor(s))-rc,axis=Cross(rt,tangent);
 float denominator=1.f+Dot(rt,tangent);
 if(length<1e-5f||denominator<.01f)return PDSuspensionMassFiniteDifference(s,arm,n,gradient,material);
 V3 rotationGradient{};float projection=Dot(axis,offset);
 for(int j=0;j<3;j++){
  V3 direction=j==0?V3{1,0,0}:j==1?V3{0,1,0}:V3{0,0,1},da=Cross(rt,direction);
  float dc=Dot(rt,direction);V3 derivative=offset*dc+Cross(da,offset)+(da*projection+axis*Dot(da,offset))/denominator-axis*(projection*dc/(denominator*denominator));
  rotationGradient=rotationGradient+direction*Dot(n,derivative);
 }
 V3 tangentGradient=(rotationGradient-tangent*Dot(tangent,rotationGradient))/length;
 for(int i=0;i<4;i++)gradient[span-1+i]=(n*weights[i]+tangentGradient*derivatives[i])*-1.f;
 float w=PDEffectiveMass(pdBody0+s,arm,n);for(int i=2;i<shaftNodeCount;i++)w+=pdInvMass[i]*Dot(gradient[i],gradient[i]);return w;
}
static void PDApplySuspension(int s,V3 arm,V3 n,const V3 gradient[shaftNodeCount],float dl){
 PDApply(pdBody0+s,arm,n*dl);
 for(int i=2;i<shaftNodeCount;i++){pdPosition[i]=pdPosition[i]+gradient[i]*(pdInvMass[i]*dl);shaftNodes[i]=pdPosition[i];}
}
// The surrounding suspension tissue carries shear as well as tension.
// Two compliant transverse strains prevent an unphysical free orbit about
// a zero-bending-stiffness tether. They share the rod's reaction Jacobian.
struct PDSuspensionData {float rest,hard,compliance,shearCompliance,ratio;};
static PDSuspensionData PDPrepareSuspension(int s){
 float rest=max(2.45f,Length(constraintBallRest[s]+CPRestSlack(s)-RestBallAnchor(s)));
 float stiff=max(0.f,min(1.f,physUI[4]/100.f));
 return {rest,rest*1.12f+CPRadii(s).y*.15f,.000053f*expf(-3.f*stiff),
  .000022f*expf((.5f-physUI[4]/100.f)*3.f),.10f+.60f*(1.f-max(0.f,min(1.f,physUI[6]/100.f)))};
}
static void PDSuspensionShear(int s,float lambda[2],float dt,const PDSuspensionData& data){
 int id=pdBody0+s;V3 target=PDMaterialTarget(s),axis=Unit(target-BallAnchor(s));
 V3 directions[2]={Unit(Cross({0,1,0},axis)),{}};directions[1]=Cross(axis,directions[0]);
 // Let contacting lobes settle laterally without fighting a tight rest frame.
 // A modestly firmer neck web checks large swings without a travel barrier.
 float compliance=data.shearCompliance;
 for(int k=0;k<2;k++){
  V3 arm=cpBasis[s][2]*(CPRadii(s).z*.23f),n=directions[k],gradient[shaftNodeCount];target=PDMaterialTarget(s);
  float value=Dot(pdPosition[id]+arm-target,n),w=PDSuspensionMass(s,arm,n,gradient,true),alpha=compliance/(dt*dt);
  float gamma=2.1f*sqrtf(compliance/max(w,1e-8f))/dt;
  float rate=Dot(pdPosition[id]+arm-PDPreviousPoint(id,arm)-(target-pdPreviousMaterial[s]),n);
  float dl=(-value-alpha*lambda[k]-gamma*rate)/((1+gamma)*w+alpha);lambda[k]+=dl;
  PDApplySuspension(s,arm,n,gradient,dl);
 }
}
static void PDSuspension(int s,float& lambda,float& stop,float dt,const PDSuspensionData& data){
 int id=pdBody0+s;V3 arm=cpBasis[s][2]*(CPRadii(s).z*.23f),anchor=BallAnchor(s),delta=pdPosition[id]+arm-anchor;
 float distance=Length(delta);if(distance<1e-7f)return;
 V3 n=delta/distance,gradient[shaftNodeCount];float w=PDSuspensionMass(s,arm,n,gradient);
 float rest=data.rest,compliance=data.compliance,alpha=compliance/(dt*dt),ratio=data.ratio;
 float gamma=2.f*ratio*sqrtf(compliance/max(w,1e-8f))/dt;
 float rate=Dot(n,pdPosition[id]+arm-PDPreviousPoint(id,arm)-(anchor-pdPreviousAnchor[s]));
 float next=min(0.f,lambda+(-(distance-rest)-alpha*lambda-gamma*rate)/((1+gamma)*w+alpha));
 PDApplySuspension(s,arm,n,gradient,next-lambda);lambda=next;
 arm=cpBasis[s][2]*(CPRadii(s).z*.23f);delta=pdPosition[id]+arm-BallAnchor(s);distance=Length(delta);n=Unit(delta);
 w=PDSuspensionMass(s,arm,n,gradient);float hard=data.hard,nextStop=min(0.f,stop-(distance-hard)/max(w,1e-8f));
 PDApplySuspension(s,arm,n,gradient,nextStop-stop);stop=nextStop;
}
static V3 PDClosest(V3 p,V3 a,V3 b,float& t){V3 e=b-a;t=max(0.f,min(1.f,Dot(p-a,e)/max(1e-8f,Dot(e,e))));return a+e*t;}
// Symmetric shaft contact also admits a perched solution above the rod.
// Bound the contents below the proximal attachment in character space.
// Keeping down vertical avoids driving them into the pelvis when the shaft
// points down and its rotated ventral direction points backwards.
static float PDVentralHeight(const V3* positions){
 constexpr float u=.12f*(shaftNodeCount-1);constexpr int span=int(u);
 constexpr float q=u-span,q2=q*q,q3=q2*q;
 return positions[span-1].z*(-.5f*(q3-2*q2+q))+
  positions[span].z*(2*q3-3*q2+1-.5f*(q3-q2))+
  positions[span+1].z*(-2*q3+3*q2+.5f*(q3-2*q2+q))+
  positions[span+2].z*(.5f*(q3-q2));
}
static void PDKeepPouchVentral(float correction[2]){
 float ceiling=PDVentralHeight(pdPosition)-logicalShaftBodyRadius*.10f;
 for(int s=0;s<2;s++)if(pdPosition[pdBody0+s].z>ceiling){
  correction[s]+=ceiling-pdPosition[pdBody0+s].z;pdPosition[pdBody0+s].z=ceiling;
 }
}
static V3 PDSkinSupport(int s,V3 n){
 V3 r=CPRadii(s),scale=CPDiv(r,s?V3{5.724f,4.86f,7.81f}:V3{5.724f,4.86f,7.93f});
 // Match the enclosing skin used by ApplyPouchSurface, not an enlarged
 // sphere or the smaller internal support alone.
 V3 outer=r+CPMul({.80f,.95f,.65f},scale);
 return CPTransform(CPSupportLocal(CPTranspose(n,s),outer),s);
}
static void PDRecoverPouchVentral(){
 float height=PDVentralHeight(pdPosition);
 for(int s=0;s<2;s++){
  int id=pdBody0+s;float depth=height-pdPosition[id].z;
  if(depth>=0.f)continue;
  // Recover an existing invalid state outside the shaft before recording
  // history. Merely putting its centre on the underside leaves the whole
  // ovoid inside the rod, making contact separation supply a large impulse.
  float clearance=logicalShaftBodyRadius+PDSkinSupport(s,{0,0,1}).z+.05f;
  V3 target=PDMaterialTarget(s)-cpBasis[s][2]*(CPRadii(s).z*.23f);
  target.z=min(target.z,height-clearance);
  pdPosition[id]=target;
  pdVelocity[id]={};cpOmega[s]={};
 }
}
static float pdBroadRadius[2];
static bool PDCapsuleMayContact(const PDConstraint& c,int s,V3 a,V3 b,float radius){
 if(c.normal>0.f)return true;
 float t;V3 q=PDClosest(pdPosition[pdBody0+s],a,b,t);
 float limit=pdBroadRadius[s]+radius+.01f;
 return Dot(pdPosition[pdBody0+s]-q,pdPosition[pdBody0+s]-q)<=limit*limit;
}
// A centre-to-centre normal is only correct for spheres. For the ovoids,
// find the separating support plane; its stationary tangential gradient
// supplies the actual surface normal and the correct rotational moment arm.
static V3 PDContactNormal(PDConstraint& c,int s,V3 a,V3 b,V3& arm,V3& q,float& t){
 V3 center=pdPosition[pdBody0+s];q=PDClosest(center,a,b,t);
 V3 n=Length(c.geometryNormal)>.5f?c.geometryNormal:Unit(center-q);
 float bound=Length(CPRadii(s))*2.f+Length(center-q);
 for(int k=0;k<6;k++){
  arm=PDSkinSupport(s,n*-1.f);q=PDClosest(center+arm,a,b,t);
  V3 delta=center+arm-q,gradient=delta-n*Dot(delta,n);
  n=Unit(n+gradient/max(bound,1.f));
 }
 c.geometryNormal=n;arm=PDSkinSupport(s,n*-1.f);q=PDClosest(center+arm,a,b,t);return n;
}
static void PDPair(PDConstraint& c,float dt){
 PDSync();V3 delta=pdPosition[pdBody0+1]-pdPosition[pdBody0];
 V3 n=Length(c.geometryNormal)>.5f?c.geometryNormal:Unit(delta),ra{},rb{};
 float bound=Length(delta)+2.f*(Length(CPRadii(0))+Length(CPRadii(1)));
 for(int k=0;k<6;k++){
  ra=CPSupport(0,n);rb=CPSupport(1,n*-1.f);V3 v=delta+rb-ra;
  n=Unit(n+(v-n*Dot(v,n))/max(bound,1.f));
 }
 c.geometryNormal=n;ra=CPSupport(0,n);rb=CPSupport(1,n*-1.f);float gap=Dot(pdPosition[pdBody0+1]+rb-pdPosition[pdBody0]-ra,n);
 PDContact(c,pdBody0+1,rb,pdBody0,ra,{},{},n,gap-.025f,0.f,dt);
}
static void PDBodyCapsule(PDConstraint& c,int s,V3 a,V3 b,V3 oldA,V3 oldB,float radius,float dt){
 if(!PDCapsuleMayContact(c,s,a,b,radius))return;
 int id=pdBody0+s;float t;V3 q{},arm{},n=PDContactNormal(c,s,a,b,arm,q,t);
 float gap=Dot(pdPosition[id]+arm-q,n)-radius;
 PDContact(c,id,arm,-1,{},q,oldA+(oldB-oldA)*t,n,gap,.000002f,dt);
}
static void PDRodBody(PDConstraint& c,int s,int j,float dt){
 // The measured radius is a mean barrel radius, not the ventral/raphe extent.
 // Cover the authored bulge (up to 1.35x in posed captures) with modest slack.
 float radius=logicalShaftBodyRadius*(1.20f+.20f*min(1.f,float(j)/4.f));
 if(!PDCapsuleMayContact(c,s,pdPosition[j],pdPosition[j+1],radius))return;
 int id=pdBody0+s;float t;V3 q{},arm{},n=PDContactNormal(c,s,pdPosition[j],pdPosition[j+1],arm,q,t);
 float gap=Dot(pdPosition[id]+arm-q,n)-radius;
 PDRecord(c,id,arm,-1,{},n,{},j,t);
 float w0=pdInvMass[j]*(1-t)*(1-t),w1=pdInvMass[j+1]*t*t,body=PDEffectiveMass(id,arm,n),alpha=.0000001f/(dt*dt);
 float next=max(0.f,c.normal-(gap+alpha*c.normal)/(body+w0+w1+alpha)),dl=next-c.normal;c.normal=next;
 PDAnchorContact(c,id,arm,-1,{},{},{},j,t);
 PDApply(id,arm,n*dl);pdPosition[j]=pdPosition[j]-n*(dl*pdInvMass[j]*(1-t));pdPosition[j+1]=pdPosition[j+1]-n*(dl*pdInvMass[j+1]*t);
 PDStaticFriction(c,id,arm,-1,{},{},{},n,j,t);
}
// Render input is sampled once and interpolated on the simulation clock.
// In particular, changing a rest shape must not teleport a dynamic centre.
struct PDInput {
 float length,radius,angle; V3 radii[2],rest[2],centers[shaftRestSampleCount],thigh[4];
 float gait,side;
};
static PDInput pdInput;
static bool pdInputReady=false;
static PDInput PDReadInput(float gait,float side){
 PDInput x{};x.length=constraintRestLength;x.radius=logicalShaftBodyRadius;x.angle=sliderValues[4];x.gait=gait;x.side=side;
 memcpy(x.radii,eggRadii,sizeof(x.radii));memcpy(x.rest,constraintBallRest,sizeof(x.rest));memcpy(x.centers,shaftRestCenters,sizeof(x.centers));
 CollisionCapsules(x.thigh[0],x.thigh[1],x.thigh[2],x.thigh[3]);return x;
}
static void PDSetInput(const PDInput& a,const PDInput& b,float t){
 constraintRestLength=a.length+(b.length-a.length)*t;logicalShaftBodyRadius=a.radius+(b.radius-a.radius)*t;sliderValues[4]=a.angle+(b.angle-a.angle)*t;
 for(int i=0;i<2;i++){eggRadii[i]=a.radii[i]+(b.radii[i]-a.radii[i])*t;constraintBallRest[i]=a.rest[i]+(b.rest[i]-a.rest[i])*t;}
 for(int i=0;i<shaftRestSampleCount;i++)shaftRestCenters[i]=a.centers[i]+(b.centers[i]-a.centers[i])*t;
 memcpy(pdOldThigh,pdThigh,sizeof(pdThigh));for(int i=0;i<4;i++)pdThigh[i]=a.thigh[i]+(b.thigh[i]-a.thigh[i])*t;
}
static void StepConstraintSolver(float dt,float gait,float side){
 if(!constraintSolverReady)InitializeConstraintSolver();CPEnsure();constraintSolverState=physicsState;StepRootSuspension(dt,gait,side);
 for(int s=0;s<2;s++){
  V3 r=CPRadii(s),scale=CPDiv(r,s?V3{5.724f,4.86f,7.81f}:V3{5.724f,4.86f,7.93f});
  // The tapered ovoid lies within max(1.13*radial, axial). Each of the two
  // bounded pressure transforms expands lengths by at most 1/sqrt(.935).
  V3 outer=r+CPMul({.80f,.95f,.65f},scale);
  pdBroadRadius[s]=max(1.13f*max(outer.x,outer.y),outer.z)*1.08f;
 }
 cpNormal=Unit(CPCenter(1)-CPCenter(0));
 // Deformation is frozen during a constraint solve. Updating it after the
 // last contact would reintroduce penetration and force feedback jitter.
 for(int s=0;s<2;s++){
  float blend=1.f-expf(-dt/.12f);cpCompression[s]+=(min(.065f,pdPairForce*.000035f)-cpCompression[s])*blend;
  lobeCompression[s]+=(min(.065f,pdShaftForce[s]*.000025f)-lobeCompression[s])*blend;
  lobePressureNormal[s]=Unit(lobePressureNormal[s]*(1.f-blend)+pdPressureDirection[s]*blend);
 }
 bool initialized=pdReady;
 if(!pdReady){for(int i=0;i<pdCount;i++)pdVelocity[i]={};for(int s=0;s<2;s++)pdPreviousAnchor[s]=BallAnchor(s);for(int s=0;s<2;s++)pdPreviousMaterial[s]=PDMaterialTarget(s);pdReady=true;}
 float shaftMass=.75f+physValues[1]*.0125f,bodyMass=.75f+physValues[5]*.0125f;
 for(int i=0;i<shaftNodeCount;i++){pdPosition[i]=shaftNodes[i];pdInvMass[i]=i<2?0.f:1.f/shaftMass;}
 // Include the attached neck tissue in angular inertia: contact impulses
 // should move the suspended contents more readily than roll them over.
 for(int s=0;s<2;s++){if(!initialized)pdPosition[pdBody0+s]=CPCenter(s);pdInvMass[pdBody0+s]=1.f/bodyMass;pdInvInertia[s]=1.5f/(bodyMass*Dot(CPRadii(s),CPRadii(s)));for(int j=0;j<3;j++)pdOldBasis[s][j]=cpBasis[s][j];}
 PDRecoverPouchVentral();
 memcpy(pdOldPosition,pdPosition,sizeof(pdPosition));
 float shaftDrag=.9f+(100.f-physUI[2])*.018f,bodyDrag=1.f+(100.f-physUI[6])*.025f;
 for(int i=2;i<pdCount;i++){
  float response=i<pdBody0?(.65f+physValues[3]*.009f):(.65f+physValues[7]*.009f),gravity=i<pdBody0?ModeValue(5.f,42.f,110.f):72.f;
  V3 acceleration{0.f,side*86.f*response,gait*86.f*response-gravity};float drag=i<pdBody0?shaftDrag:bodyDrag;
  pdVelocity[i]=(pdVelocity[i]+acceleration*dt)*expf(-drag*dt);pdPosition[i]=pdPosition[i]+pdVelocity[i]*dt;
 }
 for(int s=0;s<2;s++){
  // The suspension has a material frame: free axial spin would twist the
  // attached skin indefinitely even after the centre of mass had settled.
  V3 up=cpBasis[s][2],forward=Unit(Cross({0,1,0},up));
  float twist=atan2f(Dot(up,Cross(cpBasis[s][0],forward)),Dot(cpBasis[s][0],forward));
  float torsion=20.f*expf((physUI[4]/100.f-.5f)*2.f);
  // Reinforce upright support through the existing orientation spring only.
  // Stronger restoring torque and damping resist inversion without new work.
  V3 torque=Cross(up,CPDesiredUp(s))*20000.f+up*(twist*torsion);cpOmega[s]=(cpOmega[s]+torque*dt)*expf(-120.f*dt);PDRotateBody(s,cpOmega[s]*dt);
 }
 V3 root=ShaftRoot(),direction=LiveRootDirection();float segment=constraintRestLength/(shaftNodeCount-1);
 pdPosition[0]=root;pdPosition[1]=root+direction*segment;
 float lengthLambda[shaftNodeCount]{},tetherLambda[2]{},stopLambda[2]{},shearLambda[2][2]{};V3 bendLambda[shaftNodeCount]{};float ventralCorrection[2]{};
 PDSuspensionData suspensionData[2]={PDPrepareSuspension(0),PDPrepareSuspension(1)};
 PDConstraint pair,thigh[2][2],pelvis[2],rodContact[2][shaftNodeCount],rodThigh[shaftNodeCount][2];
 float stiffness=max(0.f,min(1.f,physUI[0]/100.f));float bendCompliance=ModeValue(.00000001f,.00008f,.0015f)*expf((.5f-stiffness)*3.f);
 // These depend on this substep's rest inputs and masses, not the evolving
 // positions. Prepare once; preserve all 24 coupled constraint iterations.
 PDBendData bendData[shaftNodeCount];
 for(int i=1;i<shaftNodeCount-1;i++){float t=float(i-1)/(shaftNodeCount-2);bendData[i]=PDPrepareBend(i,bendCompliance*(.02f+.98f*t*t)*RapheTubeBendMultiplier(float(i)/(shaftNodeCount-1)),dt);}
 pdContactCount=0;
 for(int iteration=0;iteration<24;iteration++){
  pdVelocityPass=iteration==23;
  for(int i=0;i<shaftNodeCount-1;i++)PDDistance(i,i+1,segment,.0000001f,lengthLambda[i],dt);
  // A rod supported at the pelvis has a reinforced proximal section. Its
  // flexural compliance increases continuously toward the free end.
  for(int i=1;i<shaftNodeCount-1;i++)PDBendPrepared(i,bendData[i],bendLambda[i]);
  for(int i=0;i<shaftNodeCount;i++)shaftNodes[i]=pdPosition[i];PDSync();
  for(int s=0;s<2;s++){PDSuspensionShear(s,shearLambda[s],dt,suspensionData[s]);PDSuspension(s,tetherLambda[s],stopLambda[s],dt,suspensionData[s]);}
  for(int s=0;s<2;s++){
   for(int j=0;j<2;j++)PDBodyCapsule(thigh[s][j],s,pdThigh[j*2],pdThigh[j*2+1],pdOldThigh[j*2],pdOldThigh[j*2+1],7.2f,dt);
   PDBodyCapsule(pelvis[s],s,{3.f,0.f,70.f},{5.4f,0.f,86.f},{3.f,0.f,70.f},{5.4f,0.f,86.f},6.4f,dt);
   for(int j=1;j<shaftNodeCount-2;j++)PDRodBody(rodContact[s][j],s,j,dt);
  }
  for(int i=2;i<shaftNodeCount;i++)for(int j=0;j<2;j++){
   float t;V3 q=PDClosest(pdPosition[i],pdThigh[j*2],pdThigh[j*2+1],t),n=Unit(pdPosition[i]-q),old=pdOldThigh[j*2]+(pdOldThigh[j*2+1]-pdOldThigh[j*2])*t;
   float gap=Length(pdPosition[i]-q)-7.2f-logicalShaftBodyRadius*.85f;
   PDContact(rodThigh[i][j],i,{},-1,{},q,old,n,gap,.000002f,dt);
  }
  PDPair(pair,dt);
  PDKeepPouchVentral(ventralCorrection);
 }
 pdVelocityPass=false;
 // Exclude underside recovery from reconstructed velocity. A lobe that was
 // already above the shaft must recover without gaining a launch impulse.
 for(int s=0;s<2;s++)pdOldPosition[pdBody0+s].z+=ventralCorrection[s];
 // One velocity update from the accepted positions and corrected history.
 for(int i=0;i<pdCount;i++)pdVelocity[i]=(pdPosition[i]-pdOldPosition[i])/dt;
 for(int s=0;s<2;s++){
  V3 spin{};for(int j=0;j<3;j++)spin=spin+Cross(pdOldBasis[s][j],cpBasis[s][j]);cpOmega[s]=spin*(.5f/dt);
 }
 PDSolveContactVelocities(dt);
 if(ventralCorrection[0]<0.f||ventralCorrection[1]<0.f){
  float guideVelocity=(PDVentralHeight(pdPosition)-PDVentralHeight(pdOldPosition))/dt;
  for(int s=0;s<2;s++)if(ventralCorrection[s]<0.f)pdVelocity[pdBody0+s].z=min(pdVelocity[pdBody0+s].z,guideVelocity);
 }
 for(int i=0;i<shaftNodeCount;i++){shaftNodes[i]=pdPosition[i];shaftPrevious[i]=pdPosition[i]-pdVelocity[i]*dt;}
 PDSync();
 for(int s=0;s<2;s++){
  pdPreviousAnchor[s]=BallAnchor(s);pdPreviousMaterial[s]=PDMaterialTarget(s);ballPrevious[s]=ballNodes[s]-pdVelocity[pdBody0+s]*dt;V3 oldNeck=neckNodes[s],oldNut=nutNodes[s];
  neckNodes[s]=pdPreviousAnchor[s]+(ballNodes[s]-pdPreviousAnchor[s])*.43f;nutNodes[s]=pdPosition[pdBody0+s];neckPrevious[s]=oldNeck;nutPrevious[s]=oldNut;
  pdMaxTetherRatio=max(pdMaxTetherRatio,Length(ballNodes[s]-pdPreviousAnchor[s])/suspensionData[s].hard);
  // Force-derived bounded yielding, shared by collision and skin transforms.
  pdPairForce=pair.normal/(dt*dt);
  float load=0.f;int contact=1;for(int j=1;j<shaftNodeCount-2;j++)if(rodContact[s][j].normal>load){load=rodContact[s][j].normal;contact=j;}
  float t;V3 q=PDClosest(pdPosition[pdBody0+s],shaftNodes[contact],shaftNodes[contact+1],t);pdPressureDirection[s]=Unit(pdPosition[pdBody0+s]-q);
  pdShaftForce[s]=load/(dt*dt);
 }
 V3 pairNormal=pair.geometryNormal;cpLastGap=Dot(CPCenter(1)+CPSupport(1,pairNormal*-1.f)-CPCenter(0)-CPSupport(0,pairNormal),pairNormal);
 cpMinimumGap=min(cpMinimumGap,cpLastGap);pdMinimumGap=min(pdMinimumGap,cpLastGap);
}

static void UpdateCompliantDynamics(float dt,float gait,float side){
 // Menus and device resets can draw the overlay before the character's
 // authored rest frame exists. Start only after ApplyShape supplies it.
 if(!shaftRestFrameReady||!eggRestReady)return;
 if(!constraintSolverReady)InitializeConstraintSolver();
 PDInput target=PDReadInput(gait,side);
 if(!pdInputReady||!pdReady){pdInput=target;pdInputReady=true;memcpy(pdThigh,target.thigh,sizeof(pdThigh));}
 constraintAccumulator+=min(dt,.10f);const float fixed=1.f/240.f;
 int steps=min(24,int((constraintAccumulator+1e-7f)/fixed));
 for(int i=0;i<steps;i++){
  float t=float(i+1)/steps;PDSetInput(pdInput,target,t);
  StepConstraintSolver(fixed,pdInput.gait+(target.gait-pdInput.gait)*t,pdInput.side+(target.side-pdInput.side)*t);
 }
 constraintAccumulator=max(0.f,constraintAccumulator-steps*fixed);
 if(steps)pdInput=target;PDSetInput(target,target,1.f);
}
