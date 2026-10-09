"""Checked default-off angular-root transform of the immutable source closure.

Every replacement is guarded against source drift. Engine calibration and activation
remain adapter responsibilities; this provisional module has no native acceptance.
"""

import re

def transform(text, process_isolated):
 original=text
 variant='angular-modal-consistent'
 def sub(old,new,count=1):
  nonlocal text
  if text.count(old)==count:
   text=text.replace(old,new)
  else:
   pattern=r"\n[ \t]*".join(re.escape(line.strip()) for line in old.split("\n"))
   matches=list(re.finditer(pattern,text));assert len(matches)==count,(old,len(matches))
   text=re.sub(pattern,lambda _:new,text)
 sub('surfaceCollisionEnabled=false,surfaceRootContactEnabled=false;','surfaceCollisionEnabled=false;')
 storage='static ' if process_isolated else 'static thread_local '
 sub(storage+'float surfaceClinicalLateralYaw=0;',storage+'float surfaceClinicalLateralYaw=0;\n'+storage+'bool surfaceRootContactEnabled=false;')
 helpers=r'''
 namespace rootjoint=::malemod::surface::root_contact;
namespace tangent=::malemod::surface::tangent_root;
 static rootjoint::Point RootPoint(V3 p){return {p.x,p.y,p.z};}
 static V3 RootVector(rootjoint::Point p){return {float(p[0]),float(p[1]),float(p[2])};}
 static double RootMass(){return (1.f+physValues[1]*.016f)*max(.65f,sqrtf(constraintRestLength/24.f));}
 static rootjoint::State RootState(){return rootjoint::FromJoint(RootPoint(ShaftRoot()),RootPoint(pdPosition[1]),{},rootDriveAngle*3.1415926535/180.+shaftSpring.pitch);}
 static void RootJacobians(V3& p,V3& y){auto s=RootState();double l=constraintRestLength/(shaftNodeCount-1),sp=sin(s.pitch),cp=cos(s.pitch),sy=sin(s.yaw),cy=cos(s.yaw);p={float(-l*sp*cy),0,float(-l*cp*cy)};y={float(-l*cp*sy),float(l*cy),float(l*sp*sy)};}
 static V3 RootMobility(V3 impulse){V3 p,y;RootJacobians(p,y);return (p*Dot(p,impulse)+y*Dot(y,impulse))/float(RootMass());}
 static void RootSet(rootjoint::State s){const double drive=rootDriveAngle*3.1415926535/180.,imposed=teachingTimeline.active?teachingFluid.MainLateralYaw(teachingTimeline.time):0.;shaftSpring.pitch=float(s.pitch-drive);shaftSpring.yaw=float(s.yaw-imposed);pdPosition[1]=ShaftRoot()+RootVector(rootjoint::Direction(s))*float(constraintRestLength/(shaftNodeCount-1));}
 static void RootApply(V3 impulse){auto s=RootState();V3 p,y;RootJacobians(p,y);s.pitch+=Dot(p,impulse)/RootMass();s.yaw+=Dot(y,impulse)/RootMass();RootSet(s);}
 static void RootStops(float dt,float lambda[2]){
  if(!surfaceRootContactEnabled)return;
  auto s=RootState();const double drive=rootDriveAngle*3.1415926535/180.,imposed=teachingTimeline.active?teachingFluid.MainLateralYaw(teachingTimeline.time):0.;
  double values[2]={s.pitch-drive,s.yaw-imposed},limits[2]={ModeValue(.42f,.62f,.92f),.65f};
  // The exact existing soft-stop acceleration 90*excess corresponds to k=90*I.
  const double inertia=RootMass(),alpha=1./(90.*inertia*dt*dt);
  for(unsigned a=0;a<2;a++){const double sign=values[a]<0?-1:1,c=std::abs(values[a])-limits[a];double next=std::min(0.,double(lambda[a])-(c+alpha*lambda[a])/(1./inertia+alpha));double correction=sign*(next-lambda[a])/inertia;lambda[a]=float(next);if(a==0)s.pitch+=correction;else s.yaw+=correction;}
  RootSet(s);
 }
 '''
 sub('static float PDEffectiveMass(int id,V3 arm,V3 direction){',helpers+'\nstatic float PDEffectiveMass(int id,V3 arm,V3 direction){')
 sub('if(id<0)return 0.f;float w=pdInvMass[id];','if(id<0)return 0.f;if(id==1&&surfaceRootContactEnabled)return Dot(direction,RootMobility(direction));float w=pdInvMass[id];')
 sub('if(id<0)return;pdPosition[id]=pdPosition[id]+impulse*pdInvMass[id];','if(id<0)return;if(id==1&&surfaceRootContactEnabled){RootApply(impulse);return;}pdPosition[id]=pdPosition[id]+impulse*pdInvMass[id];')
 sub('if(id<0)return;pdVelocity[id]=pdVelocity[id]+impulse*pdInvMass[id];','if(id<0)return;if(id==1&&surfaceRootContactEnabled){pdVelocity[id]=pdVelocity[id]+RootMobility(impulse);return;}pdVelocity[id]=pdVelocity[id]+impulse*pdInvMass[id];')
 # All scalar rod contact masses and impulses must use the same angular Jacobian.
 sub('pdInvMass[c.rod]*(1-c.t)*(1-c.t)+pdInvMass[c.rod+1]*c.t*c.t','PDEffectiveMass(c.rod,{},n)*(1-c.t)*(1-c.t)+PDEffectiveMass(c.rod+1,{},n)*c.t*c.t')
 sub('pdInvMass[rod]*(1-t)*(1-t)+pdInvMass[rod+1]*t*t','PDEffectiveMass(rod,{},tangent)*(1-t)*(1-t)+PDEffectiveMass(rod+1,{},tangent)*t*t')
 sub('pdPosition[rod]=pdPosition[rod]-impulse*(pdInvMass[rod]*(1-t));pdPosition[rod+1]=pdPosition[rod+1]-impulse*(pdInvMass[rod+1]*t);','if(surfaceRootContactEnabled){PDApply(rod,{},impulse*(-(1-t)));PDApply(rod+1,{},impulse*(-t));}else{pdPosition[rod]=pdPosition[rod]-impulse*(pdInvMass[rod]*(1-t));pdPosition[rod+1]=pdPosition[rod+1]-impulse*(pdInvMass[rod+1]*t);}',2)
 sub('float alpha=compliance/(dt*dt),dl=(-(distance-rest)-alpha*lambda)/(pdInvMass[a]+pdInvMass[b]+alpha);','V3 n=delta/distance;float alpha=compliance/(dt*dt),dl=(-(distance-rest)-alpha*lambda)/((surfaceRootContactEnabled?PDEffectiveMass(a,{},n)+PDEffectiveMass(b,{},n):pdInvMass[a]+pdInvMass[b])+alpha);')
 sub('V3 impulse=delta*(dl/distance);pdPosition[a]=pdPosition[a]-impulse*pdInvMass[a];pdPosition[b]=pdPosition[b]+impulse*pdInvMass[b];','V3 impulse=delta*(dl/distance);if(surfaceRootContactEnabled){PDApply(a,{},impulse*-1.f);PDApply(b,{},impulse);}else{pdPosition[a]=pdPosition[a]-impulse*pdInvMass[a];pdPosition[b]=pdPosition[b]+impulse*pdInvMass[b];}')
 sub('static void PDBendPrepared(int i,const PDBendData& data,V3& lambda){',r'''static void PDBendPrepared(int i,const PDBendData& data,V3& lambda){
  if(surfaceRootContactEnabled&&i<=2){
   V3 value=pdPosition[i-1]-pdPosition[i]*2.f+pdPosition[i+1],rhs=value+lambda*data.alpha+(value-data.oldValue)*data.gamma;
   Eigen::Matrix3d mobility=Eigen::Matrix3d::Zero();const int ids[3]={i-1,i,i+1};const double coeff[3]={1,-2,1};
   for(unsigned a=0;a<3;a++){
    if(ids[a]==1){V3 p,y;RootJacobians(p,y);Eigen::Vector3d jp(p.x,p.y,p.z),jy(y.x,y.y,y.z);mobility+=(jp*jp.transpose()+jy*jy.transpose())*(coeff[a]*coeff[a]/RootMass());}
    else mobility+=Eigen::Matrix3d::Identity()*(coeff[a]*coeff[a]*pdInvMass[ids[a]]);
   }
   Eigen::Vector3d r(rhs.x,rhs.y,rhs.z);Eigen::Vector3d correction=((1.+data.gamma)*mobility+Eigen::Matrix3d::Identity()*data.alpha).ldlt().solve(-r);
   V3 dl{float(correction.x()),float(correction.y()),float(correction.z())};lambda=lambda+dl;
   for(unsigned a=0;a<3;a++)PDApply(ids[a],{},dl*float(coeff[a]));return;
  }
 ''')
 # Make the same proximal Hermite material reaction available to the angular DOF.
 sub('for(int i=max(2,span-1);i<=min(shaftNodeCount-1,span+2);i++)','for(int i=max(surfaceRootContactEnabled?1:2,span-1);i<=min(shaftNodeCount-1,span+2);i++)')
 sub('for(int i=2;i<shaftNodeCount;i++)w+=pdInvMass[i]*Dot(gradient[i],gradient[i]);','for(int i=surfaceRootContactEnabled?1:2;i<shaftNodeCount;i++)w+=surfaceRootContactEnabled&&i==1?PDEffectiveMass(i,{},gradient[i]):pdInvMass[i]*Dot(gradient[i],gradient[i]);',2)
 sub('for(int i=2;i<shaftNodeCount;i++){pdPosition[i]=pdPosition[i]+gradient[i]*(pdInvMass[i]*dl);shaftNodes[i]=pdPosition[i];}','for(int i=surfaceRootContactEnabled?1:2;i<shaftNodeCount;i++){if(surfaceRootContactEnabled)PDApply(i,{},gradient[i]*dl);else pdPosition[i]=pdPosition[i]+gradient[i]*(pdInvMass[i]*dl);shaftNodes[i]=pdPosition[i];}')
 sub('float w0=pdInvMass[j]*(1-t)*(1-t),w1=pdInvMass[j+1]*t*t,','float w0=(surfaceRootContactEnabled?PDEffectiveMass(j,{},n):pdInvMass[j])*(1-t)*(1-t),w1=(surfaceRootContactEnabled?PDEffectiveMass(j+1,{},n):pdInvMass[j+1])*t*t,')
 sub('PDApply(id,arm,n*dl);pdPosition[j]=pdPosition[j]-n*(dl*pdInvMass[j]*(1-t));pdPosition[j+1]=pdPosition[j+1]-n*(dl*pdInvMass[j+1]*t);','PDApply(id,arm,n*dl);if(surfaceRootContactEnabled){PDApply(j,{},n*(-dl*(1-t)));PDApply(j+1,{},n*(-dl*t));}else{pdPosition[j]=pdPosition[j]-n*(dl*pdInvMass[j]*(1-t));pdPosition[j+1]=pdPosition[j+1]-n*(dl*pdInvMass[j+1]*t);}')
 # The root point no longer has isotropic translational inverse mass.
 sub('if(i==1&&surfaceRootContactEnabled){float segment=constraintRestLength/(shaftNodeCount-1);pdInvMass[i]=float(::malemod::surface::root_contact::PointInverseMass(segment,(1.f+physValues[1]*.016f)*max(.65f,sqrtf(constraintRestLength/24.f))));}','')
 sub('float lengthLambda[shaftNodeCount]{},tetherLambda[2]','float rootStopLambda[2]{};float lengthLambda[shaftNodeCount]{},tetherLambda[2]')
 sub('  PDKeepPouchVentral(ventralCorrection);','  PDKeepPouchVentral(ventralCorrection);RootStops(dt,rootStopLambda);')
 # Leave the existing soft-stop torque exact in the disabled reference branch.
 sub('if(fabsf(shaftSpring.pitch)>limit)','if(!surfaceRootContactEnabled&&fabsf(shaftSpring.pitch)>limit)')
 sub('if(fabsf(shaftSpring.yaw)>.65f)','if(!surfaceRootContactEnabled&&fabsf(shaftSpring.yaw)>.65f)')
 if variant.startswith('angular-modal-'):
  sub('static void RootStops(float dt,float lambda[2]){',r'''ROOT_STORAGESpring2 rootOldSpring{};
 static double RootModalInertia(){double inertia=0,mass=.75f+physValues[1]*.0125f,segment=constraintRestLength/(shaftNodeCount-1);for(int i=2;i<shaftNodeCount;i++)inertia+=mass*(i*segment)*(i*segment);return inertia;}
 static void RootRegular(float dt,float lambda[2]){
  if(!surfaceRootContactEnabled)return;
  auto s=RootState();const double drive=rootDriveAngle*3.1415926535/180.,imposed=teachingTimeline.active?teachingFluid.MainLateralYaw(teachingTimeline.time):0.;
  const double mass=RootMass(),scale=RootModalInertia()/mass,k=ModeValue(38.f,22.f,10.f),damp=2.*sqrt(k*mass)*ModeValue(.60f,.72f,.86f);
  const double target[2]={ModeValue(.015f,.11f,.28f),0},weight[2]={1,.8},old[2]={rootOldSpring.pitch,rootOldSpring.yaw},values[2]={s.pitch-drive,s.yaw-imposed};
  for(unsigned a=0;a<2;a++){
   const double physicalK=k*weight[a]*scale,alpha=1./(physicalK*dt*dt),gamma=(damp*scale)/(physicalK*dt),c=values[a]-target[a],oldC=old[a]-target[a];
   const double dl=(-c-alpha*lambda[a]-gamma*(c-oldC))/((1.+gamma)/mass+alpha);lambda[a]+=float(dl);if(a==0)s.pitch+=dl/mass;else s.yaw+=dl/mass;
  }
  RootSet(s);
 }
 static void RootStops(float dt,float lambda[2]){''')
  sub('StepRootSuspension(dt,gait,side);','rootOldSpring=shaftSpring;StepRootSuspension(dt,gait,side);')
  sub('float rootStopLambda[2]{};','float rootRegularLambda[2]{},rootStopLambda[2]{};')
  sub('RootStops(dt,rootStopLambda);','RootRegular(dt,rootRegularLambda);RootStops(dt,rootStopLambda);')
  # Root angular restoring/damping now enters the coupled solve implicitly.
  sub('shaftSpring.pitchVelocity+=(k*(droop-shaftSpring.pitch)-damping*shaftSpring.pitchVelocity-gait*drive)*dt/mass;','if(surfaceRootContactEnabled)shaftSpring.pitchVelocity+=(-gait*drive)*dt/mass;else shaftSpring.pitchVelocity+=(k*(droop-shaftSpring.pitch)-damping*shaftSpring.pitchVelocity-gait*drive)*dt/mass;')
  sub('shaftSpring.yawVelocity+=(-k*.8f*shaftSpring.yaw-damping*shaftSpring.yawVelocity+side*drive)*dt/mass;','if(surfaceRootContactEnabled)shaftSpring.yawVelocity+=(side*drive)*dt/mass;else shaftSpring.yawVelocity+=(-k*.8f*shaftSpring.yaw-damping*shaftSpring.yawVelocity+side*drive)*dt/mass;')
 if variant=='angular-modal-consistent':
  sub('double inertia=0,mass=.75f+physValues[1]*.0125f','double inertia=RootMass(),mass=.75f+physValues[1]*.0125f')
  sub('const double inertia=RootMass(),alpha=1./(90.*inertia*dt*dt);','const double inertia=RootMass(),alpha=1./(90.*RootModalInertia()*dt*dt);')
  sub('static rootjoint::State RootState(){return rootjoint::FromJoint(RootPoint(ShaftRoot()),RootPoint(pdPosition[1]),{},rootDriveAngle*3.1415926535/180.+shaftSpring.pitch);}','ROOT_STORAGErootjoint::State pdRootJointState{};ROOT_STORAGEdouble rootOldDriveRadians=0;\nstatic rootjoint::State RootState(){return pdRootJointState;}')
  sub('static void RootSet(rootjoint::State s){const double drive=','static void RootSet(rootjoint::State s){pdRootJointState=s;const double drive=')
  sub('rootOldSpring=shaftSpring;StepRootSuspension(dt,gait,side);','rootOldDriveRadians=(rootDriveReady?rootDriveAngle:sliderValues[4])*3.1415926535/180.;rootOldSpring=shaftSpring;StepRootSuspension(dt,gait,side);')
  sub('pdPosition[0]=root;pdPosition[1]=root+direction*segment;','pdPosition[0]=root;pdPosition[1]=root+direction*segment;if(surfaceRootContactEnabled)pdRootJointState={rootDriveAngle*3.1415926535/180.+shaftSpring.pitch,shaftSpring.yaw+(teachingTimeline.active?teachingFluid.MainLateralYaw(teachingTimeline.time):0.f),0,0};')
  sub(' PDSolveContactVelocities(dt);',''' if(surfaceRootContactEnabled){auto accepted=RootState();V3 p,y;RootJacobians(p,y);const double oldPitch=rootOldDriveRadians+rootOldSpring.pitch,oldYaw=rootOldSpring.yaw+surfaceClinicalLateralYaw;pdVelocity[1]=p*float((accepted.pitch-oldPitch)/dt)+y*float((accepted.yaw-oldYaw)/dt);}
  PDSolveContactVelocities(dt);''')
  sub('V3 first=Unit(pdPosition[1]-root)*segment;pdPosition[1]=root+first;','')
  sub('const auto state=contact::FromJoint(cp(root),cp(pdPosition[1]),cp(pdVelocity[1]),drive+shaftSpring.pitch);','auto state=RootState();V3 jp,jy;RootJacobians(jp,jy);state.pitchVelocity=Dot(pdVelocity[1],jp)/max(1e-10f,Dot(jp,jp));state.yawVelocity=Dot(pdVelocity[1],jy)/max(1e-10f,Dot(jy,jy));')
  sub('drive,imposedYaw,rootDriveVelocity*3.1415926535/180.,(imposedYaw-surfaceClinicalLateralYaw)/dt','drive,imposedYaw,(drive-rootOldDriveRadians)/dt,(imposedYaw-surfaceClinicalLateralYaw)/dt')
 
 text=text.replace('ROOT_STORAGE',storage)
 

 sub('static void RootJacobians(V3& p,V3& y){auto s=RootState();double l=constraintRestLength/(shaftNodeCount-1),sp=sin(s.pitch),cp=cos(s.pitch),sy=sin(s.yaw),cy=cos(s.yaw);p={float(-l*sp*cy),0,float(-l*cp*cy)};y={float(-l*cp*sy),float(l*cy),float(l*sp*sy)};}',
     'static void RootJacobians(V3& p,V3& y){const auto basis=tangent::Jacobian(RootState(),constraintRestLength/(shaftNodeCount-1));p=RootVector(basis.pitch);y=RootVector(basis.yaw);}')
 sub('V3 p,y;RootJacobians(p,y);Eigen::Vector3d jp(p.x,p.y,p.z),jy(y.x,y.y,y.z);mobility+=(jp*jp.transpose()+jy*jy.transpose())*(coeff[a]*coeff[a]/RootMass());',
     'const auto tensor=tangent::Mobility(RootBasis(),RootMass());for(unsigned row=0;row<3;row++)for(unsigned column=0;column<3;column++)mobility(row,column)+=tensor[row][column]*coeff[a]*coeff[a];')
 sub('static V3 RootMobility(V3 impulse){V3 p,y;RootJacobians(p,y);return (p*Dot(p,impulse)+y*Dot(y,impulse))/float(RootMass());}',
     'static tangent::Basis RootBasis(){V3 p,y;RootJacobians(p,y);return {RootPoint(p),RootPoint(y)};}\nstatic V3 RootMobility(V3 impulse){return RootVector(tangent::Response(RootBasis(),RootMass(),RootPoint(impulse)));}')
 sub('auto s=RootState();V3 p,y;RootJacobians(p,y);s.pitch+=Dot(p,impulse)/RootMass();s.yaw+=Dot(y,impulse)/RootMass();RootSet(s);',
     'RootSet(tangent::PositionImpulse(RootState(),RootBasis(),RootMass(),RootPoint(impulse)));')
 sub('double inertia=RootMass(),mass=.75f+physValues[1]*.0125f,segment=constraintRestLength/(shaftNodeCount-1);for(int i=2;i<shaftNodeCount;i++)inertia+=mass*(i*segment)*(i*segment);return inertia;',
     'const double mass=.75f+physValues[1]*.0125f,segment=constraintRestLength/(shaftNodeCount-1);std::vector<tangent::Station> stations;for(int i=2;i<shaftNodeCount;i++)stations.push_back({mass,i*segment});return tangent::ModalInertia(RootMass(),stations);')
 sub('const double mass=RootMass(),scale=RootModalInertia()/mass,k=ModeValue(38.f,22.f,10.f),damp=2.*sqrt(k*mass)*ModeValue(.60f,.72f,.86f);',
     'const double mass=RootMass(),modal=RootModalInertia(),k=ModeValue(38.f,22.f,10.f),damp=2.*sqrt(k*mass)*ModeValue(.60f,.72f,.86f);')
 sub('const double physicalK=k*weight[a]*scale,alpha=1./(physicalK*dt*dt),gamma=(damp*scale)/(physicalK*dt),c=values[a]-target[a],oldC=old[a]-target[a];\n  const double dl=(-c-alpha*lambda[a]-gamma*(c-oldC))/((1.+gamma)/mass+alpha);lambda[a]+=float(dl);if(a==0)s.pitch+=dl/mass;else s.yaw+=dl/mass;',
     'const auto law=tangent::EquivalentLaw(mass,modal,{k*weight[a],damp});const auto correction=tangent::Spring(values[a],old[a],target[a],lambda[a],1./mass,law,dt);lambda[a]=float(correction.multiplier);if(a==0)s.pitch+=correction.angle;else s.yaw+=correction.angle;')
 sub('const double inertia=RootMass(),alpha=1./(90.*RootModalInertia()*dt*dt);',
     'const double inertia=RootMass(),stiffness=90.*RootModalInertia();')
 sub('for(unsigned a=0;a<2;a++){const double sign=values[a]<0?-1:1,c=std::abs(values[a])-limits[a];double next=std::min(0.,double(lambda[a])-(c+alpha*lambda[a])/(1./inertia+alpha));double correction=sign*(next-lambda[a])/inertia;lambda[a]=float(next);if(a==0)s.pitch+=correction;else s.yaw+=correction;}',
     'for(unsigned a=0;a<2;a++){auto correction=tangent::SoftStop(values[a],limits[a],lambda[a],1./inertia,stiffness,dt);lambda[a]=float(correction.multiplier);if(a==0)s.pitch+=correction.angle;else s.yaw+=correction.angle;}')
 sub('Eigen::Vector3d r(rhs.x,rhs.y,rhs.z);Eigen::Vector3d correction=((1.+data.gamma)*mobility+Eigen::Matrix3d::Identity()*data.alpha).ldlt().solve(-r);\n  V3 dl{float(correction.x()),float(correction.y()),float(correction.z())};lambda=lambda+dl;',
     'tangent::Matrix matrix{};for(unsigned a=0;a<3;a++)for(unsigned b=0;b<3;b++)matrix[a][b]=(1.+data.gamma)*mobility(a,b)+(a==b?data.alpha:0.);const auto correction=tangent::Solve(matrix,RootPoint(rhs*-1.f));\n  V3 dl=RootVector(correction);lambda=lambda+dl;')
 sub('auto state=RootState();V3 jp,jy;RootJacobians(jp,jy);state.pitchVelocity=Dot(pdVelocity[1],jp)/max(1e-10f,Dot(jp,jp));state.yawVelocity=Dot(pdVelocity[1],jy)/max(1e-10f,Dot(jy,jy));',
     'const auto state=tangent::Rates(RootState(),RootBasis(),RootPoint(pdVelocity[1]));')
 sub('#include <malemod/surface/root_contact.hpp>','#include <malemod/surface/root_contact.hpp>\n#include <malemod/surface/tangent_root.hpp>')
 return isolate_disabled(original,text)



def isolate_disabled(original, transformed):
 """Keep original solver functions whole, avoiding reassociation/CSE across
 active-root branches (notably delta*(dl/distance) in source PDDistance).
 Only enabled callers use the transformed function closure.
 """
 def functions(text):
  result={}
  pattern=r'(?m)^\s*static\s+(?:__forceinline\s+)?(?:[\w:]+(?:\s*[*&])?\s+)+(\w+)\([^;{}]*\)\s*\{'
  for match in re.finditer(pattern,text):
   brace=text.index('{',match.start());end=brace+1;depth=1
   while depth:
    depth+=(text[end]=='{')-(text[end]=='}');end+=1
   name=match.group(1)
   if name in result:continue
   result[name]=(match.start(),end,text[match.start():end])
  return result
 before=functions(original);after=functions(transformed)
 changed={name for name in before if name in after and before[name][2].strip()!=after[name][2].strip()}
 while True:
  callers={name for name in before if name not in changed and any(re.search(r'\b'+re.escape(c)+r'\s*\(',before[name][2]) for c in changed)}
  if not callers:break
  changed|=callers
 expected={'PDApply','PDApplySuspension','PDBendPrepared','PDBodyCapsule','PDContact','PDDistance','PDEffectiveMass','PDPair','PDRodBody','PDSolveContactVelocities','PDStaticFriction','PDSuspension','PDSuspensionMass','PDSuspensionMassFiniteDifference','PDSuspensionShear','PDVelocityImpulse','StepConstraintSolver','StepRootSuspension','UpdateCompliantDynamics','UpdateConstraintSolver'}
 if changed!=expected:raise ValueError('Angular source function closure changed: '+repr(changed^expected))
 # Helpers only refer to unchanged state/type declarations, already defined by
 # the original kernel. The active function closure preserves its source order.
 helper_start=transformed.index('namespace rootjoint=')
 helper_end=transformed.index('static float PDEffectiveMass(',helper_start)
 helper=transformed[helper_start:helper_end]
 blocks=[after[name][2] for name in sorted(changed,key=lambda n:after[n][0])]
 angular=helper+'\n'+'\n'.join(blocks)
 for name in sorted(changed,key=len,reverse=True):angular=re.sub(r'\b'+re.escape(name)+r'\b',name+'Angular',angular)
 entries=('StepConstraintSolver','UpdateConstraintSolver')
 reference=original
 for name in entries:reference=re.sub(r'\b'+name+r'\b',name+'Disabled',reference)
 wrapper='\n'.join('static void '+name+'(float dt,float gait,float side){if(surfaceRootContactEnabled)'+name+'Angular(dt,gait,side);else '+name+'Disabled(dt,gait,side);}' for name in entries)
 marker='} // namespace malemod::surface::source\n'
 if reference.count(marker)!=1:raise ValueError('Kernel namespace boundary changed')
 reference=reference.replace('#include <malemod/surface/root_contact.hpp>','#include <malemod/surface/root_contact.hpp>\n#include <malemod/surface/tangent_root.hpp>')
 return reference.replace(marker,angular+'\n'+wrapper+'\n'+marker)
