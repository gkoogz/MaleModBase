#pragma once
// Slender viscoelastic approximation for the small, adhesive preliminary flow.
// Main emissions use a separate reduced-dimensional viscous stream solver.
namespace teaching {
struct ClearStrand {
 static const int count=25;
 V3 p[count]{},velocity[count]{};bool live=false,attached=false,started=false;
 float volume=0,length=0;
 float reportedContact[count]{};
 volumeFluid::CollisionPath paths[count];
 bool worldDeposited[count]{};
 void Reset(){*this=ClearStrand{};}
 void Step(float dt,float age,V3 tip,V3 direction,const volumeFluid::Settings& cfg,
           const volumeFluid::F4* collision,float fraction,float amount,V3 origin,std::vector<volumeFluid::FluidImpact>* impacts=nullptr){
  if(age<=0||age>cfg.dropDuration+cfg.dropHold+cfg.lifetime){live=false;return;}
  if(!started){started=live=true;for(int i=0;i<count;i++)p[i]=tip+direction*(.02f+.03f*i/(count-1));}
  volume=amount*fraction;attached=age<cfg.dropDuration+cfg.dropHold;
  length=max(.04f,cfg.dropLength*fraction);float rest=length/(count-1);
  V3 anchor=tip+direction*.02f,old[count];memcpy(old,p,sizeof(p));
  V3 bulk{};if(!attached){for(int i=0;i<count;i++)bulk=bulk+velocity[i];bulk=bulk/float(count);}
  for(int i=0;i<count;i++){
   // Internal viscosity damps deformation, not the falling centre of mass.
   velocity[i]=attached?(velocity[i]+V3{0,0,-cfg.gravity}*dt)*expf(-dt*4.f)
                       :bulk+V3{0,0,-cfg.gravity}*dt+(velocity[i]-bulk)*expf(-dt*4.f);
   p[i]=p[i]+velocity[i]*dt;
  }
  float lambda[count-1]{};float alpha=.000008f/(dt*dt);
  for(int iteration=0;iteration<40;iteration++){
   if(attached){p[0]=anchor;p[1]=anchor+direction*rest;}
   // Alternate ordering so growth does not accumulate at one end.
   for(int k=0;k<count-1;k++){
    int i=iteration%2?count-2-k:k;float wa=attached&&i<2?0.f:1.f,wb=attached&&i+1<2?0.f:1.f;
    V3 delta=p[i+1]-p[i];float distance=Length(delta);if(distance<1e-8f||wa+wb==0)continue;
    float dl=(-(distance-rest)-alpha*lambda[i])/(wa+wb+alpha);lambda[i]+=dl;
    V3 impulse=delta*(dl/distance);p[i]=p[i]-impulse*wa;p[i+1]=p[i+1]+impulse*wb;
   }
   for(int i=attached?2:0;i<count;i++){
    if(cfg.catchPlane)p[i].z=max(p[i].z,origin.z-cfg.catchDepth+.08f);
    for(int j=0;j<8;j++)if(collision[j].w>0){V3 center{collision[j].x,collision[j].y,collision[j].z},delta=p[i]-center;float d=Length(delta),r=collision[j].w+.08f;if(d>1e-6f&&d<r)p[i]=center+delta*(r/d);}
   }
  }
  if(volumeFluid::collisionSweep)for(int i=attached?2:0;i<count;i++){
   if(impacts&&worldDeposited[i]&&reportedContact[i]>0&&volume/count<=reportedContact[i]+1e-6f){p[i]=old[i];continue;}
   volumeFluid::FluidImpact hit{};if(paths[i].Sweep(old[i],p[i],.08f,hit)){p[i]=hit.p+hit.n*.105f;float fresh=max(0.f,volume/count-reportedContact[i]);if(impacts&&fresh>1e-6f){hit.velocity=velocity[i];hit.volume=fresh;impacts->push_back(hit);reportedContact[i]+=fresh;worldDeposited[i]=hit.kind==volumeFluid::FLUID_IMPACT_WORLD;}}
  }
  if(attached){p[0]=anchor;p[1]=anchor+direction*rest;}
  for(int i=0;i<count;i++){velocity[i]=(p[i]-old[i])/dt;float speed=Length(velocity[i]);if(speed>cfg.speedLimit)velocity[i]=velocity[i]*(cfg.speedLimit/speed);}
  if(!attached&&p[count/2].z<origin.z-180)live=false;
 }
 float ArcLength() const {float result=0;for(int i=1;i<count;i++)result+=Length(p[i]-p[i-1]);return result;}
 void Append(std::vector<volumeFluid::Particle>& out) const {
  if(!live||volume<1e-7f)return;
  float shape[count],integral=0;
  for(int i=0;i<count;i++){float t=float(i)/(count-1);shape[i]=.7f+.9f*powf(t,6);}
  for(int i=1;i<count;i++)integral+=Length(p[i]-p[i-1])*(shape[i]*shape[i]+shape[i-1]*shape[i-1])*.5f;
  float scale=sqrtf(volume/(3.14159265f*max(.0001f,integral)));
  for(int i=0;i<count-1;i++){
   float distance=Length(p[i+1]-p[i]),r=min(shape[i],shape[i+1])*scale;
   int samples=max(1,min(128,(int)ceilf(distance/max(.008f,r*.45f))));
   for(int k=0;k<samples;k++){float t=float(k)/samples;V3 pos=p[i]+(p[i+1]-p[i])*t;float radius=(shape[i]+(shape[i+1]-shape[i])*t)*scale;
    // live=3 identifies a CPU surface sample; age carries its radius only here.
    out.push_back({pos,3,{},radius});}
  }
  out.push_back({p[count-1],3,{},shape[count-1]*scale});
 }
};
}
