struct PDBendData {float alpha,gamma,factor;V3 oldValue;};
static void PDDistance(int a,int b,float rest,float compliance,float& lambda,float dt){
 V3 delta=pdPosition[b]-pdPosition[a];float distance=Length(delta);if(distance<1e-7f)return;
 float alpha=compliance/(dt*dt),dl=(-(distance-rest)-alpha*lambda)/(pdInvMass[a]+pdInvMass[b]+alpha);lambda+=dl;
 V3 impulse=delta*(dl/distance);pdPosition[a]=pdPosition[a]-impulse*pdInvMass[a];pdPosition[b]=pdPosition[b]+impulse*pdInvMass[b];
}
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
