// Exact scalar source statements. Test oracle only; not a second runtime.
static float ModeValue(float erect,float semi,float floppy){return shaftMode<=1.f?erect+(semi-erect)*Smoother01(shaftMode):semi+(floppy-semi)*Smoother01(shaftMode-1.f);}
static PDSuspensionData PDPrepareSuspension(int s){
 float rest=max(2.45f,Length(constraintBallRest[s]+CPRestSlack(s)-RestBallAnchor(s)));
 float stiff=max(0.f,min(1.f,physUI[4]/100.f));
 return {rest,rest*1.12f+CPRadii(s).y*.15f,.000053f*expf(-3.f*stiff),
  .000022f*expf((.5f-physUI[4]/100.f)*3.f),.10f+.60f*(1.f-max(0.f,min(1.f,physUI[6]/100.f)))};
}
static std::array<float,22> SourceMaterialProfile(){
float shaftMass=.75f+physValues[1]*.0125f,bodyMass=.75f+physValues[5]*.0125f;
float shaftDrag=.9f+(100.f-physUI[2])*.018f,bodyDrag=1.f+(100.f-physUI[6])*.025f;
float shaftResponse,shaftGravity,bodyResponse,bodyGravity;
for(int i=0;i<2;i++){const int pdBody0=1;float response=i<pdBody0?(.65f+physValues[3]*.009f):(.65f+physValues[7]*.009f),gravity=i<pdBody0?ModeValue(5.f,42.f,110.f):72.f;
if(i==0){shaftResponse=response;shaftGravity=gravity;}else{bodyResponse=response;bodyGravity=gravity;}}
float stiffness=max(0.f,min(1.f,physUI[0]/100.f));float bendCompliance=ModeValue(.00000001f,.00008f,.0015f)*expf((.5f-stiffness)*3.f);
float ratio=.10f+.40f*(1.f-max(0.f,min(1.f,physUI[2]/100.f)));
float torsion=20.f*expf((physUI[4]/100.f-.5f)*2.f);
float mass=(1.f+physValues[1]*.016f)*max(.65f,sqrtf(constraintRestLength/24.f));
float k=ModeValue(38.f,22.f,10.f),damping=2.f*sqrtf(k*mass)*ModeValue(.60f,.72f,.86f);
float droop=ModeValue(.015f,.11f,.28f),drive=ModeValue(7.f,9.f,12.f);
float limit=ModeValue(.42f,.62f,.92f);
auto suspension=PDPrepareSuspension(0);
return {shaftMass,bodyMass,shaftDrag,bodyDrag,shaftResponse,bodyResponse,
shaftGravity,bodyGravity,bendCompliance,ratio,suspension.compliance,
suspension.shearCompliance,suspension.ratio,torsion,mass,k,damping,droop,
drive,limit,suspension.rest,suspension.hard};}
