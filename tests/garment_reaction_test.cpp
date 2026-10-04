#include <malemod/garments/reaction_support.hpp>
#include <malemod/surface/garment_impulse.hpp>
#include <malemod/surface/wire.hpp>
#include <cstdio>
using namespace malemod;
using namespace garments;
void Check(bool pass,const char* why){if(!pass)throw std::runtime_error(why);}
Point Vector(surface::ImpulsePoint p){return {p.x,p.y,p.z};}
bool Near(Point a,Point b,double e=1e-10){return Length(Sub(a,b))<=e;}
int main(){try{
 std::vector<Sample> tissue(3);for(unsigned i=0;i<3;i++)tissue[i].lineage.donors[0]={Surface::Anatomy,i,1};
 tissue[0].position={0,0,0};tissue[1].position={2,0,0};tissue[2].position={0,2,0};
 const std::vector<std::array<std::uint32_t,3>> faces{{0,1,2}};
 ReactionCollector collector(tissue);Point bary{.25,.25,.5},clothImpulse{0,-2,3},contact{.5,1,0};collector.Add(0,bary,clothImpulse,contact,.002,faces);
 Output cloth;cloth.style=Style::WhiteJockstrap;collector.Publish(cloth,1./120,1.8);
 Check(cloth.reactions.size()==3&&cloth.reaction.contacts==1,"Actual contact records absent");
 Check(Near(Add(cloth.reaction.clothImpulse,cloth.reaction.anatomyImpulse),{}),"Linear reaction not equal and opposite");
 Check(Near(Add(cloth.reaction.clothMoment,cloth.reaction.anatomyMoment),{}),"Angular reaction not equal and opposite");
 Point impulse{},moment{};for(const auto& r:cloth.reactions){impulse=Add(impulse,r.impulse);moment=Add(moment,r.moment);}Check(Near(impulse,{0,2,-3})&&Near(moment,{-3,1.5,1}),"Contact moment/barycentric conservation");
 auto separating=cloth_contact::Solve({2,1,0},{},{0,1,0},.02,.5,.35,1./120);
 Check(Near(separating.velocity,{2,1,0})&&Near(separating.impulse,{}),"Contact attracts separating fabric");
 auto sliding=cloth_contact::Solve({2,-1,0},{0,.5,0},{0,1,0},.01,.5,.35,1./120);
 Check(std::abs(sliding.velocity[1]-.5)<1e-12&&sliding.velocity[0]>0&&sliding.velocity[0]<2,"Moving contact/friction projection differs");
 Check(Near(sliding.impulse,Mul(Sub(sliding.velocity,{2,-1,0}),2)),"Terminal contact impulse does not equal actual mass delta velocity");
 ReactionCollector velocityCollector(tissue);velocityCollector.Add(0,bary,sliding.impulse,contact,0,faces);Output velocityOutput;velocityCollector.Publish(velocityOutput,1./120,1.8);
 Check(Near(Add(velocityOutput.reaction.clothImpulse,velocityOutput.reaction.anatomyImpulse),{}),"Terminal friction reaction not reciprocal");
 auto binding=[](Donor){MechanicalBinding b;b.weights[12]=1;return b;};auto identity=[](Point p){return p;};
 std::array<Point,2> centers{},radii{{{1,1,1},{1,1,1}}};SourceMasses masses{1.825,1.925};
 auto local=AggregateContactReactions(cloth,binding,identity,{},centers,radii,masses);
 Check(local.enabled&&local.contactReaction&&Near(Vector(local.lobeImpulseTotals[0]),impulse)&&Near(Vector(local.lobeAngularImpulseTotals[0]),moment),"Measured local lobe force/torque mapping");
 auto translated=cloth;Point translation{870,-932,512};for(auto& r:translated.reactions)r.moment=Add(r.moment,Cross(translation,r.impulse));
 auto translatedLocal=AggregateContactReactions(translated,binding,identity,Mul(translation,-1),centers,radii,masses);
 Check(Near(Vector(translatedLocal.lobeAngularImpulseTotals[0]),moment,1e-9),"World actor translation leaked into local lobe torque");
 auto rotated=[](Point p){return Point{-p[1],p[0],p[2]};};auto rotation=AggregateContactReactions(cloth,binding,rotated,{},centers,radii,masses);
 Check(Near(Vector(rotation.lobeImpulseTotals[0]),rotated(impulse))&&Near(Vector(rotation.lobeAngularImpulseTotals[0]),rotated(moment)),"Reaction rotation covariance");
 auto scaled=cloth;for(auto& r:scaled.reactions){r.impulse=Mul(r.impulse,100);r.moment=Mul(r.moment,10000);}auto inverse=[](Point p){return Mul(p,.01);};auto units=AggregateContactReactions(scaled,binding,inverse,{},centers,radii,masses);
 Check(Near(Vector(units.lobeImpulseTotals[0]),impulse)&&Near(Vector(units.lobeAngularImpulseTotals[0]),moment),"Impulse unit covariance");
 auto zero=cloth;zero.reaction.activeSeconds=0;Check(!AggregateContactReactions(zero,binding,identity,{},centers,radii,masses).enabled,"Zero-time fit fabricated force");
 zero=cloth;zero.reactions.clear();Check(!AggregateContactReactions(zero,binding,identity,{},centers,radii,masses).enabled,"Clear separated fabric fabricated force");
 for(unsigned period:{1u,2u,4u}){
  surface::GarmentImpulseLedger ledger;ledger.Reset(41);surface::GarmentImpulseCursor cursor;Point applied{},angular{};surface::Frame::GarmentSupport last;
  for(unsigned i=0;i<120;i++){last=ledger.Append(local);if(i%period==0){auto d=cursor.Consume(last);applied=Add(applied,Vector(d.lobes[0]));angular=Add(angular,Vector(d.angular[0]));auto duplicate=cursor.Consume(last);Check(Near(Vector(duplicate.lobes[0]),{}),"Busy retry replayed physical impulse");}}
  auto final=cursor.Consume(last);applied=Add(applied,Vector(final.lobes[0]));angular=Add(angular,Vector(final.angular[0]));Check(Near(applied,Mul(impulse,120))&&Near(angular,Mul(moment,120)),"30/60/120Hz cumulative coupling lost or replayed impulse");
  surface::Frame::GarmentSupport paused;cursor.Consume(paused);Check(Near(Vector(cursor.Consume(last).lobes[0]),{}),"Pause manufactured impulse");
  auto altered=last;altered.lobeImpulseTotals[0].x+=1;bool rejected=false;try{cursor.Consume(altered);}catch(const std::invalid_argument&){rejected=true;}Check(rejected&&Near(Vector(cursor.Consume(last).lobes[0]),{}),"Mutable publication identity corrupted cumulative cursor");
  ledger.Reset(42);auto restarted=ledger.Append(local);auto reset=cursor.Consume(restarted);Check(reset.reset&&Near(Vector(reset.lobes[0]),impulse),"Character/style reset replayed old impulse");
  surface::wire::Request request;request.frame.garment=restarted;auto decoded=surface::wire::DecodeRequest(surface::wire::Encode(request));Check(decoded.frame.garment.contactSerial==restarted.contactSerial&&decoded.frame.garment.contactEpoch==42&&Near(Vector(decoded.frame.garment.lobeAngularImpulseTotals[0]),moment),"Wire6 physical impulse totals changed");
 }
 auto sourceMass=CurrentSourceMasses({});Check(sourceMass.rod==double(.75f+86.f*.0125f)&&sourceMass.lobe==double(.75f+94.f*.0125f),"Measured source mass law differs");
 std::puts("PASS physical contact linear/angular conservation, exact tissue lineage, local torque, unit/rotation covariance, zero-time/separated no-force, 30/60/120Hz retries/coalescing/pause/reset and wire6");return 0;
 }catch(const std::exception& e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
