#pragma once
#include "modular_asset.hpp"
#include "pelvic_aperture.hpp"
#include "surface_stitch.hpp"
#include "suspension_hang.hpp"
#include <chrono>

namespace malemod::surface::modular {
struct DynamicResult {SurfaceJoin surface;Vec anchor{},axis{};double rootAdvance=0;std::vector<SurfaceJoinCorner> moduleVertices;std::vector<double> advances;Vec fitTranslation{},suspensionTranslation{};std::shared_ptr<const std::vector<Binding>> clearedBindings;double materialRoot=0;};
struct DynamicMount {
 // Recruit a compact measured annulus, with extra ventral clearance. Keep the
 // aperture and its tangent construction on the same dimensionless contract.
 static constexpr double rampMargin=.24,undersideRampExtra=.08;
 std::vector<Vec> body,normals,section;std::vector<std::array<unsigned,3>> faces;
 std::vector<unsigned> moduleFaces,inner;Vec origin;PresentationBasis basis;
 std::string geometryHash;
 explicit DynamicMount(const std::filesystem::path& path){
  if(std::filesystem::file_size(path)>64*1024*1024)throw std::invalid_argument("Oversize dynamic mount");std::ifstream in(path,std::ios::binary);
  auto read=[&](void* p,std::size_t n){if(!in.read(static_cast<char*>(p),n))throw std::invalid_argument("Truncated dynamic mount");};auto text=[&](unsigned n){std::string s(n,'\0');read(s.data(),n);return s;};auto u=[&]{unsigned n;read(&n,4);return n;};
  if(text(8)!="MMMDYN01")throw std::invalid_argument("Dynamic mount version differs");geometryHash=text(64);
  auto nv=u(),nf=u(),nr=u(),nm=u(),ni=u();if(!nv||nv>500000||!nf||nf>1000000||nr<8||nr>4096||!nm||nm>1000000||ni<8||ni>50000)throw std::invalid_argument("Invalid dynamic mount counts");
  body.resize(nv);normals.resize(nv);faces.resize(nf);section.resize(nr);moduleFaces.resize(nm);inner.resize(ni);
  read(body.data(),nv*24);read(normals.data(),nv*24);read(faces.data(),nf*12);read(section.data(),nr*24);read(&origin,24);read(&basis,72);read(moduleFaces.data(),nm*4);read(inner.data(),ni*4);
  if(in.peek()!=EOF)throw std::invalid_argument("Trailing dynamic mount bytes");
  for(const auto* values:{&body,&normals,&section})for(auto p:*values)for(double x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite dynamic source");
  Quaternion(basis);for(auto f:faces)for(auto i:f)if(i>=nv)throw std::invalid_argument("Dynamic body index leaves original surface");
 }
 std::shared_ptr<const std::vector<Vec>> ClearedMaterial(const Asset& asset,const State& state,const std::vector<double>& advances)const{
  if(!state.shaftMaterial)return {};
  const auto& mount=*asset.mount;auto p=std::make_shared<std::vector<Vec>>(*state.shaftMaterial);
  for(unsigned i=0;i<p->size();i++)if(mount.shaftWeight[i]>0&&ShaftBinding(asset.bindings[i])){
   const double advance=advances.at(i),span=(std::max)(mount.length*.7*state.scales[1][0],2.1*advance),station=Dot(Sub(asset.rest[i],origin),basis[0]),t=std::clamp((station-mount.station)*state.scales[1][0]/span,0.,1.);
   (*p)[i]=Add((*p)[i],Scale(state.shapeFrames[1].basis[0],advance*(1-t*t*t*(10+t*(-15+6*t)))*mount.shaftWeight[i]));
  }return p;
 }
 std::pair<double,std::shared_ptr<const std::vector<Binding>>> SocketBindings(const Asset& asset,const State& state,const std::vector<Vec>& material)const{
  std::vector<Vec> rim;std::vector<Binding> owners;
  for(auto id:inner){rim.push_back(material.at(id));owners.push_back(asset.bindings.at(id));}
  double start=0;
  // Curved rest guides converge asymptotically as the clipped guide is
  // resampled. Eight iterations left a 3e-5 station residual at slider 66.
  // Solve only the rim during this search, then bind the full surface once.
  for(unsigned iteration=0;iteration<32;iteration++){
   const auto socket=SocketMaterialState(state,start);const auto bindings=BindMaterialStations(rim,owners,socket.shapeFrames);
   std::array<double,13> station{};for(unsigned i=2;i<=12;i++){auto d=Sub(socket.shapeFrames[i].origin,socket.shapeFrames[i-1].origin);station[i]=station[i-1]+std::sqrt(Dot(d,d));}
   double remaining=0;for(const auto& binding:bindings){double s=0;for(auto v:binding.influences)if(v.weight>0){if(v.frame<1||v.frame>12)throw std::invalid_argument("Socket rim is not complete-shaft material");s+=station[v.frame]*v.weight;}remaining=(std::max)(remaining,s);}
   if(remaining<1e-8)return {start,std::make_shared<const std::vector<Binding>>(BindMaterialStations(material,asset.bindings,socket.shapeFrames))};
   start+=remaining+1e-5;
  }
  throw std::invalid_argument("Unable to anchor the complete socket circumference");
 }
 std::vector<Vec> Pose(const Asset& asset,const Plan& plan,const State& state,Vec translation,const std::vector<double>& advances,Vec suspension={},std::shared_ptr<const std::vector<Binding>> clearedBindings={},double materialRoot=-1,bool relief=true)const{
  auto moved=state;for(unsigned i=1;i<15;i++)moved.frames[i].origin=Add(moved.frames[i].origin,translation);const auto& mount=*asset.mount;
  auto clearance=[&](unsigned i){const double advance=advances.at(i),span=(std::max)(mount.length*.7*state.scales[1][0],2.1*advance),station=Dot(Sub(asset.rest[i],origin),basis[0]),t=std::clamp((station-mount.station)*state.scales[1][0]/span,0.,1.);return advance*(1-t*t*t*(10+t*(-15+6*t)))*mount.shaftWeight[i];};
  auto materialState=state;
  if(state.shaftMaterial){moved.shaftMaterial=ClearedMaterial(asset,state,advances);if(!clearedBindings||materialRoot<0){auto socket=SocketBindings(asset,state,*moved.shaftMaterial);materialRoot=socket.first;clearedBindings=socket.second;}materialState=SocketMaterialState(state,materialRoot);materialState.materialBindings=clearedBindings;for(unsigned i=1;i<=12;i++)materialState.frames[i].origin=Add(materialState.frames[i].origin,translation);}
  auto points=plan.Evaluate(moved,[&](auto& points){for(unsigned i=0;i<points.size();i++)if(mount.shaftWeight[i]>0&&!(state.shaftMaterial&&ShaftBinding(asset.bindings[i])))points[i]=Add(points[i],Scale(state.frames[1].basis[0],clearance(i)));plan.TranslateSuspension(points,suspension);},state.shaftMaterial?&materialState:nullptr);
  if(relief)asset.Relieve(points,state);return points;
 }
 std::vector<Vec> Repose(const DynamicResult& cached,const Asset& asset,const Plan& plan,const State& state,std::vector<Vec>* beforeRelief=nullptr,std::vector<Vec>* detailNormals=nullptr,std::array<double,4>* timings=nullptr)const{
  const auto begin=std::chrono::steady_clock::now();
  auto source=Pose(asset,plan,state,cached.fitTranslation,cached.advances,cached.suspensionTranslation,cached.clearedBindings,cached.materialRoot,false);auto points=cached.surface.points;
  const auto posed=std::chrono::steady_clock::now();
  if(beforeRelief){*beforeRelief=points;for(auto c:cached.moduleVertices){Vec p{};for(unsigned k=0;k<3;k++)p=Add(p,Scale(source[asset.faces[c.parent][k]],c.donor[k]));(*beforeRelief)[c.vertex]=p;}}
  std::vector<Vec> detail;asset.Relieve(source,state,detailNormals?&detail:nullptr,plan.Arena());
  const auto relieved=std::chrono::steady_clock::now();
  if(detailNormals){detailNormals->assign(points.size(),{});for(auto c:cached.moduleVertices){Vec n{};for(unsigned k=0;k<3;k++)n=Add(n,Scale(detail[asset.faces[c.parent][k]],c.donor[k]));(*detailNormals)[c.vertex]=n;}}
  for(auto c:cached.moduleVertices){Vec p{};for(unsigned k=0;k<3;k++)p=Add(p,Scale(source[asset.faces[c.parent][k]],c.donor[k]));points[c.vertex]=p;}
  const auto mapped=std::chrono::steady_clock::now();
  const auto& rings=cached.surface.rings;auto first=cached.surface.firstTangents;std::vector<Vec> last(first.size());double firstScale=1,lastScale=1;
  const auto rimAxis=state.shaftMaterial?SocketMaterialState(state,cached.materialRoot).frames[1].basis[0]:state.frames[1].basis[0];
  for(unsigned i=0;i<first.size();i++){
   auto a=points[rings.front()[i]],b=points[rings.back()[i]];const double distance=std::sqrt(Dot(Sub(a,b),Sub(a,b)));last[i]=Scale(rimAxis,distance);
   auto radial=Sub(Sub(a,cached.anchor),Scale(basis[0],Dot(Sub(a,cached.anchor),basis[0])));auto bounded=BoundRampTangents(a,b,first[i],last[i],CrossSectionRadial(radial,rimAxis),.005*std::sqrt(Dot(radial,radial)));
   if(Dot(first[i],first[i])>1e-20)firstScale=(std::min)(firstScale,std::sqrt(Dot(bounded[0],bounded[0])/Dot(first[i],first[i])));
   if(Dot(last[i],last[i])>1e-20)lastScale=(std::min)(lastScale,std::sqrt(Dot(bounded[1],bounded[1])/Dot(last[i],last[i])));
  }
  for(unsigned i=0;i<first.size();i++){first[i]=Scale(first[i],firstScale);last[i]=Scale(last[i],lastScale);}
  for(unsigned row=1;row+1<rings.size();row++){
   const double t=.5*(1-std::cos(3.14159265358979323846*row/(rings.size()-1)));
   for(unsigned i=0;i<rings[0].size();i++){auto a=points[rings.front()[i]],b=points[rings.back()[i]];auto p=RaisedRampPoint(a,b,first[i],last[i],cached.anchor,basis[0],t);const auto id=rings[row][i];
    if(id<cached.surface.entryNormals.size())p=RampEntryPoint(p,cached.surface.entryPoints[id],cached.surface.entryNormals[id],basis[0],t);
    points[id]=p;}
  }
  if(timings){const auto end=std::chrono::steady_clock::now();auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};*timings={ms(begin,posed),ms(posed,relieved),ms(relieved,mapped),ms(mapped,end)};}
  return points;
 }
 DynamicResult Evaluate(const Asset& asset,const Plan& plan,const State& state)const{
  if(!asset.mount)throw std::invalid_argument("Dynamic mount needs semantic shaft ownership");
  for(auto i:moduleFaces)if(i>=asset.faces.size())throw std::invalid_argument("Dynamic module face leaves bound source");for(auto i:inner)if(i>=asset.rest.size())throw std::invalid_argument("Dynamic inner loop leaves bound source");
  const auto& mount=*asset.mount;auto axis=state.frames[1].basis[0];const double scale=state.scales[1][1]/asset.reference.scales[1][1];
  auto localAnchor=Unrotate(asset.reference.frames[1].basis,Sub(origin,asset.reference.frames[1].origin));
  auto requestedAnchor=Add(state.frames[1].origin,Rotate(state.frames[1].basis,Components(localAnchor,state.scales[1])));
  PelvicAperturePlan aperturePlan(body,faces,requestedAnchor,basis,section,1e-8);auto aperture=aperturePlan.Solve(axis,scale,rampMargin,undersideRampExtra);
  const auto apertureChart=ChartLoop(aperture.body,aperture.anchor,{{basis[1],basis[2]}});
  auto moved=state;for(unsigned i=1;i<15;i++)moved.frames[i].origin=Add(moved.frames[i].origin,aperture.translation);
  double advance=0;std::vector<double> advances(asset.rest.size());Vec suspension{};
  auto posed=plan.Evaluate(moved,[&](auto& points){
   const auto tilt=Sub(axis,Scale(basis[0],Dot(axis,basis[0])));const double tilt2=Dot(tilt,tilt);
   double tiltActivation=std::clamp((std::sqrt(tilt2)-.1)/.2,0.,1.);tiltActivation=tiltActivation*tiltActivation*(3-2*tiltActivation);
   std::vector<std::pair<double,double>> angular;
   for(auto id:inner){
    const auto radial=Sub(Sub(asset.rest[id],origin),Scale(basis[0],Dot(Sub(asset.rest[id],origin),basis[0])));
    double angle=std::atan2(Dot(radial,basis[2]),Dot(radial,basis[1]));if(angle<0)angle+=6.2831853071795864769;
    auto upper=std::upper_bound(apertureChart.angles.begin(),apertureChart.angles.end(),angle);unsigned next=unsigned(upper-apertureChart.angles.begin())%apertureChart.loop.size(),previous=(next+unsigned(apertureChart.loop.size())-1)%apertureChart.loop.size();
    const auto a=apertureChart.coordinates[previous],b=apertureChart.coordinates[next];const double y=std::cos(angle),z=std::sin(angle),den=(b[0]-a[0])*z-(b[1]-a[1])*y;
    const double t=std::clamp(-(a[0]*z-a[1]*y)/den,0.,1.);auto outside=Add(Scale(aperture.body.points[apertureChart.loop[previous]],1-t),Scale(aperture.body.points[apertureChart.loop[next]],t));
    double local=(std::max)(0.,(Dot(Sub(outside,points[id]),basis[0])+.12*asset.dimensions.radius*scale)/Dot(axis,basis[0]));
    // Only the compressed arc needs tilt clearance. A single maximum for
    // the whole circle turns the opposite arc into a long protruding wedge.
    if(tiltActivation>0&&Dot(radial,tilt)>0)local=(std::max)(local,tiltActivation*Dot(Sub(outside,points[id]),tilt)/tilt2);
    angular.emplace_back(angle,local);advance=(std::max)(advance,local);
   }
   std::sort(angular.begin(),angular.end());
   // Zero supports cannot affect a nonnegative maximum; skip their expensive
   // trigonometry without changing the original nonzero evaluation order.
   angular.erase(std::remove_if(angular.begin(),angular.end(),[](auto sample){return sample.second==0;}),angular.end());
   for(unsigned i=0;i<points.size();i++)if(mount.shaftWeight[i]>0){
    const auto radial=Sub(asset.rest[i],origin);double angle=std::atan2(Dot(radial,basis[2]),Dot(radial,basis[1]));if(angle<0)angle+=6.2831853071795864769;
    // A nonnegative periodic support envelope is at least every measured
    // clearance at its own knot. Its zero antipode leaves the opposite arc
    // free, while avoiding narrow angular spikes that fold neighboring strips.
    double local=0;for(auto sample:angular){double support=.5*(1+std::cos(angle-sample.first));local=(std::max)(local,sample.second*support*support);}advances[i]=local;
    const double span=(std::max)(mount.length*.7*state.scales[1][0],2.1*local),station=Dot(radial,basis[0]),t=std::clamp((station-mount.station)*state.scales[1][0]/span,0.,1.);const double fade=1-t*t*t*(10+t*(-15+6*t));points[i]=Add(points[i],Scale(axis,local*fade*mount.shaftWeight[i]));
   }
   // Clearance advances the shaft and its attached upper rim. Establish the
   // suspension equilibrium from that FINAL rim, not the pre-clearance shape.
   // Evaluate is control/rest-only; Repose reuses this constant offset and
   // therefore retains the source solver's complete live swing.
   suspension=plan.SuspensionRestOffset(points,moved);
   if(state.hangControl<50){
    auto measured=points;
    // Source offsets can contain live soft-tissue motion. Measure the lower
    // endpoints from the immutable zero-motion shape used by this cache.
    if(state.shaftMaterial)for(const auto& l:asset.lofts)
     if(ShaftBinding(l.lastOwner)&&!ShaftBinding(l.firstOwner))
      measured[l.start]=Add(state.shaftMaterial->at(l.start),aperture.translation);
    suspension[2]+=SuspensionLift(measured,asset.lofts,state.sectionRadius,state.hangControl);
   }
   plan.TranslateSuspension(points,suspension);
  });
  asset.Relieve(posed,state);
  SurfaceCut module;module.points=std::move(posed);module.opening=inner;
  for(auto id:moduleFaces){SurfaceCutFace face;face.parent=id;for(unsigned k=0;k<3;k++){face.corners[k].vertex=asset.faces[id][k];face.corners[k].donor[k]=1;}module.faces.push_back(face);}
  auto bodyChart=ChartLoop(aperture.body,aperture.anchor,{{basis[1],basis[2]}}),moduleChart=ChartLoop(module,origin,{{basis[1],basis[2]}},&asset.rest);
  auto surface=StitchSurfaces(std::move(aperture.body),std::move(module),bodyChart,moduleChart,aperture.anchor,basis[0],axis,normals,faces,rampMargin,undersideRampExtra,24,&body);
  std::vector<SurfaceJoinCorner> mappings;std::vector<bool> visited(surface.points.size());for(auto face:surface.faces)for(auto c:face)if(c.domain==1&&!visited[c.vertex]){visited[c.vertex]=true;mappings.push_back(c);}
  // Clearance changes the final material coordinates. Keeping the pre-clearance
  // station makes an advanced root vertex follow a frame behind its location,
  // creating a rigid shelf next to a moving section. Bind the final shape once,
  // then reuse these immutable stations throughout live motion.
  std::shared_ptr<const std::vector<Binding>> clearedBindings;double materialRoot=0;
  if(auto material=ClearedMaterial(asset,state,advances)){auto socket=SocketBindings(asset,state,*material);materialRoot=socket.first;clearedBindings=socket.second;}
  return {std::move(surface),aperture.anchor,axis,advance,std::move(mappings),std::move(advances),aperture.translation,suspension,clearedBindings,materialRoot};
 }
};
}
