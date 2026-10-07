#include <malemod/garments/pouch_session.hpp>
#include <malemod/garments/cpu_cloth.hpp>
#include <malemod/garments/measured_capsules.hpp>
namespace malemod::garments {
struct PouchSession::Impl {
 struct Binding {std::vector<Donor> donors;Point offset{},normal{};};
 Output output;Mesh rest;Input reference;CpuCloth cloth;MeasuredCapsules colliders;
 detail::BodyCollider bodyCollider;
 std::vector<Binding> bindings;
 std::array<std::unordered_map<unsigned,unsigned>,2> indices;
 std::vector<Point> targets;
 std::vector<Capsule> capsules;
 std::vector<unsigned> bodyPatch;
 std::vector<std::array<Point,3>> bodyTriangles;
 std::vector<std::array<unsigned,3>> faces;
 std::vector<std::array<double,2>> material;
 struct Wrap {std::array<unsigned,3> nodes;Point weights;};
 std::vector<Wrap> wrap;
 std::vector<bool> pins;
 std::vector<std::pair<std::array<unsigned,2>,double>> edges;
 std::vector<std::pair<std::array<unsigned,2>,double>> renderEdges;
 double scale=0,clock=0,clearance=0,hemWidth=0,hemThickness=0;std::uint64_t restRevision=0;
 Point LocalVector(const Input& input,Point p)const{return {Dot(p,input.frame.lateral),Dot(p,input.frame.forward),Dot(p,input.frame.up)};}
 std::pair<Point,Point> Anchor(const Input& input,const Binding& binding)const{
  Point point{},normal{};for(auto d:binding.donors){const auto& surface=d.surface==Surface::Body?input.bodySurface:input.anatomy;const auto& sample=surface.at(indices[unsigned(d.surface)].at(d.vertex));point=Add(point,Mul(input.frame.Local(sample.position),d.weight/scale));normal=Add(normal,Mul(LocalVector(input,sample.normal),d.weight));}
  if(Length(normal)<1e-12)throw std::invalid_argument("Skin attachment normal degenerate");return {point,Unit(normal)};
 }
 std::vector<Point> Targets(const Input& input,const std::vector<Capsule>& obstacles)const{
  std::vector<Point> result;result.reserve(bindings.size());
  for(unsigned id=0;id<bindings.size();id++){
   const auto& b=bindings[id];auto anchor=Anchor(input,b);auto p=Add(anchor.first,anchor_transport::Offset(b.offset,b.normal,anchor.second,0));const auto g=output.layout.sheet;
   if(id<g.start+g.columns+1||id>=g.start+(g.rows+1)*(g.columns+1))for(unsigned pass=0;pass<3;pass++){
    auto query=Mul(Sub(input.frame.World(Mul(p,scale)),input.frame.origin),1/scale);auto hit=bodyCollider.Closest(query);if(hit.signedDistance>=clearance)break;p=Add(p,Mul(LocalVector(input,hit.normal),clearance-hit.signedDistance));
   }
   result.push_back(p);
  }
  // The pouch returns underneath to the measured perineal ends of the glute
  // straps. Anatomy-root donors are not garment attachment points.
  std::array<Point,2> bottom;
  for(unsigned side=0;side<2;side++){
   const auto& sample=input.rearStraps[side].back();auto normal=Unit(LocalVector(input,sample.normal));bottom[side]=Add(Mul(input.frame.Local(sample.position),1/scale),Mul(normal,clearance));
   for(unsigned pass=0;pass<4;pass++){double exit=0;for(auto c:obstacles)if(detail::CapsuleDistance(bottom[side],c)<.001)exit=(std::max)(exit,MeasuredCapsules::RayExit(bottom[side],normal,c)+.001);if(exit==0)break;bottom[side]=Add(bottom[side],Mul(normal,exit));}
  }
  if(bottom[0][0]>bottom[1][0])std::swap(bottom[0],bottom[1]);
  auto grid=output.layout.sheet;
  for(unsigned col=0;col<=grid.columns;col++){double t=double(col)/grid.columns;result[grid.start+grid.rows*(grid.columns+1)+col]=Add(Mul(bottom[0],1-t),Mul(bottom[1],t));}
  return result;
 }
 void Check(const Input& input)const{
  input.frame.Validate();if(input.characterEpoch!=output.characterEpoch||input.topologyRevision!=output.topologyRevision||input.restRevision!=restRevision)throw std::invalid_argument("Pouch material identity changed; explicit reinitialization required");
  if(input.bodySurface.size()!=reference.bodySurface.size()||input.anatomy.size()!=reference.anatomy.size()||input.bodyTriangles!=reference.bodyTriangles||input.anatomyTriangles!=reference.anatomyTriangles)throw std::invalid_argument("Pouch source topology changed without a new material identity");
  if(!Finite(input.gravity))throw std::invalid_argument("Nonfinite pouch gravity");
 }
 std::vector<Point> Sheet(const std::vector<Point>& all)const{
  const auto g=output.layout.sheet;return {all.begin()+g.start,all.begin()+g.start+(g.rows+1)*(g.columns+1)};
 }
 std::vector<Point> SimulationTargets(const std::vector<Point>& all)const{
  const auto g=output.layout.sheet;std::vector<Point> result;
  for(auto uv:material){unsigned row=unsigned(std::lround(uv[1]*g.rows));double x=uv[0]*g.columns;unsigned col=(std::min)(g.columns-1,unsigned(x));double t=x-col;unsigned id=g.start+row*(g.columns+1)+col;result.push_back(Add(Mul(all[id],1-t),Mul(all[id+1],t)));}return result;
 }
 std::vector<Point> WrapSheet(const std::vector<Point>& particles)const{
  std::vector<Point> result;for(const auto& b:wrap){Point p{};for(unsigned k=0;k<3;k++)p=Add(p,Mul(particles[b.nodes[k]],b.weights[k]));result.push_back(p);}return result;
 }
 std::vector<Point> BuildSimulation(const std::vector<Point>& sheet){
  const auto g=output.layout.sheet;std::vector<Point> points;std::vector<std::vector<unsigned>> rows;
  for(unsigned row=0;row<=g.rows;row++){
   double length=0;for(unsigned col=0;col<g.columns;col++)length+=Length(Sub(sheet[row*(g.columns+1)+col+1],sheet[row*(g.columns+1)+col]));
   unsigned columns=row?std::clamp(unsigned(std::ceil(length/.02)),2u,g.columns):g.columns;std::vector<unsigned> ids;
   for(unsigned col=0;col<=columns;col++){
    double u=double(col)/columns,x=u*g.columns;unsigned a=(std::min)(g.columns-1,unsigned(x));double t=x-a;
    ids.push_back(unsigned(points.size()));points.push_back(Add(Mul(sheet[row*(g.columns+1)+a],1-t),Mul(sheet[row*(g.columns+1)+a+1],t)));
    material.push_back({u,double(row)/g.rows});pins.push_back(row==0||(row==g.rows&&(col==0||col==columns)));
   }rows.push_back(std::move(ids));
  }
  faces.clear();for(unsigned row=0;row<g.rows;row++){
   const auto& a=rows[row];const auto& b=rows[row+1];unsigned i=0,j=0;
   while(i+1<a.size()||j+1<b.size()){
    if(i+1<a.size()&&(j+1==b.size()||material[a[i+1]][0]<=material[b[j+1]][0])){faces.push_back({a[i],a[i+1],b[j]});i++;}
    else{faces.push_back({a[i],b[j+1],b[j]});j++;}
   }
  }
  for(unsigned row=0;row<=g.rows;row++)for(unsigned col=0;col<=g.columns;col++){
   Point p{double(col)/g.columns,double(row)/g.rows,0};bool found=false;
   for(auto face:faces){std::array<Point,3> uv;for(unsigned k=0;k<3;k++)uv[k]={material[face[k]][0],material[face[k]][1],0};auto weights=drape::Barycentric(p,uv[0],uv[1],uv[2]);if((std::min)({weights[0],weights[1],weights[2]})>=-1e-9){wrap.push_back({face,weights});found=true;break;}}
   if(!found)throw std::runtime_error("Simulation mesh does not cover garment material coordinates");
  }
  return points;
 }
 std::vector<CpuCloth::MotionLimit> Limits(const std::vector<Point>& all)const{
  auto sheet=SimulationTargets(all);std::vector<CpuCloth::MotionLimit> result;for(auto p:sheet)result.push_back({p,1000});
  return result;
 }
 void BuildBodyPatch(const Input& input,const std::vector<Point>& pouch){
  Point low=pouch.front(),high=low;for(auto p:pouch)for(unsigned k=0;k<3;k++){low[k]=(std::min)(low[k],p[k]-.02);high[k]=(std::max)(high[k],p[k]+.02);}
  for(unsigned i=0;i<input.bodyTriangles.size();i++){
   std::array<Point,3> face;for(unsigned k=0;k<3;k++)face[k]=Mul(input.frame.Local(input.bodySurface[input.bodyTriangles[i][k]].position),1/scale);
   bool overlap=true;for(unsigned k=0;k<3;k++)if((std::max)({face[0][k],face[1][k],face[2][k]})<low[k]||(std::min)({face[0][k],face[1][k],face[2][k]})>high[k])overlap=false;
   if(overlap)bodyPatch.push_back(i);
  }
 }
 std::vector<std::array<Point,3>> BodyTriangles(const Input& input)const{
  std::vector<std::array<Point,3>> result;for(auto id:bodyPatch){std::array<Point,3> face;for(unsigned k=0;k<3;k++)face[k]=Mul(input.frame.Local(input.bodySurface[input.bodyTriangles[id][k]].position),1/scale);auto n=Cross(Sub(face[1],face[0]),Sub(face[2],face[0]));if(Length(n)>1e-12){auto offset=Mul(Unit(n),clearance);for(auto& p:face)p=Add(p,offset);}result.push_back(face);}return result;
 }
 void Render(const Input& input){
  auto points=targets;auto particles=cloth.Positions();auto sheet=WrapSheet(particles);const auto grid=output.layout.sheet;
  for(unsigned i=0;i<sheet.size();i++)points[grid.start+i]=sheet[i];
  std::vector<Point> normals(sheet.size());for(auto f:cloth_detail::SheetTriangles(0,grid.rows,grid.columns)){auto n=Cross(Sub(sheet[f[1]],sheet[f[0]]),Sub(sheet[f[2]],sheet[f[0]]));for(auto id:f)normals[id]=Add(normals[id],n);}for(auto& n:normals)n=Length(n)>1e-12?Unit(n):Point{0,1,0};
  // The side binding is a narrow elastic hem, driven by its actual sewn sheet
  // edge. The rear straps follow measured skin except for their short join.
  for(unsigned side=0;side<2;side++){
   auto hem=output.layout.sideHems[side];const auto& edge=output.layout.sideBoundary[side];
   std::vector<Sample> path;for(auto id:edge){auto n=normals[id-grid.start];auto query=Mul(Sub(input.frame.World(Mul(points[id],scale)),input.frame.origin),1/scale);auto hit=bodyCollider.Closest(query);if(hit.distance<hemWidth*2){auto skin=Unit(LocalVector(input,hit.normal));double t=std::clamp(hit.distance/(hemWidth*2),0.,1.);auto blended=Add(Mul(n,t),Mul(skin,1-t));if(Length(blended)>1e-9)n=Unit(blended);}path.push_back({Add(points[id],Mul(n,.6*hemThickness)),n,output.mesh.vertices[id].lineage});}
   Mesh tube;detail::Tube(tube,path,hemWidth,hemThickness,false,Frame{},MaterialSlot::WhiteElastic);
   for(unsigned i=0;i<hem.sections*hem.corners;i++)points[hem.start+i]=tube.vertices[i].position;
   for(unsigned endpoint:{0u,hem.sections-1}){
    auto name="hem-"+std::to_string(side)+(endpoint?"-bottom":"-top");auto joint=std::find_if(output.layout.authoredJoints.begin(),output.layout.authoredJoints.end(),[&](const auto& j){return j.name==name;});
    if(joint!=output.layout.authoredJoints.end()){auto n=path[endpoint].normal;joint->restOffset=Mul(Add(Mul(input.frame.lateral,n[0]),Add(Mul(input.frame.forward,n[1]),Mul(input.frame.up,n[2]))),hemThickness*.6*scale);}
   }
   auto strap=output.layout.straps[side];
   const auto joint=std::find_if(output.layout.authoredJoints.begin(),output.layout.authoredJoints.end(),[&](const auto& j){return j.name=="strap-"+std::to_string(side)+"-bottom";});
   if(joint!=output.layout.authoredJoints.end()){
    Point sewn{},cap{};for(unsigned k=0;k<joint->b.vertices.size();k++)sewn=Add(sewn,Mul(points[joint->b.vertices[k]],joint->b.weights[k]));
    for(unsigned k=0;k<joint->a.vertices.size();k++)cap=Add(cap,Mul(targets[joint->a.vertices[k]],joint->a.weights[k]));
    Point delta=Sub(sewn,cap);
    for(unsigned section=0;section<strap.sections;section++){
     double t=double(section)/(std::max)(1u,strap.sections-1);double blend=detail::Smooth(std::clamp((t-.875)/.125,0.,1.));
     for(unsigned c=0;c<strap.corners;c++){unsigned id=strap.start+section*strap.corners+c;points[id]=Add(targets[id],Mul(delta,blend));}
    }
   }
  }
  for(unsigned i=0;i<points.size();i++)output.mesh.vertices[i].position=input.frame.World(Mul(points[i],scale));
  detail::Shading(output.mesh);
  auto& telemetry=output.physics;telemetry.active=true;telemetry.stateReady=true;telemetry.nodes=unsigned(particles.size());telemetry.accumulatedSeconds=clock;telemetry.maxStretchRatio=1;
  for(const auto& e:edges){double length=Length(Sub(particles[e.first[0]],particles[e.first[1]]));double ratio=length/e.second;if(ratio>telemetry.maxStretchRatio){telemetry.maxStretchRatio=ratio;telemetry.worstStretchA=e.first[0];telemetry.worstStretchB=e.first[1];telemetry.worstStretchRestLength=e.second;telemetry.worstStretchCurrentLength=length;}}
  telemetry.maxRenderStretchRatio=1;
  for(const auto& e:renderEdges){double ratio=Length(Sub(sheet[e.first[0]],sheet[e.first[1]]))/e.second;if(ratio>telemetry.maxRenderStretchRatio){telemetry.maxRenderStretchRatio=ratio;telemetry.worstRenderA=grid.start+e.first[0];telemetry.worstRenderB=grid.start+e.first[1];}}
  telemetry.maxSeamGap=0;for(const auto& joint:output.layout.authoredJoints){auto measure=[&](const MaterialPoint& p){Point v{};for(unsigned k=0;k<p.vertices.size();k++)v=Add(v,Mul(output.mesh.vertices[p.vertices[k]].position,p.weights[k]));return v;};telemetry.maxSeamGap=(std::max)(telemetry.maxSeamGap,Length(Sub(Sub(measure(joint.a),measure(joint.b)),joint.restOffset)));}
  telemetry.materialBudgetSatisfied=telemetry.maxStretchRatio<=1.15&&telemetry.maxRenderStretchRatio<=1.15&&telemetry.maxSeamGap<=scale*output.layout.bandThicknessNormalized*.25;
  output.contactBudgetSatisfied=true;output.coverageMargin=1e100;output.bodyIntersectionCount=output.pouchPenetrationCount=0;output.bodyIntersectionsByPart={};
  for(auto face:cloth_detail::SheetTriangles(0,grid.rows,grid.columns))for(auto capsule:capsules){Point a,b;capsule.radius-=clearance;double gap=detail::TriangleCapsule(sheet[face[0]],sheet[face[1]],sheet[face[2]],capsule,a,b);output.coverageMargin=(std::min)(output.coverageMargin,gap*scale);if(gap<0){output.contactBudgetSatisfied=false;output.pouchPenetrationCount++;}}
  // Skinned trim can still intersect an animated thigh. Include the actual
  // complete body resource in the acceptance check; proxy success alone is
  // not a full garment contact pass.
  for(const auto& face:output.mesh.triangles){std::array<Point,3> triangle;for(unsigned k=0;k<3;k++)triangle[k]=Mul(Sub(output.mesh.vertices[face.vertices[k]].position,input.frame.origin),1/scale);if(bodyCollider.ClosestFace(triangle,1e-7).distance<1e-8){output.contactBudgetSatisfied=false;output.bodyIntersectionCount++;auto id=face.vertices[0];unsigned part=id<grid.start?0:id<grid.start+sheet.size()?1:id<output.layout.sideHems[0].start?2:3;output.bodyIntersectionsByPart[part]++;}}
  // This backend is conventional one-way garment contact. Never emit invented
  // tissue impulses; a coupled backend must implement its own measured load.
  output.reactions.clear();output.support.clear();output.reaction={};
 }
};
PouchSession::PouchSession(Parameters parameters):impl_(new Impl),parameters_(parameters){}
PouchSession::~PouchSession()=default;
void PouchSession::Reset(){impl_=std::make_unique<Impl>();}
const Output& PouchSession::InitializeDraped(const Input& reference,const Input& current){
 if(reference.characterEpoch!=current.characterEpoch||reference.topologyRevision!=current.topologyRevision||reference.restRevision!=current.restRevision||reference.bodyTriangles!=current.bodyTriangles||reference.anatomyTriangles!=current.anatomyTriangles)throw std::invalid_argument("Dressing reference and current material identities differ");
 Reset();auto& s=*impl_;auto parameters=parameters_;parameters.simulate=false;s.clearance=parameters_.clearance;s.hemWidth=parameters_.hemWidth;s.hemThickness=parameters_.hemThickness;
 Session fitter(parameters);s.output=fitter.Update(Style::WhiteJockstrap,reference);s.reference=reference;s.rest=s.output.mesh;s.scale=s.output.measuredCircumference;s.restRevision=reference.restRevision;
 if(!s.output.layout.sheet.rows)throw std::invalid_argument("CPU pouch requires the measured sheet pattern");
 for(unsigned kind=0;kind<2;kind++){const auto& surface=kind?reference.anatomy:reference.bodySurface;for(unsigned i=0;i<surface.size();i++)for(auto d:surface[i].lineage.donors)if(d.weight>.999999)s.indices[unsigned(d.surface)][d.vertex]=i;}
 s.bodyCollider.Update(reference.bodySurface,reference.bodyTriangles,reference.frame.origin,s.scale);
 const auto layout=s.output.layout.sheet;const unsigned sheetEnd=layout.start+(layout.rows+1)*(layout.columns+1);
 for(unsigned id=0;id<s.rest.vertices.size();id++){
  auto& vertex=s.rest.vertices[id];Impl::Binding binding;
  const bool bodyBound=id<layout.start||id>=sheetEnd||id<layout.start+layout.columns+1;
  if(bodyBound){
   auto hit=s.bodyCollider.Closest(Mul(Sub(vertex.position,reference.frame.origin),1/s.scale));auto weights=s.bodyCollider.ContactWeights(hit);auto face=reference.bodyTriangles.at(hit.triangle);
   std::map<unsigned,double> donors;for(unsigned k=0;k<3;k++)for(auto d:reference.bodySurface[face[k]].lineage.donors)if(d.weight>0){if(d.surface!=Surface::Body)throw std::invalid_argument("Body wrap received a non-body source donor");donors[d.vertex]+=d.weight*weights[k];}
   vertex.lineage={};unsigned at=0;for(auto d:donors)if(d.second>0){if(at>=vertex.lineage.donors.size())throw std::invalid_argument("Body wrap donor capacity exceeded");vertex.lineage.donors[at++]={Surface::Body,d.first,d.second};}
   s.output.mesh.vertices[id].lineage=vertex.lineage;
  }
  for(auto d:vertex.lineage.donors)if(d.weight>0)binding.donors.push_back(d);auto anchor=s.Anchor(reference,binding);binding.normal=anchor.second;binding.offset=Sub(Mul(reference.frame.Local(vertex.position),1/s.scale),anchor.first);s.bindings.push_back(binding);
 }
 s.bodyCollider.Update(current.bodySurface,current.bodyTriangles,current.frame.origin,s.scale);s.colliders.Build(reference);s.capsules=s.colliders.Fit(current,s.scale,parameters_.clearance);s.targets=s.Targets(current,s.capsules);
 auto points=s.Sheet(s.targets);const auto grid=s.output.layout.sheet;
 s.faces=cloth_detail::SheetTriangles(0,grid.rows,grid.columns);s.pins.resize(points.size());
 for(unsigned x=0;x<=grid.columns;x++)s.pins[x]=true;
 // Both ends are measured body attachments, not pins in moving anatomy.
 s.pins[grid.rows*(grid.columns+1)]=true;s.pins.back()=true;
 std::set<std::array<unsigned,2>> unique;
 for(auto f:s.faces)for(unsigned i=0;i<3;i++){std::array<unsigned,2> edge{f[i],f[(i+1)%3]};std::sort(edge.begin(),edge.end());unique.insert(edge);}
 for(auto edge:unique){double length=Length(Sub(points[edge[0]],points[edge[1]]));if(length>1e-10)s.edges.push_back({edge,length});}
 Point origin{};for(const auto& sample:reference.opening)origin=Add(origin,Mul(reference.frame.Local(sample.position),1/s.scale));origin=Mul(origin,1./reference.opening.size());
 for(unsigned i=0;i<points.size();i++)if(!s.pins[i]){
  unsigned row=i/(grid.columns+1),col=i%(grid.columns+1);double t=double(row)/grid.rows;
  auto rest=Mul(reference.frame.Local(s.rest.vertices[grid.start+i].position),1/s.scale);
  auto oldBottom=Mul(reference.frame.Local(s.rest.vertices[grid.start+grid.rows*(grid.columns+1)+col].position),1/s.scale);
  auto newBottom=points[grid.rows*(grid.columns+1)+col];rest=Add(rest,Mul(Sub(newBottom,oldBottom),t*t));
  auto direction=Unit(Sub(rest,origin));double radius=Length(Sub(rest,origin));
  for(auto c:s.capsules)radius=(std::max)(radius,MeasuredCapsules::RayExit(origin,direction,c)+.003);
  points[i]=Add(origin,Mul(direction,radius));
 }
 for(unsigned i=0;i<points.size();i++)if(s.pins[i])for(unsigned c=0;c<s.capsules.size();c++)if(detail::CapsuleDistance(points[i],s.capsules[c])<-parameters_.clearance)throw std::runtime_error("Pouch attachment inside measured proxy: pin="+std::to_string(i)+" capsule="+std::to_string(c)+" gap="+std::to_string(detail::CapsuleDistance(points[i],s.capsules[c])));
 s.pins.clear();points=s.BuildSimulation(points);unique.clear();
 for(auto f:s.faces)for(unsigned i=0;i<3;i++){std::array<unsigned,2> edge{f[i],f[(i+1)%3]};std::sort(edge.begin(),edge.end());unique.insert(edge);}
 s.edges.clear();for(auto edge:unique){double length=Length(Sub(points[edge[0]],points[edge[1]]));if(length>1e-10)s.edges.push_back({edge,length});}
 s.cloth.Initialize(points,s.faces,s.pins);
 s.BuildBodyPatch(current,points);s.bodyTriangles=s.BodyTriangles(current);
 s.bodyCollider.Update(current.bodySurface,current.bodyTriangles,current.frame.origin,s.scale);
 // A bounded warm start runs the same simulation that will run during play.
 for(unsigned step=0;step<240;step++)s.cloth.Step(1./120,s.SimulationTargets(s.targets),s.capsules,{},s.Limits(s.targets),{},s.bodyTriangles);
 // Cook the fitted three-dimensional sewing pattern once. Fitting precedes
 // runtime material measurement; animated frames never rewrite rest lengths.
 constexpr double patternEase=1.25;
 points=s.cloth.Positions();s.cloth.Initialize(points,s.faces,s.pins,patternEase);
 s.edges.clear();for(auto edge:unique){double length=Length(Sub(points[edge[0]],points[edge[1]]));if(length>1e-10)s.edges.push_back({edge,length*patternEase});}
 auto renderedRest=s.WrapSheet(points);std::set<std::array<unsigned,2>> fineEdges;
 for(auto face:cloth_detail::SheetTriangles(0,grid.rows,grid.columns))for(unsigned k=0;k<3;k++){std::array<unsigned,2> edge{face[k],face[(k+1)%3]};std::sort(edge.begin(),edge.end());fineEdges.insert(edge);}
 for(auto edge:fineEdges){double length=Length(Sub(renderedRest[edge[0]],renderedRest[edge[1]]));if(length<1e-10)throw std::runtime_error("Fitted render mesh contains a collapsed material edge");s.renderEdges.push_back({edge,length*patternEase});}
 for(unsigned step=0;step<60;step++)s.cloth.Step(1./120,s.SimulationTargets(s.targets),s.capsules,{},s.Limits(s.targets),{},s.bodyTriangles);
 s.Render(current);return s.output;
}
void PouchSession::PlacePrepared(const Input&,const Input& current){auto reference=impl_->reference;InitializeDraped(reference,current);}
const Output& PouchSession::Update(Style style,const Input& input,TimeContinuity continuity){
 auto& s=*impl_;if(style==Style::Naked){Reset();return impl_->output;}
 if(style!=Style::WhiteJockstrap)throw std::invalid_argument("Unknown pouch style");
 if(!s.scale)throw std::logic_error("Pouch must be initialized from measured reference");s.Check(input);
 if(!std::isfinite(input.deltaTime)||input.deltaTime<0||input.deltaTime>2)throw std::invalid_argument("Pouch elapsed time needs an explicit discontinuity reset");
 if(continuity==TimeContinuity::Unverified&&input.deltaTime>.1)throw std::invalid_argument("Long pouch span lacks verified active-time continuity");
 auto began=std::chrono::steady_clock::now();s.bodyCollider.Update(input.bodySurface,input.bodyTriangles,input.frame.origin,s.scale);auto capsules=s.colliders.Fit(input,s.scale,parameters_.clearance);auto targets=s.Targets(input,capsules);auto bodyTriangles=s.BodyTriangles(input);
 for(unsigned i=0;i<capsules.size();i++)if(Dot(Sub(capsules[i].b,capsules[i].a),Sub(s.capsules[i].b,s.capsules[i].a))<0)std::swap(capsules[i].a,capsules[i].b);
 unsigned steps=unsigned(std::ceil(input.deltaTime*60));
 for(unsigned step=1;step<=steps;step++){
  double t=double(step)/steps;auto blendTargets=targets;auto blendCapsules=capsules;
  for(unsigned i=0;i<targets.size();i++)blendTargets[i]=Add(Mul(s.targets[i],1-t),Mul(targets[i],t));
  for(unsigned i=0;i<capsules.size();i++){blendCapsules[i].a=Add(Mul(s.capsules[i].a,1-t),Mul(capsules[i].a,t));blendCapsules[i].b=Add(Mul(s.capsules[i].b,1-t),Mul(capsules[i].b,t));blendCapsules[i].radius=s.capsules[i].radius*(1-t)+capsules[i].radius*t;}
  auto blendTriangles=bodyTriangles;for(unsigned i=0;i<bodyTriangles.size();i++)for(unsigned k=0;k<3;k++)blendTriangles[i][k]=Add(Mul(s.bodyTriangles[i][k],1-t),Mul(bodyTriangles[i][k],t));
  s.cloth.Step(input.deltaTime/steps,s.SimulationTargets(blendTargets),blendCapsules,Mul(s.LocalVector(input,input.gravity),1/s.scale),s.Limits(blendTargets),{},blendTriangles);
 }
 s.clock+=input.deltaTime;s.targets=std::move(targets);s.capsules=std::move(capsules);s.bodyTriangles=std::move(bodyTriangles);s.Render(input);
 s.output.physics.substeps=steps;s.output.physics.advancedSeconds=input.deltaTime;s.output.physics.solverMilliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();return s.output;
}
}
