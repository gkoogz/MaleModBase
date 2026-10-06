#pragma once
namespace malemod::garments {
inline const Output& Session::FitSheet(Style style,const Input& input){
 using namespace detail;
 output_=Output{};output_.style=style;output_.characterEpoch=input.characterEpoch;output_.topologyRevision=input.topologyRevision;
 if(style==Style::Naked)return output_;if(style!=Style::WhiteJockstrap)throw std::invalid_argument("Unknown measured garment style");
 input.frame.Validate();ValidateSamples(input.waist,8,128);ValidateSamples(input.opening,8,128);ValidateSamples(input.anatomy,4,131072);for(auto& path:input.rearStraps)ValidateSamples(path,3,128);drape::ValidateRegions(input);
 if(input.bodySurface.empty())throw std::invalid_argument("Measured sheet requires the actual body surface");
 double C=0;for(unsigned i=0;i<input.waist.size();i++)C+=Length(Sub(input.waist[i].position,input.waist[(i+1)%input.waist.size()].position));if(!std::isfinite(C)||C<1e-12)throw std::invalid_argument("Degenerate measured material waist");output_.measuredCircumference=C;
 const unsigned rows=parameters_.pouchRings,columns=parameters_.pouchSegments;const double gap=parameters_.clearance*C,margin=parameters_.clearance*.3,thick=parameters_.bandThickness*C;auto& mesh=output_.mesh;const auto& frame=input.frame;
 auto band=BuildBand(input,parameters_,output_);const unsigned waist=unsigned(band.measured.front().size()),bandCount=unsigned(mesh.vertices.size()),bandFaces=unsigned(mesh.triangles.size());output_.layout.revision=2;output_.layout.upperArcRadians=2*parameters_.panelHalfAngle;output_.layout.sideCoverageExtra=parameters_.sideCoverageExtra;output_.layout.band={0,6,waist};output_.layout.bandLayers=2;output_.layout.jointRevision=1;output_.layout.measuredCircumference=C;output_.layout.bandThicknessNormalized=parameters_.bandThickness;
 auto normalize=[&](Point p){return Mul(Sub(p,frame.origin),1/C);};auto world=[&](Point p){return Add(frame.origin,Mul(p,C));};
 bodyCollider_.Update(input.bodySurface,input.bodyTriangles,frame.origin,C);anatomyCollider_.Update(input.anatomy,input.anatomyTriangles,frame.origin,C);ClassifySurfaces(input,frame.origin,C,true);
 // Tilt the finite band into the ACTUAL recruited pelvic ramp. The measured
 // contours retain skin lineage; only a local complete-contact shortfall may
 // add clearance. Moving the entire belt radially discards the ramp's vertical
 // normal and makes unrelated waist regions float. Both thickness layers and
 // native UV aliases share each local correction.
 const unsigned bandLayer=7*(waist+1);
 auto bandColumn=[&](unsigned id){return ((id%(bandLayer))/(waist+1))*(waist+1)+(id%(waist+1))%waist;};
 auto moveBand=[&](unsigned column,Point delta){
  for(unsigned layer=0;layer<2;layer++){auto id=layer*bandLayer+column;mesh.vertices[id].position=Add(mesh.vertices[id].position,delta);if(column%(waist+1)==0)mesh.vertices[id+waist].position=mesh.vertices[id].position;}
 };
 for(unsigned pass=0;pass<24;pass++){
  bool changed=false;
  for(unsigned id=bandLayer;id<bandCount;id++)if(id%(waist+1)!=waist)for(auto* physical:{&bodyCollider_,&anatomyCollider_}){
   auto point=normalize(mesh.vertices[id].position);auto hit=physical->Closest(point);double signedDistance=ClassifiedDistance(hit,point,physical==&anatomyCollider_);
   if(signedDistance<margin+2e-5){moveBand(bandColumn(id),Mul(hit.normal,(margin+3e-5-signedDistance)*C));changed=true;}
  }
  for(unsigned fi=0;fi<bandFaces;fi++)for(auto* physical:{&bodyCollider_,&anatomyCollider_}){
   auto f=mesh.triangles[fi];std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalize(mesh.vertices[f.vertices[k]].position);
   auto hit=physical->ClosestFace(points,margin+2e-5);if(hit.distance>=margin+2e-5)continue;
   auto difference=Sub(hit.clothPoint,hit.point);auto direction=Length(difference)>1e-12?Unit(difference):hit.normal;if(Dot(direction,hit.normal)<0)direction=hit.normal;
   auto weights=drape::Barycentric(hit.clothPoint,points[0],points[1],points[2]);std::map<unsigned,double> grouped;
   for(unsigned k=0;k<3;k++)grouped[bandColumn(f.vertices[k])]+=(std::max)(0.,weights[k]);
   double square=0;for(auto term:grouped)square+=term.second*term.second;if(square<=1e-20)throw std::invalid_argument("Measured band contact has no material support");
   for(auto term:grouped)if(term.second>0)moveBand(term.first,Mul(direction,(margin+3e-5-hit.distance)*C*term.second/square));changed=true;
  }
  if(!changed)break;
 }
 double lateralCenter=0;for(auto s:input.waist)lateralCenter+=Dot(normalize(s.position),frame.lateral);lateralCenter/=input.waist.size();
 auto projectAt=[&](Sample& sample,double target){
  const Point original=normalize(sample.position);Point point=original;
  const double side=Dot(original,frame.lateral)-lateralCenter;
  // Initial lanes retain their measured side of the waist. An escape through
  // the opposite thigh would invert adjacent material columns. This is only
  // rest construction; the persistent cloth has no midplane/body attraction.
  auto clear=[&](Point p){if(std::abs(side)>1e-12&&side*(Dot(p,frame.lateral)-lateralCenter)<-1e-12)return false;for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->Closest(p);if(ClassifiedDistance(hit,p,collider==&anatomyCollider_)<target+1e-5)return false;}return true;};
  if(clear(point))return;
  for(unsigned pass=0;pass<16;pass++){for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->Closest(point);hit.signedDistance=ClassifiedDistance(hit,point,collider==&anatomyCollider_);if(hit.signedDistance<target+1e-5)point=Add(point,Mul(hit.normal,target-hit.signedDistance+2e-5));}if(clear(point)){sample.position=world(point);return;}}
  // At overlapping tissue boundaries an alternating nearest projection can
  // oscillate. Find a real outside point of the complete measured union.
  std::vector<Point> directions{Unit(sample.normal),frame.forward,Mul(frame.forward,-1),frame.up,Mul(frame.up,-1),frame.lateral,Mul(frame.lateral,-1)};
  for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto h=collider->Closest(original);ClassifiedDistance(h,original,collider==&anatomyCollider_);directions.push_back(h.normal);}
  double best=1e100;Point escaped{};for(auto direction:directions){double low=0,high=target*2;for(unsigned bracket=0;bracket<20&&!clear(Add(original,Mul(direction,high)));bracket++)high*=2;if(high>=best||!clear(Add(original,Mul(direction,high))))continue;for(unsigned step=0;step<24;step++){double middle=(low+high)*.5;if(clear(Add(original,Mul(direction,middle))))high=middle;else low=middle;}if(high<best){best=high;escaped=Add(original,Mul(direction,high+2e-5));}}
  if(best==1e100||!clear(escaped))throw std::invalid_argument("Measured tissue support cannot escape the physical union");sample.position=world(escaped);
 };

 auto project=[&](Sample& sample){projectAt(sample,margin);};
 auto supportProject=[&](Sample& sample){projectAt(sample,gap/C);};

 std::vector<Sample> lowerBand;for(unsigned k=0;k<waist;k++){const auto& v=mesh.vertices[k];lowerBand.push_back({v.position,band.measured.front()[k].normal,v.lineage});}Point waistCenter{};for(auto s:lowerBand)waistCenter=Add(waistCenter,frame.Local(s.position));waistCenter=Mul(waistCenter,1./lowerBand.size());
 std::vector<Sample> top;for(unsigned col=0;col<=columns;col++)top.push_back(WaistAngle(lowerBand,waistCenter,frame,(2.*col/columns-1)*parameters_.panelHalfAngle));
 auto coverageGrowth=drape::MeasureCoverageGrowth(top,input.anatomy,frame,parameters_.sideCoverageExtra);output_.layout.sideCoverageAspect=coverageGrowth.aspect;output_.layout.sideCoverageEffective=coverageGrowth.effectiveExtra;
 double centers[2]{};for(unsigned side=0;side<2;side++){for(auto index:input.anatomyRegions[2+side])centers[side]+=frame.Local(input.anatomy[index].position)[0];centers[side]/=input.anatomyRegions[2+side].size();}unsigned negative=centers[0]<centers[1]?2:3,positive=5-negative;
 std::array<Sample,2> bottom;
 for(unsigned side=0;side<2;side++){auto direction=Add(Add(Mul(frame.forward,.15),Mul(frame.up,-1)),Mul(frame.lateral,side?.2:-.2));bottom[side]=drape::Support(input,side?positive:negative,0,direction);bottom[side].position=Add(bottom[side].position,Mul(Unit(bottom[side].normal),gap));supportProject(bottom[side]);}
 auto contact=[&](Point a,Point b)->std::optional<Sample>{
  std::array<Point,3> edge{normalize(a),normalize(b),normalize(b)};auto body=bodyCollider_.ClosestFace(edge,margin),anatomy=anatomyCollider_.ClosestFace(edge,margin);bool anatomical=anatomy.distance<body.distance;auto hit=anatomical?anatomy:body;if(hit.distance>=margin)return {};
  const auto& faces=anatomical?input.anatomyTriangles:input.bodyTriangles;const auto& surface=anatomical?input.anatomy:input.bodySurface;auto triangle=faces.at(hit.triangle);auto x=surface[triangle[0]],y=surface[triangle[1]],z=surface[triangle[2]];auto weights=drape::Barycentric(world(hit.point),x.position,y.position,z.position);for(auto& w:weights)w=std::clamp(w,0.,1.);double sum=weights[0]+weights[1]+weights[2];weights=Mul(weights,1/(std::max)(sum,1e-20));
  // A normal offset can coincide with an existing endpoint when the
  // incoming chord crosses the face almost normally. Advance perpendicular
  // to the material chord as well as outside the measured support plane.
  auto tangent=Unit(Sub(b,a)),direction=Sub(hit.normal,Mul(tangent,Dot(hit.normal,tangent)));double perpendicular=Length(direction);
  if(perpendicular>1e-6)direction=Mul(direction,1/perpendicular);else{direction=Sub(frame.up,Mul(tangent,Dot(frame.up,tangent)));if(Length(direction)<1e-6)direction=Sub(frame.lateral,Mul(tangent,Dot(frame.lateral,tangent)));direction=Unit(direction);}
  double normalComponent=(std::max)(.1,Dot(direction,hit.normal));
  Sample sample;sample.position=Add(world(hit.point),Mul(direction,gap/normalComponent));sample.normal=hit.normal;double first=weights[0]+weights[1];sample.lineage=first>1e-14?Blend(x.lineage,y.lineage,weights[1]/first):z.lineage;if(weights[2])sample.lineage=Blend(sample.lineage,z.lineage,weights[2]);
  // Escape the complete measured union in the plane perpendicular to this
  // chord. An unrestricted nearest projection can return almost to its old
  // endpoint and insert hundreds of ineffective detours at thigh/lobe overlap.
  const double sideA=Dot(normalize(a),frame.lateral)-lateralCenter,sideB=Dot(normalize(b),frame.lateral)-lateralCenter,side=sideA*sideB>0?(sideA>0?1.:-1.):0.;
  auto clearSupport=[&](Point p){p=normalize(p);if(side*(Dot(p,frame.lateral)-lateralCenter)<-1e-12)return false;for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto h=collider->Closest(p);if(ClassifiedDistance(h,p,collider==&anatomyCollider_)<gap/C+1e-5)return false;}return true;};
  if(!clearSupport(sample.position)){
   auto base=world(hit.point);double best=1e100;Point escaped{};
   for(auto candidate:{direction,frame.forward,Mul(frame.forward,-1),frame.up,Mul(frame.up,-1),frame.lateral,Mul(frame.lateral,-1)}){candidate=Sub(candidate,Mul(tangent,Dot(candidate,tangent)));if(Length(candidate)<1e-8)continue;candidate=Unit(candidate);double low=0,high=gap;for(unsigned bracket=0;bracket<20&&high<best&&!clearSupport(Add(base,Mul(candidate,high)));bracket++)high*=2;if(high>=best||!clearSupport(Add(base,Mul(candidate,high))))continue;for(unsigned refine=0;refine<24;refine++){double middle=(low+high)*.5;if(clearSupport(Add(base,Mul(candidate,middle))))high=middle;else low=middle;}if(high<best){best=high;escaped=Add(base,Mul(candidate,high+C*2e-5));}}
   if(best==1e100||!clearSupport(escaped))throw std::invalid_argument("Material chord cannot escape the measured union");sample.position=escaped;
  }
  return sample;
 };
 // Sewn garment samples are boundary conditions, not new tissue pressure
 // points. The envelope contains original measured tissue samples only.
 auto hullPoints=input.anatomy;
 // Retain the actual body pressure samples underlying the broad sewn front
 // boundary, including every supplied alias. Derived garment arc samples do
 // not become pressure vertices. The entire native body stays in collision.
 std::set<std::pair<Surface,unsigned>> stitchedSources;for(auto sample:top)for(auto donor:sample.lineage.donors)if(donor.weight>0)stitchedSources.insert({donor.surface,donor.vertex});
 for(auto sample:input.bodySurface){bool stitched=false;for(auto donor:sample.lineage.donors)if(donor.weight>0&&stitchedSources.count({donor.surface,donor.vertex})){stitched=true;break;}if(stitched)hullPoints.push_back(sample);}
 auto hull=drape::TensionHull(std::move(hullPoints));Input tensionInput=input;tensionInput.anatomy=std::move(hull.samples);tensionInput.anatomyTriangles=std::move(hull.faces);
 drape::HullSurface chartHull{std::move(tensionInput.anatomy),std::move(tensionInput.anatomyTriangles)};
 drape::RadialChart chart(chartHull,frame,gap);auto sheet=chart.Make(top,bottom,rows,coverageGrowth.effectiveExtra);
 // A single ordered chart of one measured exterior envelope bridges hollow
 // regions without independent lane fans or row reparameterization. It is
 // only initialization; moving cloth retains its own measured rest metric.
 const unsigned sheetStart=unsigned(mesh.vertices.size());output_.layout.sheet={sheetStart,rows,columns};output_.layout.sheetFaces={unsigned(mesh.triangles.size()),rows*columns*2};
 for(unsigned row=0;row<=rows;row++)for(unsigned col=0;col<=columns;col++){auto s=sheet.vertices[row*(columns+1)+col];VertexAt(mesh,s.position,double(col)/columns,double(row)/rows,s.lineage);if(!row)output_.layout.topSeam.push_back(unsigned(mesh.vertices.size()-1));}
 // One consistently wound, connected webbed sheet. There is no pole, radial
 // ring, annular neck connector, or separate upper suspension panel.
 for(auto face:cloth_detail::SheetTriangles(sheetStart,rows,columns))mesh.triangles.push_back({face,MaterialSlot::WhiteRibbed});
 const unsigned bottomRow=sheetStart+rows*(columns+1);double bottomLength=Length(Sub(mesh.vertices[bottomRow+columns].position,mesh.vertices[bottomRow].position));unsigned seamColumns=(std::max)(2u,(std::min)(columns/2,unsigned(std::ceil(parameters_.strapWidth*C/(std::max)(bottomLength/columns,1e-12)))));
 for(unsigned side=0;side<2;side++)for(unsigned k=0;k<=seamColumns;k++)output_.layout.bottomSeams[side].push_back(bottomRow+(side?columns-seamColumns+k:k));
 std::array<unsigned,2> strapSide{};
 for(unsigned route=0;route<2;route++){
  double side=frame.Local(input.rearStraps[route].front().position)[0];unsigned end=side<waistCenter[0]?0:1;strapSide[route]=end;const auto& seam=output_.layout.bottomSeams[end];auto path=input.rearStraps[route];const auto& a=mesh.vertices[seam.front()];const auto& b=mesh.vertices[seam.back()];Sample sewn{Mul(Add(a.position,b.position),.5),bottom[end].normal,Blend(a.lineage,b.lineage,.5)};path.push_back(sewn);
  path=drape::SmoothRoute(std::move(path),24,project);
  unsigned count=(std::max)(33u,unsigned(input.rearStraps[route].size())*4+1);std::vector<Sample> refined;for(unsigned k=0;k<count;k++){auto s=drape::Resample(path,double(k)/(count-1));project(s);refined.push_back(s);}refined=drape::SmoothRoute(std::move(refined),12,project);unsigned start=unsigned(mesh.vertices.size());Tube(mesh,refined,parameters_.strapWidth*C,thick*.5,false,frame,MaterialSlot::WhiteElastic);output_.layout.straps[route]={start,count,4};
 }
 // Two continuous elastic side hems share the walked sheet's actual edge.
 // They are not additional skin pins, nor a closed top/bottom trim.
 Shading(mesh);
 for(unsigned side=0;side<2;side++){
  const double hemThickness=parameters_.hemThickness*C;
  std::vector<Sample> edge;for(unsigned row=0;row<=rows;row++){unsigned id=sheetStart+row*(columns+1)+(side?columns:0);output_.layout.sideBoundary[side].push_back(id);const auto& v=mesh.vertices[id];edge.push_back({Add(v.position,Mul(v.normal,hemThickness*.6)),v.normal,v.lineage});}
  unsigned start=unsigned(mesh.vertices.size());Tube(mesh,edge,parameters_.hemWidth*C,hemThickness,false,frame,MaterialSlot::WhiteElastic);output_.layout.sideHems[side]={start,rows+1,4};
  for(unsigned endpoint:{0u,rows}){unsigned first=start+endpoint*4;AuthoredJoint joint;joint.name="hem-"+std::to_string(side)+(endpoint?"-bottom":"-top");joint.a={{first,first+1,first+2,first+3},{.25,.25,.25,.25}};joint.b={{output_.layout.sideBoundary[side][endpoint]},{1}};joint.restOffset=Mul(edge[endpoint].normal,hemThickness*.6);output_.layout.authoredJoints.push_back(std::move(joint));}
 }
 for(unsigned route=0;route<2;route++){auto ribbon=output_.layout.straps[route];unsigned cap=ribbon.start+(ribbon.sections-1)*4;const auto& seam=output_.layout.bottomSeams[strapSide[route]];AuthoredJoint joint;joint.name="strap-"+std::to_string(route)+"-bottom";joint.a={{cap,cap+1,cap+2,cap+3},{.25,.25,.25,.25}};joint.b={{seam.front(),seam.back()},{.5,.5}};output_.layout.authoredJoints.push_back(std::move(joint));}
 // Solve actual material coordinates, never stale copies of sewn aliases.
 // Ribbon corners share one translation, preserving their thin cross section.
 auto hemNode=[&](unsigned id)->unsigned{for(unsigned side=0;side<2;side++){auto ribbon=output_.layout.sideHems[side];if(id>=ribbon.start&&id<ribbon.start+ribbon.sections*4)return output_.layout.sideBoundary[side][(id-ribbon.start)/4];}return id;};
 auto group=[&](unsigned id){id=hemNode(id);for(const auto& ribbon:output_.layout.straps)if(id>=ribbon.start&&id<ribbon.start+ribbon.sections*4)return ribbon.start+(id-ribbon.start)/4*4;return id;};
 std::vector<std::vector<unsigned>> sheetIncident((rows+1)*(columns+1));
 for(unsigned f=output_.layout.sheetFaces.start;f<output_.layout.sheetFaces.start+output_.layout.sheetFaces.count;f++)for(auto id:mesh.triangles[f].vertices)sheetIncident.at(id-sheetStart).push_back(f);
 std::vector<double> sheetAngularArea(output_.layout.sheetFaces.count);
 for(unsigned f=0;f<sheetAngularArea.size();f++){auto ids=mesh.triangles[output_.layout.sheetFaces.start+f].vertices;std::array<Point,3> directions{};for(unsigned k=0;k<3;k++)directions[k]=Unit(Sub(mesh.vertices[ids[k]].position,chart.center));sheetAngularArea[f]=Dot(directions[0],Cross(directions[1],directions[2]));if(sheetAngularArea[f]<=0)throw std::invalid_argument("Rest material chart begins with a reversed spherical triangle");}
 auto moveSheetVertex=[&](unsigned id,Point delta){
  if(id>=sheetStart&&id<sheetStart+(rows+1)*(columns+1)&&Length(delta)>1e-14){
   // Rest contact must preserve the injective angular material chart. A
   // positive radial scaling cannot reverse its spherical triangles. This is
   // only initial fitting: physical cloth remains free in all three axes.
   auto radial=Unit(Sub(mesh.vertices[id].position,chart.center));unsigned at=id-sheetStart,row=at/(columns+1),col=at%(columns+1);bool freeEdge=row>0&&row<rows&&(col==0||col==columns),accepted=false;
   if(freeEdge){
    // A soft side guide cannot forbid the tangential body-contact direction.
    // Permit full spatial hem motion, retaining a positive, conditioned
    // spherical chart through a local line search. Sewn endpoints stay out of
    // this freedom; every finite material face is still checked afterward.
    Point trial=delta;
    for(unsigned step=0;step<24;step++){
     bool valid=true;
     for(auto fi:sheetIncident[at]){
      auto face=mesh.triangles[fi].vertices;std::array<Point,3> newDirections{};
      for(unsigned k=0;k<3;k++){auto p=mesh.vertices[face[k]].position;newDirections[k]=Unit(Sub(face[k]==id?Add(p,trial):p,chart.center));}
      double newArea=Dot(newDirections[0],Cross(newDirections[1],newDirections[2])),longest=0;
      for(unsigned k=0;k<3;k++){auto e=Sub(newDirections[k],newDirections[(k+1)%3]);longest=(std::max)(longest,Dot(e,e));}
      // The floor is tied to the original chart, never repeatedly halved
      // against the last accepted step. Repeated small moves cannot collapse
      // a material triangle while individually appearing locally positive.
      double minimumArea=(std::max)(1e-5/(rows*columns),(std::min)(sheetAngularArea[fi-output_.layout.sheetFaces.start]*.5,.015*longest));
      if(newArea<minimumArea){valid=false;break;}
     }
     if(valid){delta=trial;accepted=true;break;}trial=Mul(trial,.5);
    }
   }
   if(!accepted){double advance=Dot(radial,delta),square=Dot(delta,delta);
    if(std::abs(advance)>Length(delta)*1e-6){double scalar=square/advance;scalar=std::clamp(scalar,-C*.02,C*.02);double radius=Length(Sub(mesh.vertices[id].position,chart.center));if(radius+scalar<=C*1e-8)throw std::invalid_argument("Rest contact would collapse its material chart radius");delta=Mul(radial,scalar);}
    else delta=Point{};
   }
  }
  mesh.vertices[id].position=Add(mesh.vertices[id].position,delta);
  for(unsigned side=0;side<2;side++)for(unsigned section=0;section<output_.layout.sideBoundary[side].size();section++)if(output_.layout.sideBoundary[side][section]==id){auto first=output_.layout.sideHems[side].start+section*4;for(unsigned k=0;k<4;k++)mesh.vertices[first+k].position=Add(mesh.vertices[first+k].position,delta);}
  for(unsigned route=0;route<2;route++){const auto& seam=output_.layout.bottomSeams[strapSide[route]];if(id!=seam.front()&&id!=seam.back())continue;auto ribbon=output_.layout.straps[route];unsigned cap=ribbon.start+(ribbon.sections-1)*4;for(unsigned k=0;k<4;k++)mesh.vertices[cap+k].position=Add(mesh.vertices[cap+k].position,Mul(delta,.5));}
 };
 auto moveVertex=[&](unsigned id,Point delta){
  id=hemNode(id);for(unsigned route=0;route<2;route++){const auto& ribbon=output_.layout.straps[route];if(id>=ribbon.start&&id<ribbon.start+ribbon.sections*4){unsigned first=ribbon.start+(id-ribbon.start)/4*4;if(first==ribbon.start+(ribbon.sections-1)*4){const auto& seam=output_.layout.bottomSeams[strapSide[route]];moveSheetVertex(seam.front(),delta);moveSheetVertex(seam.back(),delta);}else for(unsigned k=0;k<4;k++)mesh.vertices[first+k].position=Add(mesh.vertices[first+k].position,delta);return;}}
  moveSheetVertex(id,delta);
 };
 // Only skip a repeated point projection while its closed measured tissue
 // separation, minus the point's travel, still proves the complete margin.
 // The face pass and final exact clearance checks remain unchanged.
 std::array<std::vector<proximity::PointCertificate>,2> restPointClearance;
 for(auto& memo:restPointClearance)memo.resize(mesh.vertices.size());
 for(unsigned sweep=0;sweep<96;sweep++){
  if(sweep<84){
   // Tension rounds rest corrugations instead of following every local hull
   // crease. A signed bending descent may relax an outward pressure peak or
   // bridge an inward recess. The complete unilateral point/face contact pass
   // below still rejects penetration; there is no attraction to tissue.
   std::vector<Point> previous;previous.reserve((rows+1)*(columns+1));for(unsigned k=0;k<(rows+1)*(columns+1);k++)previous.push_back(mesh.vertices[sheetStart+k].position);
   for(unsigned row=1;row<rows;row++)for(unsigned col=0;col<=columns;col++){
    unsigned at=row*(columns+1)+col;auto radial=Unit(Sub(previous[at],chart.center));Point average=Add(previous[at-columns-1],previous[at+columns+1]);unsigned count=2;
    if(col>0&&col<columns){average=Add(average,Add(previous[at-1],previous[at+1]));count=4;}
    double difference=Dot(radial,Sub(Mul(average,1./count),previous[at]));moveSheetVertex(sheetStart+at,Mul(radial,std::clamp(difference*.25,-C*.002,C*.002)));
   }
  }
  if(sweep<84){for(const auto& ribbon:output_.layout.straps)drape::BendRibbonGroups(mesh,ribbon,moveVertex);for(const auto& ribbon:output_.layout.sideHems)drape::BendRibbonGroups(mesh,ribbon,moveVertex);}
  bool changed=false;
  for(unsigned id=bandCount;id<mesh.vertices.size();id++){
   auto point=normalize(mesh.vertices[id].position);
   auto& bodyClass=closedBody_.Empty()?bodyCollider_:closedBody_;
   auto& anatomyClass=closedAnatomy_.Empty()?anatomyCollider_:closedAnatomy_;
   if(restPointClearance[0][id].ProvesClear({point},margin+1e-5,bodyClass.MotionStamp())&&restPointClearance[1][id].ProvesClear({point},margin+1e-5,anatomyClass.MotionStamp()))continue;
   Sample sample{mesh.vertices[id].position,mesh.vertices[id].normal,mesh.vertices[id].lineage};project(sample);auto delta=Sub(sample.position,mesh.vertices[id].position);if(Length(delta)>1e-12){moveVertex(id,delta);changed=true;}
   point=normalize(mesh.vertices[id].position);
   for(auto* classifier:{&bodyClass,&anatomyClass}){
    auto hit=classifier->Closest(point);const bool outside=classifier->Classify(point,true)==BodyCollider::Side::Outside;
    restPointClearance[classifier==&anatomyClass][id].Remember({point},hit.distance,outside,classifier->MotionStamp());
   }
  }
  for(unsigned fi=bandFaces;fi<mesh.triangles.size();fi++)for(auto* collider:{&bodyCollider_,&anatomyCollider_}){
   auto face=mesh.triangles[fi];std::array<Point,3> pts;for(unsigned k=0;k<3;k++)pts[k]=normalize(mesh.vertices[face.vertices[k]].position);auto hit=collider->ClosestFace(pts,margin+3e-5);if(hit.distance>=margin+2e-5)continue;
   auto separation=Sub(hit.clothPoint,hit.point);Point direction=Length(separation)>1e-12?Unit(separation):hit.normal;if(Dot(direction,hit.normal)<0)direction=hit.normal;auto delta=Mul(direction,(margin-hit.distance+4e-5)*C);
   std::array<unsigned,3> moved{UINT32_MAX,UINT32_MAX,UINT32_MAX};for(unsigned k=0;k<3;k++){unsigned id=face.vertices[k],g=group(id);bool seen=false;for(unsigned previous=0;previous<k;previous++)seen=seen||moved[previous]==g;moved[k]=g;if(!seen)moveVertex(id,delta);}changed=true;
  }
  if(!changed&&sweep>=84)break;
 }
 // Initial geometry is still required to satisfy complete physical face
 // clearance. A later dynamic step cannot legitimise invalid rest material.
 output_.coverageMargin=1e100;
#ifdef MALEMOD_GARMENT_DIAGNOSTIC
 for(unsigned i=0;i<mesh.vertices.size();i++){auto p=normalize(mesh.vertices[i].position);for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->Closest(p);double distance=ClassifiedDistance(hit,p,collider==&anatomyCollider_);if(distance<margin-1e-8)std::fprintf(stderr,"REST point %u band %d anatomy %d nativeface %u clearance %.9g required %.9g\n",i,int(i<bandCount),int(collider==&anatomyCollider_),hit.triangle,distance*C,margin*C);}}
 for(unsigned i=0;i<mesh.triangles.size();i++){const auto& face=mesh.triangles[i];std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalize(mesh.vertices[face.vertices[k]].position);for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->ClosestFace(points,margin);if(hit.distance<margin-1e-8)std::fprintf(stderr,"REST face %u band %d anatomy %d nativeface %u clearance %.9g required %.9g\n",i,int(i<bandFaces),int(collider==&anatomyCollider_),hit.triangle,hit.distance*C,margin*C);}}
#endif
 for(const auto& vertex:mesh.vertices){auto p=normalize(vertex.position);for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->Closest(p);double distance=ClassifiedDistance(hit,p,collider==&anatomyCollider_);if(distance<margin-1e-8)output_.contactBudgetSatisfied=false;if(collider==&anatomyCollider_)output_.coverageMargin=(std::min)(output_.coverageMargin,(distance-margin)*C);}}
 for(const auto& face:mesh.triangles){std::array<Point,3> points;for(unsigned k=0;k<3;k++)points[k]=normalize(mesh.vertices[face.vertices[k]].position);for(auto* collider:{&bodyCollider_,&anatomyCollider_}){auto hit=collider->ClosestFace(points,margin);if(hit.distance<margin-1e-8)output_.contactBudgetSatisfied=false;if(collider==&anatomyCollider_)output_.coverageMargin=(std::min)(output_.coverageMargin,(hit.distance-margin)*C);}}
 output_.band.minimumInnerClearance=1e100;output_.band.maximumInnerClearance=-1e100;for(unsigned i=bandCount/2;i<bandCount;i++){auto p=normalize(mesh.vertices[i].position);double d=ClassifiedDistance(bodyCollider_.Closest(p),p,false)*C;output_.band.minimumInnerClearance=(std::min)(output_.band.minimumInnerClearance,d);output_.band.maximumInnerClearance=(std::max)(output_.band.maximumInnerClearance,d);}
 Shading(mesh);return output_;
}
}

