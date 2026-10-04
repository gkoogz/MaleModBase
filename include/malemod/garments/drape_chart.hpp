#pragma once
// One rest-only angular chart of the measured convex tension envelope. A
// common chart prevents independently fanning section lanes from crossing.
// Native tissue remains the sole dynamic collision surface.
namespace malemod::garments::drape {
struct CoverageGrowth {double requestedMaximum=0,reach=0,frontArcWidth=0,aspect=0,effectiveExtra=0;};
inline CoverageGrowth MeasureCoverageGrowth(const std::vector<Sample>& top,const std::vector<Sample>& anatomy,const Frame& frame,double requestedMaximum){
 frame.Validate();
 if(top.size()<2||anatomy.empty()||!std::isfinite(requestedMaximum)||requestedMaximum<0||requestedMaximum>1.5)throw std::invalid_argument("Invalid measured coverage growth inputs");
 // Actual sewn arc width supplies the pattern unit. Reach includes every
 // retained native anatomical point, not a gameplay size or selected port.
 // Accumulate relative to an actual seam sample to avoid translating the
 // summation by a large game-world origin.
 auto origin=top.front().position;Point mean{};
 for(const auto& sample:top){if(!Finite(sample.position))throw std::invalid_argument("Nonfinite measured coverage seam");mean=Add(mean,Sub(sample.position,origin));}
 mean=Mul(mean,1./top.size());CoverageGrowth result;result.requestedMaximum=requestedMaximum;
 result.frontArcWidth=std::abs(Dot(Sub(top.back().position,origin),frame.lateral));
 if(!std::isfinite(result.frontArcWidth)||!(result.frontArcWidth>0))throw std::invalid_argument("Measured coverage seam has no transverse width");
 const double meanForward=Dot(mean,frame.forward);
 for(const auto& sample:anatomy){if(!Finite(sample.position))throw std::invalid_argument("Nonfinite measured coverage anatomy");result.reach=(std::max)(result.reach,Dot(Sub(sample.position,origin),frame.forward)-meanForward);}
 result.aspect=result.reach/result.frontArcWidth;if(!std::isfinite(result.aspect))throw std::invalid_argument("Nonfinite measured coverage aspect");
 // Quarter-width transition belongs to the common sewing pattern. Compact
 // panels keep their existing underside approach; longer projections gain
 // smooth lower-middle fullness, without modifying either sewn endpoint.
 double t=std::clamp((result.aspect-1.25)/.25,0.,1.);result.effectiveExtra=requestedMaximum*t*t*(3-2*t);
 return result;
}
struct RadialChart {
 const HullSurface& hull;const Frame& frame;Point center{};double gap=0;
 struct Plane {Point normal;double height;std::array<unsigned,3> ids;};
 std::vector<Plane> planes;
 RadialChart(const HullSurface& surface,const Frame& basis,double clearance):hull(surface),frame(basis),gap(clearance){
  frame.Validate();if(hull.faces.empty()||!std::isfinite(gap)||gap<=0)throw std::invalid_argument("Invalid measured tension chart");
  std::set<unsigned> used;for(auto f:hull.faces)for(auto i:f)used.insert(i);for(auto i:used)center=Add(center,hull.samples.at(i).position);center=Mul(center,1./used.size());
  for(auto f:hull.faces){auto a=hull.samples[f[0]].position,b=hull.samples[f[1]].position,c=hull.samples[f[2]].position;auto n=Unit(Cross(Sub(b,a),Sub(c,a)));double h=Dot(n,Sub(a,center));if(h<=0)throw std::invalid_argument("Tension chart origin must lie inside its measured hull");planes.push_back({n,h,f});}
 }
 Point Coordinates(Point position)const{return frame.Local(Add(frame.origin,Sub(position,center)));}
 Sample Cast(Point direction)const{
  direction=Unit(direction);double radius=1e100;const Plane* face=nullptr;
  for(const auto& plane:planes){double denominator=Dot(plane.normal,direction);if(denominator<=1e-14)continue;double candidate=(plane.height+gap)/denominator;if(candidate<radius){radius=candidate;face=&plane;}}
  if(!face||!std::isfinite(radius))throw std::invalid_argument("Unbounded measured tension chart ray");
  auto p=Add(center,Mul(direction,radius));const auto& a=hull.samples[face->ids[0]];const auto& b=hull.samples[face->ids[1]];const auto& c=hull.samples[face->ids[2]];
  auto w=Barycentric(Sub(p,Mul(face->normal,gap)),a.position,b.position,c.position);for(auto& weight:w)weight=std::clamp(weight,0.,1.);double total=w[0]+w[1]+w[2];w=Mul(w,1/total);double first=w[0]+w[1];Lineage lineage;
  try{lineage=first>1e-14?detail::Blend(a.lineage,b.lineage,w[1]/first):c.lineage;if(w[2])lineage=detail::Blend(lineage,c.lineage,w[2]);}catch(const std::invalid_argument& e){auto count=[](const Lineage& l){return std::count_if(l.donors.begin(),l.donors.end(),[](auto d){return d.weight>0;});};throw std::invalid_argument(std::string(e.what())+" hull supports "+std::to_string(face->ids[0])+","+std::to_string(face->ids[1])+","+std::to_string(face->ids[2])+" donor counts "+std::to_string(count(a.lineage))+","+std::to_string(count(b.lineage))+","+std::to_string(count(c.lineage)));}
  return {p,face->normal,lineage};
 }
 static Point Arc(Point a,Point b,double fraction){
  a=Unit(a);b=Unit(b);double cosine=std::clamp(Dot(a,b),-1.,1.);
  if(cosine>1-1e-10)return Unit(Add(Mul(a,1-fraction),Mul(b,fraction)));
  if(cosine<-1+1e-10)throw std::invalid_argument("Measured material boundaries have antipodal directions");
  double angle=std::acos(cosine),denominator=std::sin(angle);
  return Add(Mul(a,std::sin((1-fraction)*angle)/denominator),Mul(b,std::sin(fraction*angle)/denominator));
 }
 // Both hems share ONE row parameter. Independently equalizing the sides
 // would shear the connected sheet. Measure the mean of their real spatial
 // lengths instead, including the source collar, then invert that monotone
 // cumulative measure. The dense paths are rest construction, never motion
 // regeneration or replacement collision geometry.
 static std::vector<double> CommonRows(const std::vector<std::array<Point,2>>& paths,unsigned rows){
  if(paths.size()<2||!rows)throw std::invalid_argument("Missing measured material side paths");
  std::vector<double> cumulative(paths.size());
  for(unsigned i=1;i<paths.size();i++){
   double length=.5*(Length(Sub(paths[i][0],paths[i-1][0]))+Length(Sub(paths[i][1],paths[i-1][1])));
   if(!std::isfinite(length))throw std::invalid_argument("Nonfinite measured material side path");
   cumulative[i]=cumulative[i-1]+length;
  }
  if(!(cumulative.back()>0))throw std::invalid_argument("Collapsed measured material side paths");
  std::vector<double> parameters(rows+1);parameters.back()=1;unsigned segment=1;
  for(unsigned row=1;row<rows;row++){
   double target=cumulative.back()*double(row)/rows;
   while(segment+1<cumulative.size()&&cumulative[segment]<target)segment++;
   double length=cumulative[segment]-cumulative[segment-1];
   double fraction=length>0?(target-cumulative[segment-1])/length:0;
   parameters[row]=(double(segment-1)+fraction)/double(paths.size()-1);
  }
  return parameters;
 }
 Sheet Make(const std::vector<Sample>& top,const std::array<Sample,2>& bottom,unsigned rows,double sideCoverageExtra=0)const{
  if(top.size()<9||top.size()>129||rows<8||rows>64)throw std::invalid_argument("Measured chart grid outside bounded contract");
  if(!std::isfinite(sideCoverageExtra)||sideCoverageExtra<0||sideCoverageExtra>1.5)throw std::invalid_argument("Measured side coverage extra outside finite bounded contract");
  Sheet result;result.rows=rows;result.columns=unsigned(top.size()-1);const double pi=3.14159265358979323846;
  auto centerTop=Unit(Coordinates(top[result.columns/2].position)),centerBottom=Unit(Coordinates(Mix(bottom[0],bottom[1],.5).position));
  auto front=Unit(Mul(Add(centerTop,centerBottom),-1));
  auto lateral=Unit(Sub(Point{1,0,0},Mul(front,front[0]))),vertical=Unit(Cross(lateral,front));
  // Stereographic material coordinates have no lateral pole. Unlike a capped
  // latitude strip they can cover the lateral lobes and still return both
  // hems to the measured root. The projection pole stays inside the open rear.
  auto project=[&](Point direction){auto d=Unit(direction);double denominator=1+Dot(d,front);if(denominator<=1e-10)throw std::invalid_argument("Measured seam touches the material projection pole");return Point{Dot(d,lateral)/denominator,Dot(d,vertical)/denominator,0};};
  auto unproject=[&](Point q){double square=Dot(q,q),denominator=1+square;return Mul(Add(Add(Mul(lateral,2*q[0]),Mul(vertical,2*q[1])),Mul(front,1-square)),1/denominator);};
  const unsigned columns=result.columns,stride=columns+1;std::vector<Point> material((rows+1)*stride);
  std::array<Point,2> topSides{Coordinates(top.front().position),Coordinates(top.back().position)},bottomSides{Coordinates(bottom[0].position),Coordinates(bottom[1].position)};
  for(unsigned col=0;col<=columns;col++){material[col]=project(Coordinates(top[col].position));material[rows*stride+col]=project(Coordinates(Mix(bottom[0],bottom[1],double(col)/columns).position));}
  std::array<double,2> sideBow{};
  for(unsigned side=0;side<2;side++)for(unsigned step=1;step<64;step++){
   double t=double(step)/64;auto q=project(Arc(topSides[side],bottomSides[side],t));auto line=Add(Mul(material[side?columns:0],1-t),Mul(material[rows*stride+(side?columns:0)],t));
   sideBow[side]=(std::max)(sideBow[side],(q[0]-line[0])*(side?1:-1));
  }
  auto sideGuide=[&](unsigned side,double t){unsigned col=side?columns:0;auto q=Add(Mul(material[col],1-t),Mul(material[rows*stride+col],t));
   // Measured lower-middle fullness retains the baseline endpoint tangents.
   // The extra sin-squared term has zero value AND derivative at both sewn
   // ends, so it cannot steepen the narrow underside attachment strip.
   // Actual outward route bow supplies its scale, never a character size.
   // Preserve the literal reference operation order. Iterative chart fitting
   // can amplify a regrouped multiply even when extra coverage equals zero.
   double lower=t*t*(3-2*t);q[0]+=(side?1:-1)*sideBow[side]*(.965+.035*lower)*std::sin(pi*t);
   if(sideCoverageExtra!=0){double sine=std::sin(pi*t);q[0]+=(side?1:-1)*sideBow[side]*sideCoverageExtra*lower*sine*sine;}
   return q;};
  auto collarBlend=[](double t){double f=(std::min)(1.,t/.15);return f*f*f*(10+f*(-15+6*f));};
  constexpr unsigned guideSteps=512;std::vector<std::array<Point,2>> measuredGuides(guideSteps+1);
  for(unsigned step=0;step<=guideSteps;step++)for(unsigned side=0;side<2;side++){
   double t=double(step)/guideSteps;unsigned col=side?columns:0;
   if(!step){measuredGuides[step][side]=top[col].position;continue;}
   auto local=unproject(sideGuide(side,t));auto direction=Add(Mul(frame.lateral,local[0]),Add(Mul(frame.forward,local[1]),Mul(frame.up,local[2])));
   auto sample=Cast(direction);double start=Length(Sub(top[col].position,center)),radius=Length(Sub(sample.position,center));
   measuredGuides[step][side]=Add(center,Mul(direction,start+(radius-start)*collarBlend(t)));
  }
  auto rowParameters=CommonRows(measuredGuides,rows);
  for(unsigned row=1;row<rows;row++){
   double t=rowParameters[row];
   // A monotone longitudinal hem avoids the upper reversal of a great-circle
   // arc near its projection pole. Its lateral bow is measured from that
   // source route; slightly reducing the bow leaves a small side opening.
   for(unsigned side=0;side<2;side++){unsigned col=side?columns:0;material[row*stride+col]=sideGuide(side,t);}
   for(unsigned col=1;col<columns;col++){double u=double(col)/columns;material[row*stride+col]=Add(Mul(material[row*stride],1-u),Mul(material[row*stride+columns],u));}
  }
  // Positive-weight harmonic interpolation of this single boundary replaces
  // independent lane fans. It is REST construction only; subsequent cloth
  // motion retains its own material lengths, inertia and physical contacts.
  double scale=0;for(auto q:material)scale=(std::max)(scale,Length(q));bool converged=false;
  for(unsigned pass=0;pass<4000;pass++){
   double change=0;
   for(unsigned row=1;row<rows;row++)for(unsigned col=1;col<columns;col++){unsigned at=row*stride+col;auto average=Mul(Add(Add(material[at-1],material[at+1]),Add(material[at-stride],material[at+stride])),.25);auto delta=Mul(Sub(average,material[at]),1.85);material[at]=Add(material[at],delta);change=(std::max)(change,Length(delta));}
   if(change<scale*1e-10){converged=true;break;}
  }
  if(!converged)throw std::invalid_argument("Measured material chart did not converge");
  // A naturally seated top arc is not necessarily convex in this chart.
  // Positive harmonic weights alone do not guarantee an injective map of a
  // nonconvex boundary. Restore positive triangle areas with fixed measured
  // seam/hem coordinates before casting any fabric onto the pressure surface.
  const double areaFloor=scale*scale*1e-10/(rows*columns);
  const auto materialFaces=cloth_detail::SheetTriangles(0,rows,columns);
  // Only the broad top and the two lower suspension seams are prescribed.
  // The elastic side hems are free material, not extra tissue attachments.
  auto free=[&](unsigned at){return at/stride>0&&at/stride<rows;};
  bool unfolded=false;
  for(unsigned pass=0;pass<4096;pass++){
   unsigned violations=0;
   auto projectArea=[&](std::array<unsigned,3> ids){
    auto a=material[ids[0]],b=material[ids[1]],c=material[ids[2]];double area=-Cross(Sub(b,a),Sub(c,a))[2];if(area>=areaFloor)return;
    std::array<Point,3> gradients{Point{c[1]-b[1],b[0]-c[0],0},Point{a[1]-c[1],c[0]-a[0],0},Point{b[1]-a[1],a[0]-b[0],0}};
    double square=0;for(unsigned k=0;k<3;k++)if(free(ids[k]))square+=Dot(gradients[k],gradients[k]);if(square<=1e-30)throw std::invalid_argument("Measured boundary contains a fixed inverted material triangle");
    double multiplier=(areaFloor*16-area)/square*1.05;for(unsigned k=0;k<3;k++)if(free(ids[k]))material[ids[k]]=Add(material[ids[k]],Mul(gradients[k],multiplier));violations++;
   };
   for(auto face:materialFaces)projectArea(face);
   if(!violations){unfolded=true;break;}
  }
  if(!unfolded){unsigned worst=0,violations=0;double minimum=1e100;
   for(unsigned i=0;i<materialFaces.size();i++){auto f=materialFaces[i];double area=-Cross(Sub(material[f[1]],material[f[0]]),Sub(material[f[2]],material[f[0]]))[2];if(area<areaFloor)violations++;if(area<minimum){minimum=area;worst=i;}}
   auto f=materialFaces[worst];throw std::invalid_argument("Measured nonconvex material boundary could not unfold: "+std::to_string(violations)+" material faces, worst "+std::to_string(f[0])+","+std::to_string(f[1])+","+std::to_string(f[2]));
  }
  // A positive planar area can still reverse a triangle after a curved
  // projection when its corners nearly line up. For radial vertices the
  // exterior chord orientation is exactly three positive radii times this
  // spherical determinant. Enforce that quantity before creating the sheet.
  const double sphereFloor=1e-5/(rows*columns);
  auto sphereGradient=[&](Point q,Point g){auto d=unproject(q);double denominator=1+Dot(q,q);auto shared=Add(front,d);return Point{Dot(g,Mul(Sub(lateral,Mul(shared,q[0])),2/denominator)),Dot(g,Mul(Sub(vertical,Mul(shared,q[1])),2/denominator)),0};};
  bool spherical=false;
  for(unsigned pass=0;pass<8192;pass++){
   unsigned violations=0;
   for(auto ids:materialFaces){
    std::array<Point,3> directions{unproject(material[ids[0]]),unproject(material[ids[1]]),unproject(material[ids[2]])};auto a=directions[0],b=directions[1],c=directions[2];double determinant=Dot(a,Cross(b,c));
    // A merely positive area permits nearly singular material triangles.
    // Bound angular aspect as well, using the longest actual spherical chord
    // rather than a constant area which would distort narrow sewn boundaries.
    unsigned edge=0;double longest=0;for(unsigned k=0;k<3;k++){auto d=Sub(directions[k],directions[(k+1)%3]);double square=Dot(d,d);if(square>longest){longest=square;edge=k;}}
    constexpr double quality=.015;double qualityFloor=(std::max)(sphereFloor,quality*longest);if(determinant>=qualityFloor)continue;
    std::array<Point,3> worldGradients{Cross(b,c),Cross(c,a),Cross(a,b)};
    if(quality*longest>sphereFloor){auto edgeGradient=Mul(Sub(directions[edge],directions[(edge+1)%3]),2*quality);worldGradients[edge]=Sub(worldGradients[edge],edgeGradient);worldGradients[(edge+1)%3]=Add(worldGradients[(edge+1)%3],edgeGradient);}
    std::array<Point,3> gradients{sphereGradient(material[ids[0]],worldGradients[0]),sphereGradient(material[ids[1]],worldGradients[1]),sphereGradient(material[ids[2]],worldGradients[2])};
    double square=0;for(unsigned k=0;k<3;k++)if(free(ids[k]))square+=Dot(gradients[k],gradients[k]);if(square<=1e-30)throw std::invalid_argument("Measured boundary contains a fixed reversed spherical triangle");
    double multiplier=(qualityFloor*1.5-determinant)/square;double maximum=0;for(unsigned k=0;k<3;k++)if(free(ids[k]))maximum=(std::max)(maximum,Length(gradients[k])*std::abs(multiplier));if(maximum>scale*.01)multiplier*=scale*.01/maximum;
    for(unsigned k=0;k<3;k++)if(free(ids[k]))material[ids[k]]=Add(material[ids[k]],Mul(gradients[k],multiplier));violations++;
   }
   if(!violations){spherical=true;break;}
  }
  if(!spherical)throw std::invalid_argument("Measured spherical material boundary could not unfold");
  // Smooth angular material coordinates without undoing the injective map or
  // its quality bound. Side edges carry a mild guide preference, not a hard
  // pin: fabric tension may round their approach to the actual sewn seams.
  auto guide=material;const unsigned count=unsigned(material.size());
  std::vector<std::vector<unsigned>> neighbours(count),affected(count),incident(count);
  for(unsigned at=0;at<count;at++)if(free(at)){
   unsigned col=at%stride;neighbours[at]={at-stride,at+stride};
   if(col>0&&col<columns){neighbours[at].push_back(at-1);neighbours[at].push_back(at+1);}
   affected[at].push_back(at);
  }
  for(unsigned at=0;at<count;at++)if(free(at))for(auto neighbour:neighbours[at])if(free(neighbour))affected[neighbour].push_back(at);
  for(unsigned f=0;f<materialFaces.size();f++)for(auto at:materialFaces[f])incident[at].push_back(f);
  auto laplacian=[&](unsigned at){Point value=Mul(unproject(material[at]),-double(neighbours[at].size()));for(auto neighbour:neighbours[at])value=Add(value,unproject(material[neighbour]));return value;};
  auto energy=[&](unsigned at){auto value=laplacian(at);double result=Dot(value,value);if(at%stride==0||at%stride==columns){auto difference=Sub(unproject(material[at]),unproject(guide[at]));result+=.2*Dot(difference,difference);}return result;};
  auto valid=[&](unsigned at){for(auto f:incident[at]){auto ids=materialFaces[f];std::array<Point,3> d{unproject(material[ids[0]]),unproject(material[ids[1]]),unproject(material[ids[2]])};double longest=0;for(unsigned k=0;k<3;k++){auto edge=Sub(d[k],d[(k+1)%3]);longest=(std::max)(longest,Dot(edge,edge));}if(Dot(d[0],Cross(d[1],d[2]))<(std::max)(sphereFloor,.015*longest))return false;}return true;};
  for(unsigned pass=0;pass<96;pass++){
   double largest=0;
   for(unsigned at=0;at<count;at++)if(free(at)){
    auto gradient=Mul(laplacian(at),-double(neighbours[at].size()));
    for(auto other:affected[at])if(other!=at)gradient=Add(gradient,laplacian(other));
    if(at%stride==0||at%stride==columns)gradient=Add(gradient,Mul(Sub(unproject(material[at]),unproject(guide[at])),.2));
    auto delta=Mul(sphereGradient(material[at],gradient),-.025);double conformal=2/(1+Dot(material[at],material[at]));delta=Mul(delta,1/(conformal*conformal));double length=Length(delta);if(length>scale*.002)delta=Mul(delta,scale*.002/length);if(Length(delta)<scale*1e-12)continue;
    double before=0;for(auto other:affected[at])before+=energy(other);auto previous=material[at];
    for(unsigned trial=0;trial<16;trial++){
     material[at]=Add(previous,delta);double after=0;if(valid(at)){for(auto other:affected[at])after+=energy(other);if(after<before){largest=(std::max)(largest,Length(delta));break;}}
     material[at]=previous;delta=Mul(delta,.5);
    }
   }
   if(largest<scale*1e-10)break;
  }
  for(unsigned row=0;row<=rows;row++)for(unsigned col=0;col<top.size();col++){
   if(!row){result.vertices.push_back(top[col]);continue;}
   auto local=unproject(material[row*stride+col]);
   try{
    auto direction=Add(Mul(frame.lateral,local[0]),Add(Mul(frame.forward,local[1]),Mul(frame.up,local[2])));auto sample=Cast(direction);
    // The native sewn boundary may sit inside the global convex pressure
    // envelope. Do not jump its radius to that envelope in one material row:
    // begin with a smooth collar transition, then verify actual tissue contact.
    double blend=collarBlend(rowParameters[row]);
    double startRadius=Length(Sub(top[col].position,center)),radius=Length(Sub(sample.position,center));sample.position=Add(center,Mul(direction,startRadius+(radius-startRadius)*blend));result.vertices.push_back(sample);
   }catch(const std::invalid_argument& e){throw std::invalid_argument(std::string(e.what())+" in rest chart cell "+std::to_string(row)+","+std::to_string(col));}
  }
  return result;
 }
};
}
