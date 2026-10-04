#include <malemod/garments/root_closure.hpp>
#include <iostream>
using namespace malemod::garments;
using Face=std::array<std::uint32_t,3>;
static void Check(bool value,const char* text){if(!value)throw std::runtime_error(text);}
struct Fixture {std::vector<Sample> points,opening;std::vector<Face> faces;};
static Fixture Concave(){
 Fixture f;for(Point p:std::vector<Point>{{-2,-1,0},{2,-1,0},{2,1,.08},{0,0,-.04},{-2,1,.03}}){Sample s;s.position=p;f.points.push_back(s);f.opening.push_back(s);}
 const unsigned n=unsigned(f.points.size());for(unsigned i=0;i<n;i++){auto s=f.points[i];s.position[2]+=2;f.points.push_back(s);}
 for(unsigned i=0;i<n;i++){unsigned j=(i+1)%n;f.faces.push_back({i,j,j+n});f.faces.push_back({i,j+n,i+n});}
 for(Face face:std::vector<Face>{{0,1,3},{1,2,3},{3,4,0}}){for(auto& id:face)id+=n;f.faces.push_back(face);}return f;
}
static void Closed(const Fixture& f,const std::vector<Face>& cap){
 std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,int>> edges;
 for(const auto* faces:{&f.faces,&cap})for(auto face:*faces)for(unsigned i=0;i<3;i++){auto a=face[i],b=face[(i+1)%3];auto& e=edges[(std::minmax)(a,b)];e.first++;e.second+=a<b?1:-1;}
 for(auto e:edges)Check(e.second.first==2&&e.second.second==0,"Root closure did not reverse every unmatched edge");
}
template<class F>static void Rejected(F operation){bool failed=false;try{operation();}catch(const std::invalid_argument&){failed=true;}Check(failed,"Invalid root closure was accepted");}
int main(){try{
 auto f=Concave();auto cap=RootCap(f.points,f.faces,f.opening,4);Check(cap.size()==3,"Concave measured root triangulation count differs");Closed(f,cap);
 Check(cap==RootCap(f.points,f.faces,f.opening,4),"Root closure is nondeterministic");
 for(auto face:cap)for(auto id:face)Check(id<5,"Cap invented a vertex instead of using measured root indices");
 // Unit/world-origin transport retains exact index topology, including a
 // nonplanar concave boundary that cannot use an unconditional centroid fan.
 auto moved=f;for(auto* samples:{&moved.points,&moved.opening})for(auto& s:*samples)s.position=Add({1000,-2000,3000},Mul(s.position,100));
 Check(RootCap(moved.points,moved.faces,moved.opening,400)==cap,"Measured root closure depends on source units/world origin");
 // UV/shading aliases are position-welded only for classification. Existing
 // points and rendered topology remain untouched.
 auto aliased=f;for(auto& face:aliased.faces)for(auto& id:face){aliased.points.push_back(aliased.points[id]);id=unsigned(aliased.points.size()-1);}
 Check(RootCap(aliased.points,aliased.faces,aliased.opening,4)==cap,"Positional root aliases changed closure");
 auto complete=f;complete.faces.insert(complete.faces.end(),cap.begin(),cap.end());Check(RootCap(complete.points,complete.faces,{},4).empty(),"Already closed anatomy gained extra classification faces");
 auto wrong=f;wrong.opening[0].position[0]+=.01;Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 wrong=f;wrong.opening.pop_back();Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 wrong=f;std::swap(wrong.faces[0][1],wrong.faces[0][2]);Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 wrong=f;wrong.faces.push_back(wrong.faces.front());Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 wrong=f;wrong.faces[0][0]=999;Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 wrong=f;auto second=f;unsigned offset=unsigned(wrong.points.size());for(auto s:second.points){s.position[0]+=10;wrong.points.push_back(s);}for(auto face:second.faces){for(auto& id:face)id+=offset;wrong.faces.push_back(face);}Rejected([&]{RootCap(wrong.points,wrong.faces,wrong.opening,4);});
 // Bounded high-resolution root used by anatomical attachments, with an
 // intentionally nonplanar measured ring and no added cap centroid.
 Fixture ring;constexpr unsigned n=68;for(unsigned i=0;i<n;i++){double angle=6.283185307179586*i/n;Sample s;s.position={std::cos(angle),std::sin(angle),.03*std::cos(3*angle)};ring.points.push_back(s);ring.opening.push_back(s);}
 for(unsigned i=0;i<n;i++){auto s=ring.points[i];s.position[2]+=2;ring.points.push_back(s);}Sample tip;tip.position={0,0,3};ring.points.push_back(tip);
 for(unsigned i=0;i<n;i++){unsigned j=(i+1)%n;ring.faces.push_back({i,j,j+n});ring.faces.push_back({i,j+n,i+n});ring.faces.push_back({i+n,j+n,2*n});}
 auto rc=RootCap(ring.points,ring.faces,ring.opening,2);Check(rc.size()==66,"68-point measured root did not produce 66 closure faces");Closed(ring,rc);
 // Generic classification closure includes every measured remote opening,
 // while RootCap still insists on its one anatomically matched root loop.
 auto multiple=f;unsigned remoteOffset=unsigned(multiple.points.size());for(auto s:f.points){s.position=Add(s.position,{10,0,0});multiple.points.push_back(s);}for(auto face:f.faces){for(auto& id:face)id+=remoteOffset;multiple.faces.push_back(face);}
 auto caps=SurfaceCaps(multiple.points,multiple.faces,4);Check(caps.size()==6,"Two measured remote loops did not produce six cap faces");Closed(multiple,caps);
 Check(caps==SurfaceCaps(multiple.points,multiple.faces,4),"Multiple measured closures are nondeterministic");
 for(auto face:caps)for(auto id:face)Check(id<multiple.points.size(),"Remote closure invented a vertex");
 auto remoteAliases=multiple;for(auto& face:remoteAliases.faces)for(auto& id:face){remoteAliases.points.push_back(remoteAliases.points[id]);id=unsigned(remoteAliases.points.size()-1);}Check(SurfaceCaps(remoteAliases.points,remoteAliases.faces,4)==caps,"Remote shading aliases changed closure topology");
 auto transformed=multiple;for(auto& s:transformed.points)s.position=Add({1000,-2000,3000},Mul(s.position,100));Check(SurfaceCaps(transformed.points,transformed.faces,400)==caps,"Remote cap topology depends on unit/world origin");
 auto allClosed=multiple;allClosed.faces.insert(allClosed.faces.end(),caps.begin(),caps.end());Check(SurfaceCaps(allClosed.points,allClosed.faces,4).empty(),"Closed remote surface gained extra triangles");
 Rejected([&]{RootCap(multiple.points,multiple.faces,multiple.opening,4);});
 auto badRemote=multiple;std::swap(badRemote.faces[0][1],badRemote.faces[0][2]);Rejected([&]{SurfaceCaps(badRemote.points,badRemote.faces,4);});
 badRemote=multiple;badRemote.faces.push_back(badRemote.faces.front());Rejected([&]{SurfaceCaps(badRemote.points,badRemote.faces,4);});
 // A 20-point native opening and the actual fine-boundary pattern of 54
 // existing points: 34 additional measured points lie on original edges.
 Fixture coarse;constexpr unsigned coarseN=20;for(unsigned i=0;i<coarseN;i++){double a=6.283185307179586*i/coarseN;Sample s;s.position={std::cos(a),std::sin(a),0};coarse.points.push_back(s);}
 for(unsigned i=0;i<coarseN;i++){auto s=coarse.points[i];s.position[2]=2;coarse.points.push_back(s);}Sample roof;roof.position={0,0,3};coarse.points.push_back(roof);
 for(unsigned i=0;i<coarseN;i++){unsigned j=(i+1)%coarseN;coarse.faces.push_back({i,j,j+coarseN});coarse.faces.push_back({i,j+coarseN,i+coarseN});coarse.faces.push_back({i+coarseN,j+coarseN,2*coarseN});}
 std::vector<std::uint32_t> fine;for(unsigned i=0;i<coarseN;i++){unsigned subdivisions=i<14?3:2;for(unsigned k=0;k<subdivisions;k++){auto a=coarse.points[i],b=coarse.points[(i+1)%coarseN];a.position=Add(Mul(a.position,1-double(k)/subdivisions),Mul(b.position,double(k)/subdivisions));if(k)a.position[2]+=6.5e-8*4;fine.push_back(unsigned(coarse.points.size()));coarse.points.push_back(a);}}
 Check(fine.size()==54,"Coarse/fine fixture has wrong actual boundary counts");auto originals=coarse.faces;auto refined=RefineClassificationBoundary(coarse.points,coarse.faces,fine,4);Check(coarse.faces==originals,"Classification refinement modified physical faces");
 auto refinedBoundary=closure_detail::Extract(coarse.points,refined,4);Check(refinedBoundary.loops.size()==1&&refinedBoundary.loops.front().size()==54,"Fine root still contains T junctions");
 auto classificationCaps=SurfaceCaps(coarse.points,refined,4);Check(classificationCaps.size()==52,"Fine boundary cap count differs");auto sealed=refined;sealed.insert(sealed.end(),classificationCaps.begin(),classificationCaps.end());Check(closure_detail::Extract(coarse.points,sealed,4).loops.empty(),"Refined classification root is not closed");
 Check(refined==RefineClassificationBoundary(coarse.points,coarse.faces,fine,4),"Classification boundary refinement is nondeterministic");
 auto coarseMoved=coarse;for(auto& s:coarseMoved.points)s.position=Add({1000,-2000,3000},Mul(s.position,100));Check(RefineClassificationBoundary(coarseMoved.points,coarseMoved.faces,fine,400)==refined,"Classification refinement depends on units/world origin");
 auto coarseAlias=coarse;for(auto& face:coarseAlias.faces)for(auto& id:face){coarseAlias.points.push_back(coarseAlias.points[id]);id=unsigned(coarseAlias.points.size()-1);}auto aliasRefined=RefineClassificationBoundary(coarseAlias.points,coarseAlias.faces,fine,4);Check(SurfaceCaps(coarseAlias.points,aliasRefined,4)==classificationCaps,"Classification body aliases changed refined closure");
 auto invalidFine=fine;invalidFine.erase(invalidFine.begin());Rejected([&]{RefineClassificationBoundary(coarse.points,coarse.faces,invalidFine,4);});
 auto offEdge=coarse;offEdge.points[fine[1]].position[2]+=.0001;Rejected([&]{RefineClassificationBoundary(offEdge.points,coarse.faces,fine,4);});
 invalidFine=fine;invalidFine[1]=invalidFine[0];Rejected([&]{RefineClassificationBoundary(coarse.points,coarse.faces,invalidFine,4);});
 std::cout<<"PASS measured closures and 20-to-54 existing-index classification refinement, aliases, unit covariance and rejection\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

