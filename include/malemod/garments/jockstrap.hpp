#pragma once
// Portable, measured-surface garment contract. No game, graphics or physics SDK.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <map>
#include <unordered_map>
#include <set>
#include <queue>
#include <optional>
#include <functional>
#include "proximity_cache.hpp"
namespace malemod::garments {
using Point=std::array<double,3>;
inline Point Add(Point a,Point b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline Point Sub(Point a,Point b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline Point Mul(Point a,double s){return {a[0]*s,a[1]*s,a[2]*s};}
inline double Dot(Point a,Point b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Point Cross(Point a,Point b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline double Length(Point a){return std::sqrt(Dot(a,a));}
inline bool Finite(Point p){return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]);}
inline Point Unit(Point p){double n=Length(p);if(!std::isfinite(n)||n<1e-14)throw std::invalid_argument("Garment direction is degenerate");return Mul(p,1/n);}
struct Frame {
    Point origin{},lateral{1,0,0},forward{0,1,0},up{0,0,1};
    Point Local(Point p)const{p=Sub(p,origin);return {Dot(p,lateral),Dot(p,forward),Dot(p,up)};}
    Point World(Point p)const{return Add(origin,Add(Mul(lateral,p[0]),Add(Mul(forward,p[1]),Mul(up,p[2]))));}
    void Validate()const{if(!Finite(origin)||!Finite(lateral)||!Finite(forward)||!Finite(up)||std::abs(Dot(lateral,lateral)-1)>1e-5||std::abs(Dot(forward,forward)-1)>1e-5||std::abs(Dot(up,up)-1)>1e-5||std::abs(Dot(lateral,forward))>1e-5||std::abs(Dot(lateral,up))>1e-5||std::abs(Dot(forward,up))>1e-5||Dot(Cross(lateral,forward),up)<.9999)throw std::invalid_argument("Garment frame must be finite orthonormal and right handed");}
};
enum class Surface:std::uint32_t {Body=0,Anatomy=1};
struct Donor {Surface surface=Surface::Body;std::uint32_t vertex=0;double weight=0;};
struct Lineage {std::array<Donor,16> donors{};};
struct Sample {Point position{},normal{0,0,1};Lineage lineage{};};
struct Capsule {Point a{},b{};double radius=0;};
enum class Style:std::uint32_t {Naked=0,WhiteJockstrap=1};
enum class MaterialSlot:std::uint32_t {WhiteRibbed=0,WhiteElastic=1,RedStripe=2,BlueStripe=3};
// Optical coverage is independent of cloth compliance. Adapters must use an
// actual supported opacity path rather than treating diffuse alpha as proof.
struct Material {MaterialSlot slot;Point color;double roughness;double ribStrength;double opacity=.95;};
struct Vertex {Point position{},normal{},tangent{};std::array<double,2> uv{};Lineage lineage{};double tangentSign=1;};
struct Triangle {std::array<std::uint32_t,3> vertices{};MaterialSlot material=MaterialSlot::WhiteRibbed;};
struct Mesh {std::vector<Vertex> vertices;std::vector<Triangle> triangles;};
struct Support {Lineage lineage;Point acceleration{};double influence=1,separation=0;};
// Impulse and first moment in the caller's length units and the explicitly
// calibrated dimensionless source mass units. Moment is about world zero.
// Each record retains the actual contacted tissue vertex's full lineage.
struct ContactReaction {Lineage lineage;Point impulse{},moment{};double separation=0;};
struct ReactionTelemetry {
    unsigned contacts=0,records=0;
    double activeSeconds=0,clothMass=0;
    Point clothImpulse{},anatomyImpulse{},clothMoment{},anatomyMoment{};
    // Independently summed mass*particle velocity changes and their moments.
    // Differences from point-contact load belong to sewn/prescribed material
    // support; rotating render offsets are not independent rigid-body DOFs.
    Point particleImpulse{},particleMoment{},supportImpulse{},supportMoment{};
};
// Numerical cloth mass is relative to adapter-supplied generalized tissue mass.
// Optical opacity is independent of these mechanics. Compliance operates in
// circumference-normalized material coordinates; no kilograms are inferred.
struct ClothMechanics {
    double massFraction=.08,stretchCompliance=2e-8,bendCompliance=2e-4;
    double extensionLimit=1.05,dampingRate=3,friction=.35,hemComplianceMultiplier=.25;
};
struct Parameters {
    ClothMechanics mechanics;
    // Body-following trim with a short free underside connector. The pouch
    // remains persistent cloth. Adapters opt into this shared support contract.
    bool supportedTrim=false;
    // All dimensions are fractions of the measured waist circumference.
    double bandWidth=.065,bandThickness=.003,strapWidth=.025,hemWidth=.012,hemThickness=.0025,clearance=.008,supportFraction=.055;
    double panelHalfAngle=.6981317007977318; // measured front waistband span in radians
    double sideCoverageExtra=.45; // maximum measured lower-middle pattern growth; not a user slider
    unsigned pouchRings=32,pouchSegments=64; // material grid resolution, <=64 rows/128 columns
    bool simulate=true; // false is the explicit, offline fitted-geometry reference
};
// Shared supported-trim development profile. Body adapters supply measured geometry only.
inline Parameters SupportedPouchParameters(){Parameters p;p.supportedTrim=true;p.pouchRings=12;p.pouchSegments=24;return p;}
enum class AnatomyRegion:unsigned {Shaft=0,Glans=1,LeftLobe=2,RightLobe=3};
using AnatomyRegions=std::array<std::vector<std::uint32_t>,4>;
struct VertexRange {unsigned start=0,count=0;};
struct GridLayout {unsigned start=0,rows=0,columns=0;}; // (rows+1)*(columns+1) vertices
struct RibbonLayout {unsigned start=0,sections=0,corners=4;};
struct MaterialPoint {std::vector<std::uint32_t> vertices;std::vector<double> weights;};
struct AuthoredJoint {std::string name;MaterialPoint a,b;Point restOffset{};};
struct MaterialLayout {
    unsigned revision=0;
    double upperArcRadians=0,sideCoverageExtra=.45,sideCoverageAspect=0,sideCoverageEffective=0;
    GridLayout band;unsigned bandLayers=0;
    GridLayout sheet;
    VertexRange sheetFaces;
    std::vector<std::uint32_t> topSeam;
    std::array<std::vector<std::uint32_t>,2> bottomSeams;
    std::array<RibbonLayout,2> straps;
    std::array<std::vector<std::uint32_t>,2> sideBoundary;
    std::array<RibbonLayout,2> sideHems;
    // Independent audit declarations. Offsets come from finite authored layer
    // thickness, never from a post-contact fitted gap. Revision2 material
    // particle/binding topology is unchanged by these explicit joint receipts.
    unsigned jointRevision=0;
    double measuredCircumference=0,bandThicknessNormalized=0;
    std::vector<AuthoredJoint> authoredJoints;
};
// Authored subdivision of a measured coarse root edge. Animated skinning can
// bend the intermediate vertex off the straight chord; classification closure
// must retain its proven original edge ownership. These never replace contacts.
struct RootSubdivision {unsigned vertex=0,a=0,b=0;double t=0;};
struct Input {
    Frame frame;
    std::uint64_t characterEpoch=0,topologyRevision=0;
    std::uint64_t restRevision=0; // caller morphology revision; not an animation-frame counter
    // Ordered, closed contours, without a duplicate endpoint. Opening is the
    // measured anatomical/body join, angle around forward: lateral toward up.
    std::vector<Sample> waist,opening,anatomy;
    std::vector<std::array<std::uint32_t,3>> anatomyTriangles;
    std::vector<RootSubdivision> rootSubdivisions;
    AnatomyRegions anatomyRegions;
    // Two ordered measured curves from lateral hip midpoint, under each glute, to
    // the corresponding lower pouch opening. They are not inferred skeletons.
    std::array<std::vector<Sample>,2> rearStraps;
    std::vector<Capsule> bodyContacts; // legacy fallback when no measured surface
    std::vector<Sample> bodySurface;
    std::vector<std::array<std::uint32_t,3>> bodyTriangles;
    Point gravity{0,0,0};
    double deltaTime=0;
    // Sum of measured generalized anatomy masses, in the source solver's
    // dimensionless mass units. Zero is an uncoupled offline cloth fixture.
    double anatomyMass=0;
};
struct ClothTelemetry {
    bool active=false,reset=false,stateReady=false,materialBudgetSatisfied=true;
    unsigned nodes=0,constraints=0,substeps=0,resetCount=0,worstStretchA=0,worstStretchB=0,worstRenderA=0,worstRenderB=0;
    double advancedSeconds=0,accumulatedSeconds=0,maxStretchRatio=1,maxBendError=0,
        maxSeamGap=0,maxDisplacement=0,rmsSpeed=0,maxSpeed=0,solverMilliseconds=0,
        worstStretchRestLength=0,worstStretchCurrentLength=0,maxRenderStretchRatio=1;
};
struct BandTelemetry {
    unsigned topAttachments=0,bottomAttachments=0;
    double topCircumference=0,bottomCircumference=0,preferredClearance=0,
        minimumInnerClearance=0,maximumInnerClearance=0;
};
struct Output {
    Style style=Style::Naked;
    std::uint64_t characterEpoch=0,topologyRevision=0;
    Mesh mesh;
    std::array<Material,4> materials{{{MaterialSlot::WhiteRibbed,{.94,.94,.925},.88,.18},{MaterialSlot::WhiteElastic,{.96,.96,.94},.82,.08},{MaterialSlot::RedStripe,{.62,.035,.045},.78,.04},{MaterialSlot::BlueStripe,{.035,.09,.45},.78,.04}}};
    std::vector<Support> support;
    std::vector<ContactReaction> reactions;
    ReactionTelemetry reaction;
    double measuredCircumference=0,coverageMargin=0;
    unsigned projectedContacts=0;
    bool contactBudgetSatisfied=true;
    ClothTelemetry physics;
    BandTelemetry band;
    MaterialLayout layout;
};
// Deterministic procedural knit sample: adapters can bake this into their
// native material textures. RGB contains cloth detail, not character skin.
inline std::array<double,4> Knit(double u,double v){const double rib=.018*std::cos(u*6.283185307179586*128),weave=.004*std::cos(v*6.283185307179586*256);return {.94+rib+weave,.94+rib+weave,.925+rib+weave,.95};}
namespace detail {
// Derive cloth geometry once per exact query, then reuse it for all BVH and
// candidate tests. Keep plane and SAT normals' original arithmetic distinct:
// their algebraic equivalence does not promise identical floating-point bits.
struct SurfaceFaceQuery {
 std::array<Point,3> face,edges;Point normal,satNormal,low,high;double square,normalLength;
 explicit SurfaceFaceQuery(const std::array<Point,3>& points):face(points){
  for(unsigned k=0;k<3;k++){edges[k]=Sub(face[(k+1)%3],face[k]);low[k]=(std::min)({face[0][k],face[1][k],face[2][k]});high[k]=(std::max)({face[0][k],face[1][k],face[2][k]});}
  normal=Cross(Sub(face[1],face[0]),Sub(face[2],face[0]));square=Dot(normal,normal);normalLength=std::sqrt(square);satNormal=Cross(edges[0],edges[1]);
 }
};
class BodyCollider {
public:
 enum class Side {Outside,Inside,Boundary,Indeterminate};
 struct Hit {Point point{},normal{};double distance=1e100,signedDistance=1e100;unsigned triangle=0;Point clothPoint{};};
 class PointNeighborhood {
  friend class BodyCollider;
  proximity::NeighborhoodBound bound_;
  std::vector<unsigned> candidates_;
 };
 class FaceNeighborhood {
  friend class BodyCollider;
  proximity::NeighborhoodBound bound_;
  std::vector<unsigned> candidates_;
  // Complete bounded BVH complement, used to certify exact neighborhoods
  // against local current geometry when distant surface motion is large.
  std::vector<unsigned> exclusions_;
  std::uintptr_t owner_=0;
  std::uint64_t topology_=0;
  bool exclusionsComplete_=false;
  unsigned rebuilt_=0,reused_=0,fallback_=0;
 public:
  unsigned RebuildCount()const{return rebuilt_;}
  unsigned ReuseCount()const{return reused_;}
  unsigned FallbackCount()const{return fallback_;}
  std::size_t CandidateCount()const{return candidates_.size();}
 };
 void Update(const std::vector<Sample>& vertices,const std::vector<std::array<std::uint32_t,3>>& faces,Point origin={},double scale=1);
 Hit Closest(Point point,unsigned seedTriangle=unsigned(-1))const;
 Hit Near(Point point,double radius,unsigned seedTriangle=unsigned(-1))const;
 Hit NearCached(Point point,double radius,PointNeighborhood& neighborhood,unsigned seedTriangle=unsigned(-1))const;
 // Exact within-margin triangle contact. No hit returns the query radius,
 // a conservative lower bound; it is not a measured global closest distance.
 Hit ClosestFace(const std::array<Point,3>& face,double margin,unsigned seedTriangle=unsigned(-1))const;
 Hit ClosestFaceCached(const std::array<Point,3>& face,double margin,FaceNeighborhood& neighborhood,unsigned seedTriangle=unsigned(-1))const;
 Hit Sweep(Point from,Point to)const;
 // Caller must prove closure (including positional aliases); open stock
 // surfaces do not define a volume. Physical contact faces remain separate.
 Side Classify(Point point,bool verifiedClosed=false)const;
 proximity::Stamp MotionStamp()const{return motion_.Current();}
 bool Empty()const{return triangles_.empty();}
 void Clear(){points_.clear();triangles_.clear();order_.clear();nodes_.clear();faceNormals_.clear();faceRawNormals_.clear();faceLo_.clear();faceHi_.clear();motion_.Advance(0,true);}
private:
 struct Node {Point lo{},hi{};unsigned begin=0,end=0,left=0,right=0;};
 std::vector<Point> points_;
 proximity::Motion motion_;
 std::vector<Point> faceNormals_,faceRawNormals_,faceLo_,faceHi_;
 std::vector<std::array<std::uint32_t,3>> triangles_;
 std::vector<unsigned> order_;
 std::vector<Node> nodes_;
 unsigned Build(unsigned begin,unsigned end);
 void Refit(unsigned node);
 void Search(unsigned node,Point point,Hit& hit)const;
 bool CollectPoints(unsigned node,Point point,double radius,std::vector<unsigned>& candidates)const;
 void Consider(unsigned triangle,Point point,Hit& hit)const;
 void SearchFace(unsigned node,const SurfaceFaceQuery& query,double margin,Hit& hit)const;
 void ConsiderFace(unsigned triangle,const SurfaceFaceQuery& query,Hit& hit)const;
 bool CollectFaces(unsigned node,const SurfaceFaceQuery& query,double radius,std::vector<unsigned>& candidates,std::vector<unsigned>* exclusions=nullptr,bool* exclusionsComplete=nullptr)const;
 void SearchSweep(unsigned node,Point from,Point to,Hit& hit)const;
 void SearchRay(unsigned node,Point from,Point direction,unsigned& intersections,bool& ambiguous,bool& boundary)const;
};
}
// Adapters that have already filtered pauses, rewinds and character changes
// may coalesce a long span of active time without discarding fabric state.
enum class TimeContinuity {Unverified,Continuous};
class Session {
public:
    explicit Session(Parameters p={});
    const Output& Update(Style style,const Input& input,TimeContinuity time=TimeContinuity::Unverified);
    // Fit material in a measured reference pose, then place its physical nodes
    // once using adapter-owned skinning. Subsequent updates retain cloth state.
    // The callback must preserve units and lineage; it never becomes a force.
    const Output& Initialize(const Input& reference,const Input& current,
                             const std::function<Sample(const Sample&)>& place);
    // Unpublished dressing solve: retain reference material and move measured
    // obstacles into the live pose before permitting physical feedback.
    const Output& InitializeDraped(const Input& reference,const Input& current);
    void Reset();
private:
    const Output& Fit(Style style,const Input& input);
    const Output& FitSheet(Style style,const Input& input);
    void Cloth(const Input& input,bool rebuild,double elapsed);
    void ClassifySurfaces(const Input& input,Point origin,double scale,bool rebuildCap=false);
    void SupportedCloth(const Input& input,const std::vector<Point>& targets,double elapsed);
    double ClassifiedDistance(detail::BodyCollider::Hit& hit,Point point,bool anatomy)const;
    double ClassifiedDistance(const detail::BodyCollider::Hit& hit,Point point,bool anatomy)const;
    struct Edge {unsigned a,b;double rest,compliance,lambda=0;bool bend=false,tether=false;};
    struct Binding {std::array<unsigned,4> nodes{};std::array<double,4> weights{};Point residual{};std::array<unsigned,3> materialFrame{};Point materialResidual{},restTangent{};bool transported=false,ribbon=false;};
    struct Sew {std::array<unsigned,28> nodes{};std::array<double,28> weights{};unsigned count=0;Point residual{};};
    struct Anchor {unsigned family=0,index=0;Point residual{},referenceNormal{};std::array<unsigned,16> bodyIndices{};std::array<double,16> bodyWeights{};std::array<Surface,16> attachmentSurfaces{};bool bodyAttachment=false;};
    struct PointMemo {proximity::PointCertificate certificate;unsigned closedSeed=unsigned(-1),physicalSeed=unsigned(-1);detail::BodyCollider::PointNeighborhood physicalNeighborhood;};
    struct FaceMemo {proximity::FaceCertificate certificate;unsigned closedSeed=UINT32_MAX,physicalSeed=UINT32_MAX;detail::BodyCollider::FaceNeighborhood physicalNeighborhood,closedNeighborhood;};
    std::array<std::vector<PointMemo>,2> pointMemos_;
    std::array<std::vector<FaceMemo>,2> faceMemos_;
    detail::BodyCollider bodyCollider_,anatomyCollider_,closedBody_,closedAnatomy_;
    std::vector<std::array<std::uint32_t,3>> rootCap_,closedBodyFaces_,closedAnatomyFaces_;
    std::vector<unsigned> rootVertices_;
    Parameters parameters_;
    Output output_;
    std::vector<Point> localAnatomy_;
    Mesh restMesh_;
    Input restInput_;
    std::vector<unsigned> vertexNodes_;
    std::vector<Binding> renderBindings_;
    std::vector<std::array<unsigned,3>> solverTriangles_;
    std::vector<Point> positions_,velocities_,previousTargets_;
    std::vector<Sample> referenceNodes_;
    std::vector<Anchor> anchors_;
    std::vector<Edge> edges_;
    std::vector<Sew> sewing_;
    std::vector<std::vector<unsigned>> stretchAdjacency_;
    std::vector<bool> pinned_;
    std::vector<double> inverseMass_;
    std::vector<Capsule> previousBodies_;
    std::vector<Sample> previousBodySurface_,previousAnatomySurface_;
    double clock_=0,accumulator_=0;
    double restCircumference_=0;
    double clothMass_=0;
    unsigned resetCount_=0;
};
}
#include "waist_contours.hpp"
#include "root_closure.hpp"
#include "waist_graft.hpp"
#include "natural_band.hpp"
#include "trim_support.hpp"
#include "jockstrap_detail.hpp"
#include "body_surface.hpp"
#include "surface_classification.hpp"
#include "drape_sheet.hpp"
#include "drape_section.hpp"
#include "drape_hull.hpp"
#include "material_grid.hpp"
#include "drape_chart.hpp"
#include "sheet_fit.hpp"
#include "cloth_reaction.hpp"
#include "contact_velocity.hpp"
#include "contact_query.hpp"
#include "render_contact.hpp"
#include "band_material.hpp"
#include "cloth_stretch.hpp"
#include "anchor_transport.hpp"
#include "cloth_detail.hpp"
#include "supported_cloth.hpp"
