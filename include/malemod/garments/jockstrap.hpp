#pragma once
// Portable, measured-surface garment contract. No game, graphics or physics SDK.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
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
struct Lineage {std::array<Donor,4> donors{};};
struct Sample {Point position{},normal{0,0,1};Lineage lineage{};};
struct Capsule {Point a{},b{};double radius=0;};
enum class Style:std::uint32_t {Naked=0,WhiteJockstrap=1};
enum class MaterialSlot:std::uint32_t {WhiteRibbed=0,WhiteElastic=1,RedStripe=2,BlueStripe=3};
struct Material {MaterialSlot slot;Point color;double roughness;double ribStrength;};
struct Vertex {Point position{},normal{},tangent{};std::array<double,2> uv{};Lineage lineage{};double tangentSign=1;};
struct Triangle {std::array<std::uint32_t,3> vertices{};MaterialSlot material=MaterialSlot::WhiteRibbed;};
struct Mesh {std::vector<Vertex> vertices;std::vector<Triangle> triangles;};
struct Support {Lineage lineage;Point acceleration{};double influence=1,separation=0;};
struct Parameters {
    // All dimensions are fractions of the measured waist circumference.
    double bandWidth=.065,bandThickness=.006,strapWidth=.038,hemWidth=.006,clearance=.008,supportFraction=.055;
    unsigned pouchRings=32,pouchSegments=64; // bounded <=64/128, default ~3k vertices
};
struct Input {
    Frame frame;
    std::uint64_t characterEpoch=0,topologyRevision=0;
    // Ordered, closed contours, without a duplicate endpoint. Opening is the
    // measured anatomical/body join, angle around forward: lateral toward up.
    std::vector<Sample> waist,opening,anatomy;
    std::vector<std::array<std::uint32_t,3>> anatomyTriangles;
    // Two ordered measured curves from posterior waist, under each glute, to
    // the corresponding lower pouch opening. They are not inferred skeletons.
    std::array<std::vector<Sample>,2> rearStraps;
    std::vector<Capsule> bodyContacts;
    Point gravity{0,0,0};
    double deltaTime=0;
};
struct Output {
    Style style=Style::Naked;
    std::uint64_t characterEpoch=0,topologyRevision=0;
    Mesh mesh;
    std::array<Material,4> materials{{{MaterialSlot::WhiteRibbed,{.94,.94,.925},.88,.18},{MaterialSlot::WhiteElastic,{.96,.96,.94},.82,.08},{MaterialSlot::RedStripe,{.62,.035,.045},.78,.04},{MaterialSlot::BlueStripe,{.035,.09,.45},.78,.04}}};
    std::vector<Support> support;
    double measuredCircumference=0,coverageMargin=0;
    unsigned projectedContacts=0;
    bool contactBudgetSatisfied=true;
};
// Deterministic procedural knit sample: adapters can bake this into their
// native material textures. RGB contains cloth detail, not character skin.
inline std::array<double,4> Knit(double u,double v){const double rib=.018*std::cos(u*6.283185307179586*128),weave=.004*std::cos(v*6.283185307179586*256);return {.94+rib+weave,.94+rib+weave,.925+rib+weave,1};}
class Session {
public:
    explicit Session(Parameters p={});
    const Output& Update(Style style,const Input& input);
    void Reset();
private:
    Parameters parameters_;
    Output output_;
    std::vector<Point> localAnatomy_;
};
}
#include "jockstrap_detail.hpp"
