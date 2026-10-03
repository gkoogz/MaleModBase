#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace malemod::surface {
struct Point {float x,y,z;};
// Same public order as malemod_base.controls.ORDER, including state at index 0.
struct Controls {
 std::array<float,18> values={2,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
};
// Measured character contact envelope in calibrated source-local coordinates.
// The adapter owns bone selection and world-to-local conversion. Do not infer
// another character's dimensions from the source's reference pose.
struct CollisionCalibration {
 std::array<float,2> thighRadii;
 std::array<Point,2> pelvisEndpoints;
 float pelvisRadius;
};
struct Frame {
 float seconds=1.f/60.f,pitchForce=0,yawForce=0;
 // Four source-local endpoints: left A/B, right A/B. Empty uses the source
 // reference pose; it must not be described as observing another game's legs.
 std::optional<std::array<Point,4>> thighEndpoints;
 std::optional<CollisionCalibration> collision;
 // Target character rest samples, already calibrated into source coordinates.
 // The source evaluates its final radial target law here without donor blur.
 std::vector<Point> collarQueries;
 // Optional gentle garment load in calibrated source-local length/time^2.
 // The worker caps it at15% of the current source gravity for each body.
 // Disabled input never enters source integration arithmetic.
 struct GarmentSupport {
  bool enabled=false;
  Point shaftAcceleration{};
  std::array<Point,2> lobeAcceleration{};
 } garment;
 struct ClinicalProjection {
  bool active=false;double time=0;
  std::uint32_t throbMode=0;
  float sizeTime=0,twitchTime=0,lateralWobbleDegrees=0;
  std::array<float,4> lateralGain{};
  std::array<float,4> angleGain={1,1,1,1};
 } clinical;
};
struct Surface {
 std::vector<Point> positions,normals,tangents;
 std::vector<std::array<float,2>> uv;
 std::vector<std::uint32_t> sourceVertexIDs;
};
// Actual support frame last used by the source collar metric. Its lifecycle is
// controlled by the source solver, independently of the moving guide frame.
struct CollarMetric {
 Point root{},axis{},up{};
 float radius=0,length=0;
 std::uint32_t generation=0;
};
struct Output {
 Surface anatomy;
 std::array<Surface,2> body;
 std::vector<std::uint16_t> anatomyIndices;
 float proximalRadius=0,restLength=0;
 // Source-local mechanical state, exported with the evaluated surface so an
 // adapter does not infer guide stations from skin centroids. No engine bones.
 std::array<Point,12> shaftGuide,restGuide;
 std::array<Point,2> lobeCenters,lobeAnchors,lobeRadii;
 std::array<std::array<Point,3>,2> lobeAxes;
 Point rootDirection;
 std::array<float,10> bendMultipliers;
 CollarMetric collarMetric;
 std::vector<Point> collarDisplacements;
 Point nozzlePosition{},nozzleDirection{1,0,0};
};
struct Diagnostics {std::array<double,16> geometryMilliseconds{};};
// Each session owns a worker thread and all mutable source caches. Evaluation
// is synchronous. Do not call from a render hook while holding engine locks.
// No game/graphics SDK, engine skin palette, input or upload API is exposed.
// A process-isolated build permits one Session lifetime per process. Replace
// the process to reset its character; the ordinary TLS build supports instances.
class Session {
 public:
  explicit Session(const Controls& controls={});
  ~Session();
  Session(const Session&)=delete;
  Session& operator=(const Session&)=delete;
  void SetControls(const Controls& controls);
  void Step(const Frame& frame={});
  Output Read();
  Diagnostics ReadDiagnostics();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
