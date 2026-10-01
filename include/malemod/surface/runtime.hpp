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
struct Frame {
 float seconds=1.f/60.f,pitchForce=0,yawForce=0;
 // Four source-local endpoints: left A/B, right A/B. Empty uses the source
 // reference pose; it must not be described as observing another game's legs.
 std::optional<std::array<Point,4>> thighEndpoints;
};
struct Surface {
 std::vector<Point> positions,normals,tangents;
 std::vector<std::array<float,2>> uv;
 std::vector<std::uint32_t> sourceVertexIDs;
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
};
// Each session owns a worker thread and all mutable source caches. Evaluation
// is synchronous. Do not call from a render hook while holding engine locks.
// No game/graphics SDK, engine skin palette, input or upload API is exposed.
class Session {
 public:
  explicit Session(const Controls& controls={});
  ~Session();
  Session(const Session&)=delete;
  Session& operator=(const Session&)=delete;
  void SetControls(const Controls& controls);
  void Step(const Frame& frame={});
  Output Read();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
