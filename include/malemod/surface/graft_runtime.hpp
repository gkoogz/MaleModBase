#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace malemod::surface {
using PrecisePoint=std::array<double,3>;
struct GraftFrame {
 PrecisePoint root,axis,up;
 double radius,length,sourceLengthScale=1;
 // Optional target-character seam support, in the same calibrated units.
 // Zero preserves the historical source-only support field exactly.
 double seamSupportRadius=0;
};
struct EdgeConstraint {std::uint32_t slave,a,b;double weight;};
// Unique geometric domain. Render aliases/UVs and resource splits stay in the
// versioned binding artifact. Both sides of a seam use these same master rows.
struct GraftDomain {
 std::vector<PrecisePoint> points;
 std::vector<std::array<std::uint32_t,3>> triangles;
 std::vector<EdgeConstraint> seams;
 std::vector<std::uint32_t> protectedVertices;
 // Movable shared part-boundary masters. Their displacement is supplied by
 // the one versioned boundary state, identically for every resource and LOD.
 // These are Dirichlet rows, distinct from protected zero-displacement rows.
 std::vector<std::uint32_t> prescribedVertices;
 // Optional body triangles whose signed area must remain positive in the
 // character's rest frame. Empty retains the historical solve byte for byte.
 // Anatomy may rotate independently; the adapter identifies body topology.
 std::vector<std::uint32_t> orientationTriangles;
 // Projection target for signed rest area. Adapters with native packing and
 // subdivision can request more headroom than the historical two-percent
 // target. Exact donors and fixed boundaries remain hard constraints.
 double orientationAreaTargetRatio=.02;
 // Preserve an already smooth measured body target. Fair corrections to its
 // differential coordinates, rather than flattening the target displacement
 // against independently prescribed resource-boundary rows. Default false
 // preserves historical anatomy fitting and its numerical replay exactly.
 bool preserveTargetDifferential=false;
 // Optional local fairing of the actual surface, rather than displacement.
 // Distance uses the supplied calibrated source units; zero keeps historical
 // replay. Exact seam donors and prescribed/protected boundaries still apply.
 double surfaceFairingDistance=0,surfaceFairingStrength=0;
};
// The same support law used by the target collar matrix. Adapters can suppress
// distant body-field inputs without maintaining another copy of that law.
double GraftRecruitmentWeight(PrecisePoint,const GraftFrame&);
// C++ adoption of Base's malemod_base.collar.CollarPlan displacement solve.
// No game, graphics or skeleton SDK. Reuse the factorization until the supplied
// rest metric/frame changes; it is not a deformable-bone approximation.
class GraftPlan {
 public:
  GraftPlan(const GraftDomain&,const GraftFrame&);
  // Cached constraint-only plan for a measured cooked surface. It does not
  // build a smoothing matrix or require anatomy support dimensions.
  explicit GraftPlan(const GraftDomain&);
  ~GraftPlan();
  GraftPlan(const GraftPlan&)=delete;
  GraftPlan& operator=(const GraftPlan&)=delete;
  // Reuse immutable topology operators while updating the exact support law.
  // A different sourceLengthScale requires a new plan. No frame quantization.
  void UpdateFrame(const GraftFrame&);
  std::vector<PrecisePoint> SolveDisplacement(const std::vector<PrecisePoint>&)const;
  std::vector<PrecisePoint> ProjectDisplacement(const std::vector<PrecisePoint>&)const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
