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
};
struct EdgeConstraint {std::uint32_t slave,a,b;double weight;};
// Unique geometric domain. Render aliases/UVs and resource splits stay in the
// versioned binding artifact. Both sides of a seam use these same master rows.
struct GraftDomain {
 std::vector<PrecisePoint> points;
 std::vector<std::array<std::uint32_t,3>> triangles;
 std::vector<EdgeConstraint> seams;
 std::vector<std::uint32_t> protectedVertices;
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
  ~GraftPlan();
  GraftPlan(const GraftPlan&)=delete;
  GraftPlan& operator=(const GraftPlan&)=delete;
  // Reuse immutable topology operators while updating the exact support law.
  // A different sourceLengthScale requires a new plan. No frame quantization.
  void UpdateFrame(const GraftFrame&);
  std::vector<PrecisePoint> SolveDisplacement(const std::vector<PrecisePoint>&)const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
