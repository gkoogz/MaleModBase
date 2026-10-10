#pragma once
namespace malemod::garments {
// Cloth fitting still consumes the current pose every rendered frame. This
// budget concerns only the hidden contents integrator; elapsed simulation time
// and the reference contact iteration count are preserved.
struct PouchContentsBudget {float stepSeconds;unsigned maximumSteps;};
inline constexpr PouchContentsBudget PouchContentsSimulationBudget(bool covered){
 return covered?PouchContentsBudget{1.f/120.f,12}:PouchContentsBudget{1.f/240.f,24};
}
// A supportive garment damps the contents as well as enclosing them. These
// dimensionless multipliers leave uncovered mechanics exactly unchanged.
struct PouchSupport {float motionTransfer,rootStiffness,rootDamping,linearDrag;};
inline constexpr PouchSupport PouchContentsSupport(bool covered){
 return covered?PouchSupport{.18f,4.f,3.f,8.f}:PouchSupport{1.f,1.f,1.f,0.f};
}
// Keep the complete attachment and skin evaluator intact, but amortize its
// fine deformation under the garment. Native skeletal animation and drawing
// continue every frame. Controls and active clinical timing bypass this LOD.
inline constexpr bool UpdatePouchCoveredSurface(bool covered,bool dirty,bool clinicalActive,unsigned long long frame){
 return !covered||dirty||clinicalActive||(frame%2==0);
}
}
