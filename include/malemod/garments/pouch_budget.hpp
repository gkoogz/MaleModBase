#pragma once
namespace malemod::garments {
// Cloth fitting still consumes the current pose every rendered frame. This
// budget concerns only the hidden contents integrator; elapsed simulation time
// and the reference contact iteration count are preserved.
struct PouchContentsBudget {float stepSeconds;unsigned maximumSteps;};
inline constexpr PouchContentsBudget PouchContentsSimulationBudget(bool covered){
 return covered?PouchContentsBudget{1.f/120.f,12}:PouchContentsBudget{1.f/240.f,24};
}
// Keep the complete attachment and skin evaluator intact, but amortize its
// fine deformation under the garment. Native skeletal animation and drawing
// continue every frame. Controls and active clinical timing bypass this LOD.
inline constexpr bool UpdatePouchCoveredSurface(bool covered,bool dirty,bool clinicalActive,unsigned long long frame){
 return !covered||dirty||clinicalActive||(frame%2==0);
}
}
