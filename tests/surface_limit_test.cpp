#include <malemod/surface_limit.hpp>
#include <iostream>
#include <random>

int main() {
  using namespace malemod;
  std::mt19937 rng(721);
  std::uniform_real_distribution<float> random(-3.f,3.f);
  for(int test=0;test<2000;test++) {
    auto point=[&](){return V3{random(rng),random(rng),random(rng)};};
    V3 a=point(),b=point(),c=point(),da=point(),db=point(),dc=point();
    V3 n=Cross(b-a,c-a);
    float area=Dot(n,n), floor=.2f;
    if(area<.001f) continue;
    float limit=SurfaceCorrectionLimit(a,b,c,da,db,dc,floor);
    PreparedSurfaceLimit prepared; prepared.Prepare(a,b,c);
    if(std::abs(prepared.Evaluate(da,db,dc,floor)-limit)>1e-6f || limit<0 || limit>1) return 1;
    for(int sample=0;sample<=32;sample++) {
      float s=limit*sample/32.f;
      V3 next=Cross((b+db*s)-(a+da*s),(c+dc*s)-(a+da*s));
      if(Dot(n,next)<(floor-1e-4f)*area) {
        std::cerr << "Area bound violated at case " << test << '\n'; return 2;
      }
    }
  }
  std::cout << "PASS: 2000 random corrections, sampled continuous area bounds and prepared parity.\n";
}
