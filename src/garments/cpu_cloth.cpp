#include <malemod/garments/cpu_cloth.hpp>
#include <NvCloth/Factory.h>
#include <NvCloth/Cloth.h>
#include <NvCloth/Fabric.h>
#include <NvCloth/Solver.h>
#include <NvCloth/PhaseConfig.h>
#include <NvClothExt/ClothFabricCooker.h>
#include <foundation/PxErrorCallback.h>
#include <mutex>
#include <cstdlib>
#include <cstdio>
namespace malemod::garments {
namespace {
struct Allocator final:physx::PxAllocatorCallback {
 void* allocate(size_t bytes,const char*,const char*,int)override {
#ifdef _WIN32
  void* p=_aligned_malloc(bytes,16);
#else
  void* p=nullptr;if(posix_memalign(&p,16,bytes))p=nullptr;
#endif
  if(!p)throw std::bad_alloc();return p;
 }
 void deallocate(void* p)override {
#ifdef _WIN32
  _aligned_free(p);
#else
  free(p);
#endif
 }
};
struct Errors final:physx::PxErrorCallback {
 void reportError(physx::PxErrorCode::Enum code,const char* message,const char*,int)override {
  if(code==physx::PxErrorCode::eDEBUG_INFO||code==physx::PxErrorCode::eDEBUG_WARNING)return;
  throw std::runtime_error(std::string("NvCloth: ")+message);
 }
};
struct Assertions final:nv::cloth::PxAssertHandler {
 void operator()(const char* message,const char*,int,bool&)override {throw std::runtime_error(std::string("NvCloth assertion: ")+message);}
};
physx::PxVec3 Vec(Point p){return {float(p[0]),float(p[1]),float(p[2])};}
template<class T> nv::cloth::Range<const T> Range(const std::vector<T>& v){return {v.data(),v.data()+v.size()};}
}
struct CpuCloth::Impl {
 nv::cloth::Factory* factory=nullptr;
 nv::cloth::Fabric* fabric=nullptr;
 nv::cloth::Cloth* cloth=nullptr;
 nv::cloth::Solver* solver=nullptr;
 std::vector<bool> pinned;
 std::vector<physx::PxVec4> spheres;
 std::vector<physx::PxVec3> triangles;
 ~Impl(){if(solver&&cloth)solver->removeCloth(cloth);delete solver;delete cloth;if(fabric)fabric->decRefCount();if(factory)NvClothDestroyFactory(factory);}
};
CpuCloth::CpuCloth():impl_(new Impl){
 static Allocator allocator;static Errors errors;static Assertions assertions;static std::once_flag once;
 std::call_once(once,[&]{nv::cloth::InitializeNvCloth(&allocator,&errors,&assertions,nullptr);});
}
CpuCloth::~CpuCloth()=default;
void CpuCloth::Initialize(const std::vector<Point>& points,const std::vector<std::array<unsigned,3>>& triangles,const std::vector<bool>& pins,double restScale){
 if(!std::isfinite(restScale)||restScale<1||restScale>1.5)throw std::invalid_argument("Cloth pattern ease outside 1..1.5");
 if(points.size()!=pins.size()||points.empty()||triangles.empty())throw std::invalid_argument("Invalid cloth topology");
 auto fresh=std::make_unique<Impl>();fresh->pinned=pins;
 std::vector<physx::PxVec4> particles;std::vector<physx::PxVec3> vertices;std::vector<float> masses;
 for(unsigned i=0;i<points.size();i++){if(!Finite(points[i]))throw std::invalid_argument("Nonfinite cloth point");vertices.push_back(Vec(points[i]));masses.push_back(pins[i]?0.f:1.f);particles.emplace_back(vertices.back(),masses.back());}
 for(const auto& triangle:triangles)for(auto i:triangle)if(i>=points.size())throw std::invalid_argument("Cloth triangle index outside points");
 nv::cloth::ClothMeshDesc mesh;mesh.points.data=vertices.data();mesh.points.count=unsigned(vertices.size());mesh.points.stride=sizeof(vertices[0]);
 mesh.invMasses.data=masses.data();mesh.invMasses.count=unsigned(masses.size());mesh.invMasses.stride=sizeof(float);
 mesh.triangles.data=triangles.data();mesh.triangles.count=unsigned(triangles.size());mesh.triangles.stride=sizeof(triangles[0]);
 fresh->factory=NvClothCreateFactoryCPU();if(!fresh->factory)throw std::runtime_error("NvCloth CPU factory failed");
 nv::cloth::Vector<int32_t>::Type phaseTypes;
 fresh->fabric=NvClothCookFabricFromMesh(fresh->factory,mesh,{0,0,-1},&phaseTypes,true);
 if(!fresh->fabric)throw std::runtime_error("NvCloth fabric cooking failed");
 fresh->fabric->scaleRestvalues(float(restScale));fresh->fabric->scaleTetherLengths(float(restScale));
 fresh->cloth=fresh->factory->createCloth(Range(particles),*fresh->fabric);
 if(!fresh->cloth)throw std::runtime_error("NvCloth cloth creation failed");
 auto& cloth=*fresh->cloth;
 std::vector<nv::cloth::PhaseConfig> phases;
 for(unsigned i=0;i<fresh->fabric->getNumPhases();i++){nv::cloth::PhaseConfig phase{uint16_t(i)};phase.mStiffness=phaseTypes[i]==nv::cloth::ClothFabricPhaseType::eBENDING?.02f:1.f;phases.push_back(phase);}
 cloth.setPhaseConfig(Range(phases));cloth.setSolverFrequency(2400);cloth.setStiffnessFrequency(60);
 cloth.setDamping({.08f,.08f,.08f});cloth.setFriction(.05f);cloth.setTetherConstraintStiffness(1);cloth.setTetherConstraintScale(1.05f);
 cloth.enableContinuousCollision(true);cloth.setSleepThreshold(0);
 cloth.setRestPositions(Range(particles));cloth.setSelfCollisionDistance(.003f);cloth.setSelfCollisionStiffness(1);
 // Interior samples stop coarse faces crossing a narrow collider even when
 // their three simulation vertices remain outside it.
 std::vector<std::array<uint32_t,4>> virtuals;
 const std::vector<physx::PxVec3> weights{{1.f/3,1.f/3,1.f/3},{.5f,.5f,0},{0,.5f,.5f},{.5f,0,.5f}};
 for(auto t:triangles)for(unsigned w=0;w<weights.size();w++)virtuals.push_back({t[0],t[1],t[2],w});
 auto first=reinterpret_cast<const uint32_t(*)[4]>(virtuals.data());
 cloth.setVirtualParticles({first,first+virtuals.size()},Range(weights));
 fresh->solver=fresh->factory->createSolver();if(!fresh->solver)throw std::runtime_error("NvCloth solver creation failed");
 fresh->solver->addCloth(fresh->cloth);impl_=std::move(fresh);
}
void CpuCloth::Place(const std::vector<Point>& positions){
 auto& s=*impl_;if(!s.cloth||positions.size()!=s.pinned.size())throw std::invalid_argument("Cloth placement size changed");
 auto current=s.cloth->getCurrentParticles();auto previous=s.cloth->getPreviousParticles();
 for(unsigned i=0;i<positions.size();i++){if(!Finite(positions[i]))throw std::invalid_argument("Nonfinite cloth placement");current[i]=previous[i]=physx::PxVec4(Vec(positions[i]),s.pinned[i]?0.f:1.f);}
 s.cloth->clearInertia();s.cloth->clearInterpolation();
}
void CpuCloth::Step(double seconds,const std::vector<Point>& pins,const std::vector<Capsule>& capsules,Point gravity,const std::vector<MotionLimit>& limits,const std::vector<MotionLimit>& backstops,const std::vector<std::array<Point,3>>& triangles){
 auto& s=*impl_;if(!s.cloth||pins.size()!=s.pinned.size()||!std::isfinite(seconds)||seconds<=0||seconds>1./30+1e-9||capsules.size()>16||!Finite(gravity))throw std::invalid_argument("Invalid cloth step");
 std::vector<physx::PxVec4> spheres;std::vector<uint32_t> indices;
 for(auto c:capsules){if(!Finite(c.a)||!Finite(c.b)||!std::isfinite(c.radius)||c.radius<=0)throw std::invalid_argument("Invalid cloth collider");indices.push_back(unsigned(spheres.size()));indices.push_back(unsigned(spheres.size()+1));spheres.emplace_back(Vec(c.a),float(c.radius));spheres.emplace_back(Vec(c.b),float(c.radius));}
 if(s.spheres.size()!=spheres.size()){
  s.cloth->setCapsules({},0,s.cloth->getNumCapsules());s.cloth->setSpheres(Range(spheres),0,s.cloth->getNumSpheres());
  s.cloth->setCapsules(Range(indices),0,0);s.cloth->clearInterpolation();
 }else s.cloth->setSpheres(Range(s.spheres),Range(spheres));
 s.spheres=spheres;
 std::vector<physx::PxVec3> trianglePoints;for(auto face:triangles)for(auto p:face){if(!Finite(p))throw std::invalid_argument("Nonfinite cloth collision triangle");trianglePoints.push_back(Vec(p));}
 if(s.triangles.size()==trianglePoints.size()&&!trianglePoints.empty())s.cloth->setTriangles(Range(s.triangles),Range(trianglePoints),0);
 else s.cloth->setTriangles(Range(trianglePoints),0,s.cloth->getNumTriangles());
 s.triangles=std::move(trianglePoints);
 if(!limits.empty()){
  if(limits.size()!=pins.size())throw std::invalid_argument("Cloth motion limit size changed");
  auto constraints=s.cloth->getMotionConstraints();
  for(unsigned i=0;i<limits.size();i++){if(!Finite(limits[i].center)||!std::isfinite(limits[i].radius)||limits[i].radius<0)throw std::invalid_argument("Invalid cloth motion limit");constraints[i]=physx::PxVec4(Vec(limits[i].center),float(limits[i].radius));}
 }else s.cloth->clearMotionConstraints();
 if(!backstops.empty()){
  if(backstops.size()!=pins.size())throw std::invalid_argument("Cloth backstop size changed");auto constraints=s.cloth->getSeparationConstraints();
  for(unsigned i=0;i<backstops.size();i++){if(!Finite(backstops[i].center)||!std::isfinite(backstops[i].radius)||backstops[i].radius<0)throw std::invalid_argument("Invalid cloth backstop");constraints[i]=physx::PxVec4(Vec(backstops[i].center),float(backstops[i].radius));}
 }else s.cloth->clearSeparationConstraints();
 {auto current=s.cloth->getCurrentParticles();for(unsigned i=0;i<pins.size();i++)if(s.pinned[i]){if(!Finite(pins[i]))throw std::invalid_argument("Nonfinite cloth pin");current[i]=physx::PxVec4(Vec(pins[i]),0);}}
 s.cloth->setGravity(Vec(gravity));
 if(s.solver->beginSimulation(float(seconds))){for(unsigned chunk=0;chunk<s.solver->getSimulationChunkCount();chunk++)s.solver->simulateChunk(chunk);s.solver->endSimulation();}
}
std::vector<Point> CpuCloth::Positions()const{
 if(!impl_->cloth)throw std::logic_error("Cloth is uninitialized");
 std::vector<Point> result;auto particles=impl_->cloth->getCurrentParticles();
 for(const auto& p:particles){Point point{p.x,p.y,p.z};if(!Finite(point))throw std::runtime_error("NvCloth generated nonfinite particles");result.push_back(point);}return result;
}
}
