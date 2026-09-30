#pragma once
#include <xmmintrin.h>
#include <ppl.h>
// Bounded synchronous batches. Each callback owns distinct output vertices;
// constraints, reductions, D3D calls and simulation state remain on the caller.
// The scheduler is process-lifetime: never wait for workers from DllMain.
static Concurrency::Scheduler* GeometryScheduler(){
 static Concurrency::Scheduler* scheduler=Concurrency::Scheduler::Create(
  Concurrency::SchedulerPolicy(2,Concurrency::MaxConcurrency,4,Concurrency::MinConcurrency,1));
 return scheduler;
}
template<class Function> static void GeometryFor(unsigned count,const Function& fn){
 if(count<512){for(unsigned i=0;i<count;i++)fn(i);return;}
 GeometryScheduler()->Attach();
 struct Detach {~Detach(){Concurrency::CurrentScheduler::Detach();}} detach;
 const unsigned mxcsr=_mm_getcsr();
 // More batches than workers balances cheap retained vertices against the
 // denser edited region without changing any vertex's arithmetic order.
 constexpr unsigned batches=16;
 Concurrency::parallel_for(0u,batches,[&](unsigned batch){
  const unsigned saved=_mm_getcsr();_mm_setcsr(mxcsr);
  unsigned begin=count*batch/batches,end=count*(batch+1)/batches;
  for(unsigned i=begin;i<end;i++)fn(i);
  _mm_setcsr(saved);
 });
}

// Prepared arithmetic, not a rotation matrix: retain the scalar operation
// order so existing authored surfaces and packed normals stay reproducible.
struct GeometryRotation {
  V3 axis;float s,c;
  GeometryRotation(V3 from,V3 to){
    from=Unit(from);to=Unit(to);axis=Cross(from,to);s=Length(axis);
    c=max(-1.f,min(1.f,Dot(from,to)));
  }
  V3 Apply(V3 value) const {
    if(s<1e-5f)return c>0?value:value*-1.f;
    return value*c+Cross(axis,value)+axis*(Dot(axis,value)*(1.f-c)/(s*s));
  }
};

// A topology-bound Jacobi pass. Only writable rows are traversed; immutable
// neighbors remain available as boundary values. XYZ share SIMD instructions.
// Keep pass/neighbor order and division (no reciprocal approximation).
template<unsigned N> struct GeometryFairPass {
  struct Row {unsigned id,begin,end;float count,weight;};
  std::vector<Row> rows;std::vector<unsigned> neighbors;
  __m128 points[N],next[N];
  template<class Index> void Add(unsigned id,const Index* begin,const Index* end,float weight){
    if(begin==end||weight==0.f)return;
    Row row{id,unsigned(neighbors.size()),0,float(end-begin),weight};
    neighbors.insert(neighbors.end(),begin,end);row.end=unsigned(neighbors.size());rows.push_back(row);
  }
  void Apply(V3* target,unsigned passes,float strength=1.f){
    for(unsigned i=0;i<N;i++)points[i]=_mm_set_ps(0.f,target[i].z,target[i].y,target[i].x);
    for(unsigned pass=0;pass<passes;pass++){
      for(unsigned k=0;k<rows.size();k++){
        const Row& row=rows[k];__m128 mean=_mm_setzero_ps();
        for(unsigned j=row.begin;j<row.end;j++)mean=_mm_add_ps(mean,points[neighbors[j]]);
        next[k]=_mm_add_ps(points[row.id],_mm_mul_ps(_mm_sub_ps(_mm_div_ps(mean,_mm_set1_ps(row.count)),points[row.id]),_mm_set1_ps(strength*row.weight)));
      }
      for(unsigned k=0;k<rows.size();k++)points[rows[k].id]=next[k];
    }
    for(const Row& row:rows){float p[4];_mm_storeu_ps(p,points[row.id]);target[row.id]={p[0],p[1],p[2]};}
  }
};
