#pragma once
#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace malemod {
// Reusable bounded CPU workers for independent vertex work. Reductions remain
// explicitly ordered by the caller. Calls serialize; do not nest this arena.
class TaskArena {
 std::mutex submission_,mutex_;
 std::condition_variable work_,done_;
 std::vector<std::thread> workers_;
 std::function<void(unsigned,unsigned)> job_;
 unsigned count_=0,remaining_=0,lanes_=1;
 std::size_t generation_=0;
 bool stop_=false;
 std::exception_ptr error_;
 void Invoke(unsigned lane){
  try{job_(unsigned(std::uint64_t(count_)*lane/lanes_),unsigned(std::uint64_t(count_)*(lane+1)/lanes_));}
  catch(...){std::lock_guard<std::mutex> lock(mutex_);if(!error_)error_=std::current_exception();}
 }
 void Loop(unsigned lane){
  std::size_t seen=0;
  for(;;){
   {std::unique_lock<std::mutex> lock(mutex_);work_.wait(lock,[&]{return stop_||generation_!=seen;});if(stop_)return;seen=generation_;}
   Invoke(lane);
   {std::lock_guard<std::mutex> lock(mutex_);if(--remaining_==0)done_.notify_one();}
  }
 }
 void Stop(){
  {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}work_.notify_all();
  for(auto& worker:workers_)if(worker.joinable())worker.join();
 }
public:
 explicit TaskArena(unsigned lanes=4):lanes_(std::max(1u,std::min(lanes,8u))){
  try{for(unsigned i=1;i<lanes_;i++)workers_.emplace_back([this,i]{Loop(i);});}
  catch(...){Stop();throw;}
 }
 ~TaskArena(){Stop();}
 TaskArena(const TaskArena&)=delete;
 TaskArena& operator=(const TaskArena&)=delete;
 template<class Function> void For(unsigned count,Function&& function){
  std::lock_guard<std::mutex> submission(submission_);
  if(count<1024||lanes_==1){for(unsigned i=0;i<count;i++)function(i);return;}
  {std::lock_guard<std::mutex> lock(mutex_);count_=count;remaining_=unsigned(workers_.size());error_=nullptr;
   job_=[&](unsigned begin,unsigned end){for(unsigned i=begin;i<end;i++)function(i);};++generation_;}
  work_.notify_all();Invoke(0);
  {std::unique_lock<std::mutex> lock(mutex_);done_.wait(lock,[&]{return remaining_==0;});job_={};}
  if(error_)std::rethrow_exception(error_);
 }
};
}
