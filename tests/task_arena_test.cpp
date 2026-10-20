#include <malemod/task_arena.hpp>
#include <atomic>
#include <iostream>
#include <stdexcept>
int main(){try{
 malemod::TaskArena arena(4);
 std::vector<unsigned> output(8000);
 for(unsigned run=0;run<30;run++){
  arena.For(unsigned(output.size()),[&](unsigned i){output[i]=i+run;});
  for(unsigned i=0;i<output.size();i++)if(output[i]!=i+run)throw std::runtime_error("Task lost or crossed a generation");
 }
 bool rejected=false;try{arena.For(8000,[](unsigned i){if(i==6000)throw std::runtime_error("expected");});}catch(const std::runtime_error&){rejected=true;}
 if(!rejected)throw std::runtime_error("Worker exception lost");
 arena.For(8000,[&](unsigned i){output[i]=i*2;});
 for(unsigned i=0;i<output.size();i++)if(output[i]!=i*2)throw std::runtime_error("Arena failed to recover after exception");
 std::atomic<unsigned> count{0};
 std::thread a([&]{arena.For(8000,[&](unsigned){++count;});});
 std::thread b([&]{arena.For(8000,[&](unsigned){++count;});});
 a.join();b.join();if(count!=16000)throw std::runtime_error("Concurrent submission lost work");
 std::cout<<"PASS task generations, worker exception, recovery and serialized callers\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
