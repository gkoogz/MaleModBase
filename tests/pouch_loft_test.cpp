#include <malemod/garments/pouch_loft.hpp>
#include <cstdio>
using namespace malemod::garments::meridian;
int main(){try{
 constexpr unsigned columns=32,rows=24;std::vector<Vec> p(columns*rows+1);
 for(unsigned c=0;c<columns;c++){float a=c*6.28318531f/columns;p[c]={3*std::cos(a),2*std::sin(a),0};}
 unsigned first=unsigned(p.size());
 for(unsigned j=0;j<=24;j++)for(unsigned c=0;c<32;c++){float h=j*3.14159265f/24,a=c*6.28318531f/32;p.push_back({2*std::sin(h)*std::cos(a),std::sin(h)*std::sin(a),3-3*std::cos(h)});}
 auto raw=p;auto receipt=FitPouchLoft(p,columns,rows,first,unsigned(p.size()));
 if(receipt.sections!=16||receipt.length<6)throw std::runtime_error("Support extent lost");
 for(unsigned c=0;c<columns;c++)if(p[c]!=raw[c])throw std::runtime_error("Sewn edge moved");
 for(auto q:p)for(float v:q)if(!std::isfinite(v))throw std::runtime_error("Nonfinite loft");
 auto moved=raw;Vec offset={11,-7,3};for(auto& q:moved)q=Add(q,offset);FitPouchLoft(moved,columns,rows,first,unsigned(moved.size()));
 for(unsigned i=0;i<=columns*rows;i++){auto d=Sub(moved[i],Add(p[i],offset));if(Dot(d,d)>1e-8f)throw std::runtime_error("Translation covariance failed");}
 std::puts("PASS: exact sewn edge, finite loft and translation covariance; continuous contact unverified");return 0;
 }catch(const std::exception&e){std::printf("FAIL: %s\n",e.what());return 1;}}
