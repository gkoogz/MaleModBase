#include <malemod/garments/pouch_cage.hpp>
#include <cstdio>
#include <limits>
#include <malemod/garments/pouch_budget.hpp>
using namespace malemod::garments::meridian;
int main(){try{
 constexpr unsigned cols=64,rows=40;std::vector<Vec> p(cols*rows+1);
 for(unsigned c=0;c<cols;c++){float a=c*6.28318531f/cols;p[c]={3*std::cos(a),2*std::sin(a),0};}
 unsigned first=unsigned(p.size());
 for(unsigned j=0;j<24;j++)for(unsigned c=0;c<32;c++){float h=j*3.14159265f/24,a=c*6.28318531f/32;p.push_back({2*std::sin(h)*std::cos(a)+h*.5f,std::sin(h)*std::sin(a),3-3*std::cos(h)});}
 auto raw=p;Vec tip={1.57f,0,6};auto r=FitPouchCage(p,cols,rows,first,unsigned(p.size()),tip);
 if(r.sections!=16||r.length<=6)throw std::runtime_error("Cage extent failed");
 for(unsigned c=0;c<cols;c++)if(p[c]!=raw[c])throw std::runtime_error("Attachment moved");
 for(auto q:p)for(float v:q)if(!std::isfinite(v))throw std::runtime_error("Nonfinite cage");
 auto moved=raw;Vec offset={11,-7,3};for(auto& q:moved)q=Add(q,offset);FitPouchCage(moved,cols,rows,first,unsigned(moved.size()),Add(tip,offset));
 for(unsigned i=0;i<=cols*rows;i++){auto d=Sub(moved[i],Add(p[i],offset));if(Dot(d,d)>1e-8f)throw std::runtime_error("Translation covariance failed");}
 for(unsigned c=0;c<cols;c++)for(unsigned j=2;j<rows;j++)if(p[j*cols+c][2]<p[(j-1)*cols+c][2])throw std::runtime_error("Reversed material row");
 auto rotate=[](Vec v)->Vec{return {v[2],v[0],v[1]};};auto rotated=raw;for(auto& q:rotated)q=rotate(q);
 FitPouchCage(rotated,cols,rows,first,unsigned(rotated.size()),rotate(tip));
 for(unsigned i=0;i<=cols*rows;i++){auto d=Sub(rotated[i],rotate(p[i]));if(Dot(d,d)>1e-8f)throw std::runtime_error("Rotation covariance failed");}
 for(unsigned bad=0;bad<3;bad++){
  auto invalid=raw;Vec badTip=tip;float nan=std::numeric_limits<float>::quiet_NaN();
  if(bad==0)invalid[0][0]=nan;else if(bad==1)invalid[first][0]=nan;else badTip[0]=nan;
  bool rejected=false;try{FitPouchCage(invalid,cols,rows,first,unsigned(invalid.size()),badTip);}catch(const std::runtime_error&){rejected=true;}
  if(!rejected)throw std::runtime_error("Nonfinite inputs accepted");
 }
 constexpr auto covered=malemod::garments::PouchContentsSimulationBudget(true),bare=malemod::garments::PouchContentsSimulationBudget(false);
 static_assert(covered.stepSeconds==2*bare.stepSeconds&&covered.maximumSteps*2==bare.maximumSteps,"Contents budget must preserve catch-up capacity");
 for(unsigned frame=0;frame<8;frame++){
  using malemod::garments::UpdatePouchCoveredSurface;
  if(UpdatePouchCoveredSurface(true,false,false,frame)!=(frame%2==0)||!UpdatePouchCoveredSurface(false,false,false,frame)||!UpdatePouchCoveredSurface(true,true,false,frame)||!UpdatePouchCoveredSurface(true,false,true,frame))throw std::runtime_error("Covered surface cadence bypass failed");
 }
 std::puts("PASS fixed seam, finite input rejection, rigid covariance, monotone rows and contents budget; native contact unverified");return 0;
 }catch(const std::exception&e){std::printf("FAIL %s\n",e.what());return 1;}}
