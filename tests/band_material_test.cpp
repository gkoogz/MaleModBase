#include <malemod/garments/jockstrap.hpp>
#include <cstdio>
using namespace malemod::garments;
namespace R=render_contact;
struct Binding {std::array<unsigned,4> nodes{};std::array<double,4> weights{};Point residual{};std::array<unsigned,3> materialFrame{};Point materialResidual{},restTangent{};bool transported=false,ribbon=false;};
static void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static Point GradientAt(const R::Gradient& g,unsigned node){for(unsigned k=0;k<g.count;k++)if(g.nodes[k]==node)return g.values[k];return {};}
static Point Rotate(Point p){return {p[2],p[0],p[1]};}
static void Mechanics(Binding b,std::vector<Point> x,double C){
 Frame frame;Point n=Unit({.3,-.7,.5});auto gradient=R::Derivative(b,x,frame,C,n);
 for(unsigned k=0;k<gradient.count;k++)for(unsigned axis=0;axis<3;axis++){
  auto plus=x,minus=x;plus[gradient.nodes[k]][axis]+=1e-6;minus[gradient.nodes[k]][axis]-=1e-6;
  double actual=Dot(n,Sub(R::Evaluate(b,plus,frame,C),R::Evaluate(b,minus,frame,C)))/2e-6;
  Check(std::abs(actual-gradient.values[k][axis])<3e-8,"Band layer contact omits a director derivative");
 }
 std::vector<double> inverseMass(x.size(),1);auto applied=R::Application(gradient,x,inverseMass);
 Check(Length(Sub(applied.impulse,n))<1e-10,"Band layer linear impulse is not conserved");
 Check(Length(Sub(applied.moment,Cross(R::Evaluate(b,x,frame,C),n)))<1e-10,"Band layer angular impulse is not conserved");
 auto perturbed=x;double predicted=0;for(unsigned k=0;k<gradient.count;k++){Point d{double(k+1)*1e-8,-.7e-8,.5e-8};perturbed[gradient.nodes[k]]=Add(perturbed[gradient.nodes[k]],d);predicted+=Dot(gradient.values[k],d);}
 Check(std::abs(predicted-Dot(n,Sub(R::Evaluate(b,perturbed,frame,C),R::Evaluate(b,x,frame,C))))<1e-13,"Band layer virtual work mismatch");
 Frame rotated;rotated.lateral=Rotate(frame.lateral);rotated.forward=Rotate(frame.forward);rotated.up=Rotate(frame.up);auto turned=x;Point shift{13,-81,3};for(auto& p:turned)p=Add(Rotate(p),shift);auto rg=R::Derivative(b,turned,rotated,C,Rotate(n));
 for(unsigned k=0;k<gradient.count;k++)Check(Length(Sub(Rotate(gradient.values[k]),GradientAt(rg,gradient.nodes[k])))<1e-10,"Band contact frame covariance");
 Check(Length(Sub(R::Evaluate(b,turned,rotated,C),Add(Rotate(R::Evaluate(b,x,frame,C)),shift)))<1e-10,"Band layers do not rotate with material");
 b.residual=Mul(b.residual,100);b.materialResidual[2]*=100;auto ug=R::Derivative(b,x,frame,C*100,n);for(unsigned k=0;k<gradient.count;k++)Check(Length(Sub(gradient.values[k],GradientAt(ug,gradient.nodes[k])))<1e-10,"Band contact unit covariance");
 // Both evaluator and derivative intentionally use the same finite fallback
 // when the material director loses rank. No undefined normal is introduced.
 x[b.materialFrame[2]]=x[b.materialFrame[1]];gradient=R::Derivative(b,x,frame,C*100,n);Check(gradient.count==1&&Length(Sub(gradient.values[0],n))<1e-12&&Finite(R::Evaluate(b,x,frame,C*100)),"Band degenerate director fallback disagrees");
}
int main(){try{
 const unsigned columns=12,layer=7*(columns+1);const double C=6.4;Frame frame;
 std::vector<Point> mid(layer),outer(layer),inner(layer);std::vector<unsigned> nodes(2*layer);
 for(unsigned row=0;row<7;row++)for(unsigned col=0;col<=columns;col++){
  const unsigned i=row*(columns+1)+col;double theta=double(col%columns)/columns*2*3.141592653589793;
  mid[i]={std::cos(theta)*(1+.025*row),std::sin(theta)*(.8+.02*row),.15*row+.12*std::cos(theta)};
  Point offset=Mul(Unit({std::cos(theta),std::sin(theta),-.12*std::cos(theta)}),.008);outer[i]=Add(mid[i],offset);inner[i]=Sub(mid[i],offset);
  nodes[i]=nodes[i+layer]=row*(columns+1)+(col%columns);
  Check(Length(Sub(band_material::Midsurface(outer[i],inner[i]),mid[i]))<1e-13,"Band midpoint changed authored paired geometry");
 }
 for(unsigned i=0;i<2*layer;i++){
  Binding b;b.nodes[0]=nodes[i];b.weights[0]=1;Point target=i<layer?outer[i]:inner[i-layer];b.residual=Sub(target,mid[nodes[i]]);
  band_material::Director(b,i,columns,nodes,mid,frame,C);auto normalized=mid;for(auto& p:normalized)p=Mul(p,1/C);
  Check(Length(Sub(Mul(R::Evaluate(b,normalized,frame,C),C),target))<1e-12,"Band director fails exact rest layer reconstruction");
  if(i%(columns+1)==3){Mechanics(b,normalized,C);auto pivot=normalized[b.materialFrame[0]];normalized[b.materialFrame[1]]=Add(pivot,Rotate(Sub(normalized[b.materialFrame[1]],pivot)));normalized[b.materialFrame[2]]=Add(pivot,Rotate(Sub(normalized[b.materialFrame[2]],pivot)));Mechanics(b,normalized,C);}
  Frame moved;Point shift{500,-17,83};moved.origin=shift;moved.lateral=Rotate(frame.lateral);moved.forward=Rotate(frame.forward);moved.up=Rotate(frame.up);auto measured=mid;for(auto& p:measured)p=Add(Mul(Rotate(p),100),shift);Binding unitBinding=b;unitBinding.residual=Mul(b.residual,100);band_material::Director(unitBinding,i,columns,nodes,measured,moved,C*100);Check(std::abs(unitBinding.materialResidual[0]-b.materialResidual[0])<1e-10&&std::abs(unitBinding.materialResidual[1]-b.materialResidual[1])<1e-10&&std::abs(unitBinding.materialResidual[2]-b.materialResidual[2]*100)<1e-10,"Band authored director unit/frame covariance");
 }
 bool rejected=false;try{band_material::Midsurface({NAN,0,0},{});}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Band accepts nonfinite layer");
 Binding b;b.residual={0,0,.01};std::fill(mid.begin(),mid.end(),Point{});rejected=false;try{band_material::Director(b,0,columns,nodes,mid,frame,C);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Band accepts zero-area rest director");
 std::puts("PASS one band midsurface, paired layers and aliases, exact rest reconstruction, both-layer finite gradients/virtual work, extreme tilt, impulse/moment, units and degenerate fallback");return 0;
 }catch(const std::exception& error){std::fprintf(stderr,"%s\n",error.what());return 1;}}
