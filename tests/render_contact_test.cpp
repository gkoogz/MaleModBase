#include <malemod/garments/jockstrap.hpp>
#include <cstdio>
using namespace malemod::garments;
namespace R=render_contact;
struct Binding {std::array<unsigned,4> nodes{};std::array<double,4> weights{};Point residual{};std::array<unsigned,3> materialFrame{};Point materialResidual{},restTangent{};bool transported=false,ribbon=false;};
static void Check(bool c,const char* why){if(!c)throw std::runtime_error(why);}
static bool Near(Point a,Point b,double tolerance=2e-7){return Length(Sub(a,b))<tolerance;}
static Point Rotate(Point p){return {p[2],p[0],p[1]};}
static Point GradientAt(const R::Gradient& g,unsigned node){for(unsigned i=0;i<g.count;i++)if(g.nodes[i]==node)return g.values[i];return {};}
static void FiniteDifference(const Binding& b,const std::vector<Point>& x,const Frame& frame,double scale,Point n){
 auto gradient=R::Derivative(b,x,frame,scale,n);double work=0,measured=0;auto moved=x;
 for(unsigned node=0;node<x.size();node++)for(unsigned a=0;a<3;a++){
  auto plus=x,minus=x;const double epsilon=1e-6;plus[node][a]+=epsilon;minus[node][a]-=epsilon;
  double actual=Dot(n,Sub(R::Evaluate(b,plus,frame,scale),R::Evaluate(b,minus,frame,scale)))/(2*epsilon);
  Check(std::abs(actual-GradientAt(gradient,node)[a])<2e-7,"Rendered material-frame gradient differs from finite difference");
  double displacement=(double(node)+1)*(double(a)-.4)*1e-7;moved[node][a]+=displacement;work+=GradientAt(gradient,node)[a]*displacement;
 }
 measured=Dot(n,Sub(R::Evaluate(b,moved,frame,scale),R::Evaluate(b,x,frame,scale)));Check(std::abs(work-measured)<1e-11,"Rendered contact violates virtual work");
 std::vector<double> mass(x.size(),1);auto application=R::Application(gradient,x,mass);double weights=0;for(auto w:b.weights)weights+=w;Check(Near(application.impulse,Mul(n,weights)),"Rendered contact nodal linear sum differs");
 if(b.transported)Check(Near(application.moment,Cross(R::Evaluate(b,x,frame,scale),n)),"Transported surface nodal angular sum differs from contact lever arm");
 Point angularIncrement{.2e-7,-.4e-7,.3e-7};auto spun=x;for(auto& p:spun)p=Add(p,Cross(angularIncrement,p));
 double angularWork=Dot(n,Sub(R::Evaluate(b,spun,frame,scale),R::Evaluate(b,x,frame,scale)));
 Check(std::abs(angularWork-Dot(application.moment,angularIncrement))<1e-13,"Actual nodal angular impulse violates rotational virtual work");
 auto translated=x;Point offset{82,-13,7};for(auto& p:translated)p=Add(p,offset);auto shifted=R::Derivative(b,translated,frame,scale,n);for(unsigned i=0;i<gradient.count;i++)Check(Near(gradient.values[i],GradientAt(shifted,gradient.nodes[i])),"Contact gradient changes under translation");
 auto shiftedApplication=R::Application(shifted,translated,mass);Check(Near(shiftedApplication.moment,Add(application.moment,Cross(offset,application.impulse))),"Nodal contact torque translation covariance");
 Frame rotated;rotated.lateral=Rotate(frame.lateral);rotated.forward=Rotate(frame.forward);rotated.up=Rotate(frame.up);auto rotatedX=x;for(auto& p:rotatedX)p=Rotate(p);auto rg=R::Derivative(b,rotatedX,rotated,scale,Rotate(n));
 Check(Near(R::Evaluate(b,rotatedX,rotated,scale),Rotate(R::Evaluate(b,x,frame,scale))),"Rendered rotation covariance");for(unsigned i=0;i<gradient.count;i++)Check(Near(GradientAt(rg,gradient.nodes[i]),Rotate(gradient.values[i])),"Contact derivative rotation covariance");
 auto scaledBinding=b;scaledBinding.residual=Mul(b.residual,100);scaledBinding.materialResidual[2]*=100;auto ug=R::Derivative(scaledBinding,x,frame,scale*100,n);for(unsigned i=0;i<gradient.count;i++)Check(Near(gradient.values[i],GradientAt(ug,gradient.nodes[i])),"Contact gradient unit covariance");
 Check(std::isfinite(gradient.InverseMass(mass))&&gradient.InverseMass(mass)>0,"Rendered effective mass absent");
 mass[gradient.nodes[0]]=0;auto free=R::Application(gradient,x,mass);Check(Near(free.impulse,Sub(application.impulse,gradient.values[0])),"Pinned material support leaked into free particle impulse");
}
int main(){try{
 Frame frame;Point n=Unit({.4,-.3,.8});std::vector<Point> x{{0,0,0},{1,.4,.2},{.1,1,.3},{.3,.5,.7},{-.2,.1,.4}};
 Binding b;b.nodes={0,1,3,4};b.weights={.15,.2,.4,.25};b.materialFrame={0,1,2};b.transported=true;b.materialResidual={.2,-.13,.17};FiniteDifference(b,x,frame,2,n);
 b.transported=false;b.ribbon=true;b.restTangent={1,0,0};b.residual={0,.2,.1};for(double angle:{0.,.2,1.,2.5}){x[1]={std::cos(angle),std::sin(angle),.15};FiniteDifference(b,x,frame,2,n);}
 b.ribbon=false;b.residual={};FiniteDifference(b,x,frame,2,n);
 auto a=R::Derivative(b,x,frame,2,n),merged=a;R::Merge(merged,a,.4);for(unsigned i=0;i<a.count;i++)Check(Near(merged.values[i],Mul(a.values[i],1.4)),"Fine-face contact merge loses repeated material nodes");
 std::puts("PASS exact rendered scalar contact gradients, finite differences, virtual work, nodal linear/angular sums, pin exclusion, translation/rotation/units and fine-face merged support");return 0;
 }catch(const std::exception& error){std::fprintf(stderr,"%s\n",error.what());return 1;}}
