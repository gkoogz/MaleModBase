#pragma once
// Scalar contact derivatives of the actual rendered cloth surface. Material
// frame nodes participate even when they are absent from translational donors.
namespace malemod::garments::render_contact {
struct Gradient {
 std::array<unsigned,21> nodes{};std::array<Point,21> values{};unsigned count=0;
 void Add(unsigned node,Point value){if(!Finite(value))throw std::invalid_argument("Nonfinite rendered contact gradient");unsigned i=0;while(i<count&&nodes[i]!=node)i++;if(i==count){if(count==nodes.size())throw std::invalid_argument("Rendered contact support capacity");nodes[count++]=node;}auto next=garments::Add(values[i],value);if(!Finite(next))throw std::invalid_argument("Unstable merged rendered contact gradient");values[i]=next;}
 double InverseMass(const std::vector<double>& mass)const{double out=0;for(unsigned i=0;i<count;i++){double m=mass.at(nodes[i]);if(!std::isfinite(m)||m<0)throw std::invalid_argument("Invalid rendered contact inverse mass");out+=m*Dot(values[i],values[i]);}if(!std::isfinite(out))throw std::invalid_argument("Unstable rendered contact effective mass");return out;}
};
inline Point RestVector(const Frame& frame,Point p){return Add(Add(Mul(frame.lateral,p[0]),Mul(frame.forward,p[1])),Mul(frame.up,p[2]));}
template<class Binding> inline Point Evaluate(const Binding& b,const std::vector<Point>& x,const Frame& frame,double scale){
 Point p=Mul(RestVector(frame,b.residual),1/scale);
 if(b.ribbon){auto from=RestVector(frame,b.restTangent),to=Sub(x.at(b.materialFrame[1]),x.at(b.materialFrame[0]));if(Length(to)>1e-14){to=Unit(to);double cosine=std::clamp(Dot(from,to),-1.,1.);auto axis=Cross(from,to);if(cosine>-.999999)p=Add(Add(p,Cross(axis,p)),Mul(Cross(axis,Cross(axis,p)),1/(1+cosine)));else{axis=Sub(p,Mul(from,Dot(p,from)));if(Length(axis)>1e-14){axis=Unit(axis);p=Sub(Mul(axis,2*Dot(axis,p)),p);}}}}
 else if(b.transported){auto a=x.at(b.materialFrame[0]),u=Sub(x.at(b.materialFrame[1]),a),v=Sub(x.at(b.materialFrame[2]),a),cross=Cross(u,v);if(Length(u)>1e-14&&Length(cross)>1e-14)p=Add(Add(Mul(u,b.materialResidual[0]),Mul(v,b.materialResidual[1])),Mul(Unit(cross),b.materialResidual[2]/scale));}
 for(unsigned k=0;k<4;k++)if(b.weights[k])p=Add(p,Mul(x.at(b.nodes[k]),b.weights[k]));return p;
}
template<class Binding> inline Gradient Derivative(const Binding& b,const std::vector<Point>& x,const Frame& frame,double scale,Point direction){
 if(!Finite(direction)||!std::isfinite(scale)||scale<=0)throw std::invalid_argument("Rendered contact gradient input");
 Gradient out;for(unsigned k=0;k<4;k++)if(b.weights[k])out.Add(b.nodes[k],Mul(direction,b.weights[k]));
 if(b.ribbon){
  auto from=RestVector(frame,b.restTangent),difference=Sub(x.at(b.materialFrame[1]),x.at(b.materialFrame[0]));double length=Length(difference);
  if(length>1e-14){auto to=Mul(difference,1/length),p=Mul(RestVector(frame,b.residual),1/scale);double cosine=std::clamp(Dot(from,to),-1.,1.);
   if(cosine>-.999999){auto axis=Cross(from,to),twice=Cross(axis,Cross(axis,p));Point gradient{};
    for(unsigned j=0;j<3;j++){Point unit{};unit[j]=1;auto dto=Mul(Sub(unit,Mul(to,to[j])),1/length),daxis=Cross(from,dto);double dc=Dot(from,dto);auto derivative=Add(Cross(daxis,p),Sub(Mul(Add(Cross(daxis,Cross(axis,p)),Cross(axis,Cross(daxis,p))),1/(1+cosine)),Mul(twice,dc/((1+cosine)*(1+cosine)))));gradient[j]=Dot(direction,derivative);}
    out.Add(b.materialFrame[0],Mul(gradient,-1));out.Add(b.materialFrame[1],gradient);
   }
   // The exact antiparallel renderer uses the prescribed rest residual axis;
   // that branch is locally constant, so its derivative is zero. The branch
   // discontinuity is disclosed and independently tested away from its switch.
  }
 }else if(b.transported){
  auto a=x.at(b.materialFrame[0]),u=Sub(x.at(b.materialFrame[1]),a),v=Sub(x.at(b.materialFrame[2]),a),cross=Cross(u,v);double length=Length(cross);
  if(Length(u)>1e-14&&length>1e-14){auto n=Mul(cross,1/length);Point gu{},gv{};
   for(unsigned j=0;j<3;j++){Point unit{};unit[j]=1;auto du=Cross(unit,v),dv=Cross(u,unit);du=Mul(Sub(du,Mul(n,Dot(n,du))),1/length);dv=Mul(Sub(dv,Mul(n,Dot(n,dv))),1/length);gu[j]=Dot(direction,Add(Mul(unit,b.materialResidual[0]),Mul(du,b.materialResidual[2]/scale)));gv[j]=Dot(direction,Add(Mul(unit,b.materialResidual[1]),Mul(dv,b.materialResidual[2]/scale)));}
   out.Add(b.materialFrame[0],Mul(Add(gu,gv),-1));out.Add(b.materialFrame[1],gu);out.Add(b.materialFrame[2],gv);
  }
 }
 return out;
}
inline void Merge(Gradient& into,const Gradient& from,double weight){for(unsigned i=0;i<from.count;i++)into.Add(from.nodes[i],Mul(from.values[i],weight));}
struct FreeApplication {Point impulse{},moment{};};
inline FreeApplication Application(const Gradient& gradient,const std::vector<Point>& x,const std::vector<double>& inverseMass){FreeApplication out;for(unsigned i=0;i<gradient.count;i++)if(inverseMass.at(gradient.nodes[i])>0){out.impulse=Add(out.impulse,gradient.values[i]);out.moment=Add(out.moment,Cross(x.at(gradient.nodes[i]),gradient.values[i]));}return out;}
}
