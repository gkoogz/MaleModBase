#pragma once
// One physical waistband midsurface; the original inner/outer render vertices
// keep their authored finite offsets. This changes neither measured skin donors
// nor the static fitted band. All director nodes remain contact participants.
namespace malemod::garments::band_material {
inline Point Midsurface(Point outer,Point inner){
 if(!Finite(outer)||!Finite(inner))throw std::invalid_argument("Nonfinite waistband layer");
 return Add(outer,Mul(Sub(inner,outer),.5));
}
template<class Binding> inline void Director(Binding& binding,unsigned vertex,unsigned columns,
 const std::vector<unsigned>& vertexNodes,const std::vector<Point>& positions,
 const Frame& frame,double circumference){
 if(!columns||!std::isfinite(circumference)||circumference<=0)throw std::invalid_argument("Invalid waistband director scale");
 const unsigned layer=7*(columns+1),material=vertex%layer,row=material/(columns+1),col=material%(columns+1);
 if(vertex>=2*layer)throw std::invalid_argument("Waistband director vertex outside layers");
 const unsigned side=col==columns?1:(col+1)%columns,vertical=row==6?5:row+1;
 const unsigned aNode=vertexNodes.at(material),uNode=vertexNodes.at(row*(columns+1)+side),vNode=vertexNodes.at(vertical*(columns+1)+col);
 const auto a=positions.at(aNode),u=Sub(positions.at(uNode),a),v=Sub(positions.at(vNode),a),cross=Cross(u,v);
 if(!Finite(a)||!Finite(u)||!Finite(v)||!Finite(binding.residual))throw std::invalid_argument("Nonfinite waistband material frame");
 const double uu=Dot(u,u),uv=Dot(u,v),vv=Dot(v,v),det=uu*vv-uv*uv;
 if(det<=std::pow(circumference,4)*1e-24||Length(cross)<circumference*circumference*1e-12)throw std::invalid_argument("Degenerate waistband material director");
 const auto offset=render_contact::RestVector(frame,binding.residual);
 binding.materialFrame={aNode,uNode,vNode};
 binding.materialResidual={(vv*Dot(offset,u)-uv*Dot(offset,v))/det,(uu*Dot(offset,v)-uv*Dot(offset,u))/det,Dot(offset,Unit(cross))};
 binding.transported=true;
}
}
