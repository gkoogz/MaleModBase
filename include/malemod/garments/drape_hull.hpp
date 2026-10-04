#pragma once
// A measured rest-tension envelope, never an anatomy or collision substitute.
namespace malemod::garments::drape {
struct HullSurface {std::vector<Sample> samples;std::vector<std::array<unsigned,3>> faces;};
inline HullSurface TensionHull(std::vector<Sample> points){
 if(points.size()<4)throw std::invalid_argument("Incomplete measured tension hull");
 for(const auto& sample:points){if(!Finite(sample.position)||!Finite(sample.normal))throw std::invalid_argument("Nonfinite measured tension support");double total=0;for(auto donor:sample.lineage.donors){if(!std::isfinite(donor.weight)||donor.weight<0)throw std::invalid_argument("Invalid measured tension donor");total+=donor.weight;}if(std::abs(total-1)>1e-5)throw std::invalid_argument("Measured tension donor weights do not sum to one");}
 unsigned a=0,b=0,c=0,d=0;for(unsigned i=1;i<points.size();i++)if(points[i].position[0]<points[a].position[0])a=i;
 double farthest=0;for(unsigned i=0;i<points.size();i++){double q=Dot(Sub(points[i].position,points[a].position),Sub(points[i].position,points[a].position));if(q>farthest){farthest=q;b=i;}}
 const double scale=std::sqrt(farthest),epsilon=scale*1e-10;if(!std::isfinite(scale)||scale<=1e-14)throw std::invalid_argument("Degenerate measured tension hull");
 auto axis=Sub(points[b].position,points[a].position);farthest=0;for(unsigned i=0;i<points.size();i++){auto q=Cross(axis,Sub(points[i].position,points[a].position));double square=Dot(q,q);if(square>farthest){farthest=square;c=i;}}
 if(std::sqrt(farthest)<=epsilon*scale)throw std::invalid_argument("Collinear measured tension hull");
 auto normal=Unit(Cross(axis,Sub(points[c].position,points[a].position)));farthest=0;for(unsigned i=0;i<points.size();i++){double q=std::abs(Dot(normal,Sub(points[i].position,points[a].position)));if(q>farthest){farthest=q;d=i;}}
 if(farthest<=epsilon)throw std::invalid_argument("Planar measured tension hull");
 Point interior=Mul(Add(Add(points[a].position,points[b].position),Add(points[c].position,points[d].position)),.25);
 struct Face{std::array<unsigned,3> ids;Point normal;};std::vector<Face> faces;
 auto make=[&](unsigned x,unsigned y,unsigned z){auto n=Cross(Sub(points[y].position,points[x].position),Sub(points[z].position,points[x].position));if(Dot(n,Sub(interior,points[x].position))>0){std::swap(y,z);n=Mul(n,-1);}return Face{{x,y,z},Unit(n)};};
 faces={make(a,b,c),make(a,d,b),make(b,d,c),make(c,d,a)};
 for(unsigned point=0;point<points.size();point++){
  if(point==a||point==b||point==c||point==d)continue;std::vector<unsigned char> visible(faces.size());bool outside=false;
  for(unsigned j=0;j<faces.size();j++)if(Dot(faces[j].normal,Sub(points[point].position,points[faces[j].ids[0]].position))>epsilon){visible[j]=1;outside=true;}
  if(!outside)continue;
  std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,unsigned>> boundary;
  std::vector<Face> next;next.reserve(faces.size()+8);for(unsigned j=0;j<faces.size();j++){if(!visible[j]){next.push_back(faces[j]);continue;}for(unsigned k=0;k<3;k++){unsigned x=faces[j].ids[k],y=faces[j].ids[(k+1)%3];auto key=std::minmax(x,y);auto at=boundary.find(key);if(at==boundary.end())boundary[key]={x,y};else boundary.erase(at);}}
  for(auto edge:boundary){auto x=edge.second.first,y=edge.second.second;auto cross=Cross(Sub(points[y].position,points[x].position),Sub(points[point].position,points[x].position));if(Length(cross)<=epsilon*scale)throw std::invalid_argument("Ill-conditioned measured tension hull horizon");next.push_back(make(x,y,point));}
  faces.swap(next);
 }
 HullSurface result;result.samples=std::move(points);for(auto face:faces)result.faces.push_back(face.ids);return result;
}
}
