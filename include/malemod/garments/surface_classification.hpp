#pragma once
// Virtual measured weld closure disambiguates sign; it is never a contact surface.
namespace malemod::garments {
inline void Session::ClassifySurfaces(const Input& input,Point origin,double scale,bool rebuildCap){
 if(input.anatomyTriangles.empty()){closedAnatomy_.Clear();closedBody_.Clear();rootCap_.clear();rootVertices_.clear();return;}
 if(rebuildCap||closedAnatomyFaces_.empty()){
  rootCap_=RootCap(input.anatomy,input.anatomyTriangles,input.opening,scale);
  std::set<unsigned> vertices;for(auto f:rootCap_)for(auto id:f)vertices.insert(id);rootVertices_.assign(vertices.begin(),vertices.end());
  closedAnatomyFaces_=input.anatomyTriangles;closedAnatomyFaces_.insert(closedAnatomyFaces_.end(),rootCap_.begin(),rootCap_.end());
  closedBodyFaces_=input.bodyTriangles;std::map<unsigned,unsigned> index;for(unsigned i=0;i<rootVertices_.size();i++)index[rootVertices_[i]]=unsigned(input.bodySurface.size())+i;
  if(!input.bodySurface.empty()){
   auto body=input.bodySurface;for(auto id:rootVertices_)body.push_back(input.anatomy[id]);std::vector<unsigned> rootIndices;for(unsigned i=0;i<rootVertices_.size();i++)rootIndices.push_back(unsigned(input.bodySurface.size())+i);
   std::vector<RootSubdivision> subdivisions;std::set<unsigned> seen;
   for(auto s:input.rootSubdivisions){if(!index.count(s.vertex)||!index.count(s.a)||!index.count(s.b)||!seen.insert(s.vertex).second)throw std::invalid_argument("Authored root subdivision is not in the measured anatomical opening");subdivisions.push_back({index.at(s.vertex),index.at(s.a),index.at(s.b),s.t});}
   if(!rootIndices.empty())closedBodyFaces_=RefineClassificationBoundary(body,closedBodyFaces_,rootIndices,scale,subdivisions);
   for(auto f:rootCap_)closedBodyFaces_.push_back({index[f[2]],index[f[1]],index[f[0]]});
   auto remote=SurfaceCaps(body,closedBodyFaces_,scale);closedBodyFaces_.insert(closedBodyFaces_.end(),remote.begin(),remote.end());
  }
 }
 closedAnatomy_.Update(input.anatomy,closedAnatomyFaces_,origin,scale);
 // Classification consumes positions only. Do not clone every native donor
 // vector when appending the virtual root closure on each numerical substep.
 if(input.bodySurface.empty())closedBody_.Clear();else {auto& body=classificationBody_;body.resize(input.bodySurface.size()+rootVertices_.size());for(unsigned i=0;i<input.bodySurface.size();i++)body[i].position=input.bodySurface[i].position;for(unsigned i=0;i<rootVertices_.size();i++)body[input.bodySurface.size()+i].position=input.anatomy[rootVertices_[i]].position;closedBody_.Update(body,closedBodyFaces_,origin,scale);}
}
inline double Session::ClassifiedDistance(detail::BodyCollider::Hit& hit,Point point,bool anatomy)const{
 const auto& closure=anatomy?closedAnatomy_:closedBody_;if(closure.Empty())return hit.signedDistance;
 auto side=closure.Classify(point,true);
 if(side==detail::BodyCollider::Side::Indeterminate)throw std::invalid_argument("Verified closed garment surface membership is indeterminate");
 if(side==detail::BodyCollider::Side::Boundary)return 0;
 const bool outside=side==detail::BodyCollider::Side::Outside;auto separation=outside?Sub(point,hit.point):Sub(hit.point,point);if(Length(separation)>1e-14)hit.normal=Unit(separation);
 return outside?hit.distance:-hit.distance;
}
inline double Session::ClassifiedDistance(const detail::BodyCollider::Hit& hit,Point point,bool anatomy)const{auto corrected=hit;return ClassifiedDistance(corrected,point,anatomy);}
}
