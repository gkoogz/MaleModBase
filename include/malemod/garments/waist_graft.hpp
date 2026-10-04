#pragma once
// Actual joined tissue for transverse garment cuts. Classification caps never
// enter these cuts, and neither adapter draw nor physical contact buffers change.
namespace malemod::garments::waist {
struct JoinedSurface {
 std::vector<Sample> samples;
 std::vector<std::array<std::uint32_t,3>> triangles;
 std::size_t originalBodyVertices=0,originalAnatomyVertices=0,refinedBodyTriangles=0;
};
inline JoinedSurface JoinedPhysicalSurface(const Input& input,double circumference){
 if(input.bodySurface.empty()||input.bodyTriangles.empty()||input.anatomy.empty()||input.anatomyTriangles.empty())throw std::invalid_argument("Joined waist cut requires both actual tissue surfaces");
 auto cap=RootCap(input.anatomy,input.anatomyTriangles,input.opening,circumference);
 std::set<unsigned> originalRoot;for(auto face:cap)for(auto id:face)originalRoot.insert(id);
 JoinedSurface out;out.originalBodyVertices=input.bodySurface.size();out.originalAnatomyVertices=input.anatomy.size();out.samples=input.bodySurface;out.samples.insert(out.samples.end(),input.anatomy.begin(),input.anatomy.end());
 const auto offset=std::uint32_t(input.bodySurface.size());std::vector<std::uint32_t> joinedRoot;for(auto id:originalRoot)joinedRoot.push_back(offset+id);
 out.triangles=RefineClassificationBoundary(out.samples,input.bodyTriangles,joinedRoot,circumference);out.refinedBodyTriangles=out.triangles.size();
 for(auto face:input.anatomyTriangles){for(auto& id:face)id+=offset;out.triangles.push_back(face);}
 // The virtual cap above identified and validated the exact anatomical edge.
 // No cap triangle is appended to the actual joined cut surface.
 return out;
}
}
