#pragma once
// MIT. Continuous swept surface; no sphere splats or screen-space reconstruction.
namespace teaching {
struct LiquidVertex {V3 p,n;};
struct LiquidMesh {
 std::vector<LiquidVertex> vertices;std::vector<unsigned> indices;
 unsigned opaqueIndices=0;int components=0;
 void Clear(){vertices.clear();indices.clear();opaqueIndices=0;components=0;}
 void Triangle(unsigned a,unsigned b,unsigned c){indices.push_back(a);indices.push_back(b);indices.push_back(c);}
 void Tube(const std::vector<V3>& points,const std::vector<float>& radii,int sides,float targetVolume=0){
  if(points.size()<2||points.size()!=radii.size())return;
  std::vector<V3> centers,tangents;std::vector<float> widths;
  auto add=[&](V3 p,float r){if(centers.empty()||Length(p-centers.back())>1e-5f){centers.push_back(p);widths.push_back(max(.00001f,r));}};
  // Rounded ends belong to the same indexed surface as the stream.
  V3 start=Unit(points[0]-points[1]),end=Unit(points.back()-points[points.size()-2]);
  float ra=radii[0],rb=radii.back();
  add(points[0]+start*ra*.924f,ra*.383f);add(points[0]+start*ra*.707f,ra*.707f);add(points[0]+start*ra*.383f,ra*.924f);
  for(size_t i=0;i+1<points.size();i++){
   V3 a=points[i],b=points[i+1],delta=b-a;
   V3 ta=i?Unit(b-points[i-1])*Length(delta):delta;
   V3 tb=i+2<points.size()?Unit(points[i+2]-a)*Length(delta):delta;
   // Three Hermite intervals remove polygonal corners without noisy radius ripples.
   for(int k=0;k<3;k++){float t=k/3.f,t2=t*t,t3=t2*t;V3 p=a*(2*t3-3*t2+1)+ta*(t3-2*t2+t)+b*(-2*t3+3*t2)+tb*(t3-t2);add(p,radii[i]+(radii[i+1]-radii[i])*t);}
  }
  add(points.back(),rb);add(points.back()+end*rb*.383f,rb*.924f);add(points.back()+end*rb*.707f,rb*.707f);add(points.back()+end*rb*.924f,rb*.383f);
  if(centers.size()<2)return;
  unsigned base=(unsigned)vertices.size(),ib=(unsigned)indices.size();
  V3 normal{};
  for(size_t i=0;i<centers.size();i++){
   V3 tangent=Unit(centers[min(i+1,centers.size()-1)]-centers[i?i-1:0]);
   // Rotation-minimizing frame: project the previous normal into the new plane.
   if(!i)normal=Unit(Cross(tangent,fabsf(tangent.z)<.85f?V3{0,0,1}:V3{0,1,0}));
   else {normal=normal-tangent*Dot(normal,tangent);if(Length(normal)<.01f)normal=Cross(tangent,V3{0,1,0});normal=Unit(normal);}
   V3 binormal=Cross(tangent,normal);
   for(int j=0;j<sides;j++){float a=j*6.283185307f/sides;V3 radial=normal*cosf(a)+binormal*sinf(a);vertices.push_back({centers[i]+radial*widths[i],{}});}
   if(i)for(int j=0;j<sides;j++){unsigned a=base+(unsigned)(i-1)*sides+j,b=base+(unsigned)(i-1)*sides+(j+1)%sides,c=a+sides,d=b+sides;Triangle(a,b,c);Triangle(b,d,c);}
  }
  unsigned head=(unsigned)vertices.size();vertices.push_back({points[0]+start*ra,{}});unsigned tail=(unsigned)vertices.size();vertices.push_back({points.back()+end*rb,{}});
  unsigned last=base+((unsigned)centers.size()-1)*sides;
  for(int j=0;j<sides;j++){Triangle(head,base+(j+1)%sides,base+j);Triangle(tail,last+j,last+(j+1)%sides);}
  // Align winding consistently; geometric normals then include taper and necking.
  double signedVolume=0;V3 origin=points[0];for(unsigned i=ib;i<indices.size();i+=3)signedVolume+=Dot(vertices[indices[i]].p-origin,Cross(vertices[indices[i+1]].p-origin,vertices[indices[i+2]].p-origin))/6.;
  if(signedVolume<0)for(unsigned i=ib;i<indices.size();i+=3)std::swap(indices[i+1],indices[i+2]);
  // Correct the cap/interpolation volume by scaling radial offsets only.
  if(targetVolume>0&&fabs(signedVolume)>1e-10){float scale=sqrtf(targetVolume/(float)fabs(signedVolume));scale=Clamp(scale,.2f,3.f);for(size_t i=0;i<centers.size();i++)for(int j=0;j<sides;j++){auto& p=vertices[base+(unsigned)i*sides+j].p;p=centers[i]+(p-centers[i])*scale;}}
  for(unsigned i=ib;i<indices.size();i+=3){auto& a=vertices[indices[i]];auto& b=vertices[indices[i+1]];auto& c=vertices[indices[i+2]];V3 n=Cross(b.p-a.p,c.p-a.p);a.n=a.n+n;b.n=b.n+n;c.n=c.n+n;}
  for(unsigned i=base;i<vertices.size();i++)vertices[i].n=Unit(vertices[i].n);
  ++components;
 }
};
}
