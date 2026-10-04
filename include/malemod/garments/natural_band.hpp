#pragma once
// Shared garment styling and measured surface fitting. No assumed body units.
namespace malemod::garments::band_seating {
constexpr unsigned revision=1;
struct Profile {
 Point center{};
 double width=0;
 std::vector<std::pair<double,double>> transverseScale;
 double Angle(Point p)const{return std::atan2(p[0]-center[0],p[1]-center[1]);}
 // A smooth front dip and hip rise inferred from the supplied garment reference;
 // this is a garment design preference, not a universal anatomical waistline.
 double Height(Point p)const{double angle=Angle(p);return width*(-.225*std::cos(angle)-.175*std::cos(2*angle));}
 double WidthScale(Point p)const{
  if(transverseScale.empty())return 1;
  const double angle=Angle(p),pi=3.14159265358979323846;
  auto at=std::upper_bound(transverseScale.begin(),transverseScale.end(),angle,[](double a,const auto& b){return a<b.first;});
  auto right=at==transverseScale.end()?transverseScale.front():*at;
  auto left=at==transverseScale.begin()?transverseScale.back():*(at-1);
  if(at==transverseScale.end())right.first+=2*pi;
  if(at==transverseScale.begin())left.first-=2*pi;
  double span=right.first-left.first;
  if(span<=1e-12)throw std::invalid_argument("Duplicate angular waist seating guide");
  double t=(angle-left.first)/span;t=t*t*(3-2*t);
  return left.second*(1-t)+right.second*t;
 }
 double Offset(Point p,double row)const{return Height(p)+row*width*WidthScale(p);}
};
inline Profile Guide(const Input& input,double width){
 Profile profile;profile.width=width;
 if(input.waist.empty()||!std::isfinite(width)||width<=0)throw std::invalid_argument("Invalid measured waistband seating guide");
 for(auto s:input.waist)profile.center=Add(profile.center,input.frame.Local(s.position));
 profile.center=Mul(profile.center,1./input.waist.size());return profile;
}
inline void MeasureWidth(Profile& profile,const waist::Contour& middle,const Frame& frame){
 for(unsigned i=0;i<middle.points.size();i++){
  const auto& sample=middle.points[i];
  auto tangent=Sub(middle.points[(i+1)%middle.points.size()].position,middle.points[(i+middle.points.size()-1)%middle.points.size()].position);
  auto across=Unit(Cross(Unit(sample.normal),Unit(tangent)));
  double vertical=std::abs(Dot(across,frame.up));
  if(!std::isfinite(vertical)||vertical<1e-4)throw std::invalid_argument("Waist surface cannot support an ordered transverse garment strip");
  profile.transverseScale.emplace_back(profile.Angle(frame.Local(sample.position)),vertical);
 }
 std::sort(profile.transverseScale.begin(),profile.transverseScale.end());
 for(unsigned i=1;i<profile.transverseScale.size();i++)if(profile.transverseScale[i].first-profile.transverseScale[i-1].first<=1e-10)throw std::invalid_argument("Waist seating contour has ambiguous angular correspondence");
}
}
