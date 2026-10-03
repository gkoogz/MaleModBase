#include <malemod/surface/garment_support.hpp>
#include <malemod/garments/numerical_support.hpp>
#include <cstdio>
using namespace malemod;
int main(){
 auto x=surface::BoundGarmentAcceleration({0,0,6.05f},110);
 if(x.z!=6.05f)return 1;
 auto y=surface::BoundGarmentAcceleration({0,0,6.05f},5);
 if(std::abs(y.z-.75f)>1e-6f)return 2;
 auto z=surface::BoundGarmentAcceleration({8,8,8},72);
 if(std::abs(Length(z)-10.8f)>2e-6f)return 3;
 auto zero=surface::BoundGarmentAcceleration({1,2,3},0);if(Length(zero)!=0)return 4;
 try{surface::BoundGarmentAcceleration({NAN,0,0},72);return 5;}catch(const std::invalid_argument&){}
 garments::Output cloth;cloth.style=garments::Style::WhiteJockstrap;
 garments::Support contact;contact.acceleration={0,0,6.05};contact.influence=2;contact.lineage.donors[0]={garments::Surface::Anatomy,3,.25};contact.lineage.donors[1]={garments::Surface::Anatomy,4,.75};cloth.support={contact,contact};
 auto classify=[](const garments::Donor& d){return d.vertex==3?garments::SupportBody::Shaft:garments::SupportBody::Lobe0;};auto convert=[](garments::Point p){return p;};
 auto force=garments::AggregateNumericalSupport(cloth,classify,convert);
 if(!force.enabled||std::abs(force.shaftAcceleration.z-6.05f*.25f)>1e-6f||std::abs(force.lobeAcceleration[0].z-6.05f*.75f)>1e-6f)return 6;
 cloth.support.push_back(contact);auto more=garments::AggregateNumericalSupport(cloth,classify,convert);
 if(more.shaftAcceleration.z!=force.shaftAcceleration.z||more.lobeAcceleration[0].z!=force.lobeAcceleration[0].z)return 7;
 cloth.style=garments::Style::Naked;if(garments::AggregateNumericalSupport(cloth,classify,convert).enabled)return 8;
 std::puts("PASS support acceleration normalization, measured gravity cap and invalid inputs");
}
