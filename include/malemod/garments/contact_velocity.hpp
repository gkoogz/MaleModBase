#pragma once
namespace malemod::garments::cloth_contact {
struct VelocityResponse {Point velocity{},impulse{};};
// Caller length units and measured dimensionless mass. The normal points out
// of the obstacle. A separating velocity is retained; Coulomb friction uses
// only this obstacle's measured positional normal impulse, never proximity.
inline VelocityResponse Solve(Point velocity,Point obstacleVelocity,Point normal,
                              double normalCorrection,double inverseMass,
                              double friction,double step){
 if(!Finite(velocity)||!Finite(obstacleVelocity)||!Finite(normal)||
    !std::isfinite(normalCorrection)||!std::isfinite(inverseMass)||inverseMass<0||
    !std::isfinite(friction)||friction<0||!std::isfinite(step)||step<=0)
  throw std::invalid_argument("Invalid physical cloth contact velocity");
 VelocityResponse out;out.velocity=velocity;if(inverseMass==0)return out;
 normal=Unit(normal);auto relative=Sub(velocity,obstacleVelocity);double speed=Dot(relative,normal);
 if(speed>1e-12)return out;
 if(speed<0)out.velocity=Sub(out.velocity,Mul(normal,speed));
 relative=Sub(out.velocity,obstacleVelocity);auto tangent=Sub(relative,Mul(normal,Dot(relative,normal)));
 double tangentSpeed=Length(tangent),normalSpeed=(std::max)(0.,normalCorrection)/step+(std::max)(0.,-speed);
 if(tangentSpeed>1e-12)out.velocity=Sub(out.velocity,Mul(tangent,(std::min)(1.,friction*normalSpeed/tangentSpeed)));
 out.impulse=Mul(Sub(out.velocity,velocity),1/inverseMass);return out;
}
}
