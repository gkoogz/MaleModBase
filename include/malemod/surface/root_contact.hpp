#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace malemod::surface::root_contact {
using Point=std::array<double,3>;
inline Point Sub(Point a,Point b){for(unsigned k=0;k<3;k++)a[k]-=b[k];return a;}
inline Point Scale(Point a,double s){for(auto& x:a)x*=s;return a;}
inline double Dot(Point a,Point b){double d=0;for(unsigned k=0;k<3;k++)d+=a[k]*b[k];return d;}
inline void Finite(Point p){for(auto x:p)if(!std::isfinite(x))throw std::invalid_argument("Nonfinite angular root joint");}
struct State {double pitch=0,yaw=0,pitchVelocity=0,yawVelocity=0;};
// A driven orientation and its rate belong to the imposed command, not to the
// contact suspension. Recover only the relative state to prevent double drive.
inline State Relative(State total,State drive){
 for(double v:{total.pitch,total.yaw,total.pitchVelocity,total.yawVelocity,drive.pitch,drive.yaw,drive.pitchVelocity,drive.yawVelocity})if(!std::isfinite(v))throw std::invalid_argument("Nonfinite angular root drive");
 return {total.pitch-drive.pitch,total.yaw-drive.yaw,total.pitchVelocity-drive.pitchVelocity,total.yawVelocity-drive.yawVelocity};
}
// Source-calibrated pitch/yaw convention: forward X, side Y, up Z.
// This is a numerical frame, never an inferred game skeleton or game unit.
inline Point Direction(const State& s){return {std::cos(s.pitch)*std::cos(s.yaw),std::sin(s.yaw),-std::sin(s.pitch)*std::cos(s.yaw)};}
// The first link represents the angular root DOF. Its fixed-length constraint
// removes radial motion; the existing rod constraints supply contact reactions.
inline double PointInverseMass(double linkLength,double angularMass){
 if(!std::isfinite(linkLength)||!std::isfinite(angularMass)||linkLength<=0||angularMass<=0)throw std::invalid_argument("Invalid root link inertia");
 return linkLength*linkLength/angularMass;
}
// Recover the angular state from the accepted rigid first link. This is a
// kinematic conversion, not a collar or full-tube clearance certificate.
inline State FromJoint(Point root,Point link,Point velocity,double previousPitch){
 Finite(root);Finite(link);Finite(velocity);auto offset=Sub(link,root);double length=std::sqrt(Dot(offset,offset));
 if(!std::isfinite(previousPitch)||length<1e-10)throw std::invalid_argument("Degenerate angular root joint");
 auto d=Scale(offset,1/length);State out;out.pitch=std::atan2(-d[2],d[0]);out.yaw=std::atan2(d[1],std::hypot(d[0],d[2]));
 constexpr double pi=3.14159265358979323846;
 while(out.pitch-previousPitch>pi)out.pitch-=2*pi;while(out.pitch-previousPitch < -pi)out.pitch+=2*pi;
 const double sp=std::sin(out.pitch),cp=std::cos(out.pitch),sy=std::sin(out.yaw),cy=std::cos(out.yaw);
 const Point dp={-sp*cy,0,-cp*cy},dy={-cp*sy,cy,sp*sy};
 out.pitchVelocity=Dot(velocity,dp)/(length*(std::max)(1e-10,Dot(dp,dp)));out.yawVelocity=Dot(velocity,dy)/length;return out;
}
}
