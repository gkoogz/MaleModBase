#pragma once
// Speculative subcutaneous support tube. This profile is shared by the hidden
// annular geometry, its skin envelope and its contribution to rod stiffness.
static float RapheTubeRadiusRatio(float s){
 float root=1.f-Smoother01((s+.035f)/.27f);
 float taper=Smoother01((s-.50f)/.50f);
 return (.30f+.28f*root)*(1.f-taper)+.10f*taper;
}
static constexpr float rapheTubeLumenRatio=.64f;
static float RapheTubeSecondMoment(float s,float radius){
 float outer=radius*RapheTubeRadiusRatio(s),inner=outer*rapheTubeLumenRatio;
 return .78539816339f*(outer*outer*outer*outer-inner*inner*inner*inner);
}
static float RapheTubeBendMultiplier(float t){
 float reference=RapheTubeSecondMoment(.35f,1.f);
 float relative=RapheTubeSecondMoment(t/.88f,1.f)/reference;
 // Parallel material stiffness in the existing XPBD rod. There is one solve,
 // not a second animated object pulling the skin in a competing direction.
 return 1.f/(1.f+ModeValue(3.2f,.90f,.16f)*relative);
}
