#pragma once
#include <array>
#include <cstdint>
#include <type_traits>

namespace malemod::surface::canonical {
// Adapter observations only. Numerical envelopes and deformation remain in the
// canonical x86 kernel. Time is the original unsigned millisecond clock.
struct Features {
 std::uint32_t throbMode=0,milliseconds=0,resetClock=0,rootJointRequested=0;
 std::uint32_t teachingActive=0;
 float teachingEnd=20.f;
 double teachingTime=0;
 std::array<float,4> angleGain{1,1,1,1},lateralGain{};
 float lateralWobbleDegrees=3.f;
};
struct FeatureOutput {
 float sizePulse=0,twitchPulse=0,angleSizePulse=0;
 std::uint32_t nozzleValid=0;
 std::array<float,3> nozzlePosition{},nozzleDirection{};
 // Canonical source menu display order, distinct from its internal slider
 // packet order. These values are animated readouts, never saved controls.
 std::array<float,18> effectiveControls{};
};
static_assert(std::is_standard_layout_v<Features> && std::is_trivially_copyable_v<Features>);
static_assert(std::is_standard_layout_v<FeatureOutput> && std::is_trivially_copyable_v<FeatureOutput>);
}
