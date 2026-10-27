#pragma once
#include <cstdint>
#include <array>
#include <type_traits>
#include "canonical_features.hpp"

namespace malemod::surface::canonical {
// Fixed-width process boundary. A request is consumed synchronously and its
// complete source output is admitted only with the identical sequence number.
// This preserves the original x86 arithmetic inside a 64-bit game host.
inline constexpr std::uint64_t Magic=0x315632434d4d554cull;
inline constexpr unsigned AnatomyVertices=17528,Body0Vertices=15168,Body1Vertices=5615;
struct Point {float x,y,z;};
struct alignas(8) Packet {
 std::uint64_t magic=Magic;
 std::uint32_t version=4,bytes=sizeof(Packet);
 std::uint64_t request=0,completed=0;
 std::uint32_t epoch=0,status=0,stop=0,collisionOverride=0;
 std::array<float,18> controls{2,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
 float seconds=1.f/60.f,pitchForce=0,yawForce=0;
 std::uint32_t garmentStyle=0; // Canonical support/substep mode; physics still runs each request.
 std::array<Point,4> thighs{};
 double physicsMilliseconds=0,surfaceMilliseconds=0;
 std::uint64_t physicsServices=0,surfaceServices=0;
 std::array<Point,12> guide{};
 std::array<unsigned char,AnatomyVertices*32> anatomy{};
 std::array<unsigned char,Body0Vertices*32> body0{};
 std::array<unsigned char,Body1Vertices*32> body1{};
 // Exact pressure-transformed outer lobe support points used by the canonical
 // Meridian recipe. The adapter poses these alongside its measured donors.
 std::array<Point,12> garmentLobeControls{};
 Features features{};
 FeatureOutput featureOutput{};
};
static_assert(std::is_standard_layout_v<Packet> && std::is_trivially_copyable_v<Packet>);
}
