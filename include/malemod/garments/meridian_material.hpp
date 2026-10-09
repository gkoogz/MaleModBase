#pragma once
// Portable authored material intent. Engine lighting and shader code belong
// in adapters. Coordinates follow the fabric, never camera/world position.
namespace malemod::garments::meridian {
struct FabricStyle {
 float white[3]={.88f,.87f,.84f};
 float red[3]={.58f,.018f,.027f};
 float blue[3]={.018f,.065f,.38f};
 float ribCount=96, ribSlope=.16f;
 float redCenter=.4275f,blueCenter=.5725f,stripeHalfWidth=.0275f;
};
inline constexpr FabricStyle classicFabric{};
// Observed seven-row waistband pattern, normalized from its physical cut.
inline constexpr float bandMaterialRows[7]={0,.4f,.455f,.545f,.6f,.89f,1};
}
