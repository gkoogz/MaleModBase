#pragma once
#include "../math.hpp"
#include <algorithm>
#include <vector>
namespace malemod::physics {
using std::min;using std::max;
struct State {std::vector<V3> position,oldPosition,velocity;std::vector<float> invMass;};
}
