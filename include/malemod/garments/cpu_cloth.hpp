#pragma once
#include "jockstrap.hpp"
#include <memory>
namespace malemod::garments {
// Engine-independent numerical interface. Implementation links the pinned
// NvCloth CPU library; no NVIDIA types or graphics SDK escape this interface.
class CpuCloth {
public:
    struct MotionLimit {Point center{};double radius=0;};
    CpuCloth();
    ~CpuCloth();
    CpuCloth(const CpuCloth&)=delete;
    CpuCloth& operator=(const CpuCloth&)=delete;
    void Initialize(const std::vector<Point>& points,
                    const std::vector<std::array<unsigned,3>>& triangles,
                    const std::vector<bool>& pinned,double restScale=1);
    void Place(const std::vector<Point>& points);
    void Step(double seconds,const std::vector<Point>& pins,
              const std::vector<Capsule>& capsules,Point gravity,
              const std::vector<MotionLimit>& limits={},
              const std::vector<MotionLimit>& backstops={},
              const std::vector<std::array<Point,3>>& triangles={});
    std::vector<Point> Positions() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
