# Optional garment load in the full source worker

This shared addition supplies a small garment load to the existing numerical
solver. It does not replace contact constraints, reshape anatomy, change
scaling, or alter source simulation timing. A fully coupled cloth solver is
not claimed.

## Production input and units

`surface::Frame::garment` contains:

```cpp
bool enabled=false;
Point shaftAcceleration{};
std::array<Point,2> lobeAcceleration{};
```

Each vector is acceleration in the worker's calibrated source-local length
units per source time squared. Transform directions and length scale explicitly
when supplying world-space garment contacts; do not apply a position-origin
offset. The reference integration uses shaft gravity `ModeValue(5,42,110)` and
lobe gravity `72`. These are observed source values, not inferred SI meters.

`BoundGarmentAcceleration` caps the vector's magnitude at 15% of the current
source gravity for that integrated body. Wire input budgets are 16.5 for the
shaft and 10.8 for each lobe. Nonfinite or oversized input is rejected before
worker state changes. Disabled input never executes the extra integration
arithmetic, including when its finite input vectors are nonzero.

The injected branch is immediately after the existing gait/gravity acceleration
calculation in `StepConstraintSolver`. It affects free shaft nodes and each
lobe, then uses the original drag, timestep, masses, constraints and accepted
velocity reconstruction. Fixed shaft root nodes receive no load.

## Contact aggregation

`garments/numerical_support.hpp::AggregateNumericalSupport` accepts the current
garment output and two caller-owned callbacks:

- `classify(Donor) -> SupportBody { None, Shaft, Lobe0, Lobe1 }`, using the
  actual source vertex lineage, not guessed engine joints;
- `toSource(acceleration) -> garments::Point`, an explicitly calibrated
  acceleration transform.

It normalizes the positive contact influences and each contact's donor weights,
then aggregates by body. Duplicating equivalent sampled contacts does not
increase support. Non-anatomical donors can map to `None`. Naked garments,
empty support and an unsatisfied contact budget yield disabled input. The
garment generator owns contact separation and influence; this helper does not
invent cloth contact locations or a bidirectional cloth constraint system.

The garment's default strength is a 0.055 gravity fraction. The aggregated
forces are gentle. Nonlinear constraints and contacts mean tip position need
not rise monotonically in every mode. Verification establishes actual force
delivery and boundedness, not a uniform visible lifting promise.

## Adapter and canonical adoption

Wire version **5** carries the explicit flag and all three vectors. Worker and
adapter must upgrade together; old wire messages are rejected explicitly.
Witcher's `RuntimeHost::Tick` and `RuntimeController::Tick` accept a final
optional `Frame::GarmentSupport` argument after clinical projection. Accepted
active requests and paused previews copy it into the numerical request.
The engine bridge owns the garment snapshot and calls those APIs.

Canonical Wolverine must consume the pinned Base helper header and add:

```cpp
static bool surfaceGarmentEnabled=false;
static V3 surfaceGarmentShaft{},surfaceGarmentLobes[2]{};
```

Include `<malemod/surface/garment_support.hpp>`. In the existing common
integration loop, keep the original acceleration expression and insert:

```cpp
V3 acceleration{0.f,side*86.f*response,gait*86.f*response-gravity};
if(surfaceGarmentEnabled){
 auto a=i<pdBody0?surfaceGarmentShaft:surfaceGarmentLobes[i-pdBody0];
 auto bounded=::malemod::surface::BoundGarmentAcceleration({a.x,a.y,a.z},gravity);
 acceleration=acceleration+V3{bounded.x,bounded.y,bounded.z};
}
float drag=i<pdBody0?shaftDrag:bodyDrag;
```

Publish normalized garment support before the next simulation step, and clear
`surfaceGarmentEnabled` on Naked/reset/character replacement. Leave geometry
evaluation and clinical clocks in their existing order. Do not edit the
immutable source snapshot. The Base extractor applies a guarded replacement
at the same source integration span and keeps per-session/process storage.

## Offline gates

- `garment_support_test`: measured acceleration cap, normalized lineage/contact
  aggregation, duplicate sample invariance, Naked and invalid input.
- `surface_wire_test`: exact wire roundtrip and malformed support flag/budget
  rejection.
- `tools/test_garment_worker_support.py`: fresh production worker processes in
  states 0/1/2, 180 steps each; enabled support changes numerical output, while
  disabled nonzero vectors produce exactly the same full wire output as zero
  vectors. Four established revision-2 cases retain byte-identical geometry,
  normals, tangents, UVs, topology and mechanical data with the support disabled.

Offline tests do not establish installed garment behavior or gameplay.
Recorded results and library provenance are in `provenance/source-surface.json`.
