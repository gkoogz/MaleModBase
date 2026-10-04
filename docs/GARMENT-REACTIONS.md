# Physical garment reaction contract

The retained anatomy and the garment remain separate meshes. A persistent
material session fits once, then advances cloth particles, elastic constraints,
sewing and contact. A motion frame does not repeat the walking fit.

## Measured units and mass

`Input.anatomyMass` is the sum of the source solver's current generalized
dimensionless masses: ten free rod nodes and two suspended lobes. It is not a
kilogram estimate. The exact source float law and control mapping are in
`garments/reaction_support.hpp`. Adapters convert observed character positions
and directions into their measured source coordinate calibration.

The shared cloth mass is `mechanics.massFraction` (currently 0.08) times this
sum. Material triangle area distributes it across free cloth particles.
The ratio is an explicit lightweight garment tuning choice, not a measured
fabric weight; full coupled stability and motion must validate its use.
Compliance is normalized against the mean area mass so changing the source
mass calibration does not arbitrarily change softness. Zero mass is an
explicit uncoupled offline compatibility input.

## Contact and feedback

Each actual physical contact records its exact current triangle, barycentric
weights and full retained native vertex lineage. Actual positional projection
impulses and terminal contact velocity/friction impulses act reciprocally on
the source. Sewing and stock body contacts do not become anatomy forces.
Relative contact velocity uses the current and preceding observed triangle
positions. Separating normal velocity is preserved; friction uses the shared
coefficient and the measured tissue normal correction, not proximity alone.

Impulse has caller length times dimensionless source mass divided by seconds.
First moment has caller length squared times mass divided by seconds. The
adapter transforms moments as pseudovectors, including calibrated reflections
and unit changes. Local lobe torque subtracts its lever arm from the affine
image of **world zero**. Using the pelvis origin instead would let actor
translation create false torque; a translated-frame regression covers this.
Each free source node receives its local impulse divided by
its actual current mass. Suspended lobes also receive torque divided by their
actual source inertia. Rod nodes zero and one are prescribed pelvis attachments.

The source force-ownership table is derived from the authored station and lobe
fields through the existing R14, rounded-surface and final native interpolation
lineage. It is a bounded force-distribution map. **It is not the complete
Jacobian of the nonlinear anatomy surface or pelvic collar.** The source rod
also has no independent axial-spin degree of freedom. These limits must remain
visible in adapter receipts and must not be described as exact free-body
angular conservation.

`ReactionTelemetry` reports point-contact linear impulse and moment, and
independently sums the actual free cloth particle impulse and moment. The
reported support difference includes prescribed/sewn material constraints and
render offsets whose orientation is supplied by a material frame. Rendering
those offsets does not introduce independent rigid rotational degrees of
freedom. An opposite point-contact moment alone is not a particle torque proof.

Contact projection differentiates the actual rendered material point: linear
weights, transported triangle normals and ribbon shortest-arc rotation all
contribute their scalar gradients and effective mass. Fine-face contacts merge
the three actual corner gradients, preserving repeated material-frame nodes.
`render_contact_test` compares these derivatives with finite differences and
checks virtual work, nodal linear/angular sums and frame/unit covariance.
The renderer's antiparallel ribbon branch uses its prescribed rest axis and is
locally constant; its switch is not a smooth independent roll degree of freedom.

## Asynchronous delivery

Wire6 carries double cumulative per-rod and per-lobe impulses and lobe torque,
plus producer epoch and publication serial. The producer appends every completed
cloth solve before replacing its latest result. A source consumer subtracts its
last accepted totals. Re-reading one result does not replay it; skipped latest
results do not discard their integrated impulse. Pending increments remain
queued until an actual source numerical substep. A new garment/character epoch
resets the old queue. Validation commits cursor and queue state transactionally.

The contract conserves transported increments across 30/60/120 Hz consumption,
coalescing and busy retries. It does not remove partitioned coupling lag: a
cloth result depends on a preceding source surface, and its reaction is applied
on a following source substep. Stability and presentation age therefore need
a separate full closed-loop gate. Runaway/nonfinite input is rejected, never
silently capped into a claim of containment.

## Verification boundaries

`garment_reaction_test` checks actual reciprocal projection/friction math,
exact barycentric lineage, units and rotations, and cumulative transport.
`tools/verify_garment_source_reactions.py` verifies the actual Win32 extracted
source: disabled input exact parity, once versus retried publication, queued
zero-substep and coalesced pending impulses in all three mechanical states.
An optional prior wire5 binary checks exact disabled numerical payload parity.
The receipt binds exact binaries, source files, commit and dirty state.

Canonical adapter worker tests use synthetic impulses to isolate transport;
they do not prove cloth contacts or coupled stability. Native opacity readback
tests prove the production draw honors 0.95 once over retained anatomy and
restores graphics state; they do not prove gameplay. Actual source static,
moving cloth and full coupled source/cloth gates remain distinct, and only
observed game results establish gameplay behavior.

Canonical `tests/garment_source_coupling_test.cpp` and
`tools/Verify-GarmentSourceCoupling.py` run a separate actual source session for
each size and feedback/control pair. They retain complete source/body/native
meshes, persistent material state, measured mass and actual contacts, with
60 Hz source submissions and two 120 Hz cloth steps. Three-frame publication
coalescing and zero-step pending input every fifth frame exercise real feedback.
These prescribed authoring motion tests are not observed gait or gameplay.
