# Root articulation: preserving live physics cadence

After selecting the root-articulation preview for installation, the user
confirmed the angle was largely fixed but reported greatly reduced shaft and
testicular response. Preserve the reviewed collar while repairing that regression.

The fixed lobe rows are fixed only with respect to the angle correction: their
current simulated positions still enter each solve. Shaft handles also rotate
the current simulated surface. Neither set is frozen to the rest pose.

The added junction solve did, however, make complete updates slower than the
controller's single 0.1-second catch-up step. When every accepted update takes
0.24 seconds, that path advances physics by only 0.1 seconds per update. Its
interpolated native-pose endpoint falls progressively behind the moving character,
which also attenuates the motion filter input. Waiting for a settled photograph
does not test this behavior.

The adapter now submits bounded batches of original wire-valid physics samples.
Every intermediate source step executes in order, with interpolated measured
poses/contacts. It builds the target character only for the final sample. An
accepted batch commits its complete clock and pose; rejection changes neither.
The 32-sample bound retains any excess active interval for the next batch. Pause,
character epochs and the worker's numerical protocol remain distinct.

Base's shared Junction now retains immutable topology and sparse symbolic
analysis. UpdateReference still recomputes the weights and numerical
factorization from the current simulated geometry. It does not freeze a metric,
cache a stale pose, change the energy, or multiply the physics response.
Wolverine and other spokes can adopt this API through a tested Base pin; the
Witcher scheduling change belongs in its adapter.

Verification so far: refreshed-metric tests match fresh construction; all 189
settled cases / 378 LOD outputs are byte-identical to the installed collar
candidate. A separate-worker regression covers 24 moving poses at 0.24-second
cadence, exact source bytes, both target LODs, full clock advancement and pause.
It detects nonzero lobe/shaft motion; those synthetic-input magnitudes are not
clinical measurements or observed gameplay. Native walking/decay verification
and installation identities are recorded separately in the adapter handoff.

Private evidence directory: D:/MaleModBuilds/witcher-fixed-collar-20261014.
Full attachment, campaign and garment-force parity remain separate gates.

## October 14 - physics cadence repaired; candidate not installed

User reported reduced motion after accepting the angle fix. The controller's
single catch-up step fell behind slow target updates. Witcher now batches source
steps and composes the target once; Base reuses junction topology/symbolic
analysis while refreshing the exact current metric. See Base PHYSICS-CADENCE.md.
All 189 cases / 378 settled LOD outputs match the installed collar byte-for-byte;
24 moving-pose serial parity/pause checks and the shared refreshed-metric test
pass. Legacy runtime_controller_test was updated but not run.

Native evidence: first 57 attachment views of 203037 plus 36 valid motion frames
of 204501; 7.2403 seconds pose time / 7.2210 seconds physics time, 7.5319 m travel.
Reviewed the three attachment sheets and motion contact sheet. Shaft/lobe motion
is visible, with continuous attachment in sampled views. Later optional captures
668-680 of 203037 and motion session 203931 were invalid due C: disk exhaustion;
these are excluded from visual proof. All three sessions are closed, temporary
driver/room removed and audio/saves restored. Existing small-size and downward
compression limitations remain; full attachment/campaign/garment/performance
gate has NOT passed. No pin/commit/release.

Candidate DLL df25874dcf3f72d1c9d021bb68d8e2972db6bd855c360865bd26f0297e7cf7a8
is NOT installed. Accepted 2fde715c runtime and exact rollback remain intact;
all 39 managed installed files reverified. Evidence and visual review:
D:/MaleModBuilds/witcher-fixed-collar-20261014/physics-final-proof.json
and physics-native-motion.gif. Source identities: provenance/physics-cadence.json.

User then requested C: cleanup. Hash-identical old cooker scratch copies and
duplicate captures were deleted; original SDK assets and all evidence remain.
Deletion manifest: D:/MaleModBuilds/cleanup-20261014/.

