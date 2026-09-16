# Claims and Evidence Matrix

Status: working publication evidence index

This table separates demonstrated results from architectural expectations. A
claim should enter the paper as a result only when its evidence is reproducible
from a named model, test, or benchmark.

| Prospective claim | Primary evidence | Recorded result | Maturity and paper use |
| --- | --- | --- | --- |
| Displacement-, velocity-, and acceleration-level implicit constraint formulations reproduce the planar pendulum response | [`test/paper/ddassl_formulation_tests.jl`](../test/paper/ddassl_formulation_tests.jl) and the three implicit pendulum examples | Each formulation is compared with the reduced solution; constraint-level diagnostics are explicit | Strong formulation evidence, but rerun after freezing the paper integrator configuration |
| Constraint-derivative deficit distinguishes the mechanical work needed to expose accelerations and reactions | [`architecture/common/mathematical-architecture.md`](../architecture/common/mathematical-architecture.md) and executable deficit tests | Deficits two, one, and zero correspond to retained $\Phi$, $\dot\Phi$, and $\ddot\Phi$ | Definition is established; explain its difference from general DAE index terminology |
| Lower-level stabilization restores constraint errors but introduces specified correction dynamics | Baumgarte, first-order, GearStableV, and GearStableA examples in [`test/paper/ddassl_formulation_tests.jl`](../test/paper/ddassl_formulation_tests.jl) | Tests compare measured decay with intended critical or exponential correction; GearStableV retains position and velocity constraints and GearStableA retains all three levels | Strong qualitative evidence; the Gear satisfaction multipliers are diagnostic discrepancies rather than physical reactions |
| Component-local equations assemble into the same implicit result as programmatic monolithic examples | [`test/paper/model_development_tests.jl`](../test/paper/model_development_tests.jl) | Sparse and dense Jacobians are compared; TOML four-bar response is compared with the programmatic model | Strong architecture evidence once the paper harness is stabilized |
| The Fully Consistent formulation can solve both dynamic and fully driven kinematic mechanisms through one sparse system | [`test/core/model_program_tests.jl`](../test/core/model_program_tests.jl), slider-crank and torque-driven four-bar TOML models | Supported suite covers both zero-state and state-containing mechanisms using the unified path | Strong supported-program evidence |
| Configuration must be made consistent before state selection | Automatic-state tests in [`test/core/model_program_tests.jl`](../test/core/model_program_tests.jl) | Perturbed four-bar configuration is corrected before QR selection | Strong regression evidence; useful algorithm sequence in paper |
| Pivoted QR can select individual physical states and detect redundant constraint rows | [`architecture/common/state-selection-from-velocity-constraints.md`](../architecture/common/state-selection-from-velocity-constraints.md) and automatic-state core tests | Correct degrees of freedom, selected variables, QR diagnostics, and inactive constraint families are retained | Strong planar evidence; spatial scaling remains future work |
| A user-preferred coordinate can outperform a locally valid automatic coordinate over a long trajectory | Closed-loop section of [`benchmark/README.md`](../benchmark/README.md) | In the 10-cell, 10 s case, preferred rocker $\omega$ reduced rejected steps 61 to 1, Newton iterations 7,795 to 5,336, and time 0.895 to 0.672 s | Strong case study; supports future in-run repartitioning rather than claiming it is implemented |
| Dense QR is not the current initialization bottleneck | State-selection timing section of [`benchmark/README.md`](../benchmark/README.md) | Complete selection costs 33--49 microseconds per pass and total pre-integration work about 1 ms for the 10-link cases | Strong for present sizes; closed-loop QR growth provides the qualification |
| Dedicated sparse factorization is essential for the unreduced formulation | Pendulum-chain sections of [`benchmark/README.md`](../benchmark/README.md) | The 50-link generic-LU time of 112.2 s fell to 0.1070 s with UMFPACK | Very strong measured evidence; identify hardware and Julia version |
| Symbolic analysis can be reused while topology and selected states remain fixed | Reused-symbolic section of [`benchmark/README.md`](../benchmark/README.md) | One symbolic factorization served each smooth chain run; 50-link time and allocation fell about 7% and 10% | Strong implementation evidence |
| A slowly changing scaled Jacobian permits useful modified-Newton reuse | Reused-numerical section of [`benchmark/README.md`](../benchmark/README.md) | The 50-link case used 30 numerical factorizations for 149 attempts and fell to 0.08028 s | Strong performance evidence; include refresh/failure safeguards |
| Canonical size and sparse nonzeros grow approximately linearly for open serial chains | Pendulum-chain tables in [`benchmark/README.md`](../benchmark/README.md) | 10, 25, and 50 links give 110, 275, and 550 variables and 498, 1,278, and 2,578 nonzeros | Strong structural-scaling evidence |
| Closed-loop systems retain favorable sparse run scaling even with one global degree of freedom | Parallelogram-chain table in [`benchmark/README.md`](../benchmark/README.md) | 10 to 50 cells increased canonical size 4.8 times and complete load/run time about 6.3 times | Strong evidence; dense rank analysis grows faster but remains small in absolute time |
| Continuous compliant contact benefits from a soft BDF restart | Bouncing-ball-bank tables in [`benchmark/README.md`](../benchmark/README.md) | For 50 balls, soft restart reduced time 3.572 to 2.103 s and rejected steps 2,099 to 875, with no corrector failures | Strong evidence limited to continuous force/state transitions |
| The stiff integrator handles a smooth widening modal range with nearly linear scaling | Rotor-train table in [`benchmark/README.md`](../benchmark/README.md) | 10 to 50 rotors increased variables fivefold and time 4.6-fold; errors stayed near requested tolerances | Strong accuracy and scaling evidence because the independent modal reference is analytical |
| Relative joint coordinates are practical state candidates but are not automatically faster | Ten-second pendulum state comparison in [`benchmark/README.md`](../benchmark/README.md) | Relative coordinates enlarged the canonical system 25% and ran about 5% slower than automatic selection; preferred body $\omega$ was about 18% faster | Strong balanced result; useful evidence that physical convenience and speed are separate criteria |
| Static equilibrium can initialize a subsequent dynamic analysis | Bushing pendulum tests and [`models/planar/bushing-pendulum.toml`](../models/planar/bushing-pendulum.toml) | Direct Newton and dynamic relaxation paths are exercised; dynamic motion starts from the equilibrium position | Strong supported-program evidence; multi-analysis result storage is not yet implemented |
| Algebraic force definitions and reactions can remain in modal shapes without creating extra finite modes | [`architecture/common/modal-linear-analysis.md`](../architecture/common/modal-linear-analysis.md), modal core tests, and [`models/planar/modal-pendulum.toml`](../models/planar/modal-pendulum.toml) | One sparse shifted factorization followed by a $2n_s$ eigenproblem recovers complete canonical modes with equation errors near machine precision in exercised models | Strong initial evidence; add at least one multi-degree benchmark table before publication |
| The same component model extends naturally to spatial systems | [`architecture/common/mathematical-architecture.md`](../architecture/common/mathematical-architecture.md) | Rotation-matrix, marker-frame, virtual-power, and local Jacobian architecture is written | Architectural claim only; do not present as demonstrated until the 3D vertical slice runs |

## Evidence still needed for the paper

1. Restore a deliberately stable paper verification baseline and record its
   complete pass counts.
2. Produce one script that regenerates every publication table without relying
   on manually copied terminal output.
3. Add a machine-readable environment record: commit, Julia version, package
   manifest hash, hardware, tolerances, and warm-up policy.
4. Generate sparse-pattern and scaling figures from benchmark output.
5. Decide which formulation comparisons are accuracy comparisons and which are
   only conceptual demonstrations; do not use unmatched tolerances as speed
   evidence.
6. Add a modal benchmark with more than one degree of freedom and compare its
   frequencies with an independent reduced or analytical reference.
7. After the first spatial model works, add one concise planar/spatial
   invariance table. Until then, describe 3D as intended extension rather than
   demonstrated generality.
8. Review literature on stiff mechanical simulation, automatic
   differentiation, Jacobian-free Newton--Krylov methods, sparse ordering, and
   recursive multibody algorithms before positioning the component-local sparse
   argument as a novel contribution.
