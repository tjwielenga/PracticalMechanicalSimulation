# Planar Methods Consolidation

Status: working technical record for author review

Baseline: commit `96d8077`, September 4, 2026

## 1. Purpose

This document preserves the reasoning behind the planar simulator before work
turns to spatial mechanics. Its purpose is not to describe every implemented
type. It records the method, the alternatives that were studied, the evidence
already available, and the decisions that should survive a 3D extension.

The supported program is now a capable planar mechanism simulator rather than
only a set of formulation experiments. It reads hierarchical TOML models,
assembles component-local implicit equations into a sparse canonical system,
selects physical states, establishes consistent initial conditions, and
performs kinematic, dynamic, static, quasi-static, and modal analysis. Results
are stored in compressed HDF5 under the `.simp` extension and can be viewed,
exported to CSV, or used to recover the model.

## 2. Working definition of the Fully Consistent method

The proposed paper needs one precise definition. The following is the working
definition to be confirmed by the author.

Let the canonical mechanical variables be

$$
x=(q,v,a,\lambda,\xi),
$$

where $q$, $v$, and $a$ are explicit configuration, velocity, and acceleration
variables, $\lambda$ contains ideal-constraint reactions, and $\xi$ contains
component-local geometric, rate, load, or constitutive variables. Components
contribute local implicit equations to

$$
F(t,x,\dot x)=0.
$$

For an ideal holonomic constraint, the position, velocity, and acceleration
members of its equation family are retained together:

$$
\Phi(q,t)=0,
$$

$$
\dot\Phi(q,v,t)=0,
$$

$$
\ddot\Phi(q,v,a,t)=0.
$$

Selected physical coordinates close the differential part of the square
system with local state equations such as

$$
a_i-\dot v_i=0,
\qquad
v_i-\dot q_i=0.
$$

At a BDF corrector, the complete active system is solved through

$$
\left(F_x+c_jF_{\dot x}\right)\Delta x=-F.
$$

The matrix is **full in physical content but sparse in storage and solution**.
No global coordinate reduction is required before integration. All canonical
variables are predicted and corrected, while integration-error control normally
uses the selected physical positions and velocities and excludes reactions and
other algebraic quantities.

The defining properties of the current Fully Consistent method are therefore:

1. Explicit levels for position, velocity, acceleration, reactions, and local
   force variables.
2. Simultaneous retention of the available position-, velocity-, and
   acceleration-level constraint equations.
3. Component-local declaration of variables, equations, and Jacobian entries.
4. Automatic selection of individual physical state coordinates without
   eliminating the other physical variables.
5. A complete sparse Newton solve at each corrector.
6. BDF history for the complete solution, with selective physical-state error
   control.

This is a deficit-zero formulation: the highest mechanical derivatives and
reactions are solvable without differentiating the retained equation set
again. Unlike an acceleration-only deficit-zero formulation, it also retains
the lower-level constraint equations directly.

### 2.1 Name and scope

**Sparse Fully Consistent Modeling Method** is the formal name of the present
unreduced simultaneous formulation. After its first use, the paper uses
**Fully Consistent method** as shorthand. No acronym is introduced.

Here *fully consistent* means that all available orders of the constraint
equations are retained and satisfied simultaneously throughout the solution.
It does not refer only to establishing consistent initial conditions.
**State-selected** describes how the implementation closes its differential
equations: the complete physical variable set is retained, but only a minimal
independent set of physical state equations is added.

### 2.2 Numerical stiffness and component locality

A principal motivation for the method is that real mechanical systems are
often numerically stiff. Bushings, contact compliance, dampers, flexible
members, driveline elements, and widely separated inertial and elastic scales
can make an explicit integration method impractical. A stiff integrator
normally requires a Newton-like corrector and therefore either a Jacobian, a
Jacobian action, or an effective approximation and preconditioner.

Coordinate or component reduction can make the integrated system smaller, but
the Jacobian of that reduced system is often complicated. Eliminated local
variables reappear through chain-rule dependencies, and a change in topology or
element formulation can require new global derivative work. Automatic
differentiation reduces the burden of forming these derivatives, and
Jacobian-free Newton--Krylov methods can avoid forming the complete matrix, but
neither removes the need to represent coupling accurately or to solve the
linearized correction problem efficiently.

The Fully Consistent method takes the complementary approach. It retains
meaningful component variables and their simple local equations, assembles
their local derivatives into one sparse matrix, and lets the sparse linear
solver perform the global elimination. Sparse ordering, fill reduction,
numerical pivoting, and factorization traverse the assembled dependency
structure using current general-purpose algorithms. This can be viewed as an
implicit algebraic counterpart to an explicit traversal written for a
particular mechanism topology: the traversal is chosen from the sparse matrix
rather than encoded as a special-purpose mechanical algorithm.

This argument does not imply that reduction, automatic differentiation, or
Jacobian-free solution is obsolete. They remain possible transformations of
the canonical component model. The important claim is that modern sparse
methods make direct component-level formulation practical, particularly when
an implicit stiff solution is already required.

## 3. Constraint-derivative deficit and the compared formulations

Constraint-derivative deficit is the number of additional time
differentiations of the retained constraint equations needed to determine the
highest mechanical derivatives of interest and their reactions. Derivatives of
reaction variables are not part of this definition.

| Formulation | Retained constraint information | Deficit | Principal benefit | Principal cost or risk |
| --- | --- | ---: | --- | --- |
| Reduced coordinate | Constraints eliminated through a coordinate map | 0 after reduction | Small dynamic system | Global reduction, topology-specific transformations, and recovery of reactions |
| Displacement constraint | $\Phi=0$ | 2 | Direct position satisfaction | Higher deficit and more demanding DAE treatment |
| Velocity constraint | $\dot\Phi=0$ | 1 | Direct velocity satisfaction | Position drift without correction |
| Acceleration constraint | $\ddot\Phi=0$ | 0 | Accelerations and reactions are directly solvable | Position and velocity errors are not restored |
| Baumgarte or related stabilization | Acceleration equation with lower-level feedback | 0 | Restores accumulated errors | Introduces correction parameters and changes error dynamics |
| GearStableV | $\Phi$ and $\dot\Phi$ with a velocity-level satisfaction multiplier | 1 | Satisfies position and velocity constraints simultaneously | Permits a small discrepancy between $\dot q$ and $v$ and does not impose $\ddot\Phi$ |
| GearStableA | All three constraint levels with velocity- and acceleration-level satisfaction multipliers | 0 | Satisfies position, velocity, and acceleration constraints simultaneously | Larger formulation and discrepancies move into both derivative relationships |
| Fully Consistent | All declared levels plus selected state equations | 0 | Uniform, local, unreduced formulation | Larger sparse Newton system and sensitivity to state conditioning |

**GearStableV** names the position-and-velocity Gear formulation implemented by
`run_gear_constraint_satisfaction_pendulum`. **GearStableA** names the complete
position-, velocity-, and acceleration-level formulation implemented by
`run_complete_gear_constraint_satisfaction_pendulum`. These are paper names for
the related analysis methods, not current TOML analysis options.

## 4. State selection and redundant constraints

At a consistent initial configuration, the program evaluates and mechanically
scales the velocity-constraint partial matrix

$$
D=\frac{\partial\dot\Phi}{\partial v}.
$$

Column-pivoted QR identifies dependent velocity components. The remaining
columns are candidate independent physical velocities. Each selected velocity
is paired with its corresponding position, body-fixed pseudo-angle, or optional
relative joint coordinate. A separate QR of a transpose is used only when row
rank indicates redundant scalar constraints; complete constraint families,
including their reaction and all derivative levels, are then deactivated.

Preferred states allow an analyst to replace a locally valid but globally poor
automatic choice. The closed-loop benchmark demonstrated the importance of
this facility: a coupler vertical velocity became poorly conditioned during
the motion, while the first rocker angular velocity remained useful through a
complete rotation.

The current program selects states at initialization. The architecture explains
how to replace the closing state-equation rows while retaining the complete BDF
history, but automatic in-run repartitioning has not yet been implemented.

## 5. Initialization and analysis reuse

The current order is intentional:

1. Project the user configuration onto the position constraints with weighted
   minimum motion.
2. Determine constraint rank and physical state choices at that consistent
   configuration.
3. Assemble the active square implicit system.
4. Solve simultaneously for consistent accelerations, reactions, applied-force
   variables, and necessary derivatives.

Static equilibrium uses the same body balances, position constraints, and
force definitions with velocities and accelerations suppressed. Direct Newton
equilibrium and dynamic relaxation are available. A dynamic or modal analysis
may request static-equilibrium initialization. Saved results can transfer
named positions and, when requested, velocities into a later model with
compatible names and types.

Kinematic models now use the same complete implicit predictor-corrector path
rather than a staged position, velocity, acceleration, and force solve. A
zero-state kinematic mechanism is solved as a complete system at each requested
time; a mechanism with selected states is dynamic unless the user explicitly
requests another supported analysis.

## 6. BDF and sparse linear algebra conclusions

The Julia integrator is a fully variable-step, variable-order BDF method with
SciML-like solution behavior. It retains explicit variable and equation levels.
Level scaling is chosen so an equation's coefficients with respect to its
highest-level variables remain approximately independent of step size.

The benchmark history established three important conclusions:

- Dispatching the sparse Newton matrix to UMFPACK instead of generic sparse LU
  was decisive. For the 50-link short pendulum benchmark, run time fell from
  112.2 s to 0.1070 s, a measured speedup of roughly 1,049.
- Reusing UMFPACK symbolic analysis is valid while the sparsity pattern and
  selected states are unchanged. The smooth benchmarks normally require one
  symbolic analysis for a complete run.
- Reusing numerical factors for modified-Newton corrections reduced the
  50-link time further from 0.09945 s to 0.08028 s in the recorded comparison,
  despite requiring more inexpensive back-solves.

The remaining performance concern is allocation. The ten-second 10-link
experiments allocated roughly 3 GiB depending on state choice. This did not
prevent useful performance, but reusable implicit-equation, root, scaling, and
assembly workspaces are stronger optimization candidates than replacing the
small dense QR factorizations used during initialization.

## 7. Force transitions and stiffness

Compliant one-sided contact is represented as a stiff force rather than a
unilateral ideal constraint. Root functions locate changes in the force-law
derivative so the integrator can refresh its iteration matrix and adjust BDF
history.

For continuous compliant contact, retaining history while limiting order to
two and halving the next step was substantially better than a hard order-one
restart. In the 50-ball benchmark, the soft policy reduced time from 3.572 s to
2.103 s, allocation from 20,460 MiB to 11,850 MiB, and rejected steps from
2,099 to 875 without a corrector failure. This conclusion applies to continuous
state and force. An impulsive event still requires an explicit state jump and a
true restart, which the program does not currently implement.

The stiff torsional rotor train supplies complementary evidence. It has smooth
equations and a widening modal frequency range, yet its size, time, and
allocation scale nearly linearly and its solution agrees with an independent
modal reference.

## 8. Modal analysis

Modal analysis linearizes the complete active implicit equations at a
constraint-consistent operating point:

$$
J\,\delta x+E\,\delta\dot x=0.
$$

For $\delta x=\hat x e^{st}$,

$$
(J+sE)\hat x=0.
$$

Algebraic definitions, reactions, and applied-force variables remain in the
descriptor problem and in the recovered mode shapes. Because only selected
position and velocity variables contribute nonzero columns to $E$, the
implementation factors one sparse shifted full matrix and solves a dense
eigenproblem of order twice the number of selected states. This is neither the
dense generalized eigenvalue solution nor a physical-coordinate nullspace
reduction.

A static equilibrium is conventional but not mathematically required. At a
moving operating point the result is a frozen-time linearization. Nonsmooth
transition points do not have a unique linearization and should be avoided.

## 9. Development sequence worth preserving

The work progressed in a useful conceptual order:

1. **Formulation studies, August 30--31.** Minimal and reduced pendulums;
   displacement-, velocity-, and acceleration-level implicit constraints;
   Baumgarte, GearStableV, and GearStableA satisfaction; Euler-parameter
   relationships; and a Julia variable-step BDF reconstruction.
2. **Component formulation, September 1.** Explicit rigid-body variables,
   reactions, torsional and spanning forces, static equilibrium, analysis
   metadata, generic sparse assembly, and fixed Jacobian patterns.
3. **Supported modeling program, September 2.** Mechanisms, motion generators,
   TOML hierarchy, unified implicit kinematic and dynamic solution, HDF5
   results, automatic state selection, configuration correction, redundancy
   removal, preferred-state validation, and quasi-static analysis.
4. **Robustness and element library, September 3.** Zero-mass experiments,
   bushing damping time scale, relative joint coordinates, compliant contact,
   static initialization, dynamic relaxation, gears, joint primitives,
   spanning forces, rack and pinion, and translational generators.
5. **Systems and evidence, September 4.** Pulley belts, saved-result transfer,
   sparse factorization reuse, open- and closed-chain benchmarks, contact
   restart comparisons, stiff rotor trains, `.simp`, model graphics, and modal
   analysis.

This history explains why some executable examples are paper evidence rather
than supported interfaces. The `test/paper` suite should retain those studies
without forcing the supported `src` program to preserve obsolete APIs.

## 10. Present strengths

- One mechanical description supports ideal joints, compliant connections,
  open trees, closed loops, and loops physically opened by bushings.
- Reactions, accelerations, and component-local force quantities are direct
  solution variables and outputs.
- Components remain local; sparse ordering and factorization handle global
  coupling without making the modeling method a graph algorithm.
- Automatic analysis assembly reuses component equations across dynamic,
  kinematic, static, initialization, and modal work.
- State selection is automatic but retains physically meaningful user control.
- Stiff forces and smooth high-frequency systems are handled by the same BDF
  path.
- Stored results retain the full canonical history and original model while
  avoiding storage of internal integrator steps.

## 11. Present limitations and open decisions

- The executable model language is planar; the spatial architecture remains a
  design rather than a verified program.
- State selection is not yet revisited automatically during a long trajectory.
- Zero-mass and zero-inertia bodies are accepted in tested cases, but mobility
  diagnostics do not yet distinguish kinematic freedoms from algebraic
  force-balanced modes.
- Contact is compliant and continuous; impulse and complementarity methods are
  intentionally outside current scope.
- A `.simp` file has one primary analysis. Static, time-domain, and modal
  results cannot yet be selected from a common multi-analysis file.
- The real-gear model omits changing pressure-angle contact behavior.
- Long runs still allocate heavily.
- The paper boundary of the Fully Consistent method must remain clear: the
  component model is canonical, while state selection, BDF integration, and
  sparse factorization are cooperating solution methods.

## 12. Verification checkpoint and discovered paper-suite coupling

At commit `96d8077`, the supported suite completed 659 checks successfully.
During this consolidation, the paper suite initially failed to load because
its direct source includes did not reflect the new `ResultIO` and
`ModalAnalysis` dependencies. The working tree corrects that include order.

After the harness correction, the paper suite exposed that it is separately
invoked but not numerically frozen from the evolving `HistoricalDDASSL.jl`:

- The torque-driven four-bar paper test measured a maximum implicit-equation
  error of $1.2797\mathbin{\times}10^{-6}$ against an old
  $10^{-9}$ threshold and a maximum position-constraint error of
  $3.7305\mathbin{\times}10^{-12}$ against an old $10^{-12}$ threshold.
- The deficit-one pendulum measured a maximum velocity-constraint error of
  $2.5678\mathbin{\times}10^{-10}$ against an old $10^{-10}$ threshold.

These results were obtained after sparse symbolic and numerical factorization
reuse was added. They are not by themselves evidence of incorrect motion, but
they prevent the complete paper suite from being called green. Before paper
submission, either freeze a historical integrator configuration for formulation
comparisons or revise the thresholds with an explicit accuracy rationale. The
tests should not simply be relaxed to remove red marks.

## 13. Spatial handoff

The first 3D vertical slice should preserve these invariants:

- physical orientation is represented first by a rotation matrix;
- angular coordinates are local integration coordinates related to
  body-referenced infinitesimal rotation;
- acceleration remains explicit;
- markers have position and orientation with defaults;
- forces and ideal reactions are transmitted through marker frames;
- spatial components contribute local equation and Jacobian blocks;
- state selection acts on scaled physical velocity constraints;
- the complete active system remains the reference formulation.

A minimal implementation sequence is one free spatial rigid body, gravity and
applied torque, oriented markers, one spherical joint, and then a revolute
joint assembled from spatial primitives. A spatial pendulum should be the first
dynamic validation model, followed by a spatial bushing because it exercises
translation, rotation, stiffness, damping, and marker frames without a closed
ideal loop.
