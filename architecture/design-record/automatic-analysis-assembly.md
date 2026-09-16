# Automatic Assembly of Analysis Equation Sets

Status: design record; its central strategy is implemented in the planar program

This document preserves the sequence by which metadata-selected analyses were
developed. Later sections intentionally retain provisional interfaces and
future-tense conclusions. For the current implementation map, see the
[Technical Manual](../README.md), [`AutomaticAnalysis`](../../src/common/AutomaticAnalysis.jl),
and [`SimulationRunner`](../../src/planar/SimulationRunner.jl).

## 1. Purpose

A modular mechanical model should support several kinds of analysis without requiring every component to contain separate implementations for every analysis role. The initial set of anticipated analyses is:

- dynamic simulation;
- consistent position initial conditions;
- consistent velocity initial conditions;
- consistent acceleration initial conditions; and
- static equilibrium.

One possible design would assign roles such as `Dynamics`, `PositionICs`, `VelocityICs`, `AccelerationICs`, and `StaticEQ` to every component. This is workable, but it would repeat equations across roles and make it difficult to keep the repeated forms consistent.

The proposed alternative is to let components expose their mathematical capabilities once. System-level analysis policies then select, specialize, and augment those capabilities to build the required equation set.

This document records the strategy so that it can be reconsidered after applied forces have been introduced.

## 2. Why equation and variable levels are not sufficient

The derivative levels provide useful first-stage classification:

- level-zero variables and equations are candidates for position analysis;
- level-one variables and equations are candidates for velocity analysis; and
- level-two variables and equations are candidates for acceleration and dynamic analysis.

Levels alone do not identify:

- whether an equation is a constraint, balance equation, kinematic relation, normalization condition, or constitutive equation;
- which equations are different derivative forms of the same constraint;
- whether an equation remains meaningful in static equilibrium;
- which variables are prescribed;
- which constraint rows are redundant;
- which individual coordinates form a nonsingular dependent set; or
- how translational and rotational quantities should be scaled.

Automatic analysis assembly therefore requires equation and variable roles in addition to their levels. These roles should describe the mathematics rather than enumerate every analysis in which an item is used.

## 3. Component-supplied variable information

Each component supplies stable local variable identifiers together with information such as:

- physical kind: position, velocity, acceleration, reaction, auxiliary variable, or parameter;
- derivative level;
- component ownership;
- physical scale or characteristic unit;
- prescribed or free status, when known; and
- the corresponding variables at adjacent derivative levels, when applicable.

For example, a planar rigid body may expose

$$
R^g,\quad \theta,\quad V^g,\quad \omega,\quad a^g,\quad \alpha.
$$

These variables are declared once. An analysis policy determines which of them are unknowns for a particular calculation.

## 4. Component-supplied equation information

Each component supplies local equation blocks with information such as:

- physical kind: balance, constraint, kinematic relation, normalization, or constitutive equation;
- derivative level;
- component ownership;
- local variable dependencies;
- scaling information; and
- membership in an equation family.

Equation-family membership connects related forms. A pin connection, for example, can expose the family

$$
\Phi(q,t)=0,
$$

$$
\dot\Phi(q,\nu,t)=0,
$$

$$
\ddot\Phi(q,\nu,a,t)=0.
$$

The family identity records that these are three derivative forms of the same physical constraint. A nonholonomic constraint may begin at the velocity level and have no corresponding position equation.

## 5. Constraint-family interface

A constraint component should be able to evaluate whichever members of its family exist, together with their required partial derivatives. Conceptually, it provides:

```julia
ConstraintFamily(
    position!,
    velocity!,
    acceleration!,
    position_jacobian!,
    velocity_jacobian!,
    acceleration_jacobian!,
    holonomic,
)
```

The first implementation may use explicit component-local functions. Later, some differentiated forms or partial derivatives may be generated using automatic differentiation. The interface should not depend on how those quantities are produced.

In the executable planar catalog, each ideal scalar constraint is registered by
a `ConstraintFamilyDeclaration`. It gives the component-local family identifier
and explicitly names its reaction variable and its position-, velocity-, and
acceleration-level equations. Layout completion resolves those names to an
`AllocatedConstraintFamily` of canonical indices and validates their kinds and
levels. Redundant-row handling therefore deactivates a complete scalar family
by identity; it never infers correspondence from declaration order.

## 6. Structural balance equations

A structural component supplies its force and torque balance once. In a schematic form,

$$
Ma-f-D^T\lambda=0.
$$

The same balance equation participates in several analyses:

- in dynamics, positions, velocities, accelerations, and reactions participate in the simultaneous solution;
- in acceleration initialization, positions and velocities are known while accelerations and reactions are solved;
- in static equilibrium, velocity and acceleration are set to zero while positions and reactions are solved.

This reuse is preferable to giving a rigid body separate implementations of its dynamic, acceleration-initialization, and static-equilibrium equations.

## 7. Applied-force dependency

Reaction forces alone do not exercise the distinctions among the proposed analyses. Applied-force elements may depend on different combinations of

$$
q,\quad \nu,\quad a,\quad t,
$$

and may contain internal variables or implicit constitutive equations. Examples include gravity, springs, dampers, actuators, contact forces, and stiff bushings.

Before automatic analysis assembly is implemented, the applied-force interface must answer at least these questions:

1. Does the element contribute a force directly to structural balances, or does it also contribute implicit equations and local unknowns?
2. Which variables and derivative levels does the force require?
3. What is its meaning in static equilibrium?
4. Should velocity-dependent terms be evaluated at zero velocity, omitted, or treated through additional equilibrium conditions?
5. Does the element supply consistent position, velocity, or acceleration initialization equations for internal variables?
6. How are its component-local Jacobian contributions assembled and scaled?

For an ordinary damper, static evaluation at zero relative velocity naturally gives zero force. For an actuator, controller, contact law, or history-dependent constitutive element, the correct static interpretation may require an explicit component capability or an analysis-specific override.

Applied forces should therefore be developed before the automatic selection rules are finalized.

The first applied-force experiment is the planar torsional spring-damper documented in [`planar-torsional-spring-damper.md`](../../examples/planar/planar-torsional-spring-damper.md). It supports the proposed separation: the element supplies two markers, an explicit algebraic torque variable, a constitutive equation, and equal-and-opposite structural torque contributions. Position and velocity initialization do not select its equation as a constraint, while acceleration initialization, static equilibrium, and dynamics evaluate the same constitutive law under different known-variable contexts.

The planar spanning spring-damper in [`planar-spanning-spring-damper.md`](../../examples/planar/planar-spanning-spring-damper.md) extends this evidence to forces applied at point markers. It supplies nine local scalar variables and equations for $s^g$, $\ell$, $\hat u^g$, $\dot\ell$, $f$, and $F^g$. The applied global force is converted into both force and moment contributions in each connected structural component. This confirms that applied-force auxiliary equations must be selected for acceleration initialization, static equilibrium, and dynamics even though they are not kinematic constraints used for position or velocity consistency.

The static-equilibrium prototype in [`spanning-spring-static-equilibrium.md`](../../examples/planar/spanning-spring-static-equilibrium.md) maps a 13-variable static analysis view into the existing 20-variable dynamic component layout and evaluates the selected component equations and analytical Jacobian contributions. Acceleration, velocity, angular velocity, length rate, and BDF equations are absent from the static unknown set.

The next prototype, documented in [`automatic-spanning-pendulum-analyses.md`](../../examples/planar/automatic-spanning-pendulum-analyses.md), generates the position, velocity, acceleration, static, and dynamic selections from variable and equation metadata. It verifies all five policies on the same spring-loaded pendulum. The rigid body, revolute pin, and spanning spring register their own variable declarations and equation blocks; the catalog builder assigns canonical indices in assembly order. Component-local implicit-equation and Jacobian callbacks are associated with executable blocks, and pin and spring loads register contributions to the body balance rows. An analysis now evaluates only selected blocks and active contributions.

Equation ownership must remain distinct from equation contribution. The rigid body owns its force and torque balance rows. A pin owns its constraint rows and reaction variables but contributes reaction terms to the body balances. A spring owns its geometry and constitutive rows but contributes applied force and moment terms to the same body balances. The executable assembler therefore contains both owned equation blocks and contribution targets; assigning the balance equations themselves to every contributing component would duplicate their physical meaning.

The selective evaluator implements this distinction. A zero-length spring can remain unevaluated during position or velocity initialization because none of its equation blocks or contribution targets are active. During acceleration initialization, static equilibrium, and dynamics, its required blocks and balance contributions are activated by the selected equation rows.

The component callbacks have now been moved from the particular pendulum assembly into reusable planar component types. A rigid body owns its balance equations and its candidate state equations; the active analysis chooses the state pairs that close the system. Gravity is a separate contribution to the body's force balance. A ground revolute joint owns its position, velocity, and acceleration constraint blocks and contributes its reactions to the body balance. A spanning spring owns its geometry, rate, and load blocks and contributes its marker loads to every connected body. The pendulum assembly itself now only constructs these components, registers them in canonical order, and combines their blocks and contributions.

This is not yet a general model builder. The component objects presently receive their assigned variable and equation locations when they are constructed.

The same interfaces have now been applied to the open-tree two-body pendulum. This required generalizing the revolute joint from a special body-to-ground component to a two-marker component. Either marker may be fixed to ground; otherwise both markers contribute kinematics to the constraint equations and equal-and-opposite reactions to their bodies. The component assembly reproduces the existing 22 equations and analytical Jacobian and selects the expected position, velocity, and acceleration subsets.

The generic evaluator now also supports sparse Jacobian assembly. The same component-local callbacks write into an accumulator keyed by global row and column. Repeated contributions are added at the same location, and the selected entries are converted to `SparseMatrixCSC`. Explicitly assigned zero values are retained in the structural pattern because a term that is zero at one configuration may become nonzero as the mechanism moves. The reconstructed two-body model produces the same 82 stored locations as the hand-written sparse assembly and follows the same BDF trajectory.

For repeated solution, the sparse matrix built during initialization now serves as a structural contract. A fixed-pattern accumulator maps canonical component entries to the selected CSC locations, zeros only the stored numerical values, and accumulates the new Jacobian without changing `colptr` or `rowval`. Writing a selected entry outside the established pattern is an error. The dictionary-based path remains useful for initial pattern construction and for analysis selections with different row or column sets.

Canonical index assignment is now separated into two phases. First, components provide registrations containing only their variable and equation declarations. A model-layout builder assigns consecutive variable ranges and independently assigns equation-block ranges in the requested assembly order. Second, executable components and markers are constructed using those returned ranges. The two-body assembly therefore contains no literal global variable or equation indices, while retaining the previously verified ordering.

The present builder still requires the assembly to state the component order and the order of each component's equation blocks. This is intentional: physical components define capabilities, while the formulation chooses the equation ordering. A later convenience layer may provide a standard ordering policy, but the sparse solver remains free to reorder the completed matrix independently.

## 8. Prescribed motion and kinematic mechanisms

A rotational motion generator has been added as the first explicitly time-dependent constraint component. It retains a marker-relative angle, angular velocity, and angular acceleration, together with the torque required to impose the motion. At each kinematic level, one equation defines the relative quantity from the marker pair and a second equation equates it to the supplied motion function or its corresponding time derivative.

Adding this generator to the revolute pendulum reduces the mechanism to zero degrees of freedom. Four analysis policies solve successively for kinematic position, velocity, acceleration, and forces. The first three use only constraint, coordinate-relation, and prescribed-motion equations at their respective levels. The force analysis holds the known kinematics in the canonical context and solves body balance for the pin reactions and generator torque. Applied forces affect this last analysis but do not affect the prescribed kinematics.

The planar inplane constraint provides a complementary one-scalar ideal constraint. It enforces $(P_i^g-P_j^g)^T\hat u_j^g=0$, supplies its velocity and acceleration derivatives, and owns one scalar reaction along $\hat u_j^g$. Two parallel instances at separated body markers form a translational guide. Their velocity matrix makes angular velocity dependent and leaves a translational velocity as the independent state, confirming that state selection follows assembled constraint geometry rather than assuming angular coordinates.

The driven planar four-bar applies the same interfaces to an ideal closed loop.
Three bodies, four revolute joints, gravity, and one rotational motion generator
produce square position, velocity, acceleration, and force analyses without a
four-bar-specific equation type. A circle-intersection construction seeds the
first configuration, after which continuation uses the preceding solution's
positions, velocities, and accelerations for a second-order prediction before
constraint correction. This verifies closed-loop assembly while leaving singular-position
detection and the comparison with a loop opened by a stiff bushing as subsequent
developments. See [`driven-planar-four-bar.md`](../../examples/planar/driven-planar-four-bar.md).

Replacing the motion generator with a constant applied torque restores the
four-bar's single mechanical degree of freedom. The torque is a parameter-only
contribution: it owns neither a variable nor an equation. Selecting the crank
angle and angular velocity as the two differential states adds their two state
equations to the 33 body-balance and three-level joint-constraint equations,
giving a 35-variable deficit-zero dynamic system. See
[`torque-driven-planar-four-bar.md`](../../examples/planar/torque-driven-planar-four-bar.md).

The constant-speed slider-crank combines two revolute joints with one in-plane
constraint. The rod-end marker is constrained against the ground-fixed vertical
direction, leaving horizontal sliding motion, while a rotational generator
prescribes the crank motion. The existing metadata policies produce square
seven-variable position, velocity, and acceleration analyses and a six-variable
force analysis. No slider-crank-specific equation is required. See
[`constant-speed-slider-crank.md`](../../examples/planar/constant-speed-slider-crank.md).

Repeated layout-to-component construction and square-analysis solution have now
been collected in `PlanarModeling`. This remains a thin utility layer: examples
still state their topology, registrations, and equation-block ordering, while
the shared code constructs allocated components, assembles owned blocks and load
contributions, solves selected analyses, and predicts the next configuration.
See [`planar-modeling-utilities.md`](../planar/planar-modeling-utilities.md).

An initial TOML model-description layer now constructs the slider-crank from
named bodies, markers, connections, loads, and drivers. Mathematical time laws
are stored as strings and evaluated by a restricted arithmetic parser rather
than Julia `eval`. The loaded model reproduces the programmatic assembly while
keeping canonical locations and equation ordering out of the input file. See
[`toml-model-description.md`](../planar/toml-model-description.md).

## 9. System-level analysis policies

An analysis policy selects component-supplied variables and equations and generates any equations that belong to the numerical procedure rather than to a physical component.

### 9.1 Position initial conditions

The position-initialization policy selects:

- free position variables;
- level-zero constraints;
- normalization equations; and
- position-level relative-coordinate relations.

It generates a weighted nearest-position problem,

$$
\min_q\frac{1}{2}(q-q_0)^TW_q(q-q_0)
$$

subject to

$$
C_0(q)=0.
$$

The weighting and stationarity equations belong to the initialization procedure and are not supplied separately by every component.

### 9.2 Velocity initial conditions

At the initialized position, the velocity policy selects:

- free velocity variables;
- differentiated holonomic constraints;
- native nonholonomic constraints; and
- velocity-level coordinate relations.

It may generate the weighted projection

$$
\min_\nu\frac{1}{2}(\nu-\nu_0)^TW_\nu(\nu-\nu_0)
$$

subject to

$$
C_1(q,\nu,t)=0.
$$

### 9.3 Acceleration initial conditions

With position and velocity known, the acceleration policy selects:

- acceleration variables;
- physical reaction variables;
- structural balance equations;
- applied forces evaluated at the initialized state; and
- acceleration constraints.

For a simple constrained system this produces the familiar structure

$$
\begin{bmatrix}
M & -D^T\\
D & 0
\end{bmatrix}
\begin{bmatrix}
a\\
\lambda
\end{bmatrix}
=
\begin{bmatrix}
f\\
-\Phi^{(2)}
\end{bmatrix}.
$$

### 9.4 Static equilibrium

The static-equilibrium policy selects:

- position variables;
- physical reactions;
- position constraints;
- structural balance equations; and
- applied forces with the appropriate static interpretation.

The balances are evaluated with

$$
\nu=0,
\qquad
a=0.
$$

A weighted nearest-position condition may be required if the equilibrium is not otherwise unique.

### 9.5 Dynamics

A dynamics policy specifies the constraint formulation and state treatment. Examples include:

- acceleration-, velocity-, or displacement-level constraints;
- Baumgarte stabilization;
- GearStableV constraint satisfaction;
- GearStableA constraint satisfaction; and
- the Fully Consistent equations with selected independent states.

The policy generates the necessary BDF relationships, multiplier variables, stabilization terms, or selected-state equations. These are properties of the chosen solution formulation rather than separate physical equations supplied by every component.

## 10. Rank checking and coordinate selection

Coordinate selection must be performed on the assembled system. An individual component cannot know whether its variables or constraint rows become redundant when connected to the rest of the model.

For each initialization stage, the analysis builder should:

1. assemble and scale the relevant constraint partial matrix;
2. use pivoted QR on its transpose to identify independent constraint rows;
3. use column pivoting to select a usable set of dependent variables;
4. honor prescribed variables and preferred relative coordinates when possible; and
5. generate the appropriate projection or equilibrium equation set.

The levels identify candidate rows and columns. The factorization determines which candidates form independent sets.

## 11. Proposed assembly sequence

The complete process is:

1. Components register variables, equation blocks, constraint families, and contribution patterns.
2. An analysis policy selects the candidate variables and equations.
3. Applied-force elements are evaluated under the requested analysis context.
4. The selected constraint matrices are scaled and checked for rank.
5. Independent equations and dependent coordinates are selected.
6. The policy generates procedural equations such as weighted projection, BDF, stabilization, or selected-state equations.
7. Component-local contributions are assembled into the unreduced sparse system.

Conceptually, the user interface may eventually resemble:

```julia
model = assemble(components)

position_result = initialize!(model, PositionIC(target = q_guess))
velocity_result = initialize!(model, VelocityIC(target = v_guess))
acceleration_result = initialize!(model, AccelerationIC())
static_result = solve(model, StaticEQ())
dynamic_result = solve(model, Dynamics(formulation = FullEquations()))
```

The names of these interfaces are provisional. The important architectural boundary is that components expose mathematical capabilities, while analysis policies construct the equation set for a particular calculation.

## 12. Deferred decisions

Implementation should wait until applied-force elements clarify:

- the force-contribution interface;
- static evaluation of velocity- and history-dependent elements;
- treatment of force-element internal variables;
- component-local Jacobian interfaces; and
- initialization requirements for constitutive states.

Once those questions have been examined, this strategy should be tested first on the planar pendulum with gravity, then with a spring and damper, and finally on the modular two-body pendulum. Those examples should reveal whether the proposed metadata and analysis policies are sufficient without assigning every component a separate implementation for every analysis role.
