# Automatic Analysis Selection for the Spanning-Spring Pendulum

Status: verified metadata-driven prototype

## 1. Purpose

This prototype uses one canonical model catalog to select variables and component equations for:

- consistent position initial conditions;
- consistent velocity initial conditions;
- consistent acceleration initial conditions;
- static equilibrium; and
- dynamics.

The catalog describes mathematical roles and derivative levels. It does not list five independent implementations for each component.

## 2. Variable metadata

Each canonical variable has:

- a stable canonical index and name;
- a physical kind;
- a derivative level;
- a component owner; and
- a scale placeholder.

The physical kinds used in this example are:

| Kind | Example |
|---|---|
| Position | $R_x$, $R_y$ |
| Orientation | $\theta$ |
| Velocity | $V_x$, $V_y$ |
| Angular velocity | $\omega$ |
| Acceleration | $a_x$, $a_y$ |
| Angular acceleration | $\alpha$ |
| Reaction | $\lambda_x$, $\lambda_y$ |
| Applied geometry | $s^g$, $\ell$, $\hat u^g$ |
| Applied rate | $\dot\ell$ |
| Applied load | $f$, $F^g$ |

These distinctions allow static equilibrium to retain applied geometry and loads while omitting velocity, acceleration, and the applied rate.

The rigid body, revolute pin, and spanning spring each declare their own variables. The catalog builder assigns consecutive canonical indices when the components are assembled. For this model the resulting blocks are:

| Component | Variable indices |
|---|---:|
| Rigid body | $1{:}9$ |
| Revolute pin | $10{:}11$ |
| Spanning spring-damper | $12{:}20$ |

## 3. Equation metadata

Each canonical scalar equation has:

- a stable index and name;
- a mathematical kind;
- a derivative level;
- a component owner; and
- an equation-family identifier.

The kinds used here are balance equations, constraints, selected-state equations, and applied-force definitions. The three pin-constraint levels share one family identifier.

Components may expose more than one equation block. This permits the rigid-body balance equations and its selected-state equations to occupy different parts of the assembled equation order without duplicating the body registration. The builder produces:

| Component block | Equation indices |
|---|---:|
| Rigid-body balances | $1{:}3$ |
| Revolute-pin constraints | $4{:}9$ |
| Rigid-body selected-state equations | $10{:}11$ |
| Spanning-force definitions | $12{:}20$ |

## 4. Selection policies

### Position initial conditions

Select position and orientation variables together with level-zero constraints, normalization equations, and coordinate relations. Applied-force geometry is not selected because a spring does not constrain the initial configuration.

For this model the component selection is

$$
3\text{ variables},\qquad 2\text{ constraints}.
$$

The weighted nearest-position procedure supplies the additional stationarity equations needed for a square projection problem.

### Velocity initial conditions

Select velocity and angular-velocity variables together with level-one constraints and relations:

$$
3\text{ variables},\qquad 2\text{ constraints}.
$$

The weighted nearest-velocity procedure supplies the stationarity equations.

### Acceleration initial conditions

Select accelerations, reactions, all applied-force variables, balance equations, acceleration constraints, and all applied-force definitions. Position and velocity are supplied as fixed initialized context:

$$
14\text{ variables},\qquad 14\text{ equations}.
$$

### Static equilibrium

Select positions, reactions, applied geometry, applied loads, balance equations, position constraints, and applied-force definitions other than the level-one length-rate definition:

$$
13\text{ variables},\qquad 13\text{ equations}.
$$

Velocity, acceleration, and $\dot\ell$ are supplied as zero context values.

### Dynamics

The complete canonical formulation selects all variables and equations:

$$
20\text{ variables},\qquad 20\text{ equations}.
$$

The selected-state and BDF treatment remains the responsibility of the dynamic solution policy.

## 5. Selected sets

The automatic catalog produces:

| Analysis | Variable indices | Equation indices |
|---|---|---|
| Position IC | $7{:}9$ | $8{:}9$ |
| Velocity IC | $4{:}6$ | $6{:}7$ |
| Acceleration IC | $1{:}3,10{:}20$ | $1{:}5,12{:}20$ |
| Static equilibrium | $7{:}16,18{:}20$ | $1{:}3,8{:}9,12{:}16,18{:}20$ |
| Dynamics | $1{:}20$ | $1{:}20$ |

The static selections are the same sets previously written manually. They are now consequences of metadata and policy rules.

## 6. Verification

The executable model registers eight owned equation blocks:

- rigid-body balance;
- pin acceleration, velocity, and position constraints;
- rigid-body selected-state equations;
- spring geometry, rate, and load definitions.

It also registers two contributions to the body balance rows: the pin reaction and the applied spring force and moment. An analysis evaluator calls only owned blocks whose rows are selected, followed by contributions whose target rows are active.

A position or velocity initialization can therefore be evaluated when the canonical spring length is zero. The spring geometry block, which would reject zero length, is not called. This verifies executable selection rather than evaluation followed by row filtering.

For the complete dynamic selection, the executable residual and Jacobian agree with the original monolithic implementations to the test tolerance.

Starting from deliberately inconsistent position and velocity guesses gives:

| Check | Result |
|---|---:|
| Position-constraint error | $9.27\times10^{-15}$ |
| Velocity-constraint error | $1.33\times10^{-17}$ |
| Acceleration-initialization equation error | $6.66\times10^{-16}$ |
| Acceleration Newton corrections | 1 |
| Static Newton corrections | 3 |
| Static equilibrium angle | $34.329079^\circ$ |

The acceleration and static systems evaluate only their selected component blocks and contributions. Their Newton matrices are assembled from the corresponding component-local Jacobian callbacks.

## 7. Present boundary

The selection rules, catalog builder, and selective executable evaluator are generic. The body, gravity, pin, and spring now use reusable component definitions. The physical components supply separate registrations, equation blocks, and contributions; the pendulum-specific executable assembly only combines them. Position and velocity projections still dispatch to the existing pendulum projection procedures after verifying the selected variable layout.

The components still receive assigned variable and equation locations during construction. The next architectural step is to assemble the modular two-body pendulum with these same definitions and use that example to develop automatic allocation of those locations.

## 8. Implementation

The generic metadata types, policies, and selective evaluator are in [`AutomaticAnalysis.jl`](../../src/common/AutomaticAnalysis.jl). Reusable planar component definitions are in [`PlanarComponentAssembly.jl`](../../src/planar/PlanarComponentAssembly.jl). The spanning-pendulum catalog and component construction are in [`spanning_pendulum_analysis_catalog.jl`](spanning_pendulum_analysis_catalog.jl); [`spanning_pendulum_executable_assembly.jl`](spanning_pendulum_executable_assembly.jl) combines their executable blocks and contributions. The analysis drivers and verification output are in [`automatic_spanning_pendulum_analyses.jl`](automatic_spanning_pendulum_analyses.jl).
