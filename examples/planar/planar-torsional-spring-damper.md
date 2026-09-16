# Planar Torsional Spring-Damper

Status: verified working applied-force example

## 1. Purpose

This example introduces the first applied-force component beyond gravity. A torsional spring and damper connects an orientation marker on the pendulum body to an orientation marker on ground. It contributes torque to the structural balance but does not impose a kinematic constraint.

The implementation keeps the torque as an explicit algebraic variable. This preserves component locality and anticipates nonlinear or implicit constitutive force laws.

## 2. Orientation markers

Let marker 1 be fixed to the pendulum body and marker 2 be fixed to ground. Their planar unit axes, expressed in global coordinates, are

$$
e_1^g=
\begin{bmatrix}
\cos\theta_1\\
\sin\theta_1
\end{bmatrix},
\qquad
e_2^g=
\begin{bmatrix}
\cos\theta_2\\
\sin\theta_2
\end{bmatrix}.
$$

The components of the first marker axis in the second marker frame are

$$
x_r={e_2^g}^T e_1^g,
$$

$$
y_r=e_{2x}^g e_{1y}^g-e_{2y}^g e_{1x}^g.
$$

The relative angle is defined geometrically by

$$
\theta_r=\operatorname{atan2}(y_r,x_r).
$$

Its derivative is the relative angular velocity,

$$
\omega_r=\omega_1-\omega_2.
$$

For the present pendulum, marker 2 is fixed to ground with zero angle and angular velocity. Consequently,

$$
\theta_r=\theta,
\qquad
\omega_r=\omega.
$$

The marker formulation is retained in the implementation so that the same force element can connect two moving planar bodies.

## 3. Constitutive equation

Let $k$ be the torsional stiffness, $c$ the damping coefficient, and $\theta_0$ the free angle. Introduce the explicit torque variable $T$ and the implicit constitutive equation

$$
T-k(\theta_r-\theta_0)-c\omega_r=0.
$$

Positive $T$ produces equal-and-opposite restoring torques

$$
\tau_1=-T,
\qquad
\tau_2=+T.
$$

Only the first torque enters a structural balance in this example because the second marker is attached to ground.

## 4. Pendulum equation set

The example extends the eleven-variable full-equation pendulum by adding $T$:

$$
z=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta&
\lambda_x&\lambda_y&T
\end{bmatrix}^T.
$$

The torque balance becomes

$$
J\alpha-{d^g}^T\lambda^g+T=0.
$$

The sign follows from the definition that the spring applies $-T$ to the body marker.

| Equation block | Scalar equations |
|---|---:|
| Force and torque balances | 3 |
| Acceleration constraint | 2 |
| Velocity constraint | 2 |
| Position constraint | 2 |
| Selected angular state equations | 2 |
| Torsional constitutive equation | 1 |
| **Total** | **12** |

The torque is an algebraic, force-like level-two variable. Its constitutive equation is classified as a level-two equation because its leading coefficient with respect to $T$ is constant and it contributes directly to a level-two torque balance. Only $\omega$ and $\theta$ are differential and error-controlled variables.

## 5. Analysis behavior

The element supports the anticipated analysis policies without separate copies of its constitutive law.

| Analysis | Element behavior |
|---|---|
| Position initial conditions | No constraint equation; the spring does not restrict position |
| Velocity initial conditions | No constraint equation; the damper does not restrict velocity |
| Acceleration initial conditions | Evaluate $T$ from the initialized position and velocity and contribute it to torque balance |
| Static equilibrium | Set $\omega_r=0$, retain the spring term, and contribute $T$ to torque balance |
| Dynamics | Retain both the spring and damping terms |

This supports the proposed distinction between component capabilities and system-level analysis policies. The component supplies markers, a constitutive equation, an algebraic load variable, and structural torque contributions. The analysis determines which mechanical variables are fixed or solved.

## 6. Verification case

The five-second verification uses

$$
k=2,
\qquad
c=0.1,
\qquad
\theta_0=0,
$$

with initial angle $45$ degrees and zero angular velocity. Relative tolerance is $10^{-5}$ and absolute tolerance is $10^{-7}$.

| Quantity | Result |
|---|---:|
| Simultaneous variables | 12 |
| Differential and controlled variables | 2 |
| Accepted/rejected steps | 185 / 27 |
| Maximum BDF order | 5 |
| Maximum implicit-equation error | $8.08\times10^{-9}$ |
| Maximum constitutive torque error | $6.66\times10^{-16}$ |
| Maximum selected-state difference from reduced reference | $4.35\times10^{-4}$ |
| Minimum torque | $-1.44$ |
| Maximum torque | $1.57$ |
| Total energy change | $-1.58$ |

The reference solution is the corresponding reduced angular equation with the same spring and damping law. The negative change in total mechanical-plus-spring energy is expected because the damper dissipates energy.

## 7. Implementation

The reusable marker and force-element definitions are in [`PlanarAppliedForces.jl`](../../src/planar/PlanarAppliedForces.jl). The assembled pendulum, analytical Jacobian, reduced reference, and diagnostics are in [`torsional_spring_damper_pendulum.jl`](torsional_spring_damper_pendulum.jl).

The earlier generated browser animation was retired after completed simulations
moved to the common HDF5 result and SimpView workflow.

The present `atan2` evaluation returns its principal value. During ordinary dynamic motion, BDF prediction keeps the accepted angle history continuous. Selecting the appropriate $2\pi$-equivalent value during unusual initialization or multiple complete revolutions remains an initialization-level refinement.
