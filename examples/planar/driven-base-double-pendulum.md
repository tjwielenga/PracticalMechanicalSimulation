# Double Pendulum with a Driven Base

Status: verified full-equation, one-degree-of-freedom example

## 1. Mechanical model

The model has two planar rigid bodies. Body 1 is connected to ground by a revolute joint, and body 2 is connected to the lower marker of body 1 by a second revolute joint. A rotational motion generator prescribes the orientation of body 1 relative to ground:

$$
\theta_1=g(t).
$$

Without the generator, the open-chain double pendulum has two degrees of freedom. Prescribing the base rotation removes one, leaving one dynamic degree of freedom.

## 2. State selection with prescribed motion

Order the six body velocity components as

$$
v=
\begin{bmatrix}
V_{1x}&V_{1y}&\omega_1&V_{2x}&V_{2y}&\omega_2
\end{bmatrix}^T.
$$

The two revolute joints contribute four scalar velocity constraints. The motion generator contributes

$$
\omega_1=\dot g(t).
$$

Together they have the nonhomogeneous form

$$
D(q)v=b(t),
$$

where $D$ has five rows and six columns. The time-dependent right-hand side affects the velocity values but not the dependent-column selection. The assembled matrix has numerical rank five, so its nullity is one.

The preferred independent component is tested directly:

$$
v_i=\omega_2.
$$

Its complementary five-column block is nonsingular. The selected integration states are therefore

$$
\omega_2,qquad \theta_2,
$$

with closing equations

$$
\alpha_2-\dot\omega_2=0,
$$

$$
\omega_2-\dot\theta_2=0.
$$

The tangent mapping satisfies

$$
DP=0
$$

to machine precision. It describes allowable changes about the particular velocity imposed by $b(t)$.

## 3. Full equation set

The assembled model retains 26 simultaneous variables:

| Component | Variables | Count |
|---|---|---:|
| Two rigid bodies | accelerations, velocities, positions and orientations | 18 |
| Two revolute joints | reaction-force components | 4 |
| Motion generator | $\theta_m$, $\omega_m$, $\alpha_m$, $\tau_m$ | 4 |
| **Total** |  | **26** |

The 26 equations are:

| Equation block | Count |
|---|---:|
| Force and torque balances for two bodies | 6 |
| Position, velocity and acceleration constraints for two pins | 12 |
| Position, velocity and acceleration equations for the generator | 6 |
| Selected state equations for body 2 | 2 |
| **Total** | **26** |

The base position, velocity and acceleration remain algebraic quantities prescribed by

$$
g(t),\qquad \dot g(t),\qquad \ddot g(t).
$$

Only $\omega_2$ and $\theta_2$ are differential variables and participate in integration-error control. All 26 variables remain in the simultaneous Newton solution and in the accepted BDF history.

## 4. Initialization

The initial base angle and angular velocity come from the motion generator. The user supplies the initial angle and angular velocity of body 2. The position and velocity constraints reconstruct the two body-center states.

With those quantities fixed, a 12-equation linear system solves for:

- six body acceleration components;
- four pin-reaction components;
- the retained generator angular acceleration; and
- the generator torque.

The equations used are the six body balances, four pin acceleration constraints, and two generator acceleration equations.

## 5. Numerical result

For the initial demonstration, a two-second BDF solution gives:

| Quantity | Result |
|---|---:|
| Simultaneous variables and equations | 26 |
| Integrated and error-controlled states | 2 |
| Maximum implicit-equation error | $2.99\times10^{-14}$ |
| Maximum position-constraint error | $2.22\times10^{-16}$ |
| Maximum prescribed-angle error | $1.11\times10^{-16}$ |
| Accepted steps | 117 |
| Rejected steps | 0 |
| Maximum BDF order | 5 |

These results verify the formulation and state selection. They are not an efficiency comparison.

The implementation is in [`driven_base_double_pendulum.jl`](driven_base_double_pendulum.jl).

The earlier generated browser animation was retired after completed simulations
moved to the common HDF5 result and SimpView workflow.
