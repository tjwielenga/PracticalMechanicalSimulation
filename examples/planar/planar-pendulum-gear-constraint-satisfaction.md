# Planar Pendulum: GearStableV and GearStableA

Status: verified working example

## 1. Purpose

This example applies the Gear constraint-satisfaction formulations reviewed in
Chapter 14 of the historical notes. **GearStableV** retains the position and
velocity constraints and introduces a second set of Lagrange multiplier
variables that permits a minimum discrepancy between the velocity variables
and the derivatives of the position variables.

The formulation complements the acceleration-, velocity-, displacement-,
stabilized-, reduced-coordinate, and Fully Consistent pendulum examples. A second
experiment, **GearStableA**, extends the method to all three constraint levels.

## 2. Variables

Retain explicit acceleration, velocity, and position variables:

$$
a=
\begin{bmatrix}
a_x&a_y&\alpha
\end{bmatrix}^T,
$$

$$
\nu=
\begin{bmatrix}
V_x&V_y&\omega
\end{bmatrix}^T,
$$

$$
q=
\begin{bmatrix}
R_x&R_y&\theta
\end{bmatrix}^T.
$$

The physical pin reaction is

$$
\lambda^g=
\begin{bmatrix}
\lambda_x&\lambda_y
\end{bmatrix}^T,
$$

and the additional constraint-satisfaction multiplier is

$$
\mu^g=
\begin{bmatrix}
\mu_x&\mu_y
\end{bmatrix}^T.
$$

The complete solution vector is

$$
z=
\begin{bmatrix}
a^T&\nu^T&q^T&{\lambda^g}^T&{\mu^g}^T
\end{bmatrix}^T,
$$

and contains 13 scalar variables.

## 3. Mechanical equations

The force and torque equations are

$$
Ma-f-D^T\lambda^g=0,
$$

where

$$
D=
\begin{bmatrix}
I&d^g
\end{bmatrix}.
$$

The acceleration variables are related to the derivatives of the velocity variables by

$$
a-\dot\nu=0.
$$

The acceleration constraint is not imposed.

## 4. Position and velocity constraints

The position constraint is retained directly:

$$
\Phi(q)=R^g+r^g-p_0^g=0.
$$

The velocity constraint is also retained:

$$
\dot\Phi(q,\nu)=V^g+d^g\omega=0.
$$

Thus the body marker is constrained at both the position and velocity levels.

## 5. Modified kinematic differential equations

The usual relationship

$$
\dot q-\nu=0
$$

is replaced by

$$
\dot q-\nu+D^T\mu^g=0.
$$

Written by components,

$$
\dot R^g-V^g+\mu^g=0,
$$

$$
\dot\theta-\omega+{d^g}^T\mu^g=0.
$$

In the exact continuous solution, the ordinary kinematic relationship is compatible with both constraint levels and

$$
\mu^g=0.
$$

In the numerical solution, $\mu^g$ permits the position and velocity constraints to be satisfied simultaneously while the discrepancy between $\dot q$ and $\nu$ is minimized.

## 6. Minimum principle

For the initial unweighted formulation, minimize

$$
\frac{1}{2}(\dot q-\nu)^T(\dot q-\nu)
$$

subject to the position constraint within the BDF relationship between $q$ and $\dot q$. The stationarity equations introduce $D^T\mu^g$ into the kinematic differential equation.

The translational and rotational entries of $q$ have different physical units. The unweighted form is used here to reproduce the reviewed method directly. A general implementation should permit scaling or a weighting matrix before applying the method to systems with substantially different characteristic lengths.

## 7. Equation count and levels

| Equation block | Scalar equations |
|---|---:|
| Force and torque | 3 |
| Velocity constraint | 2 |
| Position constraint | 2 |
| Explicit acceleration definitions | 3 |
| Modified kinematic differential equations | 3 |
| **Total** | **13** |

The variable levels are

$$
\begin{bmatrix}
2&2&2&1&1&1&0&0&0&2&2&1&1
\end{bmatrix},
$$

and the equation levels are

$$
\begin{bmatrix}
2&2&2&1&1&0&0&2&2&2&1&1&1
\end{bmatrix}.
$$

The six velocity and position variables are differential variables and participate in integration-error control. Accelerations, physical reactions, and constraint-satisfaction multipliers remain algebraic solution variables, although their accepted values are retained in the BDF history.

## 8. Results

The pendulum was integrated for five seconds with relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$.

| Quantity | Result |
|---|---:|
| Accepted/rejected steps | 217 / 5 |
| Maximum BDF order | 5 |
| Newton iterations | 441 |
| Maximum implicit-equation error | $1.65\times10^{-12}$ |
| Maximum $\lVert\Phi\rVert_\infty$ | $1.11\times10^{-16}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.83\times10^{-14}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $2.33\times10^{-3}$ |
| Maximum $\lVert\dot q-\nu\rVert_\infty$ | $1.10\times10^{-4}$ |
| Maximum $\lVert\mu^g\rVert_\infty$ | $1.10\times10^{-4}$ |
| Maximum state difference from reduced reference | $5.40\times10^{-4}$ |
| Maximum reaction difference | $2.30\times10^{-3}$ |
| Maximum energy error | $1.46\times10^{-4}$ |

Both retained constraint levels are satisfied to the nonlinear-solution tolerance. The acceleration-constraint error and physical-reaction error are nearly the same as in the velocity-constraint formulation because neither method imposes $\ddot\Phi=0$.

The Gear formulation removes the position drift present in the velocity-only formulation. The price is a small discrepancy between the derivatives of the position variables and the separately retained velocity variables. The maximum discrepancy equals the scale of the added multiplier, as expected from the modified kinematic equations.

## 9. GearStableA constraint satisfaction

The extended formulation also imposes the acceleration constraint

$$
\ddot\Phi=a^g+d^g\alpha-r^g\omega^2=0.
$$

Introduce another multiplier

$$
\eta^g=
\begin{bmatrix}
\eta_x&\eta_y
\end{bmatrix}^T
$$

and replace the exact acceleration definitions with

$$
a-\dot\nu+D^T\eta^g=0.
$$

These are the stationarity equations associated with minimizing

$$
\frac{1}{2}(a-\dot\nu)^T(a-\dot\nu)
$$

subject to the acceleration constraint, in the same BDF context used for the position-and-velocity formulation. The complete vector is

$$
z=
\begin{bmatrix}
a^T&\nu^T&q^T&{\lambda^g}^T&{\mu^g}^T&{\eta^g}^T
\end{bmatrix}^T,
$$

with 15 scalar variables and 15 equations.

| Equation block | Scalar equations |
|---|---:|
| Force and torque | 3 |
| Acceleration constraint | 2 |
| Velocity constraint | 2 |
| Position constraint | 2 |
| Modified acceleration definitions | 3 |
| Modified kinematic differential equations | 3 |
| **Total** | **15** |

The variable levels are

$$
\begin{bmatrix}
2&2&2&1&1&1&0&0&0&2&2&1&1&2&2
\end{bmatrix},
$$

and the equation levels are

$$
\begin{bmatrix}
2&2&2&2&2&1&1&0&0&2&2&2&1&1&1
\end{bmatrix}.
$$

The six velocity and position variables remain the differential and error-controlled variables. Because the highest derivatives of interest are determined without numerically differentiating a retained constraint, the complete formulation has deficit zero.

Over five seconds, the complete formulation gives:

| Quantity | Result |
|---|---:|
| Accepted/rejected steps | 217 / 5 |
| Maximum BDF order | 5 |
| Newton iterations | 441 |
| Maximum implicit-equation error | $3.83\times10^{-9}$ |
| Maximum $\lVert\Phi\rVert_\infty$ | $1.11\times10^{-16}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.83\times10^{-14}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $3.83\times10^{-9}$ |
| Maximum $\lVert\dot q-\nu\rVert_\infty$ | $1.11\times10^{-4}$ |
| Maximum $\lVert a-\dot\nu\rVert_\infty$ | $2.33\times10^{-3}$ |
| Maximum state difference from reduced reference | $2.52\times10^{-4}$ |
| Maximum acceleration difference | $9.36\times10^{-4}$ |
| Maximum reaction difference | $7.55\times10^{-4}$ |
| Maximum energy error | $9.64\times10^{-5}$ |

The individual constraint-satisfaction multipliers varied over the following ranges:

| Multiplier component | Minimum | Maximum |
|---|---:|---:|
| $\mu_x$ | $-9.23\times10^{-5}$ | $7.69\times10^{-5}$ |
| $\mu_y$ | $-3.86\times10^{-5}$ | $1.11\times10^{-4}$ |
| $\eta_x$ | $-5.97\times10^{-4}$ | $2.29\times10^{-3}$ |
| $\eta_y$ | $-2.33\times10^{-3}$ | $7.01\times10^{-4}$ |

All three constraint levels are satisfied to the nonlinear-solution tolerance. The discrepancies have not disappeared; they have moved into the two modified differential relationships and are represented by $D^T\mu^g$ and $D^T\eta^g$. This is the expected consequence of enforcing more constraint equations without reducing the physical coordinate sets.

The multiplier values are not physical joint forces. The physical pin reaction remains $\lambda^g$. Instead, $D^T\mu^g$ measures the correction between the position derivatives and the velocity variables, while $D^T\eta^g$ measures the correction between the velocity derivatives and the explicit acceleration variables. Their magnitudes are therefore useful diagnostics of disagreement between adjacent derivative levels.

## 10. Comparison with related formulations

| Quantity | Velocity constraint | GearStableV | GearStableA | Fully Consistent |
|---|---:|---:|---:|---:|
| Simultaneous variables | 8 | 13 | 15 | 11 |
| Error-controlled variables | 6 | 6 | 6 | 2 |
| Retained $\Phi$ levels | $\dot\Phi$ | $\Phi$, $\dot\Phi$ | All three | All three |
| Accepted/rejected steps | 217 / 5 | 217 / 5 | 217 / 5 | 168 / 10 |
| Maximum position-constraint error | $1.05\times10^{-5}$ | $1.11\times10^{-16}$ | $1.11\times10^{-16}$ | approximately roundoff |
| Maximum velocity-constraint error | $3.83\times10^{-14}$ | $3.83\times10^{-14}$ | $3.83\times10^{-14}$ | approximately roundoff |
| Maximum acceleration-constraint error | $2.33\times10^{-3}$ | $2.33\times10^{-3}$ | $3.83\times10^{-9}$ | approximately roundoff |
| Maximum state difference | $5.35\times10^{-4}$ | $5.40\times10^{-4}$ | $2.52\times10^{-4}$ | $1.54\times10^{-4}$ |
| Maximum reaction difference | $2.30\times10^{-3}$ | $2.30\times10^{-3}$ | $7.55\times10^{-4}$ | $5.77\times10^{-4}$ |
| Maximum energy error | $1.87\times10^{-4}$ | $1.46\times10^{-4}$ | $9.64\times10^{-5}$ | $7.46\times10^{-5}$ |

GearStableA and the Fully Consistent equations both retain all three constraint
levels. They close the enlarged equation set differently. GearStableA uses two
multiplier sets to minimize discrepancies between derivative levels and
continues to integrate all six velocity and position variables. The Fully Consistent
equations instead select two independent state equations and integrate only
$\omega$ and $\theta$.

The executable implementation is in `gear_constraint_satisfaction_pendulum.jl`.
