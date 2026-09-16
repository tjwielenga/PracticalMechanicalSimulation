# Planar Pendulum: Implicit Acceleration-Constraint Formulation

Status: internal working document

Purpose: complete the three constraint-level experiments by retaining the acceleration-level pin constraint, giving a constraint-derivative deficit of zero.

## 1. Constraint selection

This formulation retains

$$
\ddot\Phi^g
=a^g+d^g(\theta)\alpha-r^g(\theta)\omega^2
=0.
$$

No further differentiation of the constraint is needed to determine the accelerations and pin reaction. The formulation therefore has a **constraint-derivative deficit of zero**.

The displacement and velocity constraints remain true for an exact consistent motion, but neither is included as a time-step equation. Numerical errors in those lower-level relationships can consequently accumulate.

## 2. Variables and equations

The eight scalar integrator variables remain

$$
y=
\begin{bmatrix}
R^g\\
\theta\\
V^g\\
\omega\\
\lambda^g
\end{bmatrix}.
$$

The continuous implicit equations are

$$
F_0(t,y,\dot y)=
\begin{bmatrix}
m a^g-\lambda^g-mg^g\\
J\alpha-(d^g)^T\lambda^g\\
\dot R^g-V^g\\
\dot\theta-\omega\\
a^g+d^g\alpha-r^g\omega^2
\end{bmatrix}
=0.
$$

The equation count is unchanged:

| Equation block | Scalar count |
|---|---:|
| $\sum F$ | 2 |
| $\sum T$ | 1 |
| $K_R$ | 2 |
| $K_\theta$ | 1 |
| $\ddot\Phi$ | 2 |
| **Total** | **8** |

## 3. Equation structure

Blank entries are zero. The acceleration constraint contains derivative coefficients as well as partials with respect to the current position and velocity variables.

| Equations / variables | $a^g$ | $\alpha$ | $\dot\lambda^g$ | $\dot R^g$ | $\dot\theta$ | $\lambda^g$ | $V^g$ | $\omega$ | $R^g$ | $\theta$ | Count |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| $\sum F$ | $mI$ |  |  |  |  | $-I$ |  |  |  |  | 2 |
| $\sum T$ |  | $J$ |  |  |  | $-(d^g)^T$ |  |  |  |  | 1 |
| $K_R$ |  |  |  | $I$ |  |  | $-I$ |  |  |  | 2 |
| $K_\theta$ |  |  |  |  | $1$ |  |  | $-1$ |  |  | 1 |
| $\ddot\Phi$ | $I$ | $d^g$ |  |  |  |  |  | $-2r^g\omega$ |  | $-r^g\alpha-d^g\omega^2$ | 2 |
| **Total** |  |  |  |  |  |  |  |  |  |  | **8** |

The table entries in the $\omega$ and $\theta$ columns are the partials of

$$
a^g+d^g\alpha-r^g\omega^2
$$

with respect to those variables.

## 4. Error control and expected drift

Positions and velocities remain differential variables and participate in the local-error test. Only the algebraic reaction variables receive relaxed absolute tolerances.

Because $\ddot\Phi=0$ is enforced directly, numerical errors in $\dot\Phi$ are not corrected. Any accumulated velocity-constraint error can then produce continuing displacement drift. This experiment should therefore distinguish:

- satisfaction of the retained acceleration constraint;
- drift in the velocity constraint;
- drift in the displacement constraint; and
- agreement with the reduced-coordinate reference motion.

## 5. Computer experiment

The Julia program [`implicit_acceleration_pendulum.jl`](implicit_acceleration_pendulum.jl) implements the deficit-zero equations and their analytical Newton matrix for the Julia DDASSL reconstruction, the SciML DASSL package, and IDA.

For the Julia reconstruction, the variable levels are

$$
\begin{bmatrix}
0&0&0&1&1&1&2&2
\end{bmatrix},
$$

for position, velocity, and reaction variables respectively. The equation levels are

$$
\begin{bmatrix}
2&2&2&1&1&1&2&2
\end{bmatrix}.
$$

Because this is a deficit-zero formulation, the six position and velocity variables participate in integration-error control. The two reaction variables remain in the BDF history and are predicted before every Newton solution, but are excluded from the integration-error norm.

The initial experiment uses relative and controlled absolute tolerances of $10^{-5}$. The reaction variables use an absolute tolerance of $10^{-3}$. This makes their local-error test one hundred times looser without allowing inaccurate reaction estimates to weaken the Newton solution excessively.

The results over five seconds are:

| Quantity | Julia reconstruction | DASSL | IDA |
|---|---:|---:|---:|
| Accepted time points | 218 | 352 | 490 |
| Maximum $\lVert\Phi\rVert_\infty$ | $5.10\times10^{-4}$ | $1.60\times10^{-4}$ | $1.95\times10^{-4}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $2.58\times10^{-4}$ | $6.09\times10^{-5}$ | $1.05\times10^{-4}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $7.66\times10^{-9}$ | $9.17\times10^{-4}$ | $3.32\times10^{-3}$ |
| Maximum state difference from reduced solution | $5.13\times10^{-4}$ | $1.59\times10^{-4}$ | $2.09\times10^{-4}$ |
| Maximum reaction difference from reduced solution | $3.79\times10^{-4}$ | $9.23\times10^{-4}$ | $2.97\times10^{-3}$ |
| Maximum energy error | $5.16\times10^{-3}$ | $1.51\times10^{-3}$ | $2.12\times10^{-3}$ |

The Julia reconstruction takes 217 accepted steps and rejects 6 attempted steps. It reaches BDF order five, performs 443 Newton iterations and 223 Jacobian factorizations, and uses step sizes between $10^{-5}$ and $3.88\times10^{-2}$ seconds. These statistics are diagnostic rather than efficiency claims; the three implementations do not yet use identical error estimators or nonlinear-solution tolerances.

Both solvers complete successfully. As expected, directly imposing $\ddot\Phi=0$ does not remove accumulated errors in $\dot\Phi$ or $\Phi$. The experiment therefore provides the deficit-zero endpoint, but it should not yet be read as a general efficiency comparison among formulations.

Run it with

```text
julia --project=. examples/planar/implicit_acceleration_pendulum.jl
```
