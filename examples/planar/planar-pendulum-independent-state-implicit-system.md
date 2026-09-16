# Planar Pendulum: Independent-State Implicit System

Status: internal working document

## 1. Purpose

This formulation retains a comparatively large simultaneous mechanical solution while integrating only an independent angular velocity and angular position. It demonstrates that the number of simultaneous solution variables need not equal the number of differential states or the number of variables controlling integration error.

The formulation has eleven simultaneous variables and eleven implicit equations, but only two differential states.

## 2. Variables

The solution vector is

$$
z=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta&\lambda_x&\lambda_y
\end{bmatrix}^{T}.
$$

The accelerations $a_x$, $a_y$, and $\alpha$ are explicit solution variables. They are not represented by the BDF derivatives of $V_x$, $V_y$, and $\omega$ in the force, torque, or acceleration-constraint equations. Only the two state relationships use BDF derivatives:

$$
\alpha-\dot\omega=0,
$$

$$
\omega-\dot\theta=0.
$$

The variable levels are

$$
\begin{bmatrix}
2&2&2&1&1&1&0&0&0&2&2
\end{bmatrix}.
$$

Only $\omega$ and $\theta$ are differential variables and participate in integration-error control:

$$
\text{differential variables}
=\text{error control}
=
\begin{bmatrix}
0&0&0&0&0&1&0&0&1&0&0
\end{bmatrix}.
$$

All eleven variables remain in the BDF history and are predicted before Newton iteration. The numerical derivatives of the nine algebraic variables are unused.

## 3. Equations

The mechanical and kinematic equations are

$$
m a^g-\lambda^g-mg^g=0,
$$

$$
J\alpha-(d^g)^T\lambda^g=0,
$$

$$
a^g+d^g\alpha-r^g\omega^2=0,
$$

$$
V^g+d^g\omega=0,
$$

$$
R^g+r^g-p_0^g=0,
$$

$$
\alpha-\dot\omega=0,
$$

and

$$
\omega-\dot\theta=0.
$$

The scalar equation count is

| Equation block | Count | Level |
|---|---:|---:|
| $\sum F$ | 2 | 2 |
| $\sum T$ | 1 | 2 |
| $\ddot\Phi$ | 2 | 2 |
| $\dot\Phi$ | 2 | 1 |
| $\Phi$ | 2 | 0 |
| $\alpha-\dot\omega$ | 1 | 2 |
| $\omega-\dot\theta$ | 1 | 1 |
| **Total** | **11** | |

Thus the equation levels are

$$
\begin{bmatrix}
2&2&2&2&2&1&1&0&0&2&1
\end{bmatrix}.
$$

The three constraint levels are not redundant in this equation selection. The displacement constraint determines $R^g$, the velocity constraint determines $V^g$, and the acceleration constraint participates in determining $a^g$, $\alpha$, and $\lambda^g$. Only $\theta$ and $\omega$ carry the motion from one accepted time to the next.

## 4. BDF relationships and scaling

The two BDF relationships used by the implicit equations are

$$
\dot\omega=CJ\,\omega+\omega'_H,
$$

$$
\dot\theta=CJ\,\theta+\theta'_H.
$$

For the level-two state equation $\alpha-\dot\omega=0$, the direct coefficient of level-two $\alpha$ remains constant. The BDF coefficient in the level-one $\omega$ column scales as

$$
h^{2-1}CJ\approx1.
$$

For the level-one equation $\omega-\dot\theta=0$, both the direct level-one $\omega$ coefficient and the scaled BDF coefficient of level-zero $\theta$ remain approximately constant. The level scaling therefore preserves the leading coefficients of the state equations as the step size changes.

## 5. Interpretation

This can be viewed as selecting the independent coordinates $\theta$ and $\omega$ for integration while retaining the larger mechanical system for simultaneous recovery of Cartesian position, velocity, acceleration, and pin reaction. It does not require the integrator to use a smaller solution vector or a specialized nonlinear solution interface:

$$
n_{\text{solution}}=n_{\text{equations}}=11,
\qquad
n_{\text{differential}}=n_{\text{error control}}=2.
$$

The computer experiment is implemented in [`independent_state_pendulum.jl`](independent_state_pendulum.jl). It compares every recovered mechanical quantity with the reduced-coordinate reference solution.

## 6. Computer results

The example was run for five seconds with relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$. Only $\omega$ and $\theta$ participate in integration-error control.

| Quantity | Result |
|---|---:|
| Accepted/rejected steps | 168 / 10 |
| Maximum BDF order | 5 |
| Newton iterations | 353 |
| Jacobian factorizations | 178 |
| Maximum implicit-equation error | $3.83\times10^{-9}$ |
| Maximum $[\theta,\omega]$ difference | $1.54\times10^{-4}$ |
| Maximum Cartesian position difference | $5.00\times10^{-5}$ |
| Maximum Cartesian velocity difference | $1.54\times10^{-4}$ |
| Maximum acceleration difference | $7.31\times10^{-4}$ |
| Maximum reaction difference | $5.77\times10^{-4}$ |
| Maximum energy error | $7.46\times10^{-5}$ |

The two integrated states follow the reduced-coordinate reference, and the algebraic displacement, velocity, acceleration, and reaction variables are recovered consistently from the simultaneous equations. Unlike the deficit-two displacement-constraint formulation, accelerations and reactions do not depend on repeated numerical differentiation of the constraint. All three constraint levels are present directly and determine different algebraic variable blocks.

The experiment confirms that the existing integrator can use a large square solution vector while restricting differential-state and error-control status to a small subset. No separate nonlinear solution interface or reduced history vector is required.
