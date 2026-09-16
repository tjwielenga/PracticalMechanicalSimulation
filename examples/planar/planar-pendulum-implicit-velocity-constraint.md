# Planar Pendulum: Implicit Velocity-Constraint Formulation

Status: internal working document

Purpose: repeat the implicit solver experiment with the velocity-level pin constraint, giving a constraint-derivative deficit of one.

## 1. Change from the deficit-two formulation

The displacement-constraint experiment retained

$$
\Phi^g=0.
$$

This experiment replaces those two scalar equations with

$$
\dot\Phi^g
=V^g+d^g(\theta)\omega
=0.
$$

One differentiation of the retained constraint produces the acceleration constraint needed to determine the accelerations and pin reaction. The formulation therefore has a **constraint-derivative deficit of one**.

All other equation and variable selections remain unchanged so the effect of changing the constraint level can be observed directly.

## 2. Variables

The integrator variables remain

$$
y=
\begin{bmatrix}
R^g\\
\theta\\
V^g\\
\omega\\
\lambda^g
\end{bmatrix},
$$

with eight scalar entries. Their derivatives are

$$
\dot y=
\begin{bmatrix}
\dot R^g\\
\dot\theta\\
a^g\\
\alpha\\
\dot\lambda^g
\end{bmatrix}.
$$

## 3. Equations

The continuous implicit equations are

$$
F_1(t,y,\dot y)=
\begin{bmatrix}
m a^g-\lambda^g-mg^g\\
J\alpha-(d^g)^T\lambda^g\\
\dot R^g-V^g\\
\dot\theta-\omega\\
V^g+d^g\omega
\end{bmatrix}
=0.
$$

The equation count is

| Equation block | Scalar count |
|---|---:|
| $\sum F$ | 2 |
| $\sum T$ | 1 |
| $K_R$ | 2 |
| $K_\theta$ | 1 |
| $\dot\Phi$ | 2 |
| **Total** | **8** |

## 4. Equation structure

Blank entries are zero. The entries shown for the nonlinear constraint row are equation partials.

| Equations / variables | $a^g$ | $\alpha$ | $\dot\lambda^g$ | $\dot R^g$ | $\dot\theta$ | $\lambda^g$ | $V^g$ | $\omega$ | $R^g$ | $\theta$ | Count |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| $\sum F$ | $mI$ |  |  |  |  | $-I$ |  |  |  |  | 2 |
| $\sum T$ |  | $J$ |  |  |  | $-(d^g)^T$ |  |  |  |  | 1 |
| $K_R$ |  |  |  | $I$ |  |  | $-I$ |  |  |  | 2 |
| $K_\theta$ |  |  |  |  | $1$ |  |  | $-1$ |  |  | 1 |
| $\dot\Phi$ |  |  |  |  |  |  | $I$ | $d^g$ |  | $-r^g\omega$ | 2 |
| **Total** |  |  |  |  |  |  |  |  |  |  | **8** |

In the default mechanical error-control rule, this deficit-one formulation controls the level-zero position and level-one velocity variables. The level-two reactions remain in the solution history, are predicted at every step, and participate fully in Newton iteration, but are excluded from the integration-error norm. The velocity constraint itself is enforced directly as an implicit equation.

The displacement constraint is not enforced directly. If the initial position is consistent and the velocity constraint is satisfied exactly, then analytically

$$
\frac{d\Phi}{dt}=\dot\Phi=0
$$

preserves the initial value of $\Phi$. Numerically, integration and nonlinear-solution errors may allow position drift. The experiment must therefore report both $\Phi$ and $\dot\Phi$.

## 5. Computer experiment

The Julia program [`implicit_velocity_pendulum.jl`](implicit_velocity_pendulum.jl) implements the deficit-one equations, their analytical Newton matrix, and runs with the Julia DDASSL reconstruction, the SciML DASSL package, and IDA. It uses exactly the same consistent initial physical state and reduced-coordinate reference as the deficit-two experiment.

For the Julia reconstruction, the variable and equation levels are

$$
\text{variable levels}=
\begin{bmatrix}
0&0&0&1&1&1&2&2
\end{bmatrix},
$$

$$
\text{equation levels}=
\begin{bmatrix}
2&2&2&1&1&1&1&1
\end{bmatrix}.
$$

Both solver runs use tight absolute tolerances for $R^g$, $\theta$, $V^g$, and $\omega$, and a large absolute tolerance for $\lambda^g$. The latter causes the reaction variables to make a negligible contribution to the local-error test. The reaction remains fully coupled to the force and torque equations in the nonlinear solution.

Run it with

```text
julia --project=. examples/planar/implicit_velocity_pendulum.jl
```

The reported quantities include the solver return code, final time, displacement drift, velocity-constraint error, acceleration-constraint error, force and torque equation errors, and differences from the reduced-coordinate solution.

## 6. Initial results

With relative tolerance $10^{-8}$, absolute tolerance $10^{-9}$ on the six position and velocity variables, and a large absolute tolerance on the two reaction variables, both installed solvers complete the five-second simulation.

| Quantity | Native DASSL 3.0.1 | Sundials IDA |
|---|---:|---:|
| Accepted time points | 1,223 | 2,005 |
| Maximum $\lVert\Phi\rVert_\infty$ | $1.30\times10^{-9}$ | $4.46\times10^{-8}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.59\times10^{-9}$ | $1.38\times10^{-8}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $3.24\times10^{-5}$ | $2.65\times10^{-5}$ |
| Maximum state difference from reduced solution | $1.04\times10^{-7}$ | $7.06\times10^{-7}$ |
| Maximum reaction difference from reduced solution | $3.01\times10^{-5}$ | $1.44\times10^{-4}$ |

These results are much better than the deficit-two experiment. Retaining the velocity constraint allows position and velocity variables to remain in the error test while only the reaction variables are relaxed. The results are still preliminary: the accepted-point counts are not direct efficiency comparisons, and the effects of tolerance selection, solver order, internal interpolation, and Jacobian handling require further study.

Both solvers use the analytical Newton matrix. A directional finite-difference test verifies its action to the test tolerance. Supplying the matrix to IDA reduced its accepted-point count only from 2,083 to 2,005 and reduced nonlinear convergence failures from 18 to 14. The Jacobian was therefore worth supplying, but it was not the principal cause of the small time steps.

## 7. Julia reconstruction result

The Julia reconstruction was run for five seconds with relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$. These tolerances differ from the earlier results above, so the step counts are not direct efficiency comparisons.

| Quantity | Julia reconstruction |
|---|---:|
| Accepted time points | 218 |
| Maximum $\lVert\Phi\rVert_\infty$ | $1.05\times10^{-5}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.83\times10^{-14}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $2.33\times10^{-3}$ |
| Maximum state difference from reduced solution | $5.35\times10^{-4}$ |
| Maximum reaction difference from reduced solution | $2.30\times10^{-3}$ |
| Maximum energy error | $1.87\times10^{-4}$ |

The run takes 217 accepted steps and rejects 5 attempted steps. It reaches BDF order five, performs 440 Newton iterations and 222 Jacobian factorizations, and uses step sizes between $10^{-5}$ and $3.86\times10^{-2}$ seconds.

The retained velocity constraint is satisfied to roundoff in the saved solutions. Position drift remains because the displacement constraint is not enforced. The acceleration-constraint value is not a solved equation in this formulation and is correspondingly larger. Controlling velocity as well as position improves the state, reaction, energy, and constraint-drift measures compared with controlling position alone.
