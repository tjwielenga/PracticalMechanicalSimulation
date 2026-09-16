# Planar Pendulum: Implicit Displacement-Constraint Formulation

Status: internal working document

Purpose: organize the planar rigid-body pendulum as a continuous implicit equation system using only the displacement-level pin constraint.

## 1. Scope

The physical model and notation are defined in *Planar Rigid-Body Pendulum: Common Mechanical Equations*. This formulation is similar to an ADAMS-type formulation: the force and torque equations are combined with the kinematic differential equations and the displacement-level constraint.

The velocity and acceleration forms of the pin constraint remain true for a consistent solution, but they are not included as additional equation rows. The time integrator will later discretize the differential equations using a method such as BDF.

Using the terminology defined in the mathematical architecture, this formulation has a **constraint-derivative deficit of two**. The retained equation $\Phi=0$ must be differentiated twice to obtain $\ddot\Phi=0$, after which the force, torque, and acceleration-constraint equations determine the accelerations and pin reactions. A derivative of the pin reaction is not a mechanical quantity of interest and is not included in this classification.

## 2. Variables presented to the integrator

The variables presented to an implicit differential-algebraic equation integrator are

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

Their derivatives are presented as

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

The acceleration remains an explicit unknown in the unreduced mechanical system through

$$
a^g=\dot V^g,
\qquad
\alpha=\dot\omega.
$$

The derivative $\dot\lambda^g$ is formally part of $\dot y$, but it does not appear in the mechanical equations. The pin reaction $\lambda^g$ is an algebraic variable rather than a differential state.

The scalar variable count is

| Variable block | Scalar count | Type |
|---|---:|---|
| $R^g$ | 2 | Differential position variable |
| $\theta$ | 1 | Differential orientation variable |
| $V^g$ | 2 | Differential velocity variable |
| $\omega$ | 1 | Differential angular-velocity variable |
| $\lambda^g$ | 2 | Algebraic pin-reaction variable |
| **Total** | **8** | Six differential and two algebraic variables |

## 3. Continuous implicit equations

The force and torque equations are

$$
m a^g-\lambda^g-mg^g=0,
$$

$$
J\alpha-(d^g)^T\lambda^g=0.
$$

The kinematic differential equations are

$$
\dot R^g-V^g=0,
$$

$$
\dot\theta-\omega=0.
$$

The displacement-level pin constraint is

$$
\Phi^g
=R^g+r^g(\theta)-p_0^g
=0.
$$

Here

$$
r^g(\theta)=A^{gb}(\theta)r_m^b,
$$

and

$$
d^g(\theta)
=\frac{\partial r^g}{\partial\theta}
=A^{gb}(\theta)Sr_m^b.
$$

## 4. Equation structure

The following table shows the leading coefficient or equation partial associated with each variable. Blank entries are zero. The first five variable columns belong to $\dot y$; the remaining five belong to $y$.

| Equations / variables | $a^g$ | $\alpha$ | $\dot\lambda^g$ | $\dot R^g$ | $\dot\theta$ | $\lambda^g$ | $V^g$ | $\omega$ | $R^g$ | $\theta$ | Equation count |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| $\sum F$ | $mI$ |  |  |  |  | $-I$ |  |  |  |  | 2 |
| $\sum T$ |  | $J$ |  |  |  | $-(d^g)^T$ |  |  |  |  | 1 |
| $K_R$ |  |  |  | $I$ |  |  | $-I$ |  |  |  | 2 |
| $K_\theta$ |  |  |  |  | $1$ |  |  | $-1$ |  |  | 1 |
| $\Phi$ |  |  |  |  |  |  |  |  | $I$ | $d^g$ | 2 |
| **Total** |  |  |  |  |  |  |  |  |  |  | **8** |

The table displays derivatives and variables in separate columns to expose the equation structure; it does not imply that they are all separate integration variables. The integrator operates on the eight scalar entries of $y$ and supplies the corresponding entries of $\dot y$.

The zero column under $\dot\lambda^g$ makes the algebraic character of the pin reaction visible. The position constraint contains no derivative variables. This is therefore a displacement-constraint formulation.

## 5. Compact implicit form

Collect the equations in

$$
F(t,y,\dot y)=
\begin{bmatrix}
m a^g-\lambda^g-mg^g\\
J\alpha-(d^g)^T\lambda^g\\
\dot R^g-V^g\\
\dot\theta-\omega\\
R^g+r^g(\theta)-p_0^g
\end{bmatrix}
=0.
$$

This is a square system of eight scalar implicit equations for the eight scalar entries of $y$. At a time step, the integration method relates $y$ and $\dot y$. That discrete relationship will be introduced separately when the BDF formulation and its Newton equations are developed.

## 6. Error-control variables

All variables participate in the implicit equation solution, but they need not all participate in the integrator's local-error test. In the intended ADAMS-type treatment, error control will be suppressed for

$$
\lambda^g,
\qquad
V^g,
\qquad
\omega.
$$

The reaction is involved only as an algebraically determined variable, and its derivative is unneeded. The velocity variables do not have to satisfy the derivative of the constraint equation explicitly and are estimated by the integrator numerically. Their derivatives also do not have to satisfy the second derivative of the constraint equation and are even more unreliable. Consequently, they may be erratic and cause large error estimates.

The numerical consequences of this choice should be demonstrated with the computer model rather than claimed from the equation structure alone.

## 7. Backward differentiation formulas

A backward differentiation formula approximates the derivative at the current time using the current variable value and previously calculated variable values. The method is implicit because the current value $y_n$ is not known when the derivative is formed.

For a constant time step

$$
h=t_n-t_{n-1},
$$

the first-order backward differentiation formula, BDF1 or backward Euler, is

$$
\dot y_n
=\frac{y_n-y_{n-1}}{h}.
$$

The second-order formula is

$$
\dot y_n
=\frac{3y_n-4y_{n-1}+y_{n-2}}{2h}.
$$

A fixed-step BDF method of order $k$ can be written

$$
\dot y_n
=\frac{1}{h}
\sum_{j=0}^{k}\beta_j y_{n-j},
$$

where the coefficients $\beta_j$ depend on the order of the method. For the first two orders,

| Method | $\beta_0$ | $\beta_1$ | $\beta_2$ |
|---|---:|---:|---:|
| BDF1 | $1$ | $-1$ |  |
| BDF2 | $3/2$ | $-2$ | $1/2$ |

It is convenient to separate the current unknown from the previously calculated history:

$$
\dot y_n=\gamma y_n+b_n,
$$

where

$$
\gamma=\frac{\beta_0}{h},
$$

and

$$
b_n
=\frac{1}{h}
\sum_{j=1}^{k}\beta_j y_{n-j}.
$$

At the current time step, $\gamma$ and $b_n$ are known. Both $y_n$ and $\dot y_n$ change during the iteration because $\dot y_n$ depends on the current iterate for $y_n$.

## 8. Application to the pendulum variables

The BDF expression is applied component by component to the variables presented to the integrator. For the position variables,

$$
\dot R_n^g=\gamma R_n^g+b_{R,n},
$$

$$
\dot\theta_n=\gamma\theta_n+b_{\theta,n}.
$$

The kinematic differential equations require

$$
\dot R_n^g-V_n^g=0,
$$

$$
\dot\theta_n-\omega_n=0.
$$

Consequently, the velocity variables are equal to the BDF estimates of the position derivatives.

For the velocity variables,

$$
a_n^g
=\dot V_n^g
=\gamma V_n^g+b_{V,n},
$$

$$
\alpha_n
=\dot\omega_n
=\gamma\omega_n+b_{\omega,n}.
$$

These BDF estimates of acceleration enter the force and torque equations directly.

The same differentiation formula can formally produce

$$
\dot\lambda_n^g
=\gamma\lambda_n^g+b_{\lambda,n},
$$

but $\dot\lambda_n^g$ does not appear in any mechanical equation. Previous values of $\lambda^g$ therefore do not enter the current time-step equations through a differential relationship. The current reaction is determined algebraically by the force, torque, and constraint equations.

## 9. Time-step implicit equations

At time $t_n$, substitute

$$
\dot y_n=\gamma y_n+b_n
$$

into the continuous implicit equations:

$$
F(t_n,y_n,\dot y_n)=0.
$$

This produces the time-step implicit equation

$$
G(y_n)
=F\left(t_n,y_n,\gamma y_n+b_n\right)
=0.
$$

For the pendulum, the equations to be solved at the current time are

$$
m\left(\gamma V_n^g+b_{V,n}\right)
-\lambda_n^g-mg^g=0,
$$

$$
J\left(\gamma\omega_n+b_{\omega,n}\right)
-(d_n^g)^T\lambda_n^g=0,
$$

$$
\gamma R_n^g+b_{R,n}-V_n^g=0,
$$

$$
\gamma\theta_n+b_{\theta,n}-\omega_n=0,
$$

and

$$
R_n^g+r^g(\theta_n)-p_0^g=0.
$$

These are eight nonlinear scalar equations for the eight scalar entries of $y_n$. The nonlinearity enters through $r^g(\theta_n)$ and $d^g(\theta_n)$.

The displacement constraint is satisfied at the end of every converged time step. The velocity and acceleration forms of the pin constraint are not included as separate time-step equations.

## 10. Newton iteration

Let $y_n^{(i)}$ be the current estimate of the solution at time $t_n$. A Newton correction solves

$$
J_G\,\Delta y=-G,
$$

and updates

$$
y_n^{(i+1)}=y_n^{(i)}+\Delta y.
$$

Because

$$
\dot y_n=\gamma y_n+b_n,
$$

a correction to $y_n$ produces

$$
\Delta\dot y_n=\gamma\Delta y.
$$

The Newton matrix is therefore

$$
J_G
=F_y+\gamma F_{\dot y}.
$$

This expression connects the partial derivatives of the continuous component equations to the matrix required by the time integrator. The coefficient $\gamma$ is supplied by the integration method and changes when the step size or BDF order changes.

Newton convergence and local-error control serve different purposes. Every component of $y_n$, including $V_n^g$, $\omega_n$, and $\lambda_n^g$, participates in the Newton solution. Excluding selected variables from the local-error test affects the choice of step size and order, but it does not remove their equations or corrections from the Newton iteration.

The first computer implementation will use existing variable-step, variable-order BDF solvers through SciML. The fixed-step BDF1 and BDF2 equations above remain useful for explaining and checking the time-step equation and its Newton matrix.

## 11. SciML solver experiment

The accompanying Julia program [`implicit_displacement_pendulum.jl`](implicit_displacement_pendulum.jl) implements the eight continuous implicit equations once and presents them to the Julia DDASSL reconstruction, native SciML DASSL, and Sundials IDA. It also supplies exactly consistent initial values for $y_0$ and $\dot y_0$, an analytical matrix

$$
F_y+\gamma F_{\dot y},
$$

and comparisons with the reduced-coordinate reference solution.

The native DASSL interface accepts vector relative and absolute tolerances. The example assigns tight tolerances to the three displacement variables and independently adjustable large absolute tolerances to the five velocity and reaction variables. The current DASSL `DAEProblem` interface does not return its derivative history, so the example uses the package's direct integration interface with the same implicit-equation function and the analytical Newton matrix.

The initial experiment is informative but is not yet a successful validation of either general-purpose solver for the deficit-two formulation:

- With DASSL 3.0.1, an extremely large ignored-variable tolerance allows integration to reach the final time, but it also weakens the weighted Newton convergence test and produces inaccurate velocities and reactions. Reducing that tolerance eventually causes the step size to collapse. The installed package then encounters a logging-interface error while reporting the small-step failure.
- With Sundials.jl 6.3.0 and SUNDIALS 7.5.0, IDA preserves the displacement constraint during its attempted startup but repeatedly fails its error test near the initial time and does not complete the requested interval.

These observations support the distinction made in Section 6: excluding variables from local-error control and solving all variables accurately in the Newton iteration are separate requirements. A single tolerance weight used for both purposes does not provide that separation.

Loosening the controlled relative and absolute tolerances to $10^{-5}$ changes the IDA behavior but does not produce a satisfactory deficit-two solution. With ordinary tolerances on all eight variables, IDA still fails near the initial time. With $10^{-5}$ on the three displacement variables and progressively larger absolute tolerances on velocities and reactions, the observed behavior is:

| Relaxed absolute tolerance | Result | Time points | Maximum position-state difference | Maximum velocity-state difference | Maximum reaction difference |
|---:|---|---:|---:|---:|---:|
| $10^{-2}$ | Error-test failure at $t=0.052$ s | 11 | -- | -- | -- |
| $10^{-1}$ | Error-test failure at $t=0.403$ s | 49 | -- | -- | -- |
| $1$ | Reaches 5 s | 454 | $1.54\times10^{-2}$ | $6.99\times10^{-2}$ | $1.12$ |
| $10$ | Reaches 5 s | 328 | $1.43\times10^{-2}$ | $5.47\times10^{-2}$ | $1.33$ |
| $10^2$ | Reaches 5 s | 307 | $2.23\times10^{-2}$ | $9.65\times10^{-2}$ | $1.40$ |

The displacement constraint itself remains accurately satisfied in the converged time-step equations, but that does not guarantee an accurate trajectory. Once velocity and reaction weights are sufficiently loose to permit larger steps, errors in those quantities feed back through the dynamics and accumulate in the position history. This is the behavior that the specialized ADAMS-type error-control and convergence strategy must avoid.

The experiment can be run from the project directory with

```text
julia --project=. examples/planar/implicit_displacement_pendulum.jl
```

The program reports the solver return code, final time, constraint errors, force and torque equation errors, and differences from the reduced-coordinate solution. These results are implementation and tolerance dependent and should be treated as solver evaluation, not as general efficiency conclusions.

## 12. Julia reconstruction result

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
2&2&2&1&1&1&0&0
\end{bmatrix}.
$$

The first six variables are differential variables and the two reactions are algebraic. With deficit two, the default error-control mask selects only the three level-zero position variables:

$$
\begin{bmatrix}
1&1&1&0&0&0&0&0
\end{bmatrix}.
$$

The reconstruction was run for five seconds with relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$.

| Quantity | Julia reconstruction |
|---|---:|
| Accepted time points | 196 |
| Maximum $\lVert\Phi\rVert_\infty$ | $2.07\times10^{-14}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.46\times10^{-4}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $2.89\times10^{-2}$ |
| Maximum state difference from reduced solution | $8.14\times10^{-4}$ |
| Maximum reaction difference from reduced solution | $2.88\times10^{-2}$ |
| Maximum energy error | $2.24\times10^{-4}$ |

The run takes 195 accepted steps and rejects 14 attempted steps. It reaches BDF order five, performs 415 Newton iterations and 209 Jacobian factorizations, and uses step sizes between $10^{-5}$ and $3.90\times10^{-2}$ seconds.

The retained displacement constraint is satisfied to roundoff, and the position and velocity trajectory remains reasonably close to the reduced solution. Acceleration and reaction errors remain appreciably larger than in the lower-deficit formulations. This is consistent with the deficit-two interpretation: velocity is obtained through one numerical differentiation level and acceleration through a second. The force equations can be satisfied using less accurate acceleration estimates and correspondingly less accurate reactions. Accurate position integration therefore does not by itself establish acceleration or reaction accuracy for this formulation.
