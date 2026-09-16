# Spanning-Spring Pendulum Static Equilibrium

Status: verified analysis-selection prototype

## 1. Purpose

This example calculates the static equilibrium of the pendulum loaded by gravity and the spanning spring. It is the first direct test of selecting a different equation and variable set from the same component definitions for a different type of analysis.

The static analysis reuses the rigid-body balance, pin constraint, and spanning-force equations developed for dynamics. It does not give the body or force element a separate static implementation.

## 2. Static assumptions

At static equilibrium,

$$
V^g=0,
\qquad
\omega=0,
\qquad
a^g=0,
\qquad
\alpha=0.
$$

The physical length rate is consequently

$$
\dot\ell=0.
$$

The damping term vanishes, but the spring force remains:

$$
f=-k(\ell-\ell_0).
$$

The damper coefficient therefore cannot affect the static solution.

## 3. Selected variables

The static unknown vector is

$$
y=
\begin{bmatrix}
R_x&R_y&\theta&\lambda_x&\lambda_y&
s_x&s_y&\ell&u_x&u_y&f&F_x&F_y
\end{bmatrix}^T.
$$

It contains 13 scalar variables. Dynamic variables not required for the analysis are omitted rather than added as unknowns with separate zero equations.

| Variable block | Scalars |
|---|---:|
| Body position and angle | 3 |
| Pin reaction | 2 |
| Spanning vector | 2 |
| Length | 1 |
| Unit direction | 2 |
| Scalar spring force | 1 |
| Global force | 2 |
| **Total** | **13** |

## 4. Selected equations

The force and torque balances are evaluated with zero acceleration:

$$
-\lambda^g-mg^g-F^g=0,
$$

$$
-{d_p^g}^T\lambda^g-{d_f^g}^TF^g=0.
$$

The pin position constraint is retained:

$$
R^g+r_p^g-p_0^g=0.
$$

The selected spanning-force equations are

$$
s^g-(P_2^g-P_1^g)=0,
$$

$$
\ell-\sqrt{{s^g}^Ts^g}=0,
$$

$$
\hat u^g-\frac{s^g}{\ell}=0,
$$

$$
f-k(\ell-\ell_0)=0,
$$

$$
F^g-\hat u^g f=0.
$$

The length-rate equation is not selected because $\dot\ell=0$ is fixed by the static analysis. The remaining blocks provide 13 scalar equations.

| Equation block | Scalars |
|---|---:|
| Force and torque equilibrium | 3 |
| Pin position constraint | 2 |
| Spanning-vector definition | 2 |
| Length definition | 1 |
| Unit-direction definition | 2 |
| Static spring law | 1 |
| Global-force definition | 2 |
| **Total** | **13** |

## 5. Reusing the dynamic component equations

The prototype maps the 13 static variables into the canonical 20-variable dynamic component state. It supplies zero values for acceleration, velocity, angular velocity, and length rate. It then evaluates only the selected component equation blocks and contributions.

The static Jacobian is assembled from the callbacks associated with those selected blocks and contributions. Unselected velocity, acceleration, length-rate, and selected-state blocks are not evaluated.

The row and column selections are now generated from the variable and equation metadata described in [`automatic-spanning-pendulum-analyses.md`](automatic-spanning-pendulum-analyses.md). This demonstrates that a system-level analysis view can reuse component equations without copying or rewriting their mathematical definitions.

## 6. Numerical solution

The same force data used in the dynamic example are retained:

$$
k=20,
\qquad
\ell_0=0.5,
\qquad
P_2^g=
\begin{bmatrix}
0.8&-0.2
\end{bmatrix}^T.
$$

Starting from an angle estimate of $35$ degrees, Newton iteration converges in three corrections.

| Quantity | Result |
|---|---:|
| Equilibrium angle | $34.329079^\circ$ |
| Pin reaction | $[-1.19184,\ 6.65027]^T$ |
| Spring length | $0.668852$ |
| Scalar spring force | $-3.37704$ |
| Global spring force | $[1.19184,\ 3.15973]^T$ |
| Maximum equation error | $4.44\times10^{-16}$ |
| Reduced moment-equilibrium error | $4.44\times10^{-16}$ |
| Newton corrections | 3 |
| Jacobian condition estimate | $392$ |

Changing the damping coefficient from $0.5$ to $50$ leaves the static result unchanged to the test tolerance.

## 7. Implementation

The equation selection, static state mapping, analytical Jacobian selection, Newton iteration, and diagnostics are implemented in [`spanning_spring_static_equilibrium.jl`](spanning_spring_static_equilibrium.jl).

The implementation uses the shared metadata-driven selection and executable assembly policies. Its zero context values specialize the same spring load equation to static equilibrium without evaluating the spring length-rate block.
