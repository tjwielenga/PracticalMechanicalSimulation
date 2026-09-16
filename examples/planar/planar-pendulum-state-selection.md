# Planar Pendulum: State Selection from Velocity Constraints

Status: internal working example

## 1. Purpose

This example applies the state-selection procedure to the planar rigid-body pendulum. Because the pendulum has one known degree of freedom, it provides a transparent test of whether pivoted QR selects physically useful individual coordinates.

The velocity variables are

$$
\nu=
\begin{bmatrix}
V_x & V_y & \omega
\end{bmatrix}^T.
$$

The expected independent variables are the angular velocity $\omega$ and its position-level partner, the relative pin angle $\theta$.

## 2. Velocity constraint partial

Let $r^g$ be the vector from the center of mass to the pin, and define

$$
d^g=Sr^g.
$$

The pin velocity constraint is

$$
V^g+d^g\omega=0.
$$

Consequently,

$$
D\nu=0,
\qquad
D=
\begin{bmatrix}
1&0&d_x\\
0&1&d_y
\end{bmatrix}.
$$

The matrix has two rows, three columns, and rank two. Two velocity components are dependent and one is independent.

## 3. Unscaled pivoted QR

The example uses the current pendulum data,

$$
\lVert r^g\rVert=0.5\ \mathrm{m}.
$$

The column norms of the unscaled constraint partial are therefore

$$
\lVert D_{V_x}\rVert=1,
\qquad
\lVert D_{V_y}\rVert=1,
\qquad
\lVert D_\omega\rVert=0.5.
$$

Column-pivoted QR chooses the two translational columns as the dependent block. Thus its actual selection is

$$
\nu_d=
\begin{bmatrix}
V_x\\
V_y
\end{bmatrix},
\qquad
\nu_i=\omega.
$$

For this model, the unscaled factorization does choose the desired independent angular velocity. This result depends partly on the numerical length of the pendulum and should not be interpreted as a dimensionally invariant rule.

## 4. Scaling and a preferred coordinate

Translational-velocity columns are dimensionless, while the angular-velocity column contains a length. A characteristic length $L$ can be used to put the trial velocity components on comparable scales. In this example,

$$
L=\lVert r^g\rVert.
$$

The scaled matrix is formed as

$$
\overline D
=D\operatorname{diag}(L,L,1).
$$

All three scaled columns then have norm $L$. Because the pivot magnitudes are tied, their precise order can depend on the QR implementation and roundoff. Scaling removes the dimensional bias, but it does not express the mechanical preference for $\omega$.

The clearest policy is therefore to request $\omega$ as the preferred independent variable and test its complementary dependent block:

$$
D_d=D[:,\{V_x,V_y\}]=I.
$$

This block is nonsingular and has condition number one, so the preferred selection is accepted without needing the QR fallback.

## 5. Automatically constructed tangent mapping

Partitioning the constraint gives

$$
D_d
\begin{bmatrix}
V_x\\
V_y
\end{bmatrix}
+d^g\omega=0.
$$

Solving for the dependent velocities gives

$$
\begin{bmatrix}
V_x\\
V_y
\end{bmatrix}
=-d^g\omega.
$$

The automatically constructed mapping is therefore

$$
\nu=P\omega,
\qquad
P=
\begin{bmatrix}
-d_x\\
-d_y\\
1
\end{bmatrix}.
$$

It satisfies

$$
DP=0.
$$

This is exactly the velocity mapping previously obtained from the pendulum geometry.

## 6. Selection of position states

The independent angular velocity is paired with the relative pin angle:

$$
\omega-\dot\theta=0.
$$

The Cartesian center-of-mass positions remain dependent quantities determined by the pin constraint,

$$
R^g=p_0^g-r^g(\theta).
$$

Thus the velocity-constraint procedure recovers the expected two first-order integration states,

$$
\theta,
\qquad
\omega.
$$

## 7. Conclusions from the experiment

For the present pendulum:

- the unscaled QR makes the mechanically preferred selection;
- the dependent block is the identity and is perfectly conditioned;
- the constructed tangent mapping is identical to the analytical mapping;
- no redundant constraint rows are present; and
- the selected velocity and position states reproduce the known one-degree-of-freedom formulation.

The experiment also shows why scaling and explicit coordinate preferences serve different purposes. Scaling removes unit and magnitude bias. A preferred-coordinate request records the mechanical meaning desired by the model author and is accepted only after its dependent block passes a rank and conditioning check.

The executable calculation is in `pendulum_state_selection.jl`.
