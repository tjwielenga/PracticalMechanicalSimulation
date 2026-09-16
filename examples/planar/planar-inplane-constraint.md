# Planar Inplane Constraint

Status: verified component and state-selection example

## 1. Definition

The inplane constraint connects point markers $i$ and $j$ and prevents their relative motion in the direction of a unit vector $\hat u_j^g$ fixed in marker $j$. Motion remains possible in the planar direction perpendicular to this vector.

Define the spanning vector

$$
s^g=P_i^g-P_j^g.
$$

The position constraint is

$$
\Phi={s^g}^T\hat u_j^g=0.
$$

If marker $j$ belongs to a moving body, $\hat u_j^g$ rotates with that body. Define its perpendicular direction by

$$
\hat n_j^g=S\hat u_j^g,
\qquad
S=
\begin{bmatrix}
0&-1\\
1&0
\end{bmatrix}.
$$

Then

$$
\dot{\hat u}_j^g=\hat n_j^g\omega_j,
$$

and

$$
\ddot{\hat u}_j^g
=\hat n_j^g\alpha_j-\hat u_j^g\omega_j^2.
$$

## 2. Velocity and acceleration equations

Let

$$
v_r^g=V_i^g-V_j^g,
\qquad
a_r^g=a_i^g-a_j^g.
$$

The velocity constraint is

$$
\dot\Phi
={v_r^g}^T\hat u_j^g
+{s^g}^T\hat n_j^g\omega_j
=0.
$$

The acceleration constraint is

$$
\ddot\Phi
={a_r^g}^T\hat u_j^g
+2{v_r^g}^T\hat n_j^g\omega_j
+{s^g}^T
\left(
\hat n_j^g\alpha_j-\hat u_j^g\omega_j^2
\right)
=0.
$$

The point-marker accelerations in $a_r^g$ already include the translational acceleration of the owning body and the tangential and centripetal terms caused by marker offset.

## 3. Reaction variable

The constraint owns one scalar reaction variable $\lambda$. Its global reaction force is

$$
F^g=\lambda\hat u_j^g.
$$

The force is applied at marker $i$, and the equal-and-opposite force is applied at marker $j$. Each connected rigid body converts its marker force into force and torque contributions to its balance equations.

## 4. Translational state-selection example

Consider one planar body with two marker offsets

$$
r_1^b=
\begin{bmatrix}0\\0.5\end{bmatrix},
\qquad
r_2^b=
\begin{bmatrix}0\\-0.5\end{bmatrix}.
$$

Constrain both markers against ground-fixed directions

$$
\hat u_1^g=\hat u_2^g=
\begin{bmatrix}1\\0\end{bmatrix}.
$$

At $\theta=0$, order the body velocity components as

$$
v=
\begin{bmatrix}
V_x&V_y&\omega
\end{bmatrix}^T.
$$

The two velocity constraints have coefficient matrix

$$
D=
\begin{bmatrix}
1&0&-0.5\\
1&0&0.5
\end{bmatrix}.
$$

This matrix has rank two. Column-pivoted QR selects

$$
V_x,\quad\omega
$$

as dependent components and leaves

$$
V_y
$$

as the independent velocity. Its corresponding position state is the global translation $R_y$. Thus, state selection is not intrinsically biased toward angular states: the selected physical coordinate follows from the assembled constraint geometry.

The state-selection calculation is in [`inplane_constraint_state_selection.jl`](inplane_constraint_state_selection.jl). The reusable component assembly is exercised in [`parallel_inplane_constraint_components.jl`](parallel_inplane_constraint_components.jl).

## 5. Implementation note

The initial component implementation evaluates its local Jacobian entries by centered numerical differentiation over only the variables belonging to its incident bodies and reaction. This preserves component locality and the sparse structural pattern. The explicit equations above provide the basis for replacing those local numerical derivatives with analytical expressions if that becomes useful.
