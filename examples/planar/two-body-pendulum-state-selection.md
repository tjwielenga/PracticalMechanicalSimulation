# Two-Body Pendulum: State Selection from Velocity Constraints

Status: internal working example

## 1. Purpose

This example extends the planar pendulum state-selection experiment by attaching a second rigid body to the tip of the first with a revolute joint. It tests whether column-pivoted QR leaves both body angular velocities independent without being told to do so.

## 2. Model and velocity variables

Body 1 is connected to ground by an ideal pin at its upper marker. Its lower marker is connected to the upper marker of body 2 by a second ideal pin.

The body velocities are collected as

$$
\nu=
\begin{bmatrix}
V_1^g\\
\omega_1\\
V_2^g\\
\omega_2
\end{bmatrix}
=
\begin{bmatrix}
V_{1x}&V_{1y}&\omega_1&V_{2x}&V_{2y}&\omega_2
\end{bmatrix}^T.
$$

There are six scalar velocity components.

## 3. Velocity constraints

Let $d_{1u}^g$ and $d_{1l}^g$ be the infinitesimal rotational directions of the upper and lower markers on body 1. Let $d_{2u}^g$ be the corresponding direction of the upper marker on body 2.

The ground-pin velocity constraint is

$$
V_1^g+d_{1u}^g\omega_1=0.
$$

The interbody-pin velocity constraint is

$$
V_1^g+d_{1l}^g\omega_1
-V_2^g-d_{2u}^g\omega_2=0.
$$

Together, these give

$$
D\nu=0,
$$

with

$$
D=
\begin{bmatrix}
I&d_{1u}^g&0&0\\
I&d_{1l}^g&-I&-d_{2u}^g
\end{bmatrix}.
$$

The matrix has four rows, six columns, and rank four. The mechanism therefore has two independent velocity components.

## 4. Unscaled QR result

For the test configuration,

$$
\theta_1=35^\circ,
\qquad
\theta_2=-20^\circ,
$$

and each body is one meter long with its center of mass halfway between its markers.

Column-pivoted QR of the unscaled matrix produces the pivot order

$$
V_{1x},\quad V_{1y},\quad \omega_1,\quad V_{2y},
\quad V_{2x},\quad \omega_2.
$$

It therefore selects

$$
\nu_d=
\begin{bmatrix}
V_{1x}&V_{1y}&\omega_1&V_{2y}
\end{bmatrix}^T.
$$

The unrestricted independent variables are

$$
\nu_i=
\begin{bmatrix}
V_{2x}&\omega_2
\end{bmatrix}^T.
$$

Thus unrestricted QR does **not** select both angular velocities in this example. The selected dependent block is valid, with condition number approximately $3.76$, and its tangent mapping satisfies $DP$ to machine precision. QR has found a numerically independent partition, but not the partition with the clearest mechanical interpretation.

## 5. Preferred angular-velocity selection

Request the mechanically preferred independent set

$$
\nu_i=
\begin{bmatrix}
\omega_1&\omega_2
\end{bmatrix}^T.
$$

Its complementary dependent set contains all four translational velocities. The corresponding dependent block is

$$
D_d=
\begin{bmatrix}
I&0\\
I&-I
\end{bmatrix}.
$$

This matrix is nonsingular and has condition number approximately $2.62$. The preferred selection is therefore accepted. It is somewhat better conditioned than the unrestricted QR selection for this configuration.

## 6. Constructed tangent mapping

The ground-pin equation first gives

$$
V_1^g=-d_{1u}^g\omega_1.
$$

Substitution into the interbody-pin equation gives

$$
V_2^g=
\left(d_{1l}^g-d_{1u}^g\right)\omega_1
-d_{2u}^g\omega_2.
$$

The program constructs these relationships directly from the selected matrix blocks:

$$
\nu=P
\begin{bmatrix}
\omega_1\\
\omega_2
\end{bmatrix},
\qquad
DP=0.
$$

It does not form arbitrary linear combinations of velocities. The independent entries of $P$ remain the individual physical angular velocities.

## 7. Position-state selection

Each independent angular velocity is paired with its corresponding body-fixed pseudo angle:

$$
\omega_1-\dot\theta_1=0,
$$

$$
\omega_2-\dot\theta_2=0.
$$

The selected integration variables are therefore

$$
\theta_1,
\quad
\omega_1,
\quad
\theta_2,
\quad
\omega_2.
$$

The four translational positions remain dependent variables maintained by the two pin constraints.

## 8. Significance

This example demonstrates that the same local marker equations used for one body assemble naturally into a larger constraint matrix. The QR procedure correctly finds rank four and a valid two-dimensional allowable-velocity space, but unrestricted numerical pivoting does not necessarily choose the individual variables a mechanical-system author would prefer.

The preferred-coordinate procedure resolves this without replacing physical variables by linear combinations. It tests the desired angular-velocity selection, confirms that the complementary translational block is nonsingular and well-conditioned, and then constructs the mapping from the same assembled constraint matrix. No graph traversal, recursive coordinate construction, or manually derived global reduction is required.

## 10. Prescribed base motion

When a rotational motion generator prescribes $\omega_1=\dot g(t)$, its coefficient row is appended to the four joint velocity constraints. The resulting matrix has five rows and six mechanical-velocity columns. Its numerical rank is five, leaving one independent velocity.

Testing $\omega_2$ as the preferred independent component gives a nonsingular complementary block. The selected state pair is therefore $\omega_2$ and its body-fixed pseudo angle $\theta_2$. The nonzero prescribed-motion right-hand side changes the particular velocity solution but does not enter the QR column selection. The complete dynamic experiment is documented in [`driven-base-double-pendulum.md`](driven-base-double-pendulum.md).

The executable calculation is in `two_body_pendulum_state_selection.jl`.
