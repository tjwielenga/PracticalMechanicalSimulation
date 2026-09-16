# Planar Pendulum: Reduced-Coordinate Solution

Status: internal working document

Purpose: reduce the planar rigid-body pendulum to its single independent coordinate, first by direct geometry and then by a procedure that generalizes to larger constrained models.

## 1. Scope

The physical model and complete equation inventory are defined in *Planar Rigid-Body Pendulum: Common Mechanical Equations*. The minimal Cartesian acceleration solution retains five unknowns: the two components of the center-of-mass acceleration, the angular acceleration, and the two components of the pin reaction.

The pendulum has only one degree of freedom. In this formulation, the relative angular coordinate at the pin is selected as the independent coordinate. The Cartesian position, velocity, and acceleration of the body then depend on that coordinate and its derivatives.

For this simple mechanism, the dependent quantities can be found directly from geometry. That derivation is presented first because it is physically clear. A second derivation shows how the same coordinate transformation can be generated from the constraint equations. The second procedure is less necessary for the pendulum, but it can be applied to larger models for which the geometry is not as easily written by inspection.

## 2. Full and independent coordinates

Collect the Cartesian body coordinates in

$$
q=
\begin{bmatrix}
R^g\\
\theta
\end{bmatrix}.
$$

There are three scalar coordinates in $q$, but the two pin constraints leave one degree of freedom. Select the relative pin angle as the independent coordinate:

$$
s=\theta_p.
$$

For this pendulum, the relative pin angle and the body orientation angle are equal:

$$
\theta=s.
$$

Its derivatives are

$$
\omega_p=\dot s,
\qquad
\alpha_p=\ddot s.
$$

The reduced dynamic solution therefore requires only the two first-order state variables

$$
s,
\qquad
\omega_p.
$$

## 3. Direct construction from geometry

Define the vector from the center of mass to the pin:

$$
r^g(s)=A^{gb}(s)r_m^b.
$$

Because the body marker remains at the fixed ground point $p_0^g$,

$$
R^g+r^g(s)=p_0^g.
$$

Consequently,

$$
R^g=p_0^g-r^g(s),
$$

and all three Cartesian body coordinates are determined by $s$:

$$
q=
\begin{bmatrix}
R^g\\
\theta
\end{bmatrix}
=H(s)
=
\begin{bmatrix}
p_0^g-r^g(s)\\
s
\end{bmatrix}.
$$

For this example, $H(s)$ is obtained immediately from the geometry of the pin. The notation $H$ will also be useful later when the relationship must be found by solving constraint equations.

## 4. Velocity transformation

For the derivatives of the states we have:

$$
\dot q
=
\frac{\partial H}{\partial s}\dot s
=\frac{\partial }{\partial s}\begin{bmatrix}
p_0^g-r^g(s)\\
s
\end{bmatrix} \dot s
=P(s)\dot s,
$$

Define
$$
d^g(s)=\frac{\partial r^g(s)}{\partial s}
$$

Therefore:

$$
P(s)
=
\frac{\partial H}{\partial s}
=
\begin{bmatrix}
-d^g\\
1
\end{bmatrix}.
$$

Thus,

$$
V^g=-d^g\omega_p,
\qquad
\omega=\omega_p.
$$

The column of $P$ describes the allowable Cartesian body velocity produced by a unit value of the independent angular velocity.

The corresponding virtual velocity is

$$
\delta\dot q=P\,\delta\omega_p.
$$

This expression supplies the admissible virtual direction used in the virtual-power reduction.

## 5. Acceleration transformation


Differentiating the velocity transformation gives

$$
\ddot q
=P\ddot s+\dot P\dot s.
$$

The derivative of $d^g$ with respect to $s$ is

$$
\frac{dd^g}{ds}
=A^{gb}S^2r_m^b
=-\omega r^g.
$$

For the pendulum,

$$
\ddot q
=
\begin{bmatrix}
-d^g\\
1
\end{bmatrix}
\alpha_p
+
\begin{bmatrix}
r^g\omega_p^2\\
0
\end{bmatrix}.
$$

It is useful to write this as

$$
\ddot q=P\alpha_p+k,
$$

where

$$
k=
\begin{bmatrix}
r^g\omega_p^2\\
0
\end{bmatrix}.
$$

Therefore, the dependent Cartesian accelerations are

$$
a^g=-d^g\alpha_p+r^g\omega_p^2,
\qquad
\alpha=\alpha_p.
$$


## 6. Reduction by virtual power

Define the Cartesian body mass matrix and the applied-force vector by

$$
M=
\begin{bmatrix}
mI&0\\
0&J
\end{bmatrix},
\qquad
f=
\begin{bmatrix}
mg^g\\
0
\end{bmatrix}.
$$

The pin constraint partial matrix is

$$
D=
\begin{bmatrix}
I&d^g
\end{bmatrix}.
$$

The force and torque equations can be collected as

$$
M\ddot q-f-D^T\lambda^g=0.
$$

Premultiply by the transpose of the allowable virtual direction:

$$
P^T\left(M\ddot q-f-D^T\lambda^g\right)=0.
$$

The transformation satisfies

$$
DP
=
\begin{bmatrix}
I&d^g
\end{bmatrix}
\begin{bmatrix}
-d^g\\
1
\end{bmatrix}
=0.
$$

It follows that

$$
P^TD^T\lambda^g=(DP)^T\lambda^g=0.
$$

The ideal pin reaction does no virtual power in the allowable direction and therefore disappears from the reduced equation. Substitution of

$$
\ddot q=P\alpha_p+k
$$

gives

$$
P^TMP\,\alpha_p
=P^T(f-Mk).
$$

For the pendulum,

$$
P^TMP=J+m(d^g)^Td^g.
$$

The length from the center of mass to the pin is constant, so

$$
(d^g)^Td^g=(r_m^b)^Tr_m^b=\ell^2.
$$

Also, $r^g$ and $d^g$ are perpendicular:

$$
(d^g)^Tr^g=0.
$$

The velocity-dependent term therefore makes no contribution to this particular reduced equation:

$$
P^TMk=-m(d^g)^Tr^g\omega_p^2=0.
$$

The single reduced equation of motion is

$$
\boxed{
\left(J+m\ell^2\right)\alpha_p
=-m(d^g)^Tg^g
}.
$$

This equation describes rotation about the pin. The coefficient $J+m\ell^2$ is the body's mass moment of inertia about the pin, obtained here by transforming the Cartesian mass matrix rather than by introducing the parallel-axis theorem separately.

## 7. Reduced equation and variable structure

The Cartesian acceleration solution used five instantaneous unknowns and five equations. The reduced-coordinate solution has one instantaneous acceleration unknown and one dynamic equation:

| Equation / variable | $\alpha_p$ | Right side |
|---|---:|---:|
| Reduced virtual-power equation | $P^TMP=J+m\ell^2$ | $P^T(f-Mk)=-m(d^g)^Tg^g$ |

For first-order integration, use

$$
\dot s=\omega_p,
$$

$$
\dot\omega_p
=\alpha_p
=\frac{-m(d^g)^Tg^g}{J+m\ell^2}.
$$

The first-order state supplied to an ordinary differential equation integrator is therefore

$$
u=
\begin{bmatrix}
s\\
\omega_p
\end{bmatrix}.
$$

Cartesian body quantities are reconstructed from $s$, $\omega_p$, and $\alpha_p$ when they are required.

## 8. Recovering the pin reaction

The pin reaction is not required to integrate the reduced equation, but it remains a physically useful output. After calculating the reduced acceleration, reconstruct

$$
a^g=-d^g\alpha_p+r^g\omega_p^2.
$$

The force equation then gives

$$
\lambda^g=ma^g-mg^g.
$$

The reconstructed reaction should also satisfy the torque equation

$$
J\alpha_p-(d^g)^T\lambda^g=0.
$$

This provides a useful check that the reduced and Cartesian formulations describe the same mechanical system.

## 9. Generating the transformation from the constraints

The preceding construction used the pendulum geometry directly. The same $H(s)$ and $P(s)$ can be found systematically from the constraint equations.

The pin constraints are

$$
\Phi(q)=0.
$$

Add a relationship that identifies the selected independent coordinate:

$$
\Psi(q)=s.
$$

For the pendulum,

$$
\Psi(q)=\theta.
$$

At a specified value of $s$, solve

$$
\begin{bmatrix}
\Phi(q)\\
\Psi(q)-s
\end{bmatrix}
=0
$$

for the full Cartesian coordinates $q$. This numerical position solution defines

$$
q=H(s)
$$

even when a convenient explicit formula for $H$ is not available.

Define

$$
D=\frac{\partial\Phi}{\partial q},
\qquad
C=\frac{\partial\Psi}{\partial q}.
$$

Differentiation gives

$$
D\dot q=0,
\qquad
C\dot q=\dot s.
$$

Using $\dot q=P\dot s$, the transformation is obtained from

$$
\begin{bmatrix}
D\\
C
\end{bmatrix}P
=
\begin{bmatrix}
0\\
I
\end{bmatrix}.
$$

For the pendulum,

$$
D=
\begin{bmatrix}
I&d^g
\end{bmatrix},
\qquad
C=
\begin{bmatrix}
0&0&1
\end{bmatrix}.
$$

Therefore,

$$
\begin{bmatrix}
I&d^g\\
0&0&1
\end{bmatrix}P
=
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix},
$$

which gives the same result obtained directly from geometry:

$$
P=
\begin{bmatrix}
-d^g\\
1
\end{bmatrix}.
$$

The two parts of the augmented equation have distinct purposes:

$$
DP=0
$$

makes the columns of $P$ allowable velocity directions, while

$$
CP=I
$$

associates those directions with the particular independent coordinates that were selected.

## 10. Acceleration term from the constraints

The velocity-dependent acceleration $k$ can also be generated from the constraint equations. Begin with the velocity equations derived in the preceding section:

$$
D\dot q=0,
$$

$$
C\dot q=\dot s.
$$

Both $D$ and $C$ can depend on the current coordinates. Their time derivatives must therefore be included when the velocity equations are differentiated. Differentiating the first equation gives

$$
D\ddot q+\dot D\,\dot q=0,
$$

or

$$
D\ddot q=-\dot D\,\dot q.
$$

Differentiating the second velocity equation gives

$$
C\ddot q+\dot C\,\dot q=\ddot s,
$$

or

$$
C\ddot q=\ddot s-\dot C\,\dot q.
$$

Now substitute the acceleration transformation

$$
\ddot q=P\ddot s+k
$$

into the first acceleration equation:

$$
D(P\ddot s+k)=-\dot D\,\dot q.
$$

Because $DP=0$, the term containing the independent acceleration is zero, leaving

$$
Dk=-\dot D\,\dot q.
$$

Substitution into the second acceleration equation gives

$$
C(P\ddot s+k)=\ddot s-\dot C\,\dot q.
$$

Because $CP=I$, the term $CP\ddot s$ on the left is equal to $\ddot s$ and cancels the same term on the right. Therefore,

$$
Ck=-\dot C\,\dot q.
$$

Combining the two equations produces the augmented linear system

$$
\begin{bmatrix}
D\\
C
\end{bmatrix}k
=-
\begin{bmatrix}
\dot D\,\dot q\\
\dot C\,\dot q
\end{bmatrix}.
$$

For the pendulum, $C$ is constant, so $\dot C=0$. Solving this equation produces

$$
k=
\begin{bmatrix}
r^g\omega_p^2\\
0
\end{bmatrix},
$$

which is again the result obtained directly from geometry.

This acceleration construction is included to show the general procedure. It is not the simplest way to derive the pendulum equations.

## 11. Direct and systematic constructions

The two derivations perform the same reduction:

| Quantity | Direct construction | Constraint-generated construction |
|---|---|---|
| $H(s)$ | Write the body position from the mechanism geometry | Solve $\Phi(q)=0$ and $\Psi(q)=s$ |
| $P(s)$ | Differentiate $H(s)$ | Solve the augmented velocity equations |
| $k(s,\dot s)$ | Differentiate $P(s)\dot s$ | Solve the augmented acceleration equations |
| Reduced dynamics | Premultiply the Cartesian equations by $P^T$ | Premultiply the Cartesian equations by $P^T$ |

The direct construction is preferable when the relationships are simple and clear. The constraint-generated construction provides a systematic path for larger mechanisms and establishes why the transformation eliminates ideal reaction forces.

For larger models, $s$ contains several independent coordinates and $P$ has one column for each of them. The selection of those coordinates must make the augmented coordinate-and-constraint equations independent. Questions of coordinate selection, conditioning, and preservation of sparse component structure can be addressed when a larger mechanism makes them necessary.

## 12. Julia computer model

The accompanying Julia program [`reduced_coordinate_pendulum.jl`](reduced_coordinate_pendulum.jl) implements the two-state first-order model using SciML. At every function evaluation it:

1. evaluates $r^g(s)$ and $d^g(s)$;
2. calculates $\alpha_p$ from the reduced equation;
3. returns $\dot s=\omega_p$ and $\dot\omega_p=\alpha_p$ to the integrator.

Separate reconstruction functions calculate the Cartesian position, velocity, acceleration, and ideal-pin reaction. The program also integrates the existing minimal Cartesian model from the same consistent physical initial state and compares the two solutions at common output times.

The earlier standalone browser animation was retired after completed
simulations moved to the common HDF5 result and SimpView workflow.

Run the example from the project directory with

```text
julia --project=. examples/planar/reduced_coordinate_pendulum.jl
```

Run the current program regression suite with

```text
julia --project=. test/runtests.jl
```

Run the complete paper-analysis verification separately with

```text
julia --project=. test/paper/run_all.jl
```
