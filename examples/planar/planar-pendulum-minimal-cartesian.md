# Planar Pendulum: Minimal Cartesian Acceleration Solution

Status: internal working document

Purpose: develop the first solution method for the planar rigid-body pendulum using five Cartesian acceleration and reaction unknowns.

## 1. Scope

The physical model and complete equation inventory are defined in *Planar Rigid-Body Pendulum: Common Mechanical Equations*. This document selects the smallest Cartesian acceleration-level system needed to calculate the instantaneous body acceleration and pin reaction at a known, consistent position and velocity.

The relative pin variables $(\theta_p,\omega_p,\alpha_p)$ are not retained as unknowns in this first model. They may be evaluated as outputs. This is a reduced selection of variables and equations from the common inventory, but it is not yet a reduced-coordinate formulation: the body's Cartesian translational acceleration and the pin reaction remain in the solve.

## 2. Known quantities

At the instant of the acceleration solve, the following quantities are known:

$$
R^g,\qquad
\theta,\qquad
V^g,\qquad
\omega.
$$

They satisfy the pin position and velocity constraints:

$$
\Phi=0,
\qquad
\dot\Phi=0.
$$

The rotation matrix and two useful marker vectors can therefore be evaluated:

$$
A^{gb}=A^{gb}(\theta),
$$

$$
c^g=A^{gb}r_m^b,
\qquad
d^g=A^{gb}Sr_m^b.
$$

Here $c^g$ is the vector from the center of mass to the pin, and $d^g$ is the direction in which that vector moves for a positive infinitesimal body rotation.

## 3. Reduced variable set

The acceleration solve retains the following unknown blocks:

| Unknown block | Scalar count | Meaning |
|---|---:|---|
| $a^g$ | 2 | Global acceleration of the center of mass |
| $\alpha$ | 1 | Angular acceleration of the rigid body |
| $\lambda^g$ | 2 | Global reaction force exerted by the pin on the body |
| **Total** | **5** | |

Collect them as

$$
z_a=
\begin{bmatrix}
a^g\\
\alpha\\
\lambda^g
\end{bmatrix}.
$$

## 4. Reduced equation set

The selected equations are the body force balance, body torque balance, and the second derivative of the pin constraint:

| Equation block | Scalar count | Purpose |
|---|---:|---|
| $\sum F$ | 2 | Newton force balance |
| $\sum T$ | 1 | Euler torque balance about the center of mass |
| $\ddot\Phi$ | 2 | Pin acceleration constraint |
| **Total** | **5** | |

Using $c^g$ and $d^g$, these equations are

$$
m a^g-\lambda^g=m g^g,
$$

$$
J\alpha-(d^g)^T\lambda^g=0,
$$

$$
a^g+d^g\alpha=c^g\omega^2.
$$

The last equation follows from

$$
a^g+A^{gb}
\left(
Sr_m^b\alpha-r_m^b\omega^2
\right)=0.
$$

## 5. Visible block structure

The reduced equation and variable structure is

| Equations / variables | $a^g$ | $\alpha$ | $\lambda^g$ | Right side |
|---|---:|---:|---:|---:|
| $\sum F$ | $mI$ |  | $-I$ | $m g^g$ |
| $\sum T$ |  | $J$ | $-(d^g)^T$ | $0$ |
| $\ddot\Phi$ | $I$ | $d^g$ |  | $c^g\omega^2$ |

In block-matrix form,

$$
\begin{bmatrix}
mI & 0 & -I\\
0 & J & -(d^g)^T\\
I & d^g & 0
\end{bmatrix}
\begin{bmatrix}
a^g\\
\alpha\\
\lambda^g
\end{bmatrix}
=
\begin{bmatrix}
m g^g\\
0\\
c^g\omega^2
\end{bmatrix}.
$$

The transpose pair

$$
d^g
\qquad\text{and}\qquad
(d^g)^T
$$

shows the relationship between the rotational coefficient in the pin acceleration constraint and the mapping of the pin reaction into the body torque equation.

At a known position and velocity, every matrix entry and right-side term is known. The system is therefore a linear set of five scalar equations for five scalar unknowns.

## 6. Quantities recovered after the solve

Once $z_a$ is known, the solution supplies

$$
a^g,\qquad
\alpha,\qquad
\lambda^g.
$$

The relative pin quantities need not participate in the solve:

$$
\theta_p=\theta,
\qquad
\omega_p=\omega,
\qquad
\alpha_p=\alpha.
$$

They can be evaluated afterward when requested as outputs.

## 7. Consistent initial conditions

### 7.1 Initial position

A user may supply an initial estimate

$$
q_{\mathrm{user}}=
\begin{bmatrix}
R_{\mathrm{user}}^g\\
\theta_{\mathrm{user}}
\end{bmatrix}
$$

that does not satisfy the pin constraint exactly. A consistent initial position can be defined as the position nearest to the supplied estimate, subject to the constraint:

$$
\min_q
\frac{1}{2}
\left(q-q_{\mathrm{user}}\right)^T
W_q
\left(q-q_{\mathrm{user}}\right)
\qquad
\text{subject to}
\qquad
\Phi(q)=0.
$$

For the pendulum, the position-constraint partial matrix is

$$
D(q)
=
\begin{bmatrix}
I & d^g
\end{bmatrix},
$$

because

$$
d\Phi=dR^g+d^g\,d\theta.
$$

A weighted constrained-minimum iteration solves

$$
\begin{bmatrix}
H_q & -D^T\\
D & 0
\end{bmatrix}
\begin{bmatrix}
\Delta q\\
\Delta\mu
\end{bmatrix}
=
\begin{bmatrix}
-\left[W_q(q-q_{\mathrm{user}})-D^T\mu\right]\\
-\Phi(q)
\end{bmatrix},
$$

and updates

$$
q\leftarrow q+\Delta q,
\qquad
\mu\leftarrow\mu+\Delta\mu.
$$

Here $H_q$ is the Hessian of the constrained minimum. The matrix $D$, the constraint $\Phi$, and $H_q$ are reevaluated after each correction. The weights determine how the translational and rotational coordinates share the correction.

The basic acceleration-like iteration uses $H_q\approx W_q$. For the exact Newton iteration, the constraint curvature changes only the angular entry for this pendulum:

$$
H_q
=
W_q+
\begin{bmatrix}
0&0&0\\
0&0&0\\
0&0&\mu^T c^g
\end{bmatrix},
$$

where $\mu$ is the current constraint multiplier for the position projection. The curvature term is zero on the first iteration when the multiplier starts at zero.

A particularly simple choice is

$$
W_q=
\begin{bmatrix}
mI & 0\\
0 & J
\end{bmatrix}.
$$

With this choice, the basic position-correction matrix has the same block structure as the acceleration solution matrix:

$$
\begin{bmatrix}
mI & 0 & -I\\
0 & J & -(d^g)^T\\
I & d^g & 0
\end{bmatrix}.
$$

The exact iteration retains this block structure but updates the scalar rotational entry with the curvature term. More general positive weights can be used when the supplied value of one coordinate should be preserved more strongly than another. In that case, the weights replace the mass and inertia blocks on the diagonal.

### 7.2 Initial velocity

After a consistent position has been found, the initial velocity must satisfy

$$
\dot\Phi
=D(q)V_b
=0,
$$

where the body velocity block is

$$
V_b=
\begin{bmatrix}
V^g\\
\omega
\end{bmatrix}.
$$

Given a user-supplied velocity $V_{b,\mathrm{user}}$, choose the nearest consistent velocity from

$$
\min_{V_b}
\frac{1}{2}
\left(V_b-V_{b,\mathrm{user}}\right)^T
W_v
\left(V_b-V_{b,\mathrm{user}}\right)
\qquad
\text{subject to}
\qquad
D(q)V_b=0.
$$

The corresponding linear correction is

$$
\begin{bmatrix}
W_v & -D^T\\
D & 0
\end{bmatrix}
\begin{bmatrix}
\Delta V_b\\
\nu
\end{bmatrix}
=
\begin{bmatrix}
-W_v(V_b-V_{b,\mathrm{user}})\\
-D V_b
\end{bmatrix}.
$$

After solving, update

$$
V_b\leftarrow V_b+\Delta V_b.
$$

For fixed $q$, the velocity constraint is linear, so one solution of this system produces a velocity satisfying $\dot\Phi=0$ to the accuracy of the linear solve. Choosing

$$
W_v=
\begin{bmatrix}
mI & 0\\
0 & J
\end{bmatrix}
$$

again gives the same left-side block structure as the acceleration solution.

## 8. First-order integration form

A conventional first-order ODE integrator advances a state vector $u$ by repeatedly evaluating

$$
\dot u=f(u,t).
$$

For the minimal Cartesian pendulum, the integrated state is

$$
u=
\begin{bmatrix}
R^g\\
\theta\\
V^g\\
\omega
\end{bmatrix}.
$$

The two components of $R^g$, one component of $\theta$, two components of $V^g$, and one component of $\omega$ give six integrated scalar variables.

| Integrated variable | Scalar count | Derivative returned to the integrator |
|---|---:|---|
| $R^g$ | 2 | $V^g$ |
| $\theta$ | 1 | $\omega$ |
| $V^g$ | 2 | $a^g$ |
| $\omega$ | 1 | $\alpha$ |
| **Total** | **6** | **6 derivative components** |

At each evaluation of the ODE function:

1. Read $(R^g,\theta,V^g,\omega)$ from the six-component state.
2. Evaluate $A^{gb}(\theta)$, $c^g$, and $d^g$.
3. Assemble and solve the five-equation acceleration system for $(a^g,\alpha,\lambda^g)$.
4. Return

   $$
   \dot u=
   \begin{bmatrix}
   V^g\\
   \omega\\
   a^g\\
   \alpha
   \end{bmatrix}.
   $$

The acceleration solution returns the derivatives of the velocities, while the current velocities supply the derivatives of the coordinates. The pin reaction $\lambda^g$ is calculated during the right-hand-side evaluation but is not integrated. It is an algebraic output that can be saved or recomputed at requested output times.

Although the integrator advances six scalar variables, the pendulum still has only one mechanical degree of freedom and two independent state variables. The position and velocity constraints restrict the six Cartesian state components to a two-dimensional state manifold:

$$
\Phi(u)=0,
\qquad
\dot\Phi(u)=0.
$$

The initial-condition procedures in Section 7 place the starting state on this manifold. Because the first-order ODE enforces only the acceleration constraint during subsequent derivative evaluations, numerical integration error can cause the position and velocity constraints to drift. This will be measured in the first implementation and addressed by later solution methods.

In SciML form, this model naturally becomes an ODEProblem whose right-hand-side function performs the internal five-by-five linear solve and fills the six components of $\dot u$.

The corresponding transparent Julia implementation is in
[minimal_cartesian_pendulum.jl](minimal_cartesian_pendulum.jl). It includes the position and velocity consistency calculations, the acceleration solve, the SciML right-hand-side function, and constraint and energy diagnostics.

The earlier standalone browser animation was retired after completed
simulations moved to the common HDF5 result and SimpView workflow.

## 9. Next development

The next steps for this formulation are:

1. implement the position and velocity consistency solves in Julia;
2. implement the five-equation acceleration solve;
3. implement the six-state SciML ODEProblem;
4. monitor the position and velocity constraints during integration; and
5. compare the Cartesian solution and recovered pin reaction with a reduced-coordinate reference solution.
