# Planar Spanning Spring-Damper

Status: verified working applied-force example

## 1. Purpose

This example introduces a translational applied force spanning two point markers. Marker 1 is attached to the free end of the pendulum body and marker 2 is fixed to ground. The element calculates its geometry, extension rate, scalar constitutive force, and global force through nine explicit local variables and nine scalar implicit equations.

Unlike an ideal connection, the element imposes no kinematic constraint between its markers. Its global force and marker moment contribute to the structural force and torque balances.

## 2. Spanning geometry

Define the global spanning vector from marker 1 to marker 2:

$$
s^g=P_2^g-P_1^g.
$$

Introduce $s^g$ as an explicit two-component variable with

$$
S^g=s^g-(P_2^g-P_1^g)=0.
$$

Introduce the length $\ell$:

$$
L=\ell-\sqrt{{s^g}^Ts^g}=0.
$$

The element is undefined at $\ell=0$ because its axial direction would not exist. The implementation reports this condition as an error.

## 3. Unit direction and length rate

Introduce the global unit vector $\hat u^g$ with

$$
U^g=\hat u^g-\frac{s^g}{\ell}=0.
$$

The relative marker velocity is

$$
v_s^g=V_2^g-V_1^g.
$$

Introduce the length-rate variable $\dot\ell$ and calculate it from marker kinematics:

$$
L_v=\dot\ell- {\hat u^g}^T v_s^g=0.
$$

This uses the instantaneous physical marker velocities instead of obtaining $\dot\ell$ by numerically differentiating the length history.

## 4. Constitutive force and global components

For stiffness $k$, damping coefficient $c$, and free length $\ell_0$, introduce the signed scalar force $f$:

$$
F_s=f+k(\ell-\ell_0)+c\dot\ell=0.
$$

The scalar $f$ is the force on marker 1 along the line from marker 2 toward
marker 1. Positive $f$ is compression. Introduce the global force vector on
marker 1, $F^g$, with

$$
F_c^g=F^g+\hat u^g f=0.
$$

The marker forces are equal and opposite:

$$
F_1^g=+F^g,
\qquad
F_2^g=-F^g.
$$

The chosen spanning direction therefore makes a negative tensile force pull
marker 1 toward marker 2, matching the ADAMS force reported on marker $I$.

## 5. Structural contributions

For a force $F_i^g$ applied at a body marker whose body-reference-to-marker vector is $r_i^g$, the element contributes

$$
F_i^g
$$

to the body force balance and

$$
\tau_i=r_{ix}^gF_{iy}^g-r_{iy}^gF_{ix}^g
={d_i^g}^TF_i^g
$$

to the planar torque balance. The equal-and-opposite force on a ground marker is physically present but does not enter a structural balance equation.

This example therefore verifies the distinction between a marker force and its assembled structural force and torque contributions.

## 6. Local equation structure

| Local variable | Scalar variables | Level | Defining equation |
|---|---:|---:|---|
| $s^g$ | 2 | 0 | Spanning-vector definition |
| $\ell$ | 1 | 0 | Length definition |
| $\hat u^g$ | 2 | 0 | Unit-vector definition |
| $\dot\ell$ | 1 | 1 | Length-rate definition |
| $f$ | 1 | 2 | Scalar constitutive law |
| $F^g$ | 2 | 2 | Global force components |
| **Total** | **9** |  | **9 scalar equations** |

The equations remain component-local. Their Jacobian blocks involve only the element variables and the variables of the two connected markers.

## 7. Pendulum equation set

The eleven-variable full-equation pendulum is extended by the nine applied-force variables:

$$
z=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta&
\lambda_x&\lambda_y&
s_x&s_y&\ell&u_x&u_y&\dot\ell&f&F_x&F_y
\end{bmatrix}^T.
$$

The complete simultaneous system contains 20 variables and 20 equations. Only $\omega$ and $\theta$ remain differential and error-controlled variables.

## 8. Verification case

The body marker is at the free end of the one-meter pendulum and the ground marker is at

$$
P_2^g=
\begin{bmatrix}
0.8&-0.2
\end{bmatrix}^T.
$$

The force parameters are

$$
k=20,
\qquad
c=0.5,
\qquad
\ell_0=0.5.
$$

The pendulum starts at $45$ degrees with zero angular velocity and is integrated for five seconds.

| Quantity | Result |
|---|---:|
| Simultaneous variables | 20 |
| Applied-force variables and equations | 9 |
| Differential and controlled variables | 2 |
| Accepted/rejected steps | 186 / 11 |
| Maximum BDF order | 5 |
| Maximum implicit-equation error | $3.28\times10^{-9}$ |
| Maximum selected-state difference from reduced reference | $4.42\times10^{-4}$ |
| Minimum/maximum length | $0.516$ / $0.791$ |
| Minimum/maximum scalar force | $-5.86$ / $-0.311$ |
| Total mechanical-plus-spring energy change | $-0.297$ |

The reduced reference uses the same instantaneous marker-velocity expression for $\dot\ell$. The negative energy change is expected from damping.

## 9. Implementation

The reusable marker and force-element definitions are in [`PlanarAppliedForces.jl`](../../src/planar/PlanarAppliedForces.jl). The assembled example, analytical Jacobian, reduced reference, and diagnostics are in [`spanning_spring_damper_pendulum.jl`](spanning_spring_damper_pendulum.jl).

The earlier generated browser animation was retired after completed simulations
moved to the common HDF5 result and SimpView workflow.

The same component equations are reused for the static analysis in [`spanning-spring-static-equilibrium.md`](spanning-spring-static-equilibrium.md).
