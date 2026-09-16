# Modular Sparse Two-Body Pendulum

Status: verified working example

## 1. Purpose

This example is the first implementation of the modular sparse-assembly architecture. It extends the planar rigid-body pendulum with a second body connected to the tip of the first by an ideal revolute joint.

The example demonstrates:

- repeated rigid-body and marker components;
- additive force, torque, reaction, and constraint assembly;
- explicit acceleration variables;
- a complete unreduced BDF solution history;
- selected angular-velocity and angular-position states; and
- an analytical sparse Newton matrix assembled from component-local contributions.

## 2. Complete variable set

Each body owns the local variable block

$$
y_b=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta
\end{bmatrix}^T.
$$

The two revolute joints each own two reaction variables. The complete system therefore has

$$
2(9)+2(2)=22
$$

simultaneous variables.

The global ordering is

$$
y=
\begin{bmatrix}
y_1\\
y_2\\
\lambda_0^g\\
\lambda_{12}^g
\end{bmatrix},
$$

where $\lambda_0^g$ is the ground-pin reaction and $\lambda_{12}^g$ is the interbody-pin reaction.

## 3. Complete equation set

Each body contributes two force equations and one torque equation. Each revolute joint contributes its acceleration-, velocity-, and position-level constraint equations. The final four rows are the selected state equations.

| Equation block | Scalar equations |
|---|---:|
| Body 1 force and torque | 3 |
| Body 2 force and torque | 3 |
| Ground pin: $\ddot\Phi_0$, $\dot\Phi_0$, $\Phi_0$ | 6 |
| Interbody pin: $\ddot\Phi_{12}$, $\dot\Phi_{12}$, $\Phi_{12}$ | 6 |
| Selected angular state equations | 4 |
| **Total** | **22** |

The final equations are

$$
\alpha_1-\dot\omega_1=0,
\qquad
\omega_1-\dot\theta_1=0,
$$

$$
\alpha_2-\dot\omega_2=0,
\qquad
\omega_2-\dot\theta_2=0.
$$

Only

$$
\omega_1,\quad\theta_1,\quad\omega_2,\quad\theta_2
$$

participate in integration-error control. Nevertheless, accepted values of all 22 variables are retained in the BDF history.

## 4. Modular assembly

The implementation defines separate data structures for:

- `PlanarBodyBlock`;
- `PlanarBodyMarker`;
- `PlanarGroundMarker`;
- `PlanarRevoluteJoint`; and
- `ModularTwoBodySystem`.

A body contributes only its inertial and gravity terms. Each joint evaluates its two marker kinematics, contributes its six relative kinematic equations, and adds its reaction force and torque to the connected bodies.

The interbody reaction appears with equal and opposite signs on the two bodies. The moment contribution is obtained from the marker's infinitesimal rotational direction $d^g$ rather than being written as a special two-body formula.

## 5. Sparse Newton matrix

The analytical callback constructs

$$
J=F_y+c_jF_{\dot y}
$$

from body-, marker-, joint-, and state-equation contributions. The $22\times22$ matrix contains 82 stored locations out of 484 possible entries:

$$
\frac{82}{484}=0.1694.
$$

Thus approximately 17 percent of the matrix is stored. The integrator receives a sparse matrix prototype and uses sparse numerical factorization during Newton iteration.

The current reference implementation collects row-column-value entries and constructs a sparse matrix during each Jacobian evaluation. This favors clarity. A later optimization can map the local contributions directly into the numerical-value array of a fixed compressed sparse column structure.

## 6. Initial conditions and reference solution

Positions and velocities are reconstructed directly from the two angles and angular velocities. At the initial configuration, the six body accelerations and four joint-reaction components are found from the ten force, torque, and acceleration-constraint equations.

A conventional reduced reference integrates

$$
\theta_1,\quad\omega_1,\quad\theta_2,\quad\omega_2.
$$

Its angular accelerations are obtained from the same mechanical equations. It therefore supplies an independent integration formulation without introducing a second set of force and constraint conventions.

## 7. First dynamic result

The initial configuration is

$$
\theta_1=35^\circ,
\qquad
\theta_2=-20^\circ,
$$

with both angular velocities initially zero. The system is integrated for one second using relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$.

| Quantity | Result |
|---|---:|
| Return code | Success |
| Simultaneous variables and equations | 22 |
| Error-controlled states | 4 |
| Stored Jacobian locations | 82 |
| Jacobian density | 0.1694 |
| Accepted steps | 113 |
| Rejected steps | 7 |
| Maximum BDF order | 5 |
| Maximum implicit-equation error | $1.12\times10^{-10}$ |
| Maximum constraint-equation error | $1.12\times10^{-10}$ |
| Maximum selected-state difference from reference | $1.02\times10^{-3}$ |
| Maximum BDF energy drift | $6.89\times10^{-4}\ \mathrm{J}$ |
| Final BDF energy drift | $-6.01\times10^{-4}\ \mathrm{J}$ |
| Maximum reference energy drift | $4.74\times10^{-11}\ \mathrm{J}$ |

The agreement is appropriate for the requested integration tolerances. These results establish correctness of the first modular sparse formulation; they are not an efficiency comparison with another method or package.

### Reusable component reconstruction

The model has also been reconstructed using the reusable planar rigid-body, gravity, and revolute-joint definitions. The first revolute joint connects a body marker to ground. The second connects markers on the two moving bodies, and its reaction contribution is applied with opposite signs to their respective balance equations.

The reconstructed model contains 22 variables, 22 equations, ten owned equation blocks, and four contributions to body balance equations. Its complete implicit-equation vector and analytical Jacobian agree with the original modular implementation to roundoff. The analysis policies select:

| Analysis | Selected variables | Selected equations |
|---|---:|---:|
| Position initial conditions | 6 | 4 |
| Velocity initial conditions | 6 | 4 |
| Acceleration initial conditions | 10 | 10 |
| Dynamics | 22 | 22 |

The reusable reconstruction now also follows an integration path. Its component-local Jacobian callbacks assemble through a generic sparse accumulator into a `SparseMatrixCSC` with the same 82 stored entries as the original hand-written sparse matrix. Explicit structural zeros are retained so the pattern remains fixed as configuration-dependent terms pass through zero. With this Jacobian supplied to the BDF integrator, the reconstructed and original models produce the same trajectory to roundoff.

The dictionary accumulator establishes the pattern once and keeps component callbacks independent of matrix storage. During integration, a fixed-pattern accumulator maps the same canonical callback entries directly into the existing CSC numerical-value array. The matrix's column pointers and stored row indices remain unchanged. This separates occasional symbolic-pattern construction from repeated numerical Jacobian evaluation.

The reusable assembly also assigns its canonical locations from declarations. Rigid bodies and joints first declare their local variables and equation blocks without global indices. The model-layout builder assigns variable ranges in component order and equation ranges in the formulation's requested block order. Bodies, markers, and joints are then constructed from those returned ranges. This reproduces the established 22-variable ordering without embedding its global numbers in the component construction.

Over the five-second interval used by the animation, the improved step-size and order controller gives

$$
\max_t|E_{\mathrm{BDF}}(t)-E(0)|
=0.001511\ \mathrm{J},
$$

$$
E_{\mathrm{BDF}}(5)-E(0)
=-0.001479\ \mathrm{J}.
$$

The magnitude of the initial energy under the selected zero-potential convention is $16.6630\ \mathrm{J}$, so the final drift is approximately $0.0089$ percent of that magnitude. The high-accuracy reduced reference has a maximum energy drift of only $7.67\times10^{-11}\ \mathrm{J}$ over the same interval.

The original threshold-based controller dropped to order one near $t=2.5\ \mathrm{s}$ and remained there, producing $0.1111\ \mathrm{J}$ of energy loss. With candidate-order step estimates, the method remains predominantly at orders four and five:

| Time interval (s) | Mean BDF order | Mean step size (s) |
|---|---:|---:|
| 0.0–0.5 | 3.68 | 0.00752 |
| 0.5–1.0 | 4.89 | 0.01084 |
| 2.5–3.0 | 4.89 | 0.01089 |
| 3.0–3.5 | 5.00 | 0.01075 |
| 4.5–5.0 | 4.88 | 0.01177 |

The five-second run now requires 454 accepted and 29 rejected steps. The energy comparison confirms that the earlier visible damping was primarily caused by the controller becoming trapped at backward Euler, not by a force or constraint error in the model.

## 8. Verification

The automated tests verify:

- consistency of the initial 22 equations;
- the $22\times22$ Jacobian dimensions;
- use of sparse matrix storage;
- the expected 82 stored locations;
- the analytical combined Jacobian against a directional finite difference;
- successful sparse BDF integration;
- the differential-variable and error-control selections;
- satisfaction of all constraint levels; and
- agreement with the reduced four-state reference.

The executable implementation is in `modular_sparse_two_body_pendulum.jl`.

The reusable component reconstruction is in `automatic_two_body_component_assembly.jl`.

## 9. Graphics and step history

The earlier generated browser animation and step-history files were retired
after completed simulations moved to the common HDF5 result and SimpView
workflow. Equivalent graphics can be generated from saved results when needed.
