# Modular Sparse Assembly for Mechanical Systems

Status: completed design prototype; the sparse organization is now implemented

This document preserves the first two-body experiment and its then-current
limits. The production implementation now includes a general package API,
automatic and runtime state selection, fixed-pattern sparse Jacobian updates,
and larger benchmark models. See the [Technical Manual](../README.md) for the
current source path.

## 1. Purpose

The one-body examples could be written as individual dense equation sets. Larger models require a formulation that preserves component locality and exposes sparse matrix structure without changing the mechanical equations.

This document proposes the initial organization for modular sparse assembly. It is intentionally limited to the needs of the next planar examples. The design should remain compatible with three-dimensional bodies, flexible bodies, stiff force elements, ideal constraints, and alternative implicit integration methods, but those extensions should not obscure the first implementation.

The principal objectives are:

- retain accelerations as explicit unknowns;
- retain the complete unreduced set of mechanical variables;
- let bodies, connections, and force elements contribute local implicit equations;
- assemble one solver-neutral implicit system;
- expose a fixed sparse Jacobian structure whenever the model topology is fixed;
- keep selected state equations in a replaceable final block; and
- avoid requiring a recursive or graph-based solution algorithm.

## 2. Assembled implicit system

The complete model is written as

$$
F(y,\dot y,t)=0.
$$

For a BDF step, the Newton matrix is

$$
J
=
\frac{\partial F}{\partial y}
+c_j\frac{\partial F}{\partial\dot y},
$$

where $c_j$ is the BDF coefficient relating a correction in $y$ to the corresponding correction in $\dot y$.

The mechanical model should not depend on a particular BDF implementation. It supplies:

- the implicit-equation vector $F$;
- the partial matrix $F_y$;
- the partial matrix $F_{\dot y}$; or
- directly, the combined Newton matrix for a supplied value of $c_j$.

The direct combined form is convenient when a component has simple local expressions for

$$
F_y+c_jF_{\dot y}.
$$

## 3. Fixed variable blocks

Every component that owns unknowns receives a fixed range in the global solution vector. The first planar implementation uses the same ordering for every rigid body:

$$
y_b=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta
\end{bmatrix}^T.
$$

These are grouped by derivative level:

$$
\begin{array}{c|c}
\text{Level}&\text{Variables}\\
\hline
2&a_x,\ a_y,\ \alpha\\
1&V_x,\ V_y,\ \omega\\
0&R_x,\ R_y,\ \theta
\end{array}
$$

An ideal planar revolute connection owns two reaction components,

$$
\lambda^g=
\begin{bmatrix}
\lambda_x&\lambda_y
\end{bmatrix}^T.
$$

Reaction variables remain in the complete solution vector and are not integration-error-control variables.

The global ordering remains fixed after model construction. State selection changes equations and control masks, not the locations or histories of physical variables.

## 4. Fixed equation blocks

Each equation-producing component receives a fixed range of rows. Permanent mechanical equations are assembled first. Selected state equations occupy the final rows.

For a planar rigid body, the permanent equation block is

$$
F_b=
\begin{bmatrix}
\sum F_x\\
\sum F_y\\
\sum T_z
\end{bmatrix}.
$$

For an ideal planar revolute connection, the first implementation retains all three kinematic levels:

$$
F_j=
\begin{bmatrix}
\ddot\Phi^g\\
\dot\Phi^g\\
\Phi^g
\end{bmatrix}.
$$

Each entry shown is a two-component vector, so a revolute connection contributes six scalar equations.

If there are $n_s$ selected independent velocity components, the closing state block contributes $2n_s$ scalar equations: one equation connects each independent velocity to its acceleration-level variable, and one connects its position-level partner to that velocity.

For an angular selection,

$$
\alpha_k-\dot\omega_k=0,
$$

$$
\omega_k-\dot\theta_k^\ast=0.
$$

For a translational selection, the analogous equations are

$$
a_k^g-\dot V_k^g=0,
$$

$$
V_k^g-\dot R_k^g=0.
$$

## 5. Component roles

### 5.1 Planar rigid body

A planar rigid body owns its nine mechanical variables and three force-and-torque equations. It stores mass and center-of-mass inertia.

The body does not need to know which joints or forces are connected to it. Its inertial terms are local:

$$
m a^g,
\qquad
J\alpha.
$$

Applied forces and marker reactions contribute additional terms to the body's existing equation rows.

### 5.2 Body marker

A body marker stores:

- the body to which it belongs; and
- its fixed location $r_m^b$ in body coordinates.

For the planar model it evaluates

$$
r_m^g=A^{gb}(\theta)r_m^b,
$$

$$
d_m^g=A^{gb}(\theta)Sr_m^b.
$$

Its position, velocity, and acceleration are

$$
p_m^g=R^g+r_m^g,
$$

$$
v_m^g=V^g+d_m^g\omega,
$$

$$
a_m^g=a^g+d_m^g\alpha-r_m^g\omega^2.
$$

The marker owns no global variables or equations. It supplies local kinematics and partial derivatives to the components that reference it.

### 5.3 Ground marker

A ground marker supplies a prescribed position and, when needed later, prescribed velocity and acceleration. It owns no solution variables.

### 5.4 Ideal revolute connection

A revolute connection references two markers. Either marker may be fixed to ground. It owns the two reaction components and contributes the relative marker equations

$$
\Phi^g=p_A^g-p_B^g=0,
$$

$$
\dot\Phi^g=v_A^g-v_B^g=0,
$$

$$
\ddot\Phi^g=a_A^g-a_B^g=0.
$$

It also converts its reaction into structural force and torque contributions. With the stated definition of $\Phi$, the generalized reaction has the local coefficient pattern

$$
D^T\lambda^g.
$$

The signs in the body force-and-torque equations must be consistent with the chosen implicit-equation convention. The two connected bodies receive equal and opposite physical forces.

### 5.5 Gravity force

Gravity references a body and contributes

$$
-m g^g
$$

to that body's force implicit equations when the inertial form is written as

$$
m a^g-\sum F^g=0.
$$

Gravity owns no variables or equation rows.

### 5.6 Selected state equation

A selected state equation references existing acceleration-, velocity-, and position-level variables. It owns equation rows but no physical variables.

State equations are kept separate from body equations because their selection may change while the mechanical model and its component equations remain unchanged.

## 6. Residual assembly

The implementation allocates the complete implicit-equation vector once. Each component receives the global row and column indices that it needs.

During evaluation:

1. Set the equation vector to zero.
2. Evaluate body inertial contributions.
3. Add applied-force contributions.
4. Evaluate each connection's kinematic equations.
5. Add connection reactions to the appropriate body force-and-torque rows.
6. Evaluate the selected closing state equations.

Components add contributions rather than replacing complete global rows. This is necessary because several forces and connections can act on one body.

The software may use the term `residual` internally to match nonlinear-solver interfaces, while the accompanying mechanical documentation continues to call these implicit equations.

## 7. Sparse Jacobian assembly

The Jacobian is sparse because a component depends only on its own variables and those of directly connected components. The implementation should preserve this locality explicitly.

### 7.1 Structural assembly

Each partial derivative occupies one row and one column of the global Jacobian. A component can therefore report a contribution by giving:

- the global equation row;
- the global variable column; and
- the numerical value to add there.

For example, suppose the $x$ force equation for body 1 is global row 7 and $R_{1x}$ is global variable 12. A spring might contribute

$$
\frac{\partial F_{1x}}{\partial R_{1x}}=k_1
$$

at row 7, column 12. A second force element acting on the same body might contribute $k_2$ at that same location. The assembled Jacobian entry is then

$$
J_{7,12}=k_1+k_2.
$$

In sparse-matrix terminology, each contribution is recorded as a row-column-value entry,

$$
(i,j,v).
$$

Such entries are often called *triplets*. Repeated row-column pairs are allowed because Julia's sparse-matrix constructor adds their numerical values. Thus the two force elements could report

$$
(7,12,k_1),
\qquad
(7,12,k_2),
$$

and the resulting matrix would contain their sum at $(7,12)$.

During model construction, the collection of all reported row-column pairs identifies every Jacobian location that may be nonzero. This collection is the structural sparse pattern. The first implementation can collect the row-column-value entries from all components and use them to construct Julia's standard compressed sparse column matrix, `SparseMatrixCSC`. This is a straightforward reference implementation, although it is not yet the most efficient way to update the matrix at every integration step.

### 7.2 Numerical assembly

Repeated insertion into a `SparseMatrixCSC` should be avoided during integration. Once the reference implementation is verified, model construction should map each local contribution to a location in the CSC numerical-value array. A Jacobian evaluation then performs:

1. Set the stored numerical values to zero.
2. Ask each component to add its local values at its precomputed locations.
3. Reuse the existing column pointers and row indices.

This separates the fixed structural pattern from the changing numerical values.

### 7.3 Local partial derivatives

Each component should initially provide analytical local partial derivatives. A finite-difference Jacobian of the complete implicit system will be retained as a verification tool.

For a direction $z$, compare

$$
Jz
$$

with

$$
\frac{
F(y+\epsilon z,\dot y+c_j\epsilon z,t)-F(y,\dot y,t)
}{\epsilon}.
$$

This checks the exact combined matrix required by the BDF Newton iteration.

Automatic differentiation may later generate component-local partials, but the modular interface should not require it.

## 8. Proposed Julia data organization

The initial implementation can use small immutable descriptions for physical data and separate index records for assembled locations. Conceptually:

```julia
struct PlanarRigidBody
    mass
    inertia
    variables
    equations
end

struct BodyMarker
    body
    r_body
end

struct RevoluteJoint
    marker_a
    marker_b
    reaction_variables
    constraint_equations
end

struct MechanicalSystem
    bodies
    joints
    forces
    state_equations
    jacobian_structure
end
```

The precise Julia field types should be chosen during implementation. Physical descriptions should remain distinct from their assigned global indices so that model construction is readable and equation evaluation is efficient.

The model generator is responsible for assigning blocks. Components should not maintain a global counter or infer ownership through traversal.

## 9. Two-body pendulum equation count

The first modular sparse example contains two planar bodies and two ideal revolute connections: one ground pin and one interbody pin.

The variables are:

| Source | Variables per source | Count |
|---|---:|---:|
| Two planar bodies | 9 | 18 |
| Two revolute connections | 2 reactions | 4 |
| **Total** |  | **22** |

The equations are:

| Source | Equations per source | Count |
|---|---:|---:|
| Two planar bodies | 2 force + 1 torque | 6 |
| Two revolute connections | 2 acceleration + 2 velocity + 2 position | 12 |
| Two selected angular states | 2 state equations | 4 |
| **Total** |  | **22** |

The selected states are

$$
\theta_1,\quad\omega_1,\quad\theta_2,\quad\omega_2.
$$

All 22 variables remain in the BDF solution history. Only the four selected states participate in integration-error control.

## 10. Verification plan

The two-body implementation should be checked in stages:

1. Verify each marker's position, velocity, and acceleration formulas.
2. Verify the assembled position-, velocity-, and acceleration-level joint equations.
3. Verify equal and opposite interbody reaction contributions.
4. Verify the analytical sparse Newton matrix by directional finite differences.
5. Confirm that the matrix dimensions and structural nonzero pattern remain fixed during ordinary integration.
6. Integrate a consistent initial condition with the Julia BDF solver.
7. Compare the selected states with a conventional four-state reduced-coordinate reference solution.
8. Check constraint satisfaction, energy behavior, accelerations, and reaction forces.

The initial example will use the preferred angular states throughout. Dynamic repartitioning should be tested only after the fixed-selection sparse implementation is understood and verified.

## 11. Initial implementation limits

The first implementation will not attempt to provide:

- a general package API;
- three-dimensional rigid bodies;
- automatic state repartitioning;
- sparse symbolic reordering strategies;
- recursive multibody algorithms;
- code-generated component Jacobians; or
- performance comparisons with established packages.

These are later developments. The immediate objective is to demonstrate that the unreduced component equations assemble correctly into a modular sparse system and reproduce the known two-body dynamics.

## 12. First implementation

The first implementation is complete in [`modular_sparse_two_body_pendulum.jl`](../../examples/planar/modular_sparse_two_body_pendulum.jl), with results documented in [`modular-sparse-two-body-pendulum.md`](../../examples/planar/modular-sparse-two-body-pendulum.md).

It assembles a 22-variable system from two body blocks, three body markers, one ground marker, and two revolute-joint blocks. Its analytical $22\times22$ BDF Newton matrix has 82 stored locations. A one-second run reaches BDF order five, satisfies the implicit equations to approximately $1.12\times10^{-10}$, and agrees with a four-state reduced reference to approximately $1.02\times10^{-3}$ under relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$.

These results verify the proposed organization for this example. They do not yet establish scaling behavior or performance advantages for larger systems.
