# Mathematical Architecture for Practical Mechanical System Simulation

Status: common mathematical foundation; implemented for planar models and the first spatial rigid-body vertical slice
Purpose: reconstruct the solver-neutral mathematical architecture before selecting examples, implementations, or efficiency claims.

The planar program implements the component-local unreduced system, sparse
assembly, selected physical states, and BDF solution described here. The first
spatial vertical slice now implements free rigid bodies, oriented markers,
gravity, and rotation matrices evaluated from Euler parameters. Spatial joint
and force statements beyond that narrow slice still describe the intended
architecture rather than completed 3D features.

## 1. Scope and working position

This document defines a component-based mechanical model and the unreduced implicit equations assembled from it. The unreduced system is the reference formulation. Coordinate reduction, local elimination, recursive solution, static condensation, automatic differentiation, finite differencing, and matrix-free solution are possible later transformations or solution strategies; none defines the mechanical model itself.

The immediate scope is spatial rigid-body mechanics with ideal constraints and compliant force elements. The interface is intended to admit flexible and other structural elements later, but this document does not yet claim that every such element fits without extension.

The formulation must represent:

- structural elements and their inertia equations;
- connection points, orientations, and markers;
- ideal constraints with explicit reaction variables;
- applied and constitutive force elements, including stiff bushings;
- component-local variables and implicit equations;
- assembly of an unreduced implicit system; and
- conversion of that system to the nonlinear residual and Jacobian used at a BDF time step.

The model is solver-neutral. BDF is developed because it exposes the intended implicit solution structure and is appropriate for the stiff systems motivating the work, not because components are allowed to depend on a particular BDF implementation.

## 2. Notation and frames

### 2.1 Proper vectors and component columns

Following *1 Notation and preliminaries*, a proper vector is independent of the frame used to describe it. A bold lowercase symbol such as $\mathbf v$ denotes a proper vector. A right-handed orthonormal frame $g$ has a column of basis vectors

$$
\mathbf e^g =
\begin{Bmatrix}
\mathbf e_x^g\\
\mathbf e_y^g\\
\mathbf e_z^g
\end{Bmatrix}.
$$

The component column of $\mathbf v$ in frame $g$ is

$$
v^g = \mathbf e^g \mathbin{\cdot} \mathbf v,
\qquad
\mathbf v = (\mathbf e^g)^T v^g.
$$

This working document uses bold symbols for proper vectors and unbolded symbols with a frame superscript for component columns. This replaces the manuscript's typographic underlining of component columns, which is awkward in Markdown. Vector equations are retained as long as possible; scalar implicit equations are produced only when quantities are resolved in chosen frames.

### 2.2 Transformation matrices

The transformation matrix $A^{gb}$ maps components in frame $b$ to components in frame $g$:

$$
v^g = A^{gb} v^b.
$$

For orthonormal frames,

$$
A^{bg}=(A^{gb})^T,
\qquad
A^{gb}A^{bg}=I.
$$

The orientation of a rigid body is represented first by $A^{gb}$, where $b$ is its body-fixed frame and $g$ is the inertial or global frame.

For a vector $a$, $\widetilde a$ denotes the skew-symmetric matrix satisfying

$$
\widetilde a\,b = a\times b.
$$

### 2.3 Derivatives and local rates

An overdot denotes the inertial time derivative. A prime denotes a derivative observed in a rotating local frame. Thus

$$
\dot{\mathbf r}=\boldsymbol\omega\times\mathbf r+\mathbf r'.
$$

For a vector fixed in a rigid body, $\mathbf r'=0$.

### 2.4 Infinitesimal rotation and angular coordinates

The differential rotation $d\boldsymbol\alpha$ is a proper vector because it is infinitesimal. A general finite orientation is not represented by integrating it as an ordinary vector. Instead, a set of angular coordinates $\theta$ is related locally to the body-referenced components of differential rotation by

$$
d\alpha^b = B(\theta)\,d\theta.
$$

The corresponding kinematic relation is

$$
\omega^b = B(\theta)\,\dot\theta,
\qquad
\dot\theta=C(\theta)\,\omega^b
$$

when the selected representation admits the stated inverse or generalized inverse. The architectural equations will use $A^{gb}$, $\omega^b$, and the body-referenced components $d\alpha^b$ of the infinitesimal rotation. The angular coordinates $\theta$ and the corresponding functions $B(\theta)$ and $A^{gb}(\theta)$ are selected together. A particular choice of angular coordinates is therefore a replaceable kinematic side relation, not part of the mechanical component equations.

The differential rotation $d\boldsymbol\alpha$ must not be treated as the differential of a finite rotation vector. During numerical integration, the angular coordinates are advanced according to their kinematic differential equations, and the rotation matrix is then evaluated from the corresponding relation $A^{gb}(\theta)$. The details depend on the chosen angular coordinates and need not be fixed in the general architecture.

### 2.5 Differentials, variations, and partials

A differential $d(\cdot)$ is used to derive local linearizations. A variation $\delta(\cdot)$ is restricted to a selected set of freedoms, such as velocities in virtual power. For a point fixed on a rigid body,

$$
\mathbf p=\mathbf R+\mathbf r,
$$

where $\mathbf R$ locates the body frame and $\mathbf r$ is fixed in that frame. Its displacement differential and velocity are

$$
d\mathbf p=d\mathbf R+d\boldsymbol\alpha\times\mathbf r,
\qquad
\mathbf v_p=\mathbf V+\boldsymbol\omega\times\mathbf r.
$$

With $dR^g$ resolved globally and $d\alpha^b$ resolved in the body frame,

$$
dp^g=dR^g-A^{gb}\widetilde r^{\,b}\,d\alpha^b.
$$

This differential form is preferred to prematurely extracting and naming every partial matrix.

## 3. Mechanical entities

### 3.1 Structural elements

A structural element owns inertia and a set of motion variables. For rigid body $i$, the unreduced variables are provisionally

$$
q_i=(R_i^g,A_i^{gi}),
\qquad
v_i=(V_i^g,\omega_i^i),
\qquad
a_i=(\dot V_i^g,\dot\omega_i^i).
$$

Acceleration is an explicit unknown. The rotation matrix is the physical orientation representation; any angular coordinates used for time integration are auxiliary kinematic coordinates related to it.

A structural element supplies:

1. its kinematic state and update relations;
2. the kinematics of its connection objects;
3. inertia equations resolved in specified frames;
4. mappings from connection loads to its force and torque equations; and
5. local derivatives or derivative actions needed by a nonlinear solver.

### 3.2 Connection objects

The historical manuscript distinguishes several geometric connection objects.

- A **point** has a location and translational kinematics but no orientation.
- An **orient** has an orientation and angular kinematics but no location.
- A **marker** combines a point and an orient.
- A **line** and a **surface** may parameterize locations or directions used by contacts, constraints, and forces.

These objects are not themselves necessarily joints or force elements. They provide geometric and kinematic quantities from which such components are constructed.

For marker $m$ fixed on rigid body $i$, let $r_{im}^i$ be its fixed body-frame offset and $A^{im}$ its fixed marker orientation relative to body $i$. Then

$$
p_m^g=R_i^g+A_i^{gi}r_{im}^i,
\qquad
A_m^{gm}=A_i^{gi}A^{im}.
$$

Its linear velocity and acceleration are

$$
v_m^g
=V_i^g+A_i^{gi}(\omega_i^i\times r_{im}^i),
$$

$$
a_m^g
=\dot V_i^g
+A_i^{gi}\left(
\dot\omega_i^i\times r_{im}^i
+\omega_i^i\times(\omega_i^i\times r_{im}^i)
\right).
$$

The marker also exposes angular velocity and acceleration, transformed to whatever frame the consuming component specifies. A marker transmits applied and constraint forces and torques to its owning structural element. The structural element is responsible for transforming these marker loads into contributions to its balance equations, as described for a rigid body in Section 4.1.


### 3.3 Ideal constraints

An ideal constraint component connects one or more connection objects and contributes scalar constraint equations. For holonomic constraint component $c$,

$$
\Phi_c(q,t)=0.
$$

Its velocity and acceleration forms are written abstractly as

$$
\dot\Phi_c(q,v,t)
=D_c(q,t)v+\phi_c(q,t)=0,
$$

$$
\ddot\Phi_c(q,v,a,t)
=D_c(q,t)a+\gamma_c(q,v,t)=0.
$$

Here $D_c$ is the partial of the velocity constraint with respect to the chosen structural velocity freedoms. Its rotational columns correspond to infinitesimal rotation/angular velocity, not directly to a particular finite angular-coordinate set.

Each scalar ideal constraint has a reaction variable $\lambda_c$. By virtual power, the transpose mapping $D_c^T\lambda_c$ contributes reaction forces and torques to the connected structural-element equations.

### 3.3.1 Constraint-derivative deficit

This work uses **constraint-derivative deficit** instead of DAE index when comparing the displacement, velocity, and acceleration forms of a mechanical constraint.

The constraint-derivative deficit is the number of additional time differentiations of the retained constraint equations required to determine the highest mechanical derivatives of interest and the associated constraint reactions. Derivatives of the reaction variables are not included because they are not required by the mechanical solution.

For a holonomic constraint,

| Constraint equation retained | Constraint-derivative deficit |
|---|---:|
| $\Phi(q,t)=0$ | 2 |
| $\dot\Phi(q,v,t)=0$ | 1 |
| $\ddot\Phi(q,v,a,t)=0$ | 0 |

A deficit-two formulation retains the displacement constraint. Two differentiations produce the acceleration constraint which, together with the structural balance equations, determines the accelerations and constraint reactions. A deficit-one formulation retains the velocity constraint. A deficit-zero formulation retains the acceleration constraint directly.

This definition is intentionally tied to the mechanical quantities of interest. Some numerical-analysis literature assigns a different DAE differentiation index to the equivalent first-order equation system because it asks whether derivatives of every variable, including constraint reactions, can be determined. That is a different classification and is not the measure used here.


### 3.4 Applied force elements

An applied force element obtains relative position, orientation, velocity, and possibly internal state from its connection objects. It may define local equations to define the force. It contributes applied loads to connection elements which, in turn, transmit the loads to structural elements.


## 4. Structural-element equations

### 4.1 Rigid-body inertia equations

Using a center-of-mass body frame, resolve translational balance in the global frame and rotational balance in the body frame. For rigid body $i$,

$$
r_{F,i}
=m_i\dot V_i^g-F_i^g=0,
$$

$$
r_{T,i}
=J_i^i\dot\omega_i^i
+\widetilde{\omega_i^i}J_i^i\omega_i^i
-T_i^i=0.
$$

The force and torque totals include gravity, applied force-element loads,
ideal-constraint reactions, and other external loads. A model may use a body
reference origin away from the center of mass. The input layer translates its
markers and initial motion to the CM before assembly, so the uncoupled balance
equations above remain unchanged.

For marker $m$ fixed on rigid body $i$, a force $F_m^g$ resolved in the global frame contributes unchanged to the body's force summation:

$$
F_{i\leftarrow m}^g=F_m^g.
$$

The same force also contributes a moment about the center of mass. If a torque $T_m^m$ resolved in the marker frame is applied at the marker, the total marker contribution to the body's torque summation, resolved in body frame $i$, is

$$
T_{i\leftarrow m}^i
=
\widetilde{r_{im}^i}\,A^{ig}F_m^g
+
A^{im}T_m^m.
$$

A pure applied torque contributes to the rotational balance but not to the translational balance. Summing the marker contributions and loads applied directly to the body gives

$$
F_i^g
=
F_{i,\mathrm{direct}}^g
+
\sum_{m\in\mathcal M_i}F_{i\leftarrow m}^g,
$$

$$
T_i^i
=
T_{i,\mathrm{direct}}^i
+
\sum_{m\in\mathcal M_i}T_{i\leftarrow m}^i,
$$

where $\mathcal M_i$ is the set of markers belonging to body $i$. Gravity and any other body loads may be included in the direct terms or shown separately when that distinction is useful.

Define the six-component balance equations

$$
r_{b,i}(q_i,v_i,a_i,t)=
\begin{bmatrix}
r_{F,i}\\
r_{T,i}
\end{bmatrix}=0,
$$

where $\ell_i$ denotes the loads delivered to the body through its connections.

### 4.2 Kinematic equations

Translation has the kinematic relationship

$$
r_{K,R,i}=\dot R_i^g-V_i^g=0.
$$

Orientation is governed by the rotation-matrix/angular-velocity relation. At the continuous level this may be expressed as

$$
\dot A_i^{gi}=A_i^{gi}\widetilde{\omega_i^i},
$$

for the stated body-referenced convention. In implementation, the nonlinear step will normally use a three-component rotational increment and a rotation update rather than add nine independent matrix-entry equations plus six orthonormality constraints.

The acceleration variables are related to velocity derivatives by

$$
r_{K,V,i}=\dot V_i^g-a_i^g=0,
\qquad
r_{K,\omega,i}=\dot\omega_i^i-\Omega_i^i=0.
$$


## 5. Component-local variables and equations

Let component $k$ own local variables $z_k$. These may include:

- reaction variables for ideal constraints;
- scalar or vector loads for force elements;
- relative distance, orientation, or rate variables retained explicitly;
- internal constitutive states; and
- parameters of line or surface connection objects.

The component contributes local equations

$$
r_k(x_{N(k)},z_k,\dot z_k,t)=0,
$$

where $x_{N(k)}$ contains only variables of the structural elements and connection objects used by component $k$.

Retaining an intermediate quantity as a variable is allowed when it gives a clearer component boundary or simpler local derivatives. Eliminating it is also allowed later. The architecture therefore distinguishes:

1. **model variables**, whose meaning belongs to the physical model;
2. **local formulation variables**, introduced to express a component cleanly; and
3. **solver variables**, which remain after optional eliminations or transformations.

These sets need not be identical.

## 6. Assembly of the unreduced implicit system

### 6.1 Global unknowns

Collect structural configurations, velocities, explicit accelerations, ideal-constraint reactions, and other component-local variables as

$$
x=(q,v,a,\lambda,\xi).
$$

Here $\xi$ includes compliant-element loads, internal states, and retained geometric or constitutive intermediates. If a component has differential internal states, their derivatives are included explicitly in the arguments of the general implicit equations.

### 6.2 Implicit-equation blocks

A provisional unreduced continuous system is a set of implicit equations denoted by $R$:

$$
R(x,\dot q,\dot v,\dot\xi,t)=
\begin{bmatrix}
R_B(q,v,a,\lambda,\xi,t)\\
R_{Kq}(q,v,\dot q,t)\\
R_{Kv}(a,\dot v,t)\\
R_C(q,v,a,t)\\
R_F(q,v,a,\xi,\dot\xi,t)
\end{bmatrix}=0.
$$

The blocks are:

- $R_B$: structural force and torque balances;
- $R_{Kq}$: position/orientation kinematic relations;
- $R_{Kv}$: relations between explicit accelerations and derivatives of velocities;
- $R_C$: active ideal-constraint equations at the selected level; and
- $R_F$: compliant-force, applied-force, geometric-definition, and internal-state equations.

The choice of constraint level in $R_C$ requires care. Acceleration-level constraints fit naturally beside explicit accelerations and reactions, while position-level constraints are needed to prevent configuration drift. A robust time-step formulation may include position constraints directly, use an augmented combination of levels, or use position/velocity correction around an acceleration-level solve. This document does not yet choose among those alternatives.

### 6.3 Local assembly

Each component contributes equation entries involving only its incident structural and local variables. It also contributes local derivative blocks or derivative actions. Global assembly consists of mapping these local equation rows and variable columns into the global implicit system and its Jacobian.

This locality implies sparse dependence:

- a body balance depends on that body's state and the loads attached to it;
- a two-marker joint depends on the two owning structural elements and its reactions;
- a two-marker applied force depends on the same local neighborhood and its internal variables; and
- an internal constitutive state normally depends only on its owning force element.

No global equation ordering is part of the physical model. Modern sparse ordering and factorization may reorder the assembled system. The architecture should preserve component-local dependency information so they can do so effectively.

## 7. Three mechanical topologies

### 7.1 Open trees

An open tree has no kinematic loop closures. It may still be represented in absolute structural coordinates using the same unreduced body, connection, force, and constraint components as any other model.

A tree permits optional relative-coordinate or recursive formulations. Those are transformations of the reference model, not required model semantics. No claim is made here that the unreduced formulation is fastest for trees.

### 7.2 Ideal closed loops

An ideal closed loop retains every physical loop-closing joint as ideal constraint rows with corresponding reaction variables. The assembled Newton system therefore has saddle-point structure in its leading mechanical blocks. Closure must hold to the selected numerical tolerance, and redundant scalar constraint rows must be removed or deactivated before repeated solution.

### 7.3 Loops opened with stiff bushings

If one ideal loop-closing joint is replaced physically by a bushing, the topology of the ideal-constraint set becomes open at that location. The bushing transmits finite forces and moments through constitutive equations. It may introduce fast, strongly damped modes and a stiff time-integration problem, but it does not introduce loop-closure reaction multipliers.

This is not a numerical trick applied to the same mathematical model. It is a different mechanical idealization whose adequacy depends on the intended physical system.

## 8. Initialization and constraint-rank validation

### 8.1 Configuration and velocity consistency

Initial configuration should satisfy the active holonomic constraints:

$$
\Phi(q_0,t_0)=0.
$$

When user-supplied coordinates are inconsistent, a weighted correction can be posed as

$$
\min_q \frac12(q-q_{\mathrm{user}})^T W(q-q_{\mathrm{user}})
\quad\text{subject to}\quad
\Phi(q,t_0)=0.
$$

Rotational corrections may be expressed as infinitesimal rotations and applied through the rotation update, rather than added directly to the entries of $A^{gb}$.

Initial velocities may be inconsistent with velocity constraints.  The initial velocities should satisfy

$$
D(q_0,t_0)v_0+\phi(q_0,t_0)=0.
$$

Velocities can be made consistent with a minimization similar to that above.

Initial accelerations and reactions may be obtained from structural balances and acceleration constraints, they are not independently prescribed initial data.

### 8.2 Redundant constraint rows

At initialization, evaluate the active constraint partial matrix

$$
D_0=D(q_0,t_0).
$$

Rows should be scaled so translational and rotational constraints have comparable numerical significance. Then perform a rank-revealing factorization, provisionally a pivoted QR on the transpose:

$$
D_0^T P=QR.
$$

If the numerical rank is $r<m$, the pivoting identifies an independent subset of the $m$ scalar constraint rows and dependent candidates. QR detects dependence; component metadata should guide which row is actually deactivated. Useful metadata include parent joint, physical direction, priority, whether deactivation is permitted, and whether the reaction is an important output.

This is an initialization and diagnostic operation, not the repeated time-step factorization. It may be repeated after a topology change or when factorization failure, loss of Newton convergence, or another diagnostic suggests a singular configuration.

The QR factorization also reveals a set of coordinates that are independent and are natural states of the system. That is, if these coordinates are specified, all the other coordinates can be calculated from them using the constraint equations. The number of these coordinates is equal to the number of degrees of freedom of the mechanism.

## 9. BDF discretization

### 9.1 General implicit residual

For a first-order implicit system

$$
F(\dot y,y,t)=0,
$$

a BDF step supplies an affine relation between the derivative and the current value:

$$
\dot y_n=\frac{\alpha}{h}y_n+\beta_n,
$$

where $h$ is the step size, $\alpha$ is the current BDF leading coefficient, and $\beta_n$ is fixed during the nonlinear solve and contains the history/prediction terms.

The time-step residual is therefore

$$
G_n(y_n)=F\!\left(\frac{\alpha}{h}y_n+\beta_n,y_n,t_n\right)=0.
$$

Newton's method solves

$$
J_n\Delta y_n=-G_n,
$$

with

$$
J_n
=\frac{\alpha}{h}F_{\dot y}+F_y.
$$

### 9.2 Highest derivatives as Newton variables

The manuscript also develops the inverse affine BDF relation

$$
y_n=\frac{h}{\alpha}\dot y_n+\gamma_n,
\qquad
\Delta y_n=\frac{h}{\alpha}\Delta\dot y_n.
$$

When the highest derivatives are chosen as Newton variables,

$$
\widehat G_n(\dot y_n)
=F\!\left(\dot y_n,\frac{h}{\alpha}\dot y_n+\gamma_n,t_n\right)=0,
$$

and

$$
\widehat J_n
=F_{\dot y}+\frac{h}{\alpha}F_y.
$$

This form is especially relevant here because acceleration remains explicit in the unreduced mechanical system. The inertia terms stay (nearly) constant in the Jacobian from iteration to iteration, perhaps delaying the need to evaluate and refactor the Jacobian.

### 9.3 Mechanical chain $q$, $v$, and $a$

For translational variables with

$$
\dot q=v,
\qquad
\dot v=a,
$$

and acceleration increments used as Newton variables, the BDF relations give

$$
\Delta v=\eta\,\Delta a,
\qquad
\Delta q=\eta^2\,\Delta a,
\qquad
\eta=\frac{h}{\alpha}.
$$

Consequently, for a mechanical residual $R_M(a,v,q,z,t)$, the condensed derivative with respect to the acceleration correction is

$$
J_{M,a}^{\mathrm{BDF}}
=R_{M,a}
+\eta R_{M,v}
+\eta^2R_{M,q},
$$

plus columns for algebraic or component-local variables $z$ that are not eliminated through BDF relations.

The rotational configuration increments are body-referenced infinitesimal rotations. Their effects on a component's implicit equations are evaluated through differentials with respect to $\Delta\alpha_i^i$:
$$
\Delta\theta_i=C_i(\theta_i)\,\Delta\alpha_i^i.
$$

If $C_i(\theta_i)$ becomes singular due to a rotation-representation singularity, a new set of rotation angles can be chosen. Also,
$$
\Delta\alpha_i^i=\eta\,\Delta\omega_i^i,
$$
$$
\Delta\omega_i^i=\eta\,\Delta\dot\omega_i^i.
$$

### 9.4 Unreduced versus condensed Newton systems

Two distinct systems must not be conflated:

1. The **unreduced time-step system** retains kinematic residuals and explicit variables $(q,v,a,\lambda,\xi)$.
2. A **BDF-condensed Newton system** uses the linear BDF relations to express selected configuration and velocity corrections in terms of acceleration corrections.

The first is the canonical assembled formulation. The second is an optional exact local elimination for a particular time step. Both should be documented so that solver comparisons do not accidentally compare different mechanical models.

## 10. Provisional Newton block structure

Without choosing a constraint-level strategy, a representative unreduced Newton system has the form

$$
\begin{bmatrix}
R_{B,q} & R_{B,v} & R_{B,a} & D^T & R_{B,\xi}\\
R_{Kq,q} & R_{Kq,v} & 0 & 0 & 0\\
0 & R_{Kv,v} & R_{Kv,a} & 0 & 0\\
R_{C,q} & R_{C,v} & R_{C,a} & 0 & 0\\
R_{F,q} & R_{F,v} & R_{F,a} & 0 & R_{F,\xi}
\end{bmatrix}
\begin{bmatrix}
\Delta q\\
\Delta v\\
\Delta a\\
\Delta\lambda\\
\Delta\xi
\end{bmatrix}
=-R.
$$

Zeros in this schematic indicate structural absence at the stated abstraction level, not a promise that every future component preserves the same pattern. BDF discretization modifies the kinematic rows or permits their exact local elimination. Ideal constraints create the reaction/constraint coupling; compliant elements instead contribute through $\xi$, their constitutive rows, and the body-load columns.

## 11. Optional later transformations

The following are intentionally outside the canonical model definition:

- elimination of component-local geometry or load variables;
- BDF elimination of kinematic-definition rows;
- relative coordinates on open trees;
- coordinate partitioning for ideal constraints;
- Schur-complement condensation of reaction variables;
- recursive tree solution;
- block-triangular decomposition;
- sparse direct factorization with modern ordering;
- automatic or numerical differentiation of reduced residuals; and
- matrix-free Jacobian-vector products with suitable preconditioning.

Each may be evaluated later by total computational cost and numerical robustness. This document makes no efficiency ranking among them.

## 12. Historical traceability and cautions

The following architectural ideas are directly supported by the reviewed manuscript chapters:

- frame-independent proper vectors and frame-dependent component columns;
- rotation matrices related locally to angular coordinates through infinitesimal rotation;
- virtual power using translational and angular velocity freedoms;
- explicit ideal-constraint reactions obtained from constraint partials;
- point, orient, marker, line, and surface connection constructs;
- shared geometric constructs underlying joints and force elements;
- compliant force definitions retained as local equations;
- explicit acceleration variables and implicit structural balances;
- BDF correction equations and optional kinematic elimination; and
- constraint-rank analysis as part of initialization/model validation.

The current component terminology and the separation into model, formulation, and solver variables are a modern reconstruction of those ideas. They should not be presented later as verbatim historical terminology.
