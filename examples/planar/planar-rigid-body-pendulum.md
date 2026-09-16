# Planar Rigid-Body Pendulum: Common Mechanical Equations

Status: internal working document

Purpose: establish a simple mechanical model and the equations that remain true before choosing a formulation or solution method.

## 1. Teaching objective

The planar rigid-body pendulum will be the first model used to introduce the simulation architecture. The development begins with familiar planar mechanics. General component terminology and solver details will be introduced only as they become useful.

The same physical model will later be organized in several ways:

- an unreduced formulation with explicit accelerations and pin reactions;
- an acceleration-level solution;
- a reduced-coordinate equation in the pendulum angle; and
- an implicit time-step formulation using BDF equations.

These are different uses of the same mechanical and kinematic relationships. They are not different pendulum models.

## 2. Physical model

A planar rigid body has mass $m$ and mass moment of inertia $J$ about its center of mass. Its center of mass is located by

$$
R^g=
\begin{bmatrix}
x\\
y
\end{bmatrix},
$$

in the fixed global frame $g$. A body-fixed frame $b$ has its origin at the center of mass. The body orientation is described by the planar rotation matrix

$$
A^{gb}(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta\\
\sin\theta & \cos\theta
\end{bmatrix},
$$

which transforms body-frame components into global components.

A marker $m$ is fixed to the body at the body-frame offset

$$
r_m^b=
\begin{bmatrix}
r_x\\
r_y
\end{bmatrix}.
$$

An ideal pin constrains this marker to the fixed ground point $p_0^g$. The pin supplies a reaction force $\lambda^g$ with two global components and supplies no reaction torque. Gravity acts at the center of mass:

$$
F_G^g=m g^g.
$$

For the usual vertical convention, $g^g=[0\;\;-g]^T$.

## 3. Planar rotation notation

The body rotates about the axis perpendicular to the plane. Its angular velocity vector is therefore

$$
\boldsymbol\omega=\omega\,\mathbf e_z.
$$

For planar motion, the scalar angular velocity is the rate of change of the orientation angle:

$$
\omega=\dot\theta.
$$

If $\mathbf r$ is fixed in the rotating body, its velocity relative to the body origin is the familiar expression

$$
\boldsymbol\omega\times\mathbf r.
$$

In three-dimensional matrix notation, the same cross product is

$$
\boldsymbol\omega\times\mathbf r
=
\widetilde{\boldsymbol\omega}\,r
=
\omega\,\widetilde e_z\,r.
$$

For a vector confined to the plane, the unused third row and column can be removed. The two-dimensional representation of $\widetilde e_z$ is

$$
S=
\begin{bmatrix}
0 & -1\\
1 & 0
\end{bmatrix}.
$$

Thus, for the two in-plane components $r=[r_x\;\;r_y]^T$,

$$
Sr
=
\begin{bmatrix}
-r_y\\
r_x
\end{bmatrix}
$$

is the planar representation of $\mathbf e_z\times\mathbf r$. In two dimensions this operation turns an in-plane vector through $90^\circ$. The corresponding three-dimensional tilde matrix is a cross-product operator, not a finite three-dimensional rotation matrix.

The velocity of a body-fixed vector can now be written in planar component form as

$$
\dot r^g=A^{gb}Sr^b\,\omega.
$$

Because $r^g=A^{gb}r^b$ and $r^b$ is fixed in the body,

$$
\dot r^g=\dot A^{gb}r^b.
$$

Comparing these two expressions for every body-fixed vector gives

$$
\dot A^{gb}=A^{gb}S\omega.
$$

The angular acceleration is

$$
\alpha=\dot\omega.
$$

### 3.1 Two-component Euler parameters

The orientation angle is the simplest rotational coordinate for planar motion. An alternative description is useful because it introduces the Euler-parameter form that will later be used for three-dimensional rotation.

Define the two-component Euler-parameter vector

$$
p=
\begin{bmatrix}
p_0\\
p_3
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta/2)\\
\sin(\theta/2)
\end{bmatrix}.
$$

The subscripts $0$ and $3$ are retained from the four-component Euler parameters used in three dimensions. For planar rotation about the global $z$ axis, the other two parameters are zero.

The parameters satisfy the normalization constraint

$$
p^Tp-1
=p_0^2+p_3^2-1
=0.
$$

In terms of these parameters, the body-to-global transformation matrix is

$$
A^{gb}(p)
=
\begin{bmatrix}
p_0^2-p_3^2 & -2p_0p_3\\
2p_0p_3 & p_0^2-p_3^2
\end{bmatrix}.
$$

Using

$$
p_0^2-p_3^2=\cos\theta,
\qquad
2p_0p_3=\sin\theta,
$$

shows that this is the same transformation matrix $A^{gb}(\theta)$ defined above.

The planar kinematic-differential equation is

$$
\omega=B(p)\dot p,
$$

where

$$
B(p)
=2
\begin{bmatrix}
-p_3 & p_0
\end{bmatrix}.
$$

Consequently,

$$
\omega
=2\left(-p_3\dot p_0+p_0\dot p_3\right).
$$

Differentiating the normalization constraint also gives

$$
p^T\dot p=0,
$$

so $\dot p$ lies in the direction tangent to the unit circle at $p$. Together, $\omega=B(p)\dot p$ and $p^Tp-1=0$ determine the two Euler-parameter rates from the one planar angular velocity. The parameter pairs $p$ and $-p$ describe the same physical orientation.

## 4. Marker kinematics

The global position of the body marker is

$$
p_m^g=R^g+A^{gb}r_m^b.
$$

Because $r_m^b$ is fixed in the body, the marker velocity is

$$
v_m^g
=V^g+A^{gb}Sr_m^b\,\omega,
$$

where

$$
V^g=\dot R^g.
$$

Differentiating once more gives the marker acceleration:

$$
a_m^g
=a^g+A^{gb}
\left(
Sr_m^b\,\alpha-r_m^b\,\omega^2
\right),
$$

where

$$
a^g=\dot V^g.
$$

The two rotational terms are the tangential and centripetal accelerations of the marker relative to the center of mass.

## 5. Ideal-pin constraint

The position constraint is

$$
\Phi^g
=p_m^g-p_0^g
=R^g+A^{gb}r_m^b-p_0^g
=0.
$$

Because the ground point is fixed, the velocity constraint is

$$
\dot\Phi^g
=V^g+A^{gb}Sr_m^b\,\omega
=0.
$$

The acceleration constraint is

$$
\ddot\Phi^g
=a^g+A^{gb}
\left(
Sr_m^b\,\alpha-r_m^b\,\omega^2
\right)
=0.
$$

All three forms describe the same ideal pin at different derivative levels. They are simultaneously true for a consistent motion, but a numerical formulation need not include all three as independent equation rows.

## 6. Relative pin coordinate

The point constraints prevent translation at the pin but leave one relative rotational freedom. Define a unit direction fixed in the body marker. For this example, let it be the marker's local $x$ direction:

$$
u_m^b=
\begin{bmatrix}
1\\
0
\end{bmatrix}.
$$

Its components in the ground frame are

$$
u_m^g
=A^{gb}u_m^b
=
\begin{bmatrix}
u_x\\
u_y
\end{bmatrix}.
$$

The relative pin angle is defined locally at the joint by

$$
\theta_p=\operatorname{atan2}(u_y,u_x).
$$

The two-argument function $\operatorname{atan2}$ is used instead of $\tan^{-1}(u_y/u_x)$ so that the quadrant is retained and division by zero is avoided. Because the ground frame is fixed and the body marker is aligned with the body frame in this example,

$$
\theta_p=\theta,
\qquad
\omega_p=\dot\theta_p=\omega,
\qquad
\alpha_p=\dot\omega_p=\ddot\theta_p=\alpha.
$$

Thus, the two point-constraint equations together with the relative rotation coordinate completely describe the planar pin. The rotation is a joint variable or joint output, not an additional constraint: the ideal pin permits it to vary freely.

## 7. Applied loads and rigid-body balances

The pin reaction $\lambda^g$ contributes directly to the body's global force summation:

$$
F_{\mathrm{pin}}^g=\lambda^g.
$$

Resolved in the body frame, it produces the moment about the center of mass

$$
T_{\mathrm{pin}}^b
=r_m^b\times\left(A^{bg}\lambda^g\right).
$$

Gravity passes through the center of mass and therefore produces no moment about it. Newton's force balance is

$$
m a^g-m g^g-\lambda^g=0.
$$

Euler's torque balance about the center of mass is

$$
J\alpha-r_m^b\times\left(A^{bg}\lambda^g\right)=0.
$$

The reaction force is an unknown determined together with the body acceleration. It is not necessary to calculate it before writing the body equations.

## 8. Kinematic definition equations

The configuration, velocity, and acceleration variables are related by

$$
\dot R^g-V^g=0,
$$

$$
\dot\theta-\omega=0,
$$

$$
\dot V^g-a^g=0,
$$

$$
\dot\omega-\alpha=0.
$$

Acceleration remains an explicit unknown in the unreduced mechanical system.

## 9. Common variable set

A convenient unreduced set of variables is

$$
q=(R^g,\theta),
\qquad
v=(V^g,\omega),
\qquad
a=(a^g,\alpha),
\qquad
\lambda=\lambda^g,
$$

and the pin owns the relative variables

$$
q_p=\theta_p,
\qquad
v_p=\omega_p,
\qquad
a_p=\alpha_p.
$$

Their local definition equations are

$$
\theta_p-\operatorname{atan2}(u_y,u_x)=0,
$$

$$
\omega_p-\omega=0,
$$

$$
\alpha_p-\alpha=0.
$$

For this simple fixed-ground pin, the last two equations are the velocity and acceleration derivatives of the angular-coordinate definition. Retaining these apparently redundant relative variables makes the pin's permitted motion available to other components and gives later force elements simple local variables with respect to which partial derivatives can be calculated.

In scalar terms, the body sets $q$, $v$, and $a$ each contain three quantities, the pin reaction $\lambda$ contains two, and the pin contributes one relative variable at each kinematic level. The common equations available to later formulations are:

- two Newton force-balance equations;
- one Euler torque-balance equation;
- three kinematic definition equations between configuration and velocity;
- three kinematic definition equations between velocity and acceleration;
- two pin-constraint equations at each of the position, velocity, and acceleration levels; and
- one relative pin-coordinate definition at each of the position, velocity, and acceleration levels.

The constraint levels are related by differentiation and should not be counted as independent equations simultaneously. The next development will show how a selected formulation chooses and organizes the appropriate equations.

## 10. Degrees of freedom and the Grübler criterion

An unconstrained rigid body moving in a plane has three degrees of freedom: two translations and one rotation. For a planar mechanism containing $n$ moving rigid bodies, begin with $3n$ freedoms. If the mechanism has $c$ independent scalar constraint equations, its mobility is

$$
N_{\mathrm{DOF}}=3n-c.
$$

Here the fixed ground is not included in $n$. This is the planar form of the Grübler mobility count when the scalar constraints are independent and the mechanism is in a regular configuration.

The pendulum contains one moving rigid body and the ideal pin supplies two independent translational constraint equations. Therefore,

$$
N_{\mathrm{DOF}}=3(1)-2=1.
$$

The relative pin angle $\theta_p$ is a natural coordinate for this remaining freedom. Once the constraints and reaction force have been eliminated, the motion can theoretically be described by one second-order differential equation in one independent coordinate.

A conventional first-order state-space representation replaces each second-order equation with two coupled first-order equations. Consequently, each mechanical degree of freedom produces two state variables: a coordinate and its velocity. For this pendulum, a reduced state may be written

$$
y=
\begin{bmatrix}
\theta_p\\
\omega_p
\end{bmatrix},
$$

so the one-degree-of-freedom pendulum has two independent state variables.

The unreduced formulation contains more variables than this degree-of-freedom count. Its body translations, reaction forces, accelerations, and local pin variables are connected by algebraic and kinematic equations. They are useful unknowns in the assembled system, but they are not additional independent mechanical degrees of freedom. An implicit differential-algebraic or mixed-order method may retain these variables rather than first reducing the model symbolically to its minimum state variables.

The simple count $3n-c$ assumes that the constraint rows are independent. Redundant constraints do not remove additional degrees of freedom, and a singular configuration can change the instantaneous rank of the constraint equations. In those cases, the degree-of-freedom count must use the rank of the constraint partial matrix rather than merely the number of written constraint rows.

## 11. Planned uses of the equation set

The model can next be examined without changing its physical definition:

1. Assemble the minimal Cartesian acceleration-level equations for $(a^g,\alpha,\lambda^g)$ at a known configuration and velocity, leaving out the relative pin coordinate.
2. Eliminate the pin reaction and translational acceleration to recover a single angular equation.
3. Retain configuration, velocity, acceleration, and reaction variables in an unreduced implicit system.
4. Apply BDF relations and form the corresponding Newton equations.

More complicated models can then add forcing elements, a second body, closed loops, stiff compliant connections, and flexible bodies while retaining the same basic separation between structural equations, connection kinematics, constraint equations, and applied loads.

## 12. BDF relationships

A BDF formula gives a linear relationship between a variable at the new time point and its derivative there. For any integrated variable $z$,

$$
\dot z_n=\frac{c_0}{h}z_n+d_n,
$$

where $h$ is the time step, $c_0$ is the leading BDF coefficient, and $d_n$ contains known information from previous time points. Equivalently,

$$
z_n=\eta\,\dot z_n+z_n^{\,h},
\qquad
\eta=\frac{h}{c_0},
$$

where $z_n^{\,h}$ is known during the solution at time $t_n$. Therefore the Newton corrections satisfy

$$
\Delta z_n=\eta\,\Delta\dot z_n.
$$

For the translational chain

$$
\dot R^g=V^g,
\qquad
\dot V^g=a^g,
$$

the corresponding correction relationships are

$$
\Delta V^g=\eta\,\Delta a^g,
\qquad
\Delta R^g=\eta^2\,\Delta a^g.
$$

For the body rotation,

$$
\dot\theta=\omega,
\qquad
\dot\omega=\alpha,
$$

so

$$
\Delta\omega=\eta\,\Delta\alpha,
\qquad
\Delta\theta=\eta^2\,\Delta\alpha.
$$

The same BDF relationships apply to the pin-relative chain $(\theta_p,\omega_p,\alpha_p)$, but they are not repeated here. These equations do not change the mechanical model. They are linear time-discretization relationships used to connect corrections of variables at different derivative levels.

## 13. Full equation structure

The following table collects the equations in descending derivative order. The primary columns begin with the highest derivatives and reactions, followed by velocity and configuration variables. The columns $\omega_p$ and $\theta_p$ show the pin's local first-derivative and coordinate definitions explicitly. The $\dot p$ and $p$ columns show the alternative two-component Euler-parameter description of body orientation.

Define the row operator

$$
\rho_m^T=
\begin{bmatrix}
-r_y & r_x
\end{bmatrix},
$$

so that

$$
r_m^b\times f^b=\rho_m^T f^b.
$$

Each row of the table is written in the form

$$
\sum_j C_{kj}z_j=b_k.
$$

Some entries contain the current values of $\theta$ or $\omega$. The equation set is therefore nonlinear even though it is linear in the highest derivatives $(a^g,\alpha,\alpha_p)$ and the reaction $\lambda^g$ when position and velocity are known.

| Variables: | $a^g$ | $\alpha$ | $\alpha_p$ | $\lambda^g$ | $V^g$ | $\omega$ | $\dot p$ | $R^g$ | $\theta$ | $p$ | $\omega_p$ | $\theta_p$ | Right side $b_k$ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| $\sum{F}$ | $mI$ |  |  | $-I$ |  |  |  |  |  |  |  |  | $m g^g$ |
| $\sum{T}$ |  | $J$ |  | $-\left[A^{gb}(\theta)Sr_m^b\right]^T$ |  |  |  |  |  |  |  |  | $0$ |
| $\ddot{\Phi}$ | $I$ | $A^{gb}(\theta)Sr_m^b$ |  |  |  | $-A^{gb}(\theta)r_m^b\,\omega$ |  |  |  |  |  |  | $0$ |
| $\dot{\Phi}$ |  |  |  |  | $I$ | $A^{gb}(\theta)Sr_m^b$ |  |  |  |  |  |  | $0$ |
| $\Phi$ |  |  |  |  |  |  |  | $I$ | $A^{gb}(\theta)r_m^b$ † | $A^{gb}(p)r_m^b$ † |  |  | $p_0^g$ |
| $\ddot{\Theta}$ |  | $-1$ | $1$ |  |  |  |  |  |  |  |  |  | $0$ |
| $\dot{\Theta}$ |  |  |  |  |  | $-1$ |  |  |  |  | $1$ |  | $0$ |
| $\Theta$ |  |  |  |  |  |  |  |  | $-1$ |  |  | $1$ | $0$ |
| $K_p$ * |  |  |  |  |  | $1$ | $-B(p)$ |  |  |  |  |  | $0$ |
| $N_p$ * |  |  |  |  |  |  |  |  |  | $p^T$ |  |  | $1$ |
| $K_a$ ¶ | $-I$ | $-1$ | $-1$ |  | $\dfrac{d}{dt}I$ | $\dfrac{d}{dt}$ |  |  |  |  | $\dfrac{d}{dt}$ |  | $0$ |
| $K_v$ ‖ |  |  |  |  | $-I$ | $-1$ |  | $\dfrac{d}{dt}I$ | $\dfrac{d}{dt}$ |  | $-1$ | $\dfrac{d}{dt}$ | $0$ |
| BDFa: ‡ | $-\eta I$ | $-\eta$ | $-\eta$ |  | $I$ | $1$ |  |  |  |  | $1$ |  | $(V^{g,h},\omega^h,\omega_p^h)$ |
| BDFv: § |  |  |  |  | $-\eta I$ | $-\eta$ | $-\eta I$ | $I$ | $1$ | $I$ | $-\eta$ | $1$ | $(R^{g,h},\theta^h,p^h,\theta_p^h)$ |

† The entries $A^{gb}(\theta)r_m^b$ and $A^{gb}(p)r_m^b$ are alternative nonlinear orientation-dependent terms in the constraint, not coefficients multiplying $\theta$ or $p$. Likewise, the $\omega$ entry in the second-derivative constraint is written as $[-A^{gb}r_m^b\,\omega]\omega$ to display the complete centripetal term $-A^{gb}r_m^b\omega^2$ while retaining the requested column structure.

* The Euler-parameter rows represent the alternative orientation equations

$$
K_p:\qquad \omega-B(p)\dot p=0,
$$

and

$$
N_p:\qquad p^Tp-1=0.
$$

They replace the scalar-angle orientation relationship when $p$ is selected as the body's orientation coordinate. Both alternatives are included in the inventory because a solution method may select either one.

¶ The acceleration-to-velocity kinematic block row represents

$$
\dot V^g-a^g=0,
\qquad
\dot\omega-\alpha=0,
\qquad
\dot\omega_p-\alpha_p=0.
$$

‖ The velocity-to-position kinematic block row represents

$$
\dot R^g-V^g=0,
\qquad
\dot\theta-\omega=0,
\qquad
\dot\theta_p-\omega_p=0.
$$

‡ The acceleration-to-velocity block row represents

$$
V^g-\eta a^g=V^{g,h},
\qquad
\omega-\eta\alpha=\omega^h,
\qquad
\omega_p-\eta\alpha_p=\omega_p^h.
$$

§ The velocity-to-position block row represents

$$
R^g-\eta V^g=R^{g,h},
\qquad
\theta-\eta\omega=\theta^h,
\qquad
p-\eta\dot p=p^h,
\qquad
\theta_p-\eta\omega_p=\theta_p^h.
$$

### 13.1 Variable and equation count

The table contains 18 possible scalar variables:

$$
\begin{aligned}
a^g &: 2, & \alpha &: 1, & \alpha_p &: 1, & \lambda^g &: 2,\\
V^g &: 2, & \omega &: 1, & R^g &: 2, & \theta &: 1,\\
\dot p &: 2, & p &: 2, & \omega_p &: 1, & \theta_p &: 1.
\end{aligned}
$$

The mechanical, constraint, and relative-coordinate rows represent 12 scalar equations:

$$
\underbrace{2}_{\sum F}
+
\underbrace{1}_{\sum T}
+
\underbrace{2+2+2}_{\ddot\Phi,\dot\Phi,\Phi}
+
\underbrace{1+1+1}_{\ddot\Theta,\dot\Theta,\Theta}
=12.
$$

The Euler-parameter rows add two possible scalar equations:

$$
\underbrace{1}_{K_p}
+
\underbrace{1}_{N_p}
=2.
$$

The two continuous kinematic block rows represent eight scalar equations:

$$
\underbrace{4}_{K_a}
+
\underbrace{4}_{K_v}
=8.
$$

The two BDF block rows represent ten scalar equations when both orientation alternatives are included:

$$
\underbrace{4}_{\mathrm{BDFa}}
+
\underbrace{6}_{\mathrm{BDFv}}
=10.
$$

Thus, the complete table displays 32 possible scalar relationships involving 18 possible scalar variables. These are not 32 independent equations. The angle $\theta$ and the Euler parameters $p$ are alternative orientation descriptions. The position-, velocity-, and acceleration-level forms of $\Phi$ and $\Theta$ are related by differentiation. In addition, the BDF rows are discrete forms of the continuous kinematic rows, not additional equations to impose beside them. A solution method selects an appropriate independent subset of both variables and equations.

### 13.2 Equation selections used by different methods

The first solution is a minimal Cartesian acceleration-level model. It begins with known position and velocity and leaves out the relative pin coordinate and its definition equations. Its unknowns are

$$
(a^g,\alpha,\lambda^g),
$$

which contain five scalar quantities. It uses

$$
\sum F,\qquad
\sum T,\qquad
\ddot\Phi,
$$

giving $2+1+2=5$ scalar equations. The resulting body accelerations are then passed to a separate numerical integration procedure. The relative pin angle, velocity, and acceleration can be evaluated as outputs but are not retained as independent variables in this first model.

An unreduced position-level BDF solution retains all 14 variables and uses

$$
\sum F,\qquad
\sum T,\qquad
\Phi,\qquad
\Theta,\qquad
\mathrm{BDFa},\qquad
\mathrm{BDFv}.
$$

The count is

$$
2+1+2+1+4+4=14.
$$

A velocity-level BDF formulation replaces $(\Phi,\Theta)$ with $(\dot\Phi,\dot\Theta)$, while an acceleration-level BDF formulation uses $(\ddot\Phi,\ddot\Theta)$. Each selection again supplies 14 scalar equations. The numerical behavior differs because enforcing a differentiated constraint does not necessarily eliminate accumulated error in its lower-level forms.

A future Euler-parameter formulation can omit the scalar body angle $\theta$ and retain the two components each of $p$ and $\dot p$. If the relative pin variables are omitted for the first such example, its 14 scalar variables are

$$
(a^g,\alpha,\lambda^g,V^g,\omega,\dot p,R^g,p).
$$

A position-level BDF selection can use

$$
\sum F,\qquad
\sum T,\qquad
\Phi,\qquad
K_p,\qquad
N_p,\qquad
\mathrm{BDFa},\qquad
\mathrm{BDFv},
$$

where the selected part of $\mathrm{BDFv}$ relates $R^g$ to $V^g$ and $p$ to $\dot p$. The scalar count is

$$
2+1+2+1+1+3+4=14.
$$

A reduced-coordinate method uses the pin relationships to eliminate dependent translational variables and reactions, leaving a smaller equation set in the permitted angular coordinate. Constraint correction or stabilization methods may use information from more than one level, but must account for the dependence among those equations.

The physical relationships in the table do not change between these methods. What changes is which equations and variables are retained in the system presented to the numerical solver.
