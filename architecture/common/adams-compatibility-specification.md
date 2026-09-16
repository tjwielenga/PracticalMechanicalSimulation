# ADAMS Compatibility Specification

This document defines the ADAMS conventions that Simp follows where the two
programs describe the same mechanical quantity. It also records the places
where the current implementation intentionally differs. It is the
compatibility contract for current behavior and future implementation work.

The goal is semantic compatibility rather than identical command syntax. A
person translating a model should be able to preserve marker order, load signs,
reference frames, and mass properties without rediscovering conventions by
experiment.

## Compatibility levels

The audit uses four labels:

- **Aligned** means the current Simp definition has the same physical meaning.
- **Conditional** means it agrees under a stated geometric or modeling
  condition.
- **Different** means the present physical result or reported scalar uses a
  different convention.
- **Simp-specific** means there is no useful one-to-one ADAMS element contract.

## Canonical marker convention

For a two-ended element, the ordered markers are called $I$ and $J$.

- $I$ is the first or action side.
- $J$ is the second or reaction side.
- A reported applied force or torque is the load on the body that owns $I$.
- The body that owns $J$ receives the corresponding reaction.
- Relative motion is motion of $I$ with respect to $J$.
- Unless another reference marker is named, relative vector components are
  resolved in the $J$ frame.

This rule applies to connections, bushings, two-ended forces, and motion
generators. It also supplies the reporting convention for constraint reactions:
a positive reaction variable describes the load on the first side.

An applied vector force has three distinct roles and is therefore an exception
to the simple two-marker spelling:

- $I$ is the application marker.
- `RM` is the marker whose axes define the force direction.
- `JFLOAT`, when present, is a reaction marker on another body that remains
  coincident with $I$.

Simp currently spells these roles as `markers = [application, direction]` and
an optional `reaction_body`. This is semantically equivalent to ADAMS VFORCE,
but the second entry is `RM`, not $J$.

## Relative translation

Let $P_I$ and $P_J$ be marker positions. The ADAMS-compatible relative vector
is

$$
d_{IJ}=P_I-P_J.
$$

It points from $J$ toward $I$. Its components in reference marker $R$ are

$$
d_{IJ}^{R}=A_R^T(P_I-P_J).
$$

Thus `DX(I,J,R)`, `DY(I,J,R)`, and `DZ(I,J,R)` are components of the vector
from $J$ to $I$. The distance

$$
b=\lVert P_I-P_J\rVert
$$

is always nonnegative, and its rate is positive when the markers separate.

Planar directed distance and spatial directed distance already use this
definition. A planar directed distance uses the $y_J$ direction and a spatial
directed distance uses the $z_J$ direction.

The internal Simp span vector is $P_J-P_I$. It is explicitly a geometric vector
from the first endpoint to the second, not an implementation of ADAMS `D*`.
Its length and length rate have the same meaning in either direction. The
`s_*` and `u_*` outputs must retain their endpoint-direction description so
they are not mistaken for ADAMS relative-position components.

## Line-of-sight force

Define the ADAMS line-of-sight unit vector

$$
e_{IJ}=\frac{P_I-P_J}{b}.
$$

An ADAMS-compatible scalar line force $f$ is the actual force on $I$ along
$e_{IJ}$:

$$
F_I=f e_{IJ}, \qquad F_J=-F_I.
$$

Therefore:

- $f>0$ repels the markers, or puts the element in compression.
- $f<0$ attracts the markers, or puts the element in tension.

For a linear spring-damper,

$$
f=-k(b-b_0)-c\dot b+f_0.
$$

This is the scalar that should be stored, plotted, exported, and supplied by a
force expression.

Planar and spatial spanning forces retain a geometric unit vector $u$ from $I$
toward $J$, but their public scalar now follows the ADAMS convention:

$$
f=-k(b-b_0)-c\dot b,
$$

$$
F_I=-fu=f e_{IJ}.
$$

Spanning-motion reactions use the same reporting rule. Their scalar is the
actual force on $I$ along $e_{IJ}$, even though the sign of an unreported
Lagrange multiplier would otherwise be arbitrary.

Pulley-belt tension is different. Tension is a nonnegative material quantity,
not an ADAMS single-component force result. Belt elements may retain a positive
tension variable, but the endpoint forces must still be reported separately as
loads on their named sides.

## Relative rotation and torque

For aligned planar markers or spatial markers whose $z$ axes are parallel and
point in the same direction, the relative angle is the rotation of $I$ with
respect to $J$ about positive $z_J$. In three dimensions this is ADAMS
`AZ(I,J)`:

$$
\theta_{IJ}=\operatorname{atan2}
  (\hat x_I^T\hat y_J,\hat x_I^T\hat x_J).
$$

The corresponding relative angular velocity is

$$
\omega_{IJ}=\hat z_J^T(\Omega_I-\Omega_J).
$$

A positive scalar torque $T$ is the actual torque on $I$ by the right-hand
rule. The second side receives $-T$. A linear torsional spring-damper therefore
reports

$$
T=-k(\theta_{IJ}-\theta_0)-c\omega_{IJ}+T_0.
$$

The planar torsional spring-damper stores this actual torque on $I$ and applies
its opposite to $J$. Its physical torque and public result are **aligned**.

Planar applied torque and spatial applied torque already store and apply a
positive torque to the first side. The spatial built-in spring form also uses
the ADAMS sign. A spatial rotational spring is **conditional** because the
$I$ and $J$ axes must remain parallel and point in the same direction for the
meaning to be identical to an ADAMS rotational spring-damper.

## Vector forces and torques

For a marker-directed applied force, positive components act on $I$ along the
axes of `RM`. A reaction body, when supplied, receives the equal-and-opposite
force at a generated floating marker coincident with $I$. Planar applied force
uses $y_{RM}$ and spatial applied force uses $z_{RM}$. Both are **aligned** with
the corresponding one-component use of ADAMS VFORCE.

Applied torque follows the same action-side rule. Positive torque uses the
right-hand rule about the selected marker axis. The current planar and spatial
applied torques are **aligned**, subject to the aligned-axis condition for a
two-ended spatial rotational element.

If equal and opposite forces are applied at different points, the reaction
wrench must also preserve moment equilibrium. With

$$
L=P_I-P_J,
$$

an equivalent wrench placed at $J$ is

$$
F_J=-F_I, \qquad
T_J=-T_I-L\times F_I.
$$

Simp normally generates a floating reaction point coincident with $I$. In that
case the force application itself supplies the offset moment and the free
reaction torque is simply $-T_I$. The planar and spatial applied-force and
bushing implementations follow this construction and are **aligned**.

## Bushings

The bushing translation and translation rate are motion of $I$ relative to
$J$, resolved in $J$:

$$
r^J=A_J^T(P_I-P_J),
$$

$$
v^J=A_J^T(V_I-V_J)-\omega_J^J\times r^J.
$$

The rotating-frame term is required. For diagonal laws without preload, the
force and torque on $I$ are

$$
f^J=-K_t r^J-C_t v^J,
$$

$$
\tau^J=-K_r\alpha-C_r\omega_{IJ}^J.
$$

The first marker receives these loads and the second body receives the
opposite wrench at a floating point. Planar bushings and the translational part
of spatial bushings are **aligned**.

### Spatial bushing angles

Current Simp spatial bushings use one $z$-$y$-$x$ Bryant-angle decomposition.
That choice was made so a compliant revolute can undergo large rotation about
$z_J$ while its other rotations remain small. It is not the same as the
projected marker-angle functions used by current ADAMS documentation for a
general compound rotation.

The ADAMS element-output projection angles are independently measured as

$$
\alpha_x=\operatorname{atan2}
  (\hat y_I^T\hat z_J,\hat y_I^T\hat y_J),
$$

$$
\alpha_y=\operatorname{atan2}
  (\hat z_I^T\hat x_J,\hat z_I^T\hat z_J),
$$

$$
\alpha_z=\operatorname{atan2}
  (\hat x_I^T\hat y_J,\hat x_I^T\hat x_J).
$$

The two definitions agree to first order, but not for a general finite
rotation. In particular, an ADAMS projection angle can change branch as a
denominator changes sign even when the associated out-of-plane projection is
zero. That behavior motivated Simp's large-$z$ Bryant convention. The current
spatial bushing is therefore **different** in rotational deformation. Before
changing it, finite-rotation tests should compare both angle sets and an
imported vehicle bushing should be used to confirm the intended ADAMS behavior.

## Constraint and motion reactions

The sign of an internal Lagrange multiplier is not physical until its load
mapping is defined. Simp uses the following reporting rule:

- a positive scalar or vector reaction is the load on the first marker or
  first referenced coordinate;
- the second side receives the opposite load;
- reaction vectors stored in global components are identified as global rather
  than being presented as $J$-frame ADAMS components.

Under this rule, the current planar and spatial spherical, revolute, inplane,
perpendicular-axis, inline, orient, fixed, translational, and rotational-motion
elements are **aligned** in physical sign. Composite Simp joints retain their
primitive reaction variables instead of combining them into one ADAMS-style
six-component joint result. That is an output-organization difference, not a
mechanical incompatibility.

Redundant-row removal can make individual primitive reactions inactive. The
remaining reactions are one valid load distribution for the retained
constraint basis; an ideally redundant model does not have unique individual
reactions before a basis is chosen.

## Coordinate couplers, gears, and rack-and-pinion

Coupler constraint equations can be multiplied by $-1$ without changing their
kinematics. Compatibility therefore concerns the reported reaction rather than
the bare equation sign. Simp reports the generalized load on the first listed
coordinate and maps the opposite generalized load to the remaining
coordinates.

The gear-pair and rack-and-pinion elements use Simp geometry to create contact
points and map one scalar reaction into body forces and torques. They are
**Simp-specific** formulations, but they follow the first-side load-reporting
rule. Translation from an ADAMS model must preserve joint order, signed pitch
geometry, and the selected action side rather than assuming that two arbitrary
constraint equations have the same multiplier sign.

## Contact, tire, and belt forces

The following quantities retain their conventional engineering meanings:

- normal contact force is nonnegative and repels the contacting bodies;
- tire normal force is positive from the road toward the wheel;
- belt tension is nonnegative and pulls each tangent endpoint toward the span;
- friction and tire tangential forces are signed in their documented local
  frames.

These are **Simp-specific** element contracts. They should not be sign-flipped
merely to resemble the scalar output of an unrelated ADAMS force element. An
ADAMS importer must translate the particular source contact, tire, or belt
model explicitly.

## Rigid bodies and inertia

Simp spatial bodies accept either three principal moments or a symmetric
physical inertia tensor about the center of mass. The matrix entered in the
model is the matrix used directly in

$$
I\dot\omega+\omega\times I\omega.
$$

ADAMS named products of inertia use positive mass integrals such as
$I_{xy}=\int xy\,dm$. Those named values are not the off-diagonal entries of
the physical tensor. They map to the physical matrix

$$
I=
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz}\\
-I_{xy} & I_{yy} & -I_{yz}\\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix}.
$$

Copying positive ADAMS products directly into the off-diagonal entries of a
Simp matrix reverses their meaning. Simp will not adopt that convention. Its
`inertia` field remains an ordinary physical tensor so it can be inspected,
transformed, and used in matrix algebra without a special sign rule.

This is an intentional input-format difference. If an ADAMS command-file
importer is developed, the importer must translate named ADAMS products at the
boundary and pass the resulting physical tensor to Simp. No ADAMS-specific
interpretation belongs in the Simp model reader or rigid-body equations.

The optional Simp center-of-mass marker is compatible with an ADAMS part whose
reference frame is not at its center of mass. Marker positions remain relative
to the body reference frame, while the solver converts mass properties and
motion to its internal center-of-mass representation.

## Units

Simp currently requires a consistent unit system and its examples use SI.
ADAMS models may use another consistent unit system, including millimeters.
There is no automatic unit declaration or conversion in the current reader.
This is a **different** input capability and was a material source of error in
the first vehicle conversion.

An importer must scale quantities by dimension, not just positions:

| Quantity | Dimension |
|---|---|
| position, free length | length |
| velocity | length / time |
| acceleration | length / time$^2$ |
| force | force |
| torque | force $\times$ length |
| mass moment of inertia | mass $\times$ length$^2$ |
| translational stiffness | force / length |
| translational damping | force $\times$ time / length |
| rotational stiffness | torque / angle |
| rotational damping | torque $\times$ time / angle |

Angles in Simp equations are radians. A coefficient entered per degree must be
converted in its denominator as well as converting its torque unit. For
example, a stiffness in N-mm/degree is multiplied by
$10^{-3}(180/\pi)$ to obtain N-m/radian.

## Audit of the current library

| Simp capability | Current status | Compatibility note |
|---|---|---|
| body position and CM marker | Aligned | Body reference and CM may differ. |
| diagonal/principal inertia | Aligned | Same physical moments after unit conversion. |
| full inertia matrix | Intentionally different | Simp stores the physical tensor directly; an importer must translate ADAMS named products. |
| gravity | Aligned | Ordinary applied body force. |
| marker-directed applied force | Aligned | Simp list is `[I, RM]`; reaction body creates `JFLOAT`. |
| planar applied torque | Aligned | Stored scalar is torque on $I$. |
| spatial applied torque | Conditional | Aligned joint axes are required for ADAMS rotational-force equivalence. |
| planar torsional spring-damper | Aligned | Stored scalar is the restoring torque on $I$. |
| planar bushing | Aligned | Motion in $J$ frame; restoring load on $I$. |
| spatial bushing translation | Aligned | Includes the rotating-frame velocity term. |
| spatial bushing rotation | Different | Current Bryant angles differ for compound rotation. |
| span length and rate | Aligned | Both are independent of endpoint-vector direction. |
| span vector and unit vector | Simp-specific | Explicit geometry points $I$ to $J$ and is not named as ADAMS `D*`. |
| planar/spatial spanning force | Aligned | Scalar is the load on $I$ along the $J$-to-$I$ line of sight. |
| spanning-motion reaction | Aligned | Scalar is the load on $I$ along the $J$-to-$I$ line of sight. |
| directed-distance measures | Aligned | Motion of $I$ from $J$ along a $J$ axis. |
| ideal constraint reactions | Aligned | Positive value is load on first side; vectors may be stored globally. |
| translational/rotational generators | Aligned | Reaction is reported on first side. |
| gear, rack, and coordinate coupler | Simp-specific | Preserve first-side generalized-load reporting. |
| plane contact and rolling tire | Simp-specific | Positive normal load repels. |
| pulley and belt | Simp-specific | Positive tension is retained as a material quantity. |
| result load colors | Aligned presentation | Applied color denotes first-side load; reaction color denotes opposite side. |

## Implementation status and remaining work

The load-sign compatibility work was completed together so physical responses
remained unchanged:

1. Planar and spatial spanning-force scalars now report the force on $I$ along
   the $J$-to-$I$ line of sight. The built-in spring law is
   $-k(b-b_0)-c\dot b$.
2. Spanning-motion reaction scalars use the same first-side convention.
3. The planar torsional spring variable is the actual torque applied to $I$.

The intentional formulation choices remain:

4. Keep the spatial Bryant bushing angles. Their large-$z$ behavior is more
   useful for Simp bushings than the ADAMS projection-angle branches.
5. Keep `inertia` unconditionally defined as the physical tensor. If an ADAMS
   importer is later written, isolate named-product conversion inside that
   importer and do not expose it as another Simp inertia convention.

Possible future compatibility work is limited to interfaces:

6. Add frame and side metadata to result variables where a name alone does not
   say whether a vector is global, $J$-local, applied to $I$, or applied to $J$.
7. Add a unit-aware import layer before treating ADAMS command files as direct
   model input.

Repository expressions were converted with the sign change. External models
written for the earlier positive-tension spanning-force convention must reverse
their scalar expressions. Existing stored `.simp` results remain readable, but
their scalar histories retain the convention used when they were generated.

## Implementation sources audited

The status table was checked against the equations and load mappings in these
current implementation files:

- [Planar component assembly](../../src/planar/PlanarComponentAssembly.jl)
- [Planar applied-force primitives](../../src/planar/PlanarAppliedForces.jl)
- [Spatial applied forces and torques](../../src/spatial/SpatialAppliedForces.jl)
- [Spatial bushings](../../src/spatial/SpatialBushings.jl)
- [Spatial constraints and their load mappings](../../src/spatial/SpatialConstraints.jl)
- [Spatial directed distances](../../src/spatial/SpatialDirectedDistances.jl)
- [Spatial spans](../../src/spatial/SpatialSpans.jl)
- [Spatial motion generators](../../src/spatial/SpatialMotionGenerators.jl)
- [Spatial body and inertia input](../../src/spatial/SpatialModelIO.jl)

The audit also used the current
[planar element formulations](../planar/planar-element-formulations.md) and
[spatial element formulations](../spatial/spatial-element-formulations.md) to
separate intentional public definitions from incidental internal signs.

## Compatibility tests

Each future correction should have small tests that check physical loads and
reported values separately.

- An extended two-marker spring must attract the markers while reporting a
  negative force on $I$.
- A compressed spring must repel the markers while reporting a positive force
  on $I$.
- A positive relative hinge angle with a zero-free-angle torsional spring must
  report a negative torque on $I$.
- A vector force must follow `RM`, act at $I$, and place an equal-and-opposite
  reaction at a coincident floating marker.
- A translated and rotating bushing pair must produce zero deformation rate
  under common rigid motion.
- Compound marker rotations must distinguish and verify the selected spatial
  bushing angle convention.
- An ADAMS-import test with nonzero named products must produce the expected
  physical Simp tensor and angular momentum without changing Simp's matrix
  interpretation.
- Constraint and generator result variables must equal the actual load
  assembled on their first side.

## Authoritative references

The convention above is based on Hexagon's current ADAMS documentation:

- [Adams 2023.4.1 View Command User Guide](https://documentation-be.hexagon.com/bundle/Adams_2023.4.1_Adams_View_Command_User_Guide/raw/resource/enus/Adams_2023.4.1_Adams_View_Command_User_Guide.pdf), especially the translational and rotational spring-damper definitions and single-component-force marker rules.
- [Adams 2022.4 Solver User Guide](https://documentation-be.hexagon.com/bundle/Adams_2022.4_Adams_Solver_User_Guide/raw/resource/enus/Adams_2022.4_Adams_Solver_User_Guide.pdf), especially `DM`, `DX/DY/DZ`, `AX/AY/AZ`, `JOINT`, `MOTION`, `VFORCE`, `VTORQUE`, `BUSHING`, and part inertia definitions.
- [Adams 2023.2 View User Guide](https://documentation-be.hexagon.com/bundle/Adams_2023.2_Adams_View_User_Guide/raw/resource/enus/Adams_2023.2_Adams_View_User_Guide.pdf), especially bushing, action/reaction, and torsion-spring descriptions.
