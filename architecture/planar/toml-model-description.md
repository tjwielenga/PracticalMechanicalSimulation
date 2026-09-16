# TOML Model Description

Status: executable initial planar format

This document explains the model-language design. For a concise inventory of
every currently accepted field and default, see the
[planar TOML User's Guide](../../docs/planar/toml-reference.md).

## 1. Named elements and hierarchy

The model file describes mechanical objects, not canonical variable indices,
equation indices, sparse-matrix locations, or solver callbacks. Each object is a
named TOML table whose `type` selects a registered modeling element:

```toml
[crank]
type = "rigid_body"

[crank.end]
type = "marker"
position = [0.25, 0.0]
```

The table path is the object's qualified name. Thus `crank.end` is a marker
contained by `crank`; it does not need separate `name` or `body` fields. This
containment hierarchy is distinct from mechanical connectivity. A connection
refers to its endpoints by qualified name.

Single-bracket tables are intentional. Double brackets such as `[[marker]]`
would create an array of anonymous marker records rather than one named object.

`[model]`, `[parameters]`, `[analysis]`, `[simulation]`,
`[initial_conditions]`, `[state_selection]`, and `[graphics]` are reserved
configuration tables. All other typed tables are elements. Canonical component
allocation is deterministic by qualified name and is not exposed in the file.

Graphical definitions occupy an optional `graphics` namespace beneath their
owning mechanical element. A named graphical primitive uses a `shape` field
rather than an element `type`, so it is never registered with the mechanical
system and cannot contribute variables or equations. It follows one or two
qualified markers already present in the model. Model-level color aliases and
palettes live in the reserved top-level `[graphics]` table. Because the result
file embeds the original TOML verbatim, the stored-result viewer can reconstruct
these shapes from marker histories without enlarging the canonical history or
changing the `.simp` schema.

The optional `[graphics.loads]` table sets the initial visibility and colors
of reaction forces, applied loads, and torques. The viewer reconstructs these
physical vectors from the saved variables and embedded component definitions,
so the load graphics also require no additional simulation variables.

## 2. Markers

A marker contains both position and orientation, following the usual mechanical
modeling convention. In a planar model its optional fields are

```toml
[body.marker]
type = "marker"
position = [0.0, 0.0]
orientation = 0.0
```

Omitted position and orientation fields default to zero. Planar orientation is
one angle in radians about the positive $z$-axis. A marker nested under a
`rigid_body` is body-fixed; one nested under `ground` is fixed globally. The
loader constructs separate point and orientation views internally only because
the current equation components consume those narrower interfaces.

A `floating_marker` separates translational location from ownership. It is
nested beneath the body that receives its forces and whose orientation it uses,
but its `follows` field names an ordinary marker that supplies its global
position and velocity:

```toml
[gear.contact]
type = "floating_marker"
follows = "carrier.contact"
```

This represents a gear contact marker owned by the gear while its point follows
a contact location fixed on the carrier.

For

```toml
[guide]
type = "inplane"
markers = ["rod.end", "ground.plane"]
```

the plane passes through the second marker. Its normal is the second marker's
local $y$-axis, transformed by that marker's angle. The planar constraint is

$$
\Phi=(P_i-P_j)^T A_j\begin{bmatrix}0\\1\end{bmatrix}=0,
$$

and its scalar reaction acts along the same normal. No independent `normal` or
`direction` field is accepted. In the future spatial form, the second marker's
local $z$-axis will define the plane normal.

Ideal joints can be assembled from named constraint primitives. In the planar
model, `inplane` removes translation along the second marker's $y$-axis, while
`perp` removes relative rotation by making the first marker's $x$-axis
perpendicular to the second marker's $y$-axis. Using both primitives between
the same marker pair produces a translational joint without requiring a
separate monolithic equation component.

The user-facing `translational` joint expands into generated
`joint-name.inplane` and `joint-name.perp` components. The second marker's
local $y$-axis is the inplane normal, leaving motion along its local $x$-axis.
The primitive components retain ownership of their equations, Jacobian
contributions, and reactions; the translational joint only groups them.

The user-facing `fixed` joint applies the same composition rule. It expands
into generated `fixed-name.revolute` and `fixed-name.perp` components. Those
primitive components retain ownership of their equations, Jacobian
contributions, and reaction variables; the fixed joint only groups them behind
one marker-pair interface.

The planar `gear_pair` connection names two revolute joints and a contact
marker on the geometric carrier. The first marker of each revolute belongs to
the corresponding gear. The contact marker must belong to the base body of at
least one joint, although the two joint bases need not be the same body. The
loader creates a floating contact marker on each gear. It calculates a tangent
from the contact point and the base-side marker of the joint supported by that
carrier. The projections of the two center-to-contact vectors onto the
reference radial direction give the signed pitch radii. They have opposite
signs for an external pair and the same sign for an internal pair. The second
radius is subtracted in the constraint because the two contact forces are
equal and opposite. The scalar reaction acts on the first gear along the
tangent; its equal-and-opposite acts on the second gear. The viewer uses the
force on the first gear for its contact arrow. Applying the pair through the
generated floating markers keeps contact geometry, reaction direction, and
force application points explicit without generalized gear torques that bypass
the body force balances. This also allows a planetary contact to follow a moving
carrier when a sun or ring bearing is supported by ground. An initial phase is
calculated by default so the constraint preserves the supplied gear angles.

The planar `rack_and_pinion` connection names an ordered translational joint
and revolute joint attached to a common carrier, plus a positive pitch radius.
The translational joint's carrier-side $x$-axis defines rack motion and contact
force; its $y$-axis and the pitch radius locate contact from the carrier-side
revolute center. The element generates a carrier contact marker and floating
rack and pinion contact markers. Its scalar constraint couples rack
translation to pinion rotation, while its equal-and-opposite contact forces
enter the ordinary body force and torque balances at the generated floating
markers. Its phase may be numeric or `"initial"`; the initial value is the
default and preserves the supplied rack position and pinion angle.

## 3. Bodies, Forces, and motion

A planar `rigid_body` supplies nonnegative mass, nonnegative scalar inertia, an
initial pose, and optional initial velocity values. Initial-value fields default
to zero:

```toml
position = [0.0, 0.0]
orientation = 0.0
velocity = [0.0, 0.0]
angular_velocity = 0.0
```

For numerical state selection, the body's characteristic length defaults to
the greatest distance from its center of mass to any marker on the body. A
body with no noncentral marker uses its radius of gyration
$\sqrt{I/m}$ when it is defined and positive. A massless intermediate with no
noncentral marker uses the largest body length in the model, or one when no
geometric length is available. The uncommon case needing a deliberate
numerical scale may override this positive length:

```toml
characteristic_length = 0.25
```

Mass and inertia must be written explicitly, but either may be zero. A zero
value removes the corresponding inertial term from force or moment balance, so
that balance becomes an algebraic equation. This supports intermediate bodies
whose motion is determined by connections or compliant-force equilibrium. A
model remains responsible for supplying enough equations to determine every
massless mode.

State selection continues to describe the kinematic mobility of such a model.
It may retain a massless body's velocity and position as coordinate histories,
but those state equations do not introduce inertia. The simultaneous implicit
solution lets an algebraic force balance drive the retained histories. Tests
cover constrained massless intermediates, a massless body in a dynamic
mechanism, damped equilibrium motion, and the purely algebraic relation
$k\theta=T(t)$. If the initial position does not already satisfy its algebraic
force balance, static-equilibrium initialization provides the consistent
starting configuration.

Each body's optional initial-condition correction scale is

```toml
ic_weight_scale = 1.0
```

It is stored separately from equation scaling and integration error
tolerances. Before automatic state selection, the loader uses mass-based
weights in minimum-correction projections of the supplied configuration and
velocity onto the level-zero and level-one equations at
`simulation.start_time`. Translation has weight $s\,m$ and rotation has weight
$s\,m\,L^2$, where $s$ is `ic_weight_scale` and $L$ is the body's characteristic
length. The factor $mL^2$ is a scalar inertia approximation rather than the
body's specified moment of inertia. The same weights apply to position and
velocity correction. Larger scales make all coordinates of that body resist
correction more strongly.

The characteristic length is normally the largest body-marker offset and is
also used in automatic state-selection scaling. When no marker supplies a
length, the loader uses the radius of gyration $\sqrt{J/m}$ when available and
otherwise a model-level length. A zero-mass body receives a small positive IC
mass based on the smallest positive body mass in the model.

An element-local `[element.initial.impose]` table removes named position or
velocity variables from the correction columns and assigns their exact initial
values. Body tables accept `R_x`, `R_y`, `theta`, `V_x`, `V_y`, and `omega`.
Enabled revolute coordinates accept `angle` and `omega`; distance coordinates
accept `distance` and `velocity`. The defining coordinate equations remain in
the projection, so imposing a relative value moves the remaining body
coordinates until the requested relative geometry or rate is satisfied. A
rank or consistency failure reports the imposed variables involved.

Applied-force elements are identified by their `type`, for example `gravity`,
`applied_force` or `applied_torque`; they do not need to live in a generic
`loads` array. Here “Force” is the general modeling term and includes torques.

The `spanning_force` exposes the fully expanded axial-force component through
TOML. Its geometry variables $(s_x,s_y,\ell,u_x,u_y)$, length rate
$\dot\ell$, scalar force $f$, and global force components $(F_x,F_y)$ remain
explicit canonical variables. A constant value, predefined linear
spring-damper, or general expression supplies the scalar constitutive law. The
component contributes nine local implicit equations, then applies equal and
opposite point forces to its two marker owners. Linear-law partials are
analytical; expression partials use local forward-mode differentiation.

The `torsional_spring_damper` uses the orientations of two ordinary or floating
markers. It contributes one explicit torque variable and one local
constitutive implicit equation, with an analytical Jacobian. Equal-and-opposite
torques enter the two body moment balances as separate component-local
contributions. Its optional damping time scale supplies the estimate $c=\tau k$
when an explicit damping coefficient is omitted.

A planar bushing is an ordinary marker-to-marker force element:

```toml
[support]
type = "bushing"
markers = ["body.mount", "ground.mount"]
translational_stiffness = [100.0, 100.0]
rotational_stiffness = 20.0
damping_time_scale = 0.1
free_position = [0.0, -1.0]
free_angle = 0.0
```

`free_position` and the two translational coefficient vectors are expressed in
the second marker's coordinate system. The reported `F_x`, `F_y`, and `T` are
global force components and torque acting on the first marker;
equal-and-opposite values act on the second. The optional local
`damping_time_scale` estimates each omitted damping coefficient as stiffness
times the stated time scale. An explicit translational or rotational damping
coefficient overrides its corresponding estimate. Omitted damping scales,
rotational coefficients, `free_position`, and `free_angle` default to zero. A
bushing contributes no constraint equations and has no special assembly
behavior.

A common motion law uses a predefined function:

```toml
[crank_drive]
type = "rotational_motion"
markers = ["crank.ground_end", "ground.origin"]
function = "constant_speed"
initial_angle = 0.0
angular_velocity = "crank_speed"
```

The loader supplies consistent angle, angular-velocity, and angular-acceleration
functions and wraps the prescribed relative angle to its principal value. The
body orientation itself remains continuous through a complete rotation.

The corresponding `translational_motion` component uses the second marker's
local $y$-axis and defines

$$
d=(P_i-P_j)^T\hat y_j.
$$

It retains explicit `distance_m`, `velocity_m`, `acceleration_m`, and `force_m`
variables. At each level one local equation defines the coordinate from marker
kinematics and a second prescribes its time history. The reaction acts only
along $\hat y_j$, so the component remains a one-direction primitive rather
than a complete prismatic joint. These coordinate kinematics can also be
used by the rack-and-pinion coupling.

An unprescribed `distance_coordinate` uses the same marker projection but owns
only `distance`, `velocity`, and `acceleration`. Its three local equations
define those values from the marker kinematics. It adds no reaction and no
constraint by itself. Its velocity may be selected as a state in the same way
as an enabled revolute-joint `omega`.

The general `coupler` connects two or more coordinate ports through

$$
\sum_k c_kq_k-b=0.
$$

This relation is assembled at position, velocity, and acceleration levels and
owns one scalar reaction. Each coordinate receives generalized reaction
$c_k\lambda$. Revolute coordinates convert it to marker torque; distance
coordinates convert it to marker force. The coupler is therefore an ordinary
component-local ideal constraint rather than a new analysis method. The
initial weighted coordinate sum is the default offset, so a ratio can be added
without disturbing the model's supplied phase. Mixed rotation and distance
ports describe elements such as a screw; the coefficient multiplying rotation
then has the dimensions of travel per radian and converts the common reaction
between force and torque consistently.

For an advanced prescribed rotational motion, `function = "expression"`
accepts `angle`, `angular_velocity`, and `angular_acceleration` strings. The
translational form correspondingly accepts `distance`, `velocity`, and
`acceleration`. The restricted evaluator supports numeric literals, `t`,
numeric parameters, arithmetic, `pi`, `e`, and the approved elementary
functions. Prescribed-motion expressions reject assignments, property access,
arbitrary calls, and general Julia evaluation.

Applied-force and applied-torque laws may additionally reference registered
scalar configuration and velocity variables with names such as `pin.theta`,
`pin.omega`, `travel.distance`, and `travel.velocity`. The dotted form is a
restricted qualified-name lookup, not Julia property access. The loader
resolves those names to canonical indices and gives a state-dependent law one
local load variable and constitutive implicit equation. Forward-mode dual
numbers provide the corresponding local Jacobian partials.

## 4. State selection

Before correction, an optional `[initial_conditions]` table can replace
matching body and relative-coordinate values with one sample from a `.simp`
result. Compatibility is established from qualified component names, element
types, variable names, and variable kinds rather than canonical column numbers.
This lets a static assembly model and a later dynamic model differ in their
force elements while sharing the named mechanical configuration. Velocities
are transferred only when explicitly requested; accelerations, reactions, and
force variables are always recomputed.

The semantic selector `sample = "static"` recovers the equilibrium
configuration at the beginning of a dynamic result that records
static-equilibrium initialization. It is intentionally distinct from merely
asking for the first sample: the loader verifies that the source analysis
actually performed the static stage.

When `[state_selection]` is omitted, the loader first makes the resulting
configuration constraint-consistent using the IC weights. It then assembles
the velocity-constraint partial matrix at that corrected configuration and at
the requested starting time, and applies column-pivoted QR. The non-pivot
physical velocity components become the independent states. Each selected body
`V_x`, `V_y`, or `omega` is paired with its corresponding position or
body-fixed pseudo angle. A revolute joint with `rotation_coordinates = true`
adds its relative `omega` to the candidates, paired with the joint's continuous
relative `theta` and `alpha`. A `distance_coordinate` similarly adds its
relative `velocity`, paired with `distance` and `acceleration`. Thus most
models need no state-selection input.

Before QR, each body's translational-velocity columns are scaled by its
characteristic length while its angular-velocity column is scaled by one. Each
nonzero constraint row is then normalized to unit Euclidean norm. Rank,
automatic column pivots, redundant-row detection, and preferred-state checks
all use this same scaled matrix. This prevents a change of geometric size or a
mixture of translational and rotational coefficients from deciding the state
choice merely through units. The body lengths and row and column scales are
retained in the state-selection diagnostics.

The numerical rank, rather than the written equation count, determines
mobility. After selecting dependent velocity columns, a QR factorization of
the transpose of that thin column block identifies an independent set of
actual constraint rows. For each redundant scalar ideal constraint, its
position, velocity, and acceleration equations and its indeterminate reaction
are omitted from the active solve. Their canonical result slots remain present;
the inactive reaction is recorded as zero and the suppressed velocity-equation
name is retained in the state-selection diagnostics.

An optional analysis preference has the form

```toml
[state_selection]
method = "preferred"
preferred_velocities = ["crank.omega"]
allow_fallback = false
maximum_condition_number = 6.7108864e7
```

Names select individual physical velocity components. The corresponding planar
positions, body-fixed pseudo angles, enabled revolute-joint angles, or distance
coordinates are inferred. Planar body `V_x`, `V_y`, and `omega` components,
enabled joint `omega` components, and distance-coordinate `velocity` components
are supported. For example,
`preferred_velocities = ["pin.omega"]` selects the optional relative coordinate
owned by `pin`. A fully driven kinematic model, such as the
slider-crank example, has no independent states and omits the table.

The loader tests the complementary dependent-velocity block at the corrected
initial configuration. It must have the constraint rank and a condition number
no greater than `maximum_condition_number`, whose default is
$1/\sqrt{\epsilon}$ for `Float64`. If the block fails either test,
`allow_fallback = true` selects the automatic QR states and records the
fallback; `false` reports why the preferred set is unusable. Diagnostics retain
both the requested and actually selected velocity names and the tested
condition number.

This preference initializes the analysis; it is not a permanent restriction.
If the partition becomes unhealthy during a dynamic run, the program may use
pivoted QR to choose different states while retaining every variable's BDF
history. The result file records each actual change.

## 5. Analysis choice

The default analysis mode is `automatic`. The loader determines mobility from
the assembled variable and equation inventory. Every model uses the same
complete implicit system and DDASSL predictor-corrector. A model with
independent states integrates those states; for a zero-state mechanism DDASSL
acts as a variable-step continuation solver, predicting and simultaneously
correcting every model variable.

```toml
[analysis]
mode = "automatic"
```

The current modeling program implements the `StateSelected` formulation only.
Its deficit is zero and is supplied internally rather than selected in TOML.
The alternative deficit and stabilization formulations remain paper studies.

An explicit `mode = "dynamic"` uses the same path and labels the result as
dynamic. An explicit `mode = "kinematic"` requires a zero-state mechanism.
Applied forces affect balances and reactions but do not determine mobility.

An explicit `mode = "static"` selects positions, orientations, reactions, and
static applied-force variables. It simultaneously solves force and moment
balance, position-level constraints and motion prescriptions, and static force
definitions with all velocities and accelerations zero. With multiple output
times, the program performs quasi-static continuation: time-dependent forces
and motion generators are evaluated at each time while inertial and damping
effects remain absent. Each equilibrium provides a predictor for the next, and
a failed interval is subdivided automatically. A one-sample run is an ordinary
single-point static equilibrium calculation.

A dynamic run may request an equilibrium configuration before integration:

```toml
[analysis]
mode = "dynamic"
initialization = "static_equilibrium"
```

The static equations are solved once at the starting model time with zero
velocity and acceleration. Their positions, orientations, relative positions,
reactions, and applied loads seed the dynamic canonical vector. Model-declared
velocities are restored rather than taken from the static solution, and the
ordinary simultaneous dynamic initializer then corrects accelerations,
reactions, force variables, and other algebraic quantities. Thus the static
loads are guesses for the dynamic Newton solve, not prescribed values. The
same force set is used in both stages. A different force set can instead be
used by saving the static run and naming that result in the dynamic model's
`[initial_conditions]` table. Conditional assembly forces remain a possible
later extension.

For an ideal mechanism, the first static Newton tangent can be singular when
all initial reaction estimates are zero. In that case a pivoted least-squares
step establishes reaction estimates and reduces the implicit equations; the
subsequent iterations use the ordinary sparse Newton tangent.

For configurations that remain difficult for direct Newton iteration, the
model can instead request dynamic relaxation:

```toml
[analysis]
mode = "dynamic"
initialization = "static_equilibrium"
static_method = "dynamic_relaxation"
```

The relaxation advances the ordinary implicit dynamic equations in pseudo-time
while every component continues to see the fixed physical analysis time. It
begins with zero velocity and acceleration, reduces all velocity and
acceleration variables between pseudo-time intervals, and tries a short static
Newton correction after each interval. The final answer is therefore still a
solution of the static implicit equations;
the dynamic trajectory is only a convergence device and is not included in the
simulation history. `relaxation_duration`, `relaxation_reduction_factor`, and
`relaxation_cycles` provide model-level controls when the defaults do not match
the mechanical time scale.

The first dynamic acceptance model is
[`torque-driven-four-bar.toml`](../../models/planar/torque-driven-four-bar.toml). Its one
state is selected automatically; its trajectory agrees with the earlier
programmatic dynamic reference.

## 6. Simulation

Default simulation and output times belong to the model description:

```toml
[simulation]
start_time = 0.0
end_time = 5.0
output_samples = 81
relative_tolerance = 1.0e-7
absolute_tolerance = 1.0e-9
initial_step = 1.0e-6
maximum_step = 0.005
```

If this table or one of its fields is omitted, these shown values are used.
Command-line duration and sample arguments override `end_time` and
`output_samples`; duration is measured from `start_time`.


## 7. Loading the file


Run the general command-line program with either a filename

```bash
./bin/simp2d models/planar/constant-speed-slider-crank.toml
```

or TOML supplied on standard input:

```bash
./bin/simp2d - < models/planar/constant-speed-slider-crank.toml
```

Add `--output results/examples/run.simp` to either command to retain the
completed run in the versioned HDF5 result format described in
[`simulation-result-files.md`](../common/simulation-result-files.md).

## 8. Programmer's interface


`load_planar_model` is a Julia API function, that reads a TOML model from a filename or input stream and constructs an
in-memory `LoadedPlanarModel`. During that operation it validates the hierarchy,
registers the components, allocates the unreduced canonical system, constructs
the markers and elements, and prepares the initial canonical values and
analysis metadata. It does not read a `.simp` result file; `read_result` is the
separate API for that purpose. The command-line program shown above calls
`load_planar_model` internally before running the analysis.

As a regression example, loading
[`constant-speed-slider-crank.toml`](../../models/planar/constant-speed-slider-crank.toml)
constructs the same 27-variable canonical model as the earlier slider-crank
assembled directly in Julia code.

`run_planar_model` dispatches both kinematic and dynamic planar mechanisms and
returns the same sampled result interface for HDF5 storage, graphics, and CSV
export. Registered compound elements may later use nested tables as their
private structure and expose named interface markers, without changing the
qualified-name convention. Units, reusable definitions, acceleration-dependent
loads, and spatial components remain outside this first executable format.
