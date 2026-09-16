# Adding a Planar Component

Status: developer guide for the current internal extension boundary

The current component system preserves mechanical locality, but TOML element
types are registered explicitly in the planar loader. Adding a type therefore
requires a small coordinated path through declaration, allocation, equations,
model loading, results, tests, and—when the element has visible geometry—the
stored-result viewer. The planar bushing is the compact example of that path.

## Component path

```text
TOML table
    -> validate names and fields
    -> declare local variables and equation blocks
    -> allocate fixed canonical indices
    -> construct the allocated component
    -> contribute owned equations and load terms
    -> assemble the sparse implicit system
    -> retain canonical histories in the result file
```

The component never chooses global indices. It names and declares its local
quantities before layout construction, then receives their fixed locations from
the completed `ModelLayout`.

## 1. Define the allocated component

The bushing's data type is in
[`PlanarComponentAssembly.jl`](../../src/planar/PlanarComponentAssembly.jl). It stores its
name, two markers, constitutive data, and the allocated indices of three load
variables and three equations. Marker objects provide point and orientation
kinematics without giving the bushing knowledge of the owning body layout.

Choose ownership deliberately:

- a component variable is part of the canonical solution and stored in results;
- an owned equation defines that component variable or its kinematics;
- an equation contribution adds force, torque, or reaction terms to rows owned
  by another component;
- a derived value that is neither solved nor needed in output should remain
  local to evaluation.

## 2. Declare variables and equations

`bushing_registration(name)` returns a `ComponentRegistration` with

```julia
F_x, F_y, T
```

as level-two `applied_load` variables and a `:load` equation block containing
the corresponding constitutive definitions. Declarations use local names;
allocation qualifies them with the component name and assigns canonical
indices.

Derivative level is part of both formulation and scaling. Use level zero for
positions and geometry, level one for velocities and rates, and level two for
accelerations, reactions, forces, and torques. An ideal scalar constraint also
declares a `ConstraintFamilyDeclaration` linking its reaction to its position,
velocity, and acceleration equations. That link is required for redundant-row
suppression.

## 3. Allocate and construct

The loader first collects and sorts element names, creates each registration,
and calls `allocate_component_variables!` and
`allocate_component_equation_block!`. Only after `finish_layout` does
`allocated_planar_bushing` construct the executable component from ranges
returned by `component_variable_indices` and
`component_equation_indices`.

Keep these two phases separate. Registration must depend only on the component
type and name; construction may resolve referenced markers, bodies, and
parameters.

## 4. Supply implicit equations

`executable_blocks(component)` returns rows owned by the component. For the
bushing these equations are

$$
F_x-F_x^\mathrm{law}=0,
\qquad
F_y-F_y^\mathrm{law}=0,
\qquad
T-T^\mathrm{law}=0.
$$

Each `ExecutableEquationBlock` contains

- its exact target rows;
- a function that accumulates the implicit equations; and
- a function that accumulates its Jacobian entries.

The bushing evaluates translation in marker 2's frame and writes global load
components. Its present constitutive block uses centered numerical partials for
marker-dependent terms and exact unit partials for its explicit load variables.
Other components should provide analytical local partials where practical.

The Jacobian callback receives the BDF derivative coefficient and must
accumulate

$$
\frac{\partial F}{\partial y}
+c_j\frac{\partial F}{\partial\dot y}.
$$

Only touch declared dependency columns. Fixed topology lets the sparse
assembler reuse a stable pattern and retain locality for ordering and
factorization.

## 5. Contribute to other components

`equation_contributions(component)` returns terms added to equations owned
elsewhere. The bushing applies its explicit global force and torque to the two
marker owners with equal and opposite signs. Body markers add both force and
the moment about the body origin; ground markers add no balance rows.

Keeping constitutive definitions separate from load-to-body contributions is
important. It permits the unreduced system to retain explicit force variables
and allows static, initialization, dynamic, and result-output selections to use
the same component.

## 6. Add the TOML type

The current registry is the explicit set in
[`PlanarModelIO.jl`](../../src/planar/PlanarModelIO.jl). A new type must be added to the
appropriate classification, the known-type validation set, registration and
allocation loops, field validation/construction branch, and the equation and
contribution component lists.

Validate at the model boundary:

- exact marker or body reference counts;
- referenced names and owner requirements;
- vector lengths and numeric values;
- positivity or sign restrictions;
- required fields and documented defaults; and
- any frame convention or endpoint ordering.

The executable type name and all fields belong in the
[TOML User's Guide](../../docs/planar/toml-reference.md). A representative model belongs in
`models/planar/` when it exercises a supported program feature rather than only a
paper formulation.

## 7. Preserve output and graphics

Canonical variables are stored automatically once registered. Choose stable,
mechanical variable names because `.simp` metadata, plot menus, and CSV column
names expose them as `component.variable`.

If the element has meaningful visible geometry, update
[`StoredResultViewer.jl`](../../src/viewer/StoredResultViewer.jl) to reconstruct that
geometry from the embedded TOML and canonical histories. The bushing is drawn
as a compliant connector between its two markers. Viewer support reads the
stored file; it must not rerun the model.

## 8. Test at two levels

Add focused supported behavior to `test/core/model_program_tests.jl`. A force
element should normally have at least

- loader validation;
- a simple value or equilibrium check;
- dynamic or static execution, as applicable;
- stored-result metadata/history coverage; and
- viewer-data reconstruction when visible.

Use `test/paper/` only when a test verifies an analysis formulation or an
intermediate comparison needed by the written treatment. Do not duplicate the
same end-to-end TOML acceptance test in both suites.

Run

```bash
julia --project=. test/runtests.jl
julia --project=. test/paper/run_all.jl
```

The first command is the required supported-program checkpoint. The second is a
larger verification of paper evidence and should be run when internal assembly
or integration behavior changes.

## Completion checklist

- The component owns only its local variables and equations.
- Endpoint order, coordinate frames, units, signs, defaults, and restrictions
  are documented.
- Equation and Jacobian callbacks touch only declared rows and dependencies.
- Static and dynamic selections remain square where the component applies.
- Canonical variable names are suitable for HDF5, plots, and CSV.
- A maintained TOML example and focused core tests cover the supported path.
- Visible geometry can be regenerated from a stored result.
