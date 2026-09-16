# Planar Modeling Utilities

Status: implemented allocation and assembly layer for the planar modeler

## 1. Purpose

The first component examples constructed bodies, markers, constraints, and
analysis solvers explicitly. That made each interface visible while it was being
developed, but the later four-bar and slider-crank examples began repeating the
same allocation-dependent scaffolding.

`PlanarModeling` consolidates those stable operations without changing the
mathematical component model. It is a thin layer above `AutomaticAnalysis`,
`PlanarAppliedForces`, and `PlanarComponentAssembly`.

## 2. Allocated component constructors

After a `ModelLayout` has assigned canonical locations, the utility layer
constructs bodies and fixed or floating markers; ideal constraints and compound
joints; rotational and translational motion generators; measurable relative
coordinates and coordinate couplers; gears, rack-and-pinion, and belt spans;
and the bushing, contact, spanning, and torsional force elements.

These functions translate named layout entries into the explicit index fields
used by the component kernels. They do not hide or replace the canonical layout.

## 3. Executable model assembly

`assemble_planar_model` accepts one collection of components that own equation
blocks and another collection that contributes loads to those equations. Keeping
these collections distinct preserves the established distinction between
equation ownership and equation contribution.

## 4. Analysis and continuation

`solve_planar_analysis!` performs a square Newton solve for either a standard
metadata-selected analysis or an explicitly supplied `AnalysisSelection`. It
uses the component-assembled sparse Jacobian and expands each correction back
into canonical storage.

`predict_planar_configuration!` applies the second-order configuration estimate

$$
R_{n+1}^{(0)}=R_n+hV_n+\frac{h^2}{2}a_n,
$$

$$
\theta_{n+1}^{(0)}=\theta_n+h\omega_n+\frac{h^2}{2}\alpha_n
$$

to a supplied collection of bodies. Constraint correction remains a separate
analysis step.

## 5. Scope

These utilities remain below the user-facing model-description language in
`PlanarModelIO`. `SimulationRunner` provides reusable loading and simulation,
while `CommandLine` adapts filenames or standard input to that API. The example
launcher contains no model-specific simulation logic.

All supported TOML models use this shared layer. Programmatic paper examples
may use it directly or retain their earlier explicit assembly when that form is
part of the experiment being documented.
