# Technical Manual

The Technical Manual contains the mathematical and software implementation.
It is separated by model dimension so that the mature planar program cannot be
confused with the broader but less-tested spatial program. General numerical
machinery is documented separately from both.

## Current planar implementation

The [planar Technical Manual](planar/README.md) documents the implemented 2D
model language, element equations, allocation and assembly utilities, and the
procedure for adding a planar component.

Its source path is:

```text
planar TOML model
   -> PlanarModelIO
   -> AutomaticAnalysis allocation
   -> PlanarModeling and PlanarComponentAssembly
   -> SimulationRunner
   -> ResultIO
```

The corresponding files are separated into [`src/planar`](../src/planar/) and
[`src/common`](../src/common/). The 3D implementation is independently
contained in [`src/spatial`](../src/spatial/).

This is the supported modeling program. Its user interface is documented in
the [Planar Modeler User's Guide](../docs/planar/README.md).

## Common mathematical and numerical methods

The [common Technical Manual](common/README.md) covers the solver-neutral
mechanical formulation, DDASSL integration, state selection, modal analysis,
and `.simp` result files. These methods are intended to serve both dimensions,
but each document states what has actually been implemented and tested.

## Spatial implementation

The [spatial Technical Manual](spatial/README.md) documents the implemented
rigid-body formulation, supported analyses, and 3D element library.

## Design record

The [design record](design-record/README.md) preserves prototypes and studies
that led to the current program. They remain useful evidence for the paper,
but their future-tense proposals and early limitations do not describe current
program behavior.

## Related material

- [User documentation](../docs/README.md)
- [Fully Consistent paper workspace](../paper/README.md)
- [Test-suite policy](../test/README.md)
