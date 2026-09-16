# Design Record

These documents preserve experiments and intermediate designs that led to the
current program. They support the paper and explain important decisions, but
they are not the reference for current user-visible behavior.

- [DDASSL structure and data flow](ddassl-structure-and-data-flow.md) records a
  reading of the historical Netlib Fortran solver.
- [Generalized BDF step prototype](generalized-bdf-step-prototype.md) records
  the first fixed-order experiment separating Newton unknowns from integration
  error control.
- [Modular sparse assembly](modular-sparse-assembly.md) records the first
  two-body sparse component prototype.
- [Automatic assembly of analysis equation sets](automatic-analysis-assembly.md)
  records the progression from metadata-selected analysis prototypes to the
  current canonical model.

For current behavior, use the [planar](../planar/README.md) and
[common](../common/README.md) sections of the Technical Manual.
