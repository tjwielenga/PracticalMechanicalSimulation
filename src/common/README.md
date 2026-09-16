# Common Source

This directory contains numerical and result-processing modules shared by the
planar and spatial modelers:

- canonical variable and equation metadata;
- the variable-step BDF integrator;
- modal analysis;
- `.simp` result storage;
- saved-result initialization shared by both modelers;
- CSV export and model extraction.

These files remain direct submodules of `PracticalMechanicalSimulation`.
Directory placement does not introduce an additional Julia namespace.

`AutomaticAnalysis.jl` defines the canonical allocation, equation ownership,
analysis selections, and dense or sparse assembly callbacks.
`HistoricalDDASSL.jl` advances a selected square implicit system with
variable-step, variable-order BDF formulas and retained sparse
factorizations. `ModalAnalysis.jl` obtains finite modes from the two sparse
partials of the implicit equations. `ResultIO.jl` owns the versioned `.simp`
format and keeps in-progress files readable when an analysis fails.

`AssemblyExpansion.jl`, `InputUnits.jl`, and `ScalarExpressions.jl` are input
services rather than solver layers. `SavedInitialConditions.jl` transfers
compatible named values from an earlier result without making the result file
an executable Julia object.
