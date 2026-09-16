# Test Suites

The default suite checks the supported modeling program through its package and
TOML interfaces:

```bash
julia --project=. test/runtests.jl
```

It covers representative kinematic, dynamic, static, bushing, state-selection,
result-file, CSV, model-extraction, command-line, and DDASSL behavior. Keep this
suite concise and avoid depending on standalone programs in
`examples/planar/`.

During development, run only the affected part of the core suite:

```bash
julia --project=. test/runtests.jl spatial
julia --project=. test/runtests.jl planar
julia --project=. test/runtests.jl ddassl
```

More than one name may be supplied when a change crosses boundaries. `all`
and `core` both run all three groups and are equivalent to supplying no name.
The focused suite should be the normal check while developing an element. Run
the complete core suite at larger checkpoints or before a release.

The paper verification suite preserves the alternative formulations and the
development comparisons used to support the written treatment:

```bash
julia --project=test/paper -e 'using Pkg; Pkg.instantiate()'
julia --project=test/paper test/paper/run_all.jl
```

Its environment contains DASSL, OrdinaryDiffEq, and Sundials. Those comparison
packages are intentionally not installed with the supported modeling program.

These checks intentionally exercise the executable analysis studies in
`examples/planar/`, including the deficit formulations, stabilization methods,
reduced-coordinate comparisons, Euler parameters, force-element development,
and intermediate assembly designs. A formulation should not be removed from
this suite merely because the current TOML program supersedes it.

The paper suite is operationally separate from the default suite, but it still
imports the evolving `src/common/HistoricalDDASSL.jl`; it is not a numerically frozen
historical baseline. The [planar consolidation](../paper/planar-methods-consolidation.md)
records the resulting verification issue and the choice that must be made
before publication comparisons are finalized.
