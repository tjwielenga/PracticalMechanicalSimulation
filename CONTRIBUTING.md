# Contributing

Practical Mechanical Simulation is being prepared as an open engineering and
research project. Bug reports, reproducible models, documentation corrections,
tests, and focused code changes are welcome.

Before proposing a change:

1. Read [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) for the current program scope
   and terminology.
2. Keep planar and spatial elements in their separate source, model,
   documentation, and test directories.
3. Preserve component-local equations and Jacobian contributions. The global
   system is assembled from those local definitions.
4. Do not add proprietary source material or models. The author's historical
   archive is intentionally kept outside the public repository.

## Development setup

Install Julia 1.12 and instantiate the repository environment:

```bash
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

Run the supported program tests with:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

During development, a focused suite is faster:

```bash
julia --project=. test/runtests.jl planar
julia --project=. test/runtests.jl spatial
julia --project=. test/runtests.jl viewer
julia --project=. test/runtests.jl ddassl
```

The larger paper-verification suite is intentionally separate:

```bash
julia --project=test/paper -e 'using Pkg; Pkg.instantiate()'
julia --project=test/paper test/paper/run_all.jl
```

To check the browser viewer, install a compatible Node.js release and run:

```bash
cd apps/SimpViewWeb
npm ci
npm test
npm run build
```

## Changes and documentation

- Add or update a focused test for changed behavior.
- Update both the User's Guide and Technical Manual when an interface or
  element equation changes.
- Use dollar-sign LaTeX delimiters in Markdown.
- Prefer the established mechanical terms: implicit equations, deficit,
  reaction, state selection, and Fully Consistent method.
- Keep generated `.simp`, CSV, viewer-build, and benchmark output out of Git.
- Do not commit local Word review files; the Markdown manuscript is the
  authoritative paper source.

Small, coherent changes are easier to review than broad rewrites. If a change
alters the formulation or a public model-file convention, describe the reason
and migration effect in the proposed change.
