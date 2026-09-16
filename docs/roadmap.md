# Development Roadmap

The planar modeler is mature for its intended rigid-mechanism scope. The
spatial modeler has reached a useful general foundation and can run substantial
models, but has had less use. The present phase is to make that work readable,
reproducible, and publicly available before another large modeling expansion.

## Current work: first public release

- Complete the repository license, citation information, contribution guide,
  automated tests, and clean-install instructions.
- Check the public documentation against the implemented planar and spatial
  interfaces.
- Keep generated results and local review files out of version control while
  retaining reproducible source models.
- Publish a tagged release from a clean checkout after the Julia and SimpView
  Web checks pass on macOS and Linux.
- Preserve the working methods manuscript and its executable evidence without
  making completion of the paper a prerequisite for releasing the program.

The detailed publication steps are in the repository
[release checklist](../RELEASE_CHECKLIST.md).

## Validation after release

- Exercise spatial models beyond the existing examples, particularly contact,
  tire lift-off, suspension limits, and full-vehicle behavior.
- Review difficult or failed analyses through the incremental `.simp` history
  rather than treating only successful runs as useful results.
- Separate paper-only dependencies and numerical experiments from the normal
  package environment when doing so no longer makes the historical evidence
  harder to reproduce.

## Methods paper

The journal paper will explain the Sparse Fully Consistent Modeling Method and
compare it with the executable alternative formulations. The working draft,
outline, and claims-to-evidence matrix are under [`paper/`](../paper/). The
paper is a consolidation task after the public repository is understandable
and reproducible; it is not a gate on release.

## Deliberately deferred

- Pressure-angle or load-dependent real gear contact remains future work. The
  ideal `gear_pair` is the supported gear model.
- More elaborate staged joints and contacts should be added only for a concrete
  modeling need. Current stage-dependent forces cover the known use cases.
- Additional viewer presentation controls should follow actual inspection
  difficulties rather than expanding the interface speculatively.
- A formal units system and a larger assembly-language abstraction remain
  possible extensions. Current TOML units and Lua assemblies are the supported
  input paths.

The alternative planar analysis formulations remain executable paper evidence.
Documentation cleanup must not remove them merely because the supported TOML
path uses the StateSelected Fully Consistent formulation.
