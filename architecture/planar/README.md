# Planar Technical Manual

This section documents the implemented two-dimensional modeler. All bodies,
markers, elements, variables, equations, and examples discussed here are
planar unless explicitly identified as a common numerical method.

- [TOML model description](toml-model-description.md) follows a planar model
  through validation, component registration, allocation, initialization, and
  analysis selection.
- [Planar element formulations](planar-element-formulations.md) gives the
  equations, reaction conventions, force laws, and initialization behavior of
  the supported element library.
- [Planar modeling utilities](planar-modeling-utilities.md) identifies the
  allocation and assembly boundary below the TOML front end.
- [Adding a planar component](adding-planar-component.md) follows one element
  through registration, equations, sparse assembly, results, graphics, and
  tests.

The analyst-facing interface is in the
[Planar Modeler User's Guide](../../docs/planar/README.md). Integration, state
selection, modal analysis, and result storage are described in the
[common Technical Manual](../common/README.md).
