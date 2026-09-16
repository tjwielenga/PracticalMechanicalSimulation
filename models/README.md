# Model Files

Executable model inputs are separated by dimension:

- `planar/` contains the mature two-dimensional example library and is run
  with `bin/simp2d`.
- `spatial/` contains models supported by the broader but less-validated
  three-dimensional program and is run with `bin/simp3d`.

Both use hierarchical TOML and both produce the same dimension-tagged `.simp`
result format. A model's `[model].dimension` must agree with the reader used to
load it.
