# Generated Results

This directory keeps generated analysis output separate from models, examples,
and source code.

- `examples/planar/` and `examples/spatial/` contain results and exports
  produced while running documented models. Regenerate the complete supported
  set with `bin/regenerate-example-results`.
- `benchmarks/` contains retained benchmark runs used for timing or viewing.
- `scratch/` contains short-lived experiments.

The contents of these subdirectories are ignored by Git. Only their empty
structure is retained. A result needed permanently by an automated test belongs
in `test/fixtures/` instead.

The regeneration script runs every top-level planar TOML model and every
top-level spatial TOML model. It also runs the supported Lua assembly examples,
with dependent rally-van analyses ordered after the static solution. Preliminary
models under `models/spatial/vehicle-development` are intentionally excluded.
The ignored `results/examples/regeneration-report.txt` records successes and
failures from the latest batch. The high-speed rally-van rollover is retained
as an expected partial result because its accepted history is useful for
examining the model immediately before integration fails.
