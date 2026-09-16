# Portable viewer document

SimpView should not depend on the internal objects of either the solver or one
graphics library. The simulation result remains the HDF5 `.simp` file. A
versioned viewer document provides the boundary between result reconstruction
and presentation.

The logical document contains:

- the title and planar or spatial dimension;
- one or more choices, such as modal shapes;
- the sample times for each choice;
- a renderer-independent scene graph;
- named result signals and bookmarks;
- default colors and visibility settings.

The scene graph has three parts. `meshes` contains reusable unit geometry or a
fixed indexed surface. `tracks` contains sampled position, quaternion, and
scale. `instances` place a mesh on a track, with an optional fixed local
transform. Graphics fixed to the same rigid body can therefore share one pose
track instead of repeating their world coordinates at every output sample.
Forces, torques, and compliant connectors use scale to show changing magnitude
or length. Geometry that actually deforms, such as a changing belt wrap, may
retain a small procedural description rather than pretending it is rigid.

Coordinates, scales, and vectors are three-component arrays. Quaternions are
stored as `[x, y, z, w]`. Trajectory matrices are written as `samples ×
components`. Surface face and edge indices remain one-based because they
originate in the model and are part of the portable document, not the renderer.
A renderer converts them to its native index convention.

The `.simp` file stores this information as native HDF5 datasets under
`/graphics`: fixed meshes, compact pose tracks, instances, dynamic surfaces,
follow targets, signals, and result choices. The web viewer reads these arrays
directly with HDF5 compiled to WebAssembly and builds the logical document in
memory. No opaque JSON attachment is stored in a result.

Viewer coordinates, quaternions, scales, opacities, and mesh vertices are
stored as `Float32`; they describe display geometry and do not reduce the
precision selected for the simulation history. Track rows are packed into one
compressed dataset per field with offset and count arrays, rather than hundreds
of small HDF5 groups. Each rigid body has one motion track. Body geometry,
inertia ellipsoids, markers, and other rigidly attached graphics reference that
track through fixed local transforms whenever their ownership can be
established. Loads and torques retain independent tracks because their
directions and magnitudes change.

Rewriting the graphics section rebuilds the HDF5 file around the unchanged
model and result groups. This reclaims the space occupied by the previous
graphics section instead of allowing repeated viewer preparation to enlarge the
file.

TOML and Lua previews use the same in-memory logical document, produced by the
local Julia preview service before a simulation has been run.
