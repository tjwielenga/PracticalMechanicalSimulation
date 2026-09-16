local sim3d = require "sim3d"
local model, analysis, simulation, parameters, graphics =
    sim3d.model, sim3d.analysis, sim3d.simulation,
    sim3d.parameters, sim3d.graphics
local ground, rigid_body, marker =
    sim3d.ground, sim3d.rigid_body, sim3d.marker
local applied_force, gravity = sim3d.applied_force, sim3d.gravity
local leaf_spring = require "spatial.leaf_spring"

model {
    name = "leaf_spring_assembly",
    title = "Reusable leaf-spring assembly",
    dimension = "spatial"
}

analysis {
    mode = "dynamic",
    initialization = "static_equilibrium",
    static_method = "dynamic_relaxation",
    relaxation_duration = 0.25,
    relaxation_max_cycles = 10
}

simulation {
    start_time = 0.0,
    end_time = 1.0,
    output_samples = 61,
    relative_tolerance = 1.0e-6,
    absolute_tolerance = 1.0e-8,
    initial_step = 1.0e-7,
    maximum_step = 0.002
}

parameters {
    dynamic_load = 1000.0,
    load_frequency_hz = 0.5
}

graphics {
    background = "white",
    body_palette = "colorblind"
}

ground {
    name = "ground"
}

marker {
    name = "ground.origin",
    graphics = {
        shape = "xy_frame",
        axis_length = 0.25,
        plane_size = 0.1,
        plane_color = "gray70",
        opacity = 0.16
    }
}

rigid_body {
    name = "leaf",
    mass = 3.2,
    inertia = {0.0016, 0.035, 0.0335},
    position = {0.0, 0.0, 0.0}
}

leaf_spring {
    name = "rear_spring",
    frame = "ground",
    leaf = "leaf",
    front_eye = {-0.6390, 0.0, -0.0689},
    leaf_center = {0.0, 0.0, 0.0},
    rear_shackle_eye = {0.7047, 0.0, 0.0692},
    rear_frame_eye = {0.7086, 0.0, 0.2024},
    up = {0.0, 0.0, 1.0},
    leaf_stiffness = 36532.0,
    twist_stiffness = 2740.0,
    width = 0.076,
    mount_stiffness = {7000000.0, 7000000.0, 1000000.0},
    mount_rotational_stiffness = {10000.0, 10000.0, 81.4},
    damping_time_scale = 0.006,
    nominal_load = 1000.0
}

marker {
    name = "leaf.load"
}

marker {
    name = "ground.load_axis"
}

applied_force {
    name = "vertical_load",
    markers = {"leaf.load", "ground.load_axis"},
    expression = "-dynamic_load*sin(2*pi*load_frequency_hz*t)"
}

gravity {
    name = "gravity",
    acceleration = {0.0, 0.0, -9.80665},
    bodies = {
        "leaf",
        "rear_spring.front_link",
        "rear_spring.rear_link",
        "rear_spring.shackle"
    }
}
