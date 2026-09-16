local sim3d = require "sim3d"
local model, analysis, simulation, graphics =
    sim3d.model, sim3d.analysis, sim3d.simulation, sim3d.graphics
local ground, rigid_body, marker =
    sim3d.ground, sim3d.rigid_body, sim3d.marker
local simple_body = require "spatial.simple_body"

model {
    name = "simple_body_surface",
    title = "User-defined vehicle body surface",
    dimension = "spatial"
}

analysis {mode = "dynamic"}

simulation {
    start_time = 0.0,
    end_time = 2.0,
    output_samples = 121,
    relative_tolerance = 1.0e-6,
    absolute_tolerance = 1.0e-8
}

graphics {
    background = "white",
    body_palette = "colorblind"
}

ground {name = "ground"}
marker {
    name = "ground.origin",
    graphics = {
        shape = "xy_frame",
        axis_length = 0.8,
        plane_size = 0.35,
        plane_color = "gray70",
        opacity = 0.15,
        label = "ground"
    }
}

rigid_body {
    name = "vehicle",
    mass = 1000.0,
    inertia = {400.0, 800.0, 900.0},
    angular_velocity = {0.0, 0.0, 0.25},
    graphics = {show_default = false}
}

simple_body {
    name = "bodywork",
    body = "vehicle",
    origin = {-2.5, 0.0, -0.6},
    hood_length = 1.0,
    roof_front = 1.5,
    roof_back = 3.5,
    window_back = 4.0,
    length = 5.0,
    front_hood_height = 0.8,
    windshield_height = 1.0,
    roof_height = 1.7,
    back_glass_height = 1.0,
    trunk_height = 0.8,
    rocker_width = 1.2,
    body_width = 1.8,
    roof_width = 1.5,
    body_color = "steelblue",
    body_opacity = 0.82,
    glass_color = "gray15",
    glass_opacity = 0.48,
    draw_edges = true,
    edge_color = "gray20",
    edge_width = 1.2
}
